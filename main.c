#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Provided by basic_io.c in the Keil project. */
extern void vPrintString(const char *pcString);
extern void vPrintStringAndNumber(const char *pcString, uint32_t ulValue);

#define DEBUG_VERBOSE_LOGS 1U

/*
 * Smart Parking Garage Gate System (TM4C123 + FreeRTOS)
 * Requested button mapping from user template:
 *   PF4 -> Driver OPEN      (active-low pull-up, LaunchPad SW1)
 *   PF0 -> Driver CLOSE     (active-low pull-up, LaunchPad SW2)
 *   PE0 -> Security OPEN    (active-high pull-down)
 *   PE1 -> Security CLOSE   (active-high pull-down)
 *   PB0 -> Closed Limit     (active-high pull-down)
 *   PB1 -> Open Limit       (active-high pull-down)
 *   PD1 -> Obstacle         (active-high pull-down)
 *
 * LEDs:
 *   PF1 RED   = closing
 *   PF3 GREEN = opening/reversing
 *
 */

/* ------------------------------- GPIO masks ------------------------------- */
#define LED_RED      (1U << 1)
#define LED_GREEN    (1U << 3)
#define LED_MASK     (LED_RED | LED_GREEN)

#define BTN_PF0      (1U << 0)
#define BTN_PF4      (1U << 4)
#define BTN_PE0      (1U << 0)
#define BTN_PE1      (1U << 1)
#define BTN_PB0      (1U << 0)
#define BTN_PB1      (1U << 1)
#define BTN_PD1      (1U << 1)

#define RCGCGPIO_B   (1U << 1)
#define RCGCGPIO_D   (1U << 3)
#define RCGCGPIO_E   (1U << 4)
#define RCGCGPIO_F   (1U << 5)
#define RCGCGPIO_ALL (RCGCGPIO_B | RCGCGPIO_D | RCGCGPIO_E | RCGCGPIO_F)

/* ------------------------------ timing/config ----------------------------- */
#define INPUT_PERIOD_MS           20U
#define SAFETY_PERIOD_MS          10U
#define LED_PERIOD_MS             30U
#define STATUS_PERIOD_MS          200U
#define DEBOUNCE_SAMPLES          2U
#define ONE_TOUCH_MAX_MS          300U
#define REVERSE_DURATION_MS       500U

/* --------------------------------- states --------------------------------- */
typedef enum
{
    IDLE_OPEN = 0,
    IDLE_CLOSED,
    OPENING,
    CLOSING,
    STOPPED_MIDWAY,
    REVERSING
} GateState;

typedef enum
{
    MOTION_NONE = 0,
    MOTION_MANUAL,
    MOTION_AUTO
} MotionMode;

typedef enum
{
    SRC_DRIVER = 0,
    SRC_SECURITY
} CommandSource;

typedef enum
{
    EV_OPEN_PRESS = 0,
    EV_OPEN_RELEASE,
    EV_CLOSE_PRESS,
    EV_CLOSE_RELEASE,
    EV_STOP_CONFLICT,
    EV_OBSTACLE
} EventType;

typedef struct
{
    EventType type;
    CommandSource src;
    TickType_t tick;
} GateEvent;

/* ----------------------------- shared RTOS vars ---------------------------- */
volatile GateState g_gateState = IDLE_CLOSED;
volatile MotionMode g_motionMode = MOTION_NONE;

static volatile CommandSource g_activeSrc = SRC_DRIVER;
static volatile EventType g_activeDir = EV_OPEN_PRESS;
static volatile TickType_t g_pressTick = 0U;

static QueueHandle_t g_eventQueue;
static SemaphoreHandle_t g_stateMutex;
static SemaphoreHandle_t g_openLimitSem;
static SemaphoreHandle_t g_closeLimitSem;

/* Optional debug snapshots for Keil Watch window. */
volatile uint32_t g_dbgState = (uint32_t)IDLE_CLOSED;
volatile uint32_t g_dbgMode = (uint32_t)MOTION_NONE;
volatile uint32_t g_dbgLastEvent = 0U;

/*
 * Keil uVision Watch + Logic Analyzer: use global volatile scalars (0/1 or enum as uint32).
 * In Debug: View -> Watch Windows -> Watch 1 — add names below.
 * Logic Analyzer: Debug toolbar -> Logic Analyzer -> Setup -> add same symbols, type usually "uint32".
 */
volatile uint32_t g_watchGateState = (uint32_t)IDLE_CLOSED;
volatile uint32_t g_watchMotionMode = (uint32_t)MOTION_NONE;
volatile uint32_t g_watchActiveSource = (uint32_t)SRC_DRIVER;
volatile uint32_t g_watchActiveDir = (uint32_t)EV_OPEN_PRESS;
volatile uint32_t g_watchLedGreen = 0U;
volatile uint32_t g_watchLedRed = 0U;
volatile uint32_t g_watchPortFData = 0U;
volatile uint32_t g_watchBtnDriverOpen = 0U;
volatile uint32_t g_watchBtnDriverClose = 0U;
volatile uint32_t g_watchBtnSecurityOpen = 0U;
volatile uint32_t g_watchBtnSecurityClose = 0U;
volatile uint32_t g_watchBtnOpenLimit = 0U;
volatile uint32_t g_watchBtnClosedLimit = 0U;
volatile uint32_t g_watchBtnObstacle = 0U;
volatile uint32_t g_watchSysTick = 0U;
volatile uint32_t g_watchQueueMsgsWaiting = 0U;

static const char *StateName(GateState s)
{
    switch (s)
    {
        case IDLE_OPEN: return "IDLE_OPEN";
        case IDLE_CLOSED: return "IDLE_CLOSED";
        case OPENING: return "OPENING";
        case CLOSING: return "CLOSING";
        case STOPPED_MIDWAY: return "STOPPED_MIDWAY";
        case REVERSING: return "REVERSING";
        default: return "UNKNOWN_STATE";
    }
}

static const char *ModeName(MotionMode m)
{
    switch (m)
    {
        case MOTION_NONE: return "MOTION_NONE";
        case MOTION_MANUAL: return "MOTION_MANUAL";
        case MOTION_AUTO: return "MOTION_AUTO";
        default: return "UNKNOWN_MODE";
    }
}

static const char *EventName(EventType e)
{
    switch (e)
    {
        case EV_OPEN_PRESS: return "EV_OPEN_PRESS";
        case EV_OPEN_RELEASE: return "EV_OPEN_RELEASE";
        case EV_CLOSE_PRESS: return "EV_CLOSE_PRESS";
        case EV_CLOSE_RELEASE: return "EV_CLOSE_RELEASE";
        case EV_STOP_CONFLICT: return "EV_STOP_CONFLICT";
        case EV_OBSTACLE: return "EV_OBSTACLE";
        default: return "UNKNOWN_EVENT";
    }
}

static const char *SourceName(CommandSource s)
{
    return (s == SRC_SECURITY) ? "SRC_SECURITY" : "SRC_DRIVER";
}

static void Debug_LogStateLocked(const char *reason)
{
#if DEBUG_VERBOSE_LOGS
    vPrintString("\r\n[STATE] ");
    vPrintString(reason);
    vPrintString(" | ");
    vPrintString(StateName(g_gateState));
    vPrintString(" | ");
    vPrintString(ModeName(g_motionMode));
    vPrintString(" | ");
    vPrintString(SourceName(g_activeSrc));
    vPrintString(" | ");
    vPrintString(EventName(g_activeDir));
    vPrintString("\r\n");
    vPrintStringAndNumber("Tick: ", (uint32_t)xTaskGetTickCount());
#else
    (void)reason;
#endif
}

static void Debug_LogEventQueued(EventType type, CommandSource src, TickType_t tick, BaseType_t sent)
{
#if DEBUG_VERBOSE_LOGS
    vPrintString("\r\n[EVENT] ");
    vPrintString(EventName(type));
    vPrintString(" | ");
    vPrintString(SourceName(src));
    vPrintString(" | ");
    vPrintString((sent == pdPASS) ? "queued" : "dropped");
    vPrintString("\r\n");
    vPrintStringAndNumber("Tick: ", (uint32_t)tick);
    vPrintStringAndNumber("QueueDepth: ", (uint32_t)uxQueueMessagesWaiting(g_eventQueue));
#else
    (void)type;
    (void)src;
    (void)tick;
    (void)sent;
#endif
}

static void Watch_UpdateGateLocked(void)
{
    static uint32_t prevState = 0xFFFFFFFFU;
    static uint32_t prevMode = 0xFFFFFFFFU;
    static GateState prevStateEnum = (GateState)0x7FFFFFFF;
    static MotionMode prevModeEnum = (MotionMode)0x7FFFFFFF;

    g_watchGateState = (uint32_t)g_gateState;
    g_watchMotionMode = (uint32_t)g_motionMode;
    g_watchActiveSource = (uint32_t)g_activeSrc;
    g_watchActiveDir = (uint32_t)g_activeDir;
    g_dbgState = g_watchGateState;
    g_dbgMode = g_watchMotionMode;

    if (g_watchGateState != prevState)
    {
        prevState = g_watchGateState;
        vPrintString("\r\n[TRANSITION] ");
        if (prevStateEnum != (GateState)0x7FFFFFFF)
        {
            vPrintString(StateName(prevStateEnum));
            vPrintString(" -> ");
        }
        vPrintString(StateName(g_gateState));
        vPrintString("\r\n");
        prevStateEnum = g_gateState;
        Debug_LogStateLocked("State transition");
    }

    if (g_watchMotionMode != prevMode)
    {
        prevMode = g_watchMotionMode;
        vPrintString("\r\n[MODE] ");
        if (prevModeEnum != (MotionMode)0x7FFFFFFF)
        {
            vPrintString(ModeName(prevModeEnum));
            vPrintString(" -> ");
        }
        vPrintString(ModeName(g_motionMode));
        vPrintString("\r\n");
        prevModeEnum = g_motionMode;
        Debug_LogStateLocked("Mode transition");
    }
}

static void GPIO_Init(void)
{
    SYSCTL_RCGCGPIO_R |= RCGCGPIO_ALL;
    while ((SYSCTL_PRGPIO_R & RCGCGPIO_ALL) != RCGCGPIO_ALL) { }

    GPIO_PORTF_LOCK_R = 0x4C4F434BU;
    GPIO_PORTF_CR_R |= (BTN_PF0 | BTN_PF4 | LED_MASK);

    GPIO_PORTF_AMSEL_R &= ~(BTN_PF0 | BTN_PF4 | LED_MASK);
    GPIO_PORTF_PCTL_R &= ~0x000FFFFFU;
    GPIO_PORTF_AFSEL_R &= ~(BTN_PF0 | BTN_PF4 | LED_MASK);
    GPIO_PORTF_DIR_R |= LED_MASK;
    GPIO_PORTF_DIR_R &= ~(BTN_PF0 | BTN_PF4);
    GPIO_PORTF_PUR_R |= (BTN_PF0 | BTN_PF4);
    GPIO_PORTF_DEN_R |= (BTN_PF0 | BTN_PF4 | LED_MASK);
    GPIO_PORTF_DATA_R &= ~LED_MASK;

    GPIO_PORTE_AMSEL_R &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_PCTL_R &= ~0x000000FFU;
    GPIO_PORTE_AFSEL_R &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_DIR_R &= ~(BTN_PE0 | BTN_PE1);
    GPIO_PORTE_PDR_R |= (BTN_PE0 | BTN_PE1);
    GPIO_PORTE_DEN_R |= (BTN_PE0 | BTN_PE1);

    GPIO_PORTB_AMSEL_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_PCTL_R &= ~0x000000FFU;
    GPIO_PORTB_AFSEL_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_DIR_R &= ~(BTN_PB0 | BTN_PB1);
    GPIO_PORTB_PDR_R |= (BTN_PB0 | BTN_PB1);
    GPIO_PORTB_DEN_R |= (BTN_PB0 | BTN_PB1);

    GPIO_PORTD_LOCK_R = 0x4C4F434BU;
    GPIO_PORTD_CR_R |= BTN_PD1;
    GPIO_PORTD_AMSEL_R &= ~BTN_PD1;
    GPIO_PORTD_PCTL_R &= ~0x000000F0U;
    GPIO_PORTD_AFSEL_R &= ~BTN_PD1;
    GPIO_PORTD_DIR_R &= ~BTN_PD1;
    GPIO_PORTD_PDR_R |= BTN_PD1;
    GPIO_PORTD_DEN_R |= BTN_PD1;
}

static inline uint32_t ReadPF(void) { return GPIO_PORTF_DATA_R; }
static inline uint32_t ReadPE(void) { return GPIO_PORTE_DATA_R; }
static inline uint32_t ReadPB(void) { return GPIO_PORTB_DATA_R; }
static inline uint32_t ReadPD(void) { return GPIO_PORTD_DATA_R; }

static inline void LED_Set(uint32_t mask)
{
    GPIO_PORTF_DATA_R = (GPIO_PORTF_DATA_R & ~LED_MASK) | (mask & LED_MASK);
}

/* ------------------------------ button helpers ----------------------------- */
static inline uint8_t Btn_DriverOpen(void)   { return ((ReadPF() & BTN_PF4) == 0U) ? 1U : 0U; } /* active-low */
static inline uint8_t Btn_DriverClose(void)  { return ((ReadPF() & BTN_PF0) == 0U) ? 1U : 0U; } /* active-low */
static inline uint8_t Btn_SecurityOpen(void) { return ((ReadPE() & BTN_PE0) != 0U) ? 1U : 0U; }
static inline uint8_t Btn_SecurityClose(void){ return ((ReadPE() & BTN_PE1) != 0U) ? 1U : 0U; }
static inline uint8_t Btn_OpenLimit(void)    { return ((ReadPB() & BTN_PB1) != 0U) ? 1U : 0U; }
static inline uint8_t Btn_ClosedLimit(void)  { return ((ReadPB() & BTN_PB0) != 0U) ? 1U : 0U; }
static inline uint8_t Btn_Obstacle(void)     { return ((ReadPD() & BTN_PD1) != 0U) ? 1U : 0U; }

static void SendEvent(EventType type, CommandSource src, TickType_t tick)
{
    GateEvent ev;
    BaseType_t sent;
    ev.type = type;
    ev.src = src;
    ev.tick = tick;
    g_dbgLastEvent = (uint32_t)type;
    sent = xQueueSend(g_eventQueue, &ev, pdMS_TO_TICKS(20));
    Debug_LogEventQueued(type, src, tick, sent);
}

/* --------------------------------- tasks ---------------------------------- */
static void vInputTask(void *pvParameters)
{
    uint8_t stable[4] = {0U, 0U, 0U, 0U};     /* driver open/close, security open/close */
    uint8_t prevStable[4] = {0U, 0U, 0U, 0U};
    uint8_t dbCount[4] = {0U, 0U, 0U, 0U};
    uint8_t prevSecConflict = 0U;
    uint8_t prevDrvConflict = 0U;
    uint8_t prevOpenLimit = 0U;
    uint8_t prevClosedLimit = 0U;
    uint8_t i;
    (void)pvParameters;

    for (;;)
    {
        TickType_t now = xTaskGetTickCount();
        uint8_t raw[4];
        uint8_t dOpen;
        uint8_t dClose;
        uint8_t sOpen;
        uint8_t sClose;
        uint8_t secConflict;
        uint8_t drvConflict;
        uint8_t openLimitNow;
        uint8_t closedLimitNow;
        uint8_t obstacleNow;

        raw[0] = Btn_DriverOpen();
        raw[1] = Btn_DriverClose();
        raw[2] = Btn_SecurityOpen();
        raw[3] = Btn_SecurityClose();

        for (i = 0U; i < 4U; i++)
        {
            if (raw[i] == stable[i])
            {
                dbCount[i] = 0U;
            }
            else
            {
                dbCount[i]++;
                if (dbCount[i] >= DEBOUNCE_SAMPLES)
                {
                    stable[i] = raw[i];
                    dbCount[i] = 0U;
                }
            }
        }

        dOpen = stable[0];
        dClose = stable[1];
        sOpen = stable[2];
        sClose = stable[3];
        secConflict = (((sOpen != 0U) && (sClose != 0U)) ? 1U : 0U);
        drvConflict = (((dOpen != 0U) && (dClose != 0U)) ? 1U : 0U);

        /* Same-panel conflicting inputs => immediate safe stop. */
        if (secConflict != 0U)
        {
            if (prevSecConflict == 0U)
            {
                SendEvent(EV_STOP_CONFLICT, SRC_SECURITY, now);
            }
        }
        else if (drvConflict != 0U)
        {
            if (prevDrvConflict == 0U)
            {
                SendEvent(EV_STOP_CONFLICT, SRC_DRIVER, now);
            }
        }
        else
        {
            /* Security panel has priority whenever active. */
            if ((sOpen != 0U) && (prevStable[2] == 0U)) { SendEvent(EV_OPEN_PRESS, SRC_SECURITY, now); }
            if ((sClose != 0U) && (prevStable[3] == 0U)) { SendEvent(EV_CLOSE_PRESS, SRC_SECURITY, now); }
            if ((sOpen == 0U) && (prevStable[2] != 0U)) { SendEvent(EV_OPEN_RELEASE, SRC_SECURITY, now); }
            if ((sClose == 0U) && (prevStable[3] != 0U)) { SendEvent(EV_CLOSE_RELEASE, SRC_SECURITY, now); }

            if ((sOpen == 0U) && (sClose == 0U))
            {
                if ((dOpen != 0U) && (prevStable[0] == 0U)) { SendEvent(EV_OPEN_PRESS, SRC_DRIVER, now); }
                if ((dClose != 0U) && (prevStable[1] == 0U)) { SendEvent(EV_CLOSE_PRESS, SRC_DRIVER, now); }
                if ((dOpen == 0U) && (prevStable[0] != 0U)) { SendEvent(EV_OPEN_RELEASE, SRC_DRIVER, now); }
                if ((dClose == 0U) && (prevStable[1] != 0U)) { SendEvent(EV_CLOSE_RELEASE, SRC_DRIVER, now); }
            }
        }
        prevSecConflict = secConflict;
        prevDrvConflict = drvConflict;

        openLimitNow = Btn_OpenLimit();
        closedLimitNow = Btn_ClosedLimit();
        obstacleNow = Btn_Obstacle();

        if ((openLimitNow != 0U) && (prevOpenLimit == 0U))
        {
            (void)xSemaphoreGive(g_openLimitSem);
        }
        if ((closedLimitNow != 0U) && (prevClosedLimit == 0U))
        {
            (void)xSemaphoreGive(g_closeLimitSem);
        }
        prevOpenLimit = openLimitNow;
        prevClosedLimit = closedLimitNow;

        for (i = 0U; i < 4U; i++)
        {
            prevStable[i] = stable[i];
        }

        g_watchBtnDriverOpen = (uint32_t)dOpen;
        g_watchBtnDriverClose = (uint32_t)dClose;
        g_watchBtnSecurityOpen = (uint32_t)sOpen;
        g_watchBtnSecurityClose = (uint32_t)sClose;
        g_watchBtnOpenLimit = (uint32_t)openLimitNow;
        g_watchBtnClosedLimit = (uint32_t)closedLimitNow;
        g_watchBtnObstacle = (uint32_t)obstacleNow;

        vTaskDelay(pdMS_TO_TICKS(INPUT_PERIOD_MS));
    }
}

static void vSafetyTask(void *pvParameters)
{
    uint8_t prevObstacle = 0U;
    (void)pvParameters;

    for (;;)
    {
        uint8_t obstacleNow = Btn_Obstacle();

        if ((obstacleNow != 0U) && (prevObstacle == 0U))
        {
            GateState st;

            if (xSemaphoreTake(g_stateMutex, portMAX_DELAY) == pdTRUE)
            {
                st = g_gateState;
                (void)xSemaphoreGive(g_stateMutex);

                /* TC-08: obstacle overrides closing even if CLOSE is held manually. */
                if (st == CLOSING)
                {
                    vPrintString("\r\nSafety task: obstacle edge while closing\r\n");
                    SendEvent(EV_OBSTACLE, SRC_SECURITY, xTaskGetTickCount());
                }
            }
        }

        prevObstacle = obstacleNow;
        vTaskDelay(pdMS_TO_TICKS(SAFETY_PERIOD_MS));
    }
}

static void GateSetOpening(CommandSource src, TickType_t now)
{
    g_gateState = OPENING;
    g_motionMode = MOTION_MANUAL;
    g_activeSrc = src;
    g_activeDir = EV_OPEN_PRESS;
    g_pressTick = now;
}

static void GateSetClosing(CommandSource src, TickType_t now)
{
    g_gateState = CLOSING;
    g_motionMode = MOTION_MANUAL;
    g_activeSrc = src;
    g_activeDir = EV_CLOSE_PRESS;
    g_pressTick = now;
}

static void GateSetStoppedMidway(void)
{
    g_gateState = STOPPED_MIDWAY;
    g_motionMode = MOTION_NONE;
}

static void GateHandleManualRelease(CommandSource src, EventType pressDir, TickType_t releaseTick)
{
    TickType_t heldTicks;

    if ((g_motionMode != MOTION_MANUAL) ||
        (g_activeDir != pressDir) ||
        (g_activeSrc != src))
    {
        return;
    }

    heldTicks = releaseTick - g_pressTick;
    vPrintString("\r\nManual release captured\r\n");
    vPrintStringAndNumber("Held ticks: ", (uint32_t)heldTicks);
    if (heldTicks < pdMS_TO_TICKS(ONE_TOUCH_MAX_MS))
    {
        g_motionMode = MOTION_AUTO;
        vPrintString("Converted to AUTO mode\r\n");
    }
    else
    {
        GateSetStoppedMidway();
        vPrintString("Long hold release -> STOPPED_MIDWAY\r\n");
    }
}

static void vGateControlTask(void *pvParameters)
{
    GateEvent ev;
    (void)pvParameters;

    for (;;)
    {
        if (xSemaphoreTake(g_openLimitSem, 0U) == pdTRUE)
        {
            if (xSemaphoreTake(g_stateMutex, portMAX_DELAY) == pdTRUE)
            {
                if ((g_gateState == OPENING) || (g_gateState == REVERSING))
                {
                    g_gateState = IDLE_OPEN;
                    g_motionMode = MOTION_NONE;
                    vPrintString("\r\nOpen limit reached -> IDLE_OPEN\r\n");
                }
                Watch_UpdateGateLocked();
                (void)xSemaphoreGive(g_stateMutex);
            }
        }

        if (xSemaphoreTake(g_closeLimitSem, 0U) == pdTRUE)
        {
            if (xSemaphoreTake(g_stateMutex, portMAX_DELAY) == pdTRUE)
            {
                if (g_gateState == CLOSING)
                {
                    g_gateState = IDLE_CLOSED;
                    g_motionMode = MOTION_NONE;
                    vPrintString("\r\nClosed limit reached -> IDLE_CLOSED\r\n");
                }
                Watch_UpdateGateLocked();
                (void)xSemaphoreGive(g_stateMutex);
            }
        }

        if (xQueueReceive(g_eventQueue, &ev, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            if (xSemaphoreTake(g_stateMutex, portMAX_DELAY) == pdTRUE)
            {
                switch (ev.type)
                {
                    case EV_OPEN_PRESS:
                        if (g_gateState == CLOSING)
                        {
                            if ((ev.src == SRC_SECURITY) && (g_activeSrc == SRC_DRIVER))
                            {
                                GateSetOpening(ev.src, ev.tick);
                                vPrintString("\r\nSecurity OPEN overrides driver closing\r\n");
                            }
                            else
                            {
                                GateSetStoppedMidway();
                                vPrintString("\r\nOpposite command while closing -> gate stopped\r\n");
                            }
                        }
                        else if ((g_gateState == IDLE_CLOSED) ||
                                 (g_gateState == STOPPED_MIDWAY))
                        {
                            GateSetOpening(ev.src, ev.tick);
                        }
                        break;

                    case EV_CLOSE_PRESS:
                        if (g_gateState == OPENING)
                        {
                            if ((ev.src == SRC_SECURITY) && (g_activeSrc == SRC_DRIVER))
                            {
                                GateSetClosing(ev.src, ev.tick);
                                vPrintString("\r\nSecurity CLOSE overrides driver opening\r\n");
                            }
                            else
                            {
                                GateSetStoppedMidway();
                                vPrintString("\r\nOpposite command while opening -> gate stopped\r\n");
                            }
                        }
                        else if ((g_gateState == IDLE_OPEN) ||
                                 (g_gateState == STOPPED_MIDWAY))
                        {
                            GateSetClosing(ev.src, ev.tick);
                        }
                        break;

                    case EV_OPEN_RELEASE:
                        if (g_gateState == OPENING)
                        {
                            GateHandleManualRelease(ev.src, EV_OPEN_PRESS, ev.tick);
                        }
                        break;

                    case EV_CLOSE_RELEASE:
                        if (g_gateState == CLOSING)
                        {
                            GateHandleManualRelease(ev.src, EV_CLOSE_PRESS, ev.tick);
                        }
                        break;

                    case EV_STOP_CONFLICT:
                        GateSetStoppedMidway();
                        vPrintString("\r\nConflict detected -> gate stopped\r\n");
                        break;

                    case EV_OBSTACLE:
                        if (g_gateState == CLOSING)
                        {
                            vPrintString("\r\nObstacle detected while closing\r\n");
                            /* Explicit stop first, then reverse as required. */
                            GateSetStoppedMidway();
                            Watch_UpdateGateLocked();
                            g_gateState = REVERSING;    /* Green ON during reverse */
                            g_motionMode = MOTION_MANUAL;
                            (void)xSemaphoreGive(g_stateMutex);

                            vTaskDelay(pdMS_TO_TICKS(REVERSE_DURATION_MS));

                            if (xSemaphoreTake(g_stateMutex, portMAX_DELAY) == pdTRUE)
                            {
                                GateSetStoppedMidway();
                                vPrintString("Reverse finished -> STOPPED_MIDWAY\r\n");
                            }
                        }
                        break;

                    default:
                        break;
                }

                Watch_UpdateGateLocked();
                (void)xSemaphoreGive(g_stateMutex);
            }
        }
    }
}

static void vLEDTask(void *pvParameters)
{
    (void)pvParameters;
    for (;;)
    {
        GateState localState;

        if (xSemaphoreTake(g_stateMutex, portMAX_DELAY) == pdTRUE)
        {
            localState = g_gateState;
            (void)xSemaphoreGive(g_stateMutex);

            if ((localState == OPENING) || (localState == REVERSING))
            {
                LED_Set(LED_GREEN);
            }
            else if (localState == CLOSING)
            {
                LED_Set(LED_RED);
            }
            else
            {
                LED_Set(0U);
            }

            g_watchLedGreen = (((localState == OPENING) || (localState == REVERSING)) != 0) ? 1U : 0U;
            g_watchLedRed = (localState == CLOSING) ? 1U : 0U;
            g_watchPortFData = GPIO_PORTF_DATA_R;
        }

        vTaskDelay(pdMS_TO_TICKS(LED_PERIOD_MS));
    }
}

/* Optional low priority status task: keeps debug variables current. */
static void vStatusTask(void *pvParameters)
{
    uint32_t printDivider = 0U;
    (void)pvParameters;
    for (;;)
    {
        if (xSemaphoreTake(g_stateMutex, portMAX_DELAY) == pdTRUE)
        {
            Watch_UpdateGateLocked();
#if DEBUG_VERBOSE_LOGS
            printDivider++;
            if (printDivider >= 5U) /* about every 1 second */
            {
                printDivider = 0U;
                vPrintString("\r\n[NOW] ");
                vPrintString(StateName(g_gateState));
                vPrintString(" | ");
                vPrintString(ModeName(g_motionMode));
                vPrintString("\r\n");
            }
#endif
            (void)xSemaphoreGive(g_stateMutex);
        }
        g_watchSysTick = (uint32_t)xTaskGetTickCount();
        g_watchQueueMsgsWaiting = (uint32_t)uxQueueMessagesWaiting(g_eventQueue);
        vTaskDelay(pdMS_TO_TICKS(STATUS_PERIOD_MS));
    }
}

int main(void)
{
    GPIO_Init();
    LED_Set(0U);
    vPrintString("Smart Parking Gate started\r\n");

    g_eventQueue = xQueueCreate(24, sizeof(GateEvent));
    g_stateMutex = xSemaphoreCreateMutex();
    g_openLimitSem = xSemaphoreCreateBinary();
    g_closeLimitSem = xSemaphoreCreateBinary();

    if ((g_eventQueue == NULL) || (g_stateMutex == NULL) || (g_openLimitSem == NULL) ||
        (g_closeLimitSem == NULL))
    {
        while (1) { }
    }

    /* Required task priority order from spec. */
    (void)xTaskCreate(vSafetyTask, "Safety", 256, NULL, 5, NULL);      /* Highest */
    (void)xTaskCreate(vInputTask, "Input", 256, NULL, 4, NULL);        /* High */
    (void)xTaskCreate(vGateControlTask, "Gate", 256, NULL, 3, NULL);   /* Medium */
    (void)xTaskCreate(vLEDTask, "LED", 256, NULL, 3, NULL);            /* Medium */
    (void)xTaskCreate(vStatusTask, "Status", 256, NULL, 2, NULL);      /* Optional low */

    vTaskStartScheduler();
    while (1) { }
}
