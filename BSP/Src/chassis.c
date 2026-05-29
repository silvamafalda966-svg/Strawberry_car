#include "chassis.h"
#include "pid.h"
#include <stdlib.h>   // atof
#include <string.h>   // memset

/* ══════════════════════════════════════════════════════════════════════
   Global Variables
   ══════════════════════════════════════════════════════════════════════ */
volatile uint8_t   g_emergency_stop  = 0;
ChassisState_t     g_chassis_state   = STATE_DEBUG;
uint32_t           g_state_ticks     = 0;
int32_t            g_last_picked_enc = 0;
int32_t            g_skip_until_enc  = 0;
uint8_t            g_skip_strawberry = 0;

/* ══════════════════════════════════════════════════════════════════════
   Chassis_Move API — 只设 pid_X.Target，CalcOneWheel 统一处理
   ══════════════════════════════════════════════════════════════════════ */

void Chassis_MoveForward(float speed_mps) {
    pid_FL.Target = speed_mps;
    pid_FR.Target = speed_mps;
    pid_BL.Target = speed_mps;
    pid_BR.Target = speed_mps;
}

void Chassis_MoveBackward(float speed_mps) {
    pid_FL.Target = -speed_mps;
    pid_FR.Target = -speed_mps;
    pid_BL.Target = -speed_mps;
    pid_BR.Target = -speed_mps;
}

void Chassis_Stop(void) {
    pid_FL.Target = 0.0f;
    pid_FR.Target = 0.0f;
    pid_BL.Target = 0.0f;
    pid_BR.Target = 0.0f;
}

void Chassis_DiffTurn(float base_speed, float turn_diff) {
    pid_FL.Target = base_speed + turn_diff;
    pid_BL.Target = base_speed + turn_diff;
    pid_FR.Target = base_speed - turn_diff;
    pid_BR.Target = base_speed - turn_diff;
}

void Chassis_TurnInPlace(float speed_mps, int8_t dir) {
    pid_FL.Target =  speed_mps * dir;
    pid_BL.Target =  speed_mps * dir;
    pid_FR.Target = -speed_mps * dir;
    pid_BR.Target = -speed_mps * dir;
}

void Chassis_SetTargets(float fl, float fr, float bl, float br) {
    pid_FL.Target = fl;
    pid_FR.Target = fr;
    pid_BL.Target = bl;
    pid_BR.Target = br;
}

/* ══════════════════════════════════════════════════════════════════════
   Helper: wheel pointer lookup & state getter
   ══════════════════════════════════════════════════════════════════════ */

static PID_TypeDef *get_wheel_pid(uint8_t idx) {
    switch (idx) {
        case 0: return &pid_FL;
        case 1: return &pid_FR;
        case 2: return &pid_BL;
        case 3: return &pid_BR;
        default: return &pid_FL;
    }
}

const char *Chassis_StateName(ChassisState_t s); // fwd

/* ══════════════════════════════════════════════════════════════════════
   VOFA+ 命令解析
   M0~M9=强制切状态  MD=回DEBUG  W=选轮  P/I/D/T=调参
   ══════════════════════════════════════════════════════════════════════ */

static uint8_t cur_wheel = 4; // 0=FL,1=FR,2=BL,3=BR,4=ALL

void Chassis_VOFA_Parse(const uint8_t *cmd_buf) {
    if (cmd_buf[0] == '\0') return;

    // ── M 命令：强制切状态（所有状态可用） ──
    // 映射: MD→DEBUG, M0→IDLE, M1→LINE_FOLLOW, ..., M8→FINISH
    if (cmd_buf[0] == 'M') {
        if (cmd_buf[1] == 'D') {
            g_chassis_state = STATE_DEBUG;
        } else if (cmd_buf[1] >= '0' && cmd_buf[1] <= '8') {
            g_chassis_state = (ChassisState_t)(cmd_buf[1] - '0' + 1);
        } else if (cmd_buf[1] == '9') {
            g_chassis_state = STATE_FINISH;
        }
        return;
    }

    // ── W 命令：选轮（仅 DEBUG） ──
    if (cmd_buf[0] == 'W') {
        if (g_chassis_state != STATE_DEBUG) return;
        cur_wheel = (cmd_buf[1] == 'A') ? 4 : (cmd_buf[1] - '0');
        for (int i = 0; i < 4; i++) {
            if (cur_wheel < 4 && i != (int)cur_wheel) {
                get_wheel_pid(i)->Target = 0.0f;
            }
        }
        return;
    }

    // ── P/I/D 所有状态可用 ──
    // ── T 仅 DEBUG 可用 ──
    if (cmd_buf[0] == 'T' && g_chassis_state != STATE_DEBUG) return;

    float val = atof((const char *)&cmd_buf[1]);

    if (cur_wheel < 4) {
        PID_TypeDef *pid = get_wheel_pid(cur_wheel);
        switch (cmd_buf[0]) {
            case 'P': pid->Kp = val; break;
            case 'I': pid->Ki = val; break;
            case 'D': pid->Kd = val; break;
            case 'T': pid->Target = val; break;
        }
    } else {
        switch (cmd_buf[0]) {
            case 'P': pid_FL.Kp = pid_FR.Kp = pid_BL.Kp = pid_BR.Kp = val; break;
            case 'I': pid_FL.Ki = pid_FR.Ki = pid_BL.Ki = pid_BR.Ki = val; break;
            case 'D': pid_FL.Kd = pid_FR.Kd = pid_BL.Kd = pid_BR.Kd = val; break;
            case 'T': pid_FL.Target = pid_FR.Target = pid_BL.Target = pid_BR.Target = val; break;
        }
    }
}

/* ══════════════════════════════════════════════════════════════════════
   Entry 函数 — 进入状态时调用一次
   ══════════════════════════════════════════════════════════════════════ */

static void entry_debug(void)           { /* 保持当前 PID 状态，不操作 */ }

static void entry_idle(void)            { Chassis_Stop(); }

// ═══ Phase 1 STUB entries — 全部安全停住，Phase 2~4 逐步替换 ═══
static void entry_line_follow(void)     { Chassis_Stop(); } // TODO Phase 2: MoveForward(0.3)
static void entry_obstacle(void)        { Chassis_Stop(); }
static void entry_green_filter(void)    { Chassis_Stop(); }
static void entry_breakaway(void)       { Chassis_Stop(); }
static void entry_visual_approach(void) { Chassis_Stop(); } // TODO Phase 3: MoveForward(0.15)
static void entry_docking(void)         { Chassis_Stop(); }
static void entry_blind_return(void)    { Chassis_Stop(); } // TODO Phase 3: MoveBackward(0.3)
static void entry_finish(void)          { Chassis_Stop(); }

/* ══════════════════════════════════════════════════════════════════════
   Run 函数 — 每周期调用，返回下一状态（或同状态继续）
   ══════════════════════════════════════════════════════════════════════ */

/* ── 0. DEBUG ── */
static ChassisState_t run_debug(const SensorData_t *s,
                                 uint8_t hv, const VisionData_t *v) {
    (void)s; (void)hv; (void)v;
    // VOFA+ W/P/I/D/T 命令由 Chassis_VOFA_Parse 直接操作 pid 变量
    // CalcOneWheel 在主循环中统一调用，遥控选中轮
    return STATE_DEBUG;
}

/* ── 1. IDLE ── */
static ChassisState_t run_idle(const SensorData_t *s,
                                uint8_t hv, const VisionData_t *v) {
    (void)s; (void)hv; (void)v;
    // PID 斜坡自然归零 → 停车锁死，不消耗任何运动资源
    return STATE_IDLE;
}

/* ── 2. LINE_FOLLOW [STUB — Phase 2 实现] ── */
static ChassisState_t run_line_follow(const SensorData_t *s,
                                       uint8_t hv, const VisionData_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_LINE_FOLLOW;
}

/* ── 3. OBSTACLE [STUB — Phase 2 实现] ── */
static ChassisState_t run_obstacle(const SensorData_t *s,
                                    uint8_t hv, const VisionData_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_OBSTACLE;
}

/* ── 4. GREEN_FILTER [STUB — Phase 4 实现] ── */
static ChassisState_t run_green_filter(const SensorData_t *s,
                                        uint8_t hv, const VisionData_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_GREEN_FILTER;
}

/* ── 5. BREAKAWAY [STUB — Phase 3 实现] ── */
static ChassisState_t run_breakaway(const SensorData_t *s,
                                     uint8_t hv, const VisionData_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_BREAKAWAY;
}

/* ── 6. VISUAL_APPROACH [STUB — Phase 3 实现] ── */
static ChassisState_t run_visual_approach(const SensorData_t *s,
                                           uint8_t hv, const VisionData_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_VISUAL_APPROACH;
}

/* ── 7. DOCKING [STUB — Phase 3 实现] ── */
static ChassisState_t run_docking(const SensorData_t *s,
                                   uint8_t hv, const VisionData_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_DOCKING;
}

/* ── 8. BLIND_RETURN [STUB — Phase 3 实现] ── */
static ChassisState_t run_blind_return(const SensorData_t *s,
                                        uint8_t hv, const VisionData_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_BLIND_RETURN;
}

/* ── 9. FINISH [STUB — Phase 4 实现] ── */
static ChassisState_t run_finish(const SensorData_t *s,
                                  uint8_t hv, const VisionData_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_FINISH;
}

/* ══════════════════════════════════════════════════════════════════════
   状态描述符表
   ══════════════════════════════════════════════════════════════════════ */

static const StateDescriptor_t state_table[STATE_COUNT] = {
    [STATE_DEBUG]           = { "DEBUG",           entry_debug,           run_debug,           1 },
    [STATE_IDLE]            = { "IDLE",            entry_idle,            run_idle,            0 },
    [STATE_LINE_FOLLOW]     = { "LINE_FOLLOW",     entry_line_follow,     run_line_follow,     0 },
    [STATE_OBSTACLE]        = { "OBSTACLE",        entry_obstacle,        run_obstacle,        0 },
    [STATE_GREEN_FILTER]    = { "GREEN_FILTER",    entry_green_filter,    run_green_filter,    0 },
    [STATE_BREAKAWAY]       = { "BREAKAWAY",       entry_breakaway,       run_breakaway,       0 },
    [STATE_VISUAL_APPROACH] = { "VISUAL_APPROACH", entry_visual_approach, run_visual_approach, 0 },
    [STATE_DOCKING]         = { "DOCKING",         entry_docking,         run_docking,         0 },
    [STATE_BLIND_RETURN]    = { "BLIND_RETURN",    entry_blind_return,    run_blind_return,    0 },
    [STATE_FINISH]          = { "FINISH",          entry_finish,          run_finish,          0 },
};

/* ══════════════════════════════════════════════════════════════════════
   Public helpers
   ══════════════════════════════════════════════════════════════════════ */

const char *Chassis_StateName(ChassisState_t s) {
    if (s >= STATE_COUNT) return "UNKNOWN";
    return state_table[s].name;
}

/* ══════════════════════════════════════════════════════════════════════
   Chassis_Init — 上电时调用一次
   ══════════════════════════════════════════════════════════════════════ */

void Chassis_Init(void) {
    g_chassis_state   = STATE_DEBUG;
    g_state_ticks     = 0;
    g_emergency_stop  = 0;
    g_skip_strawberry = 0;
    g_last_picked_enc = 0;
    g_skip_until_enc  = 0;
}

/* ══════════════════════════════════════════════════════════════════════
   Chassis_StateMachine_Run — 每 10ms 主循环调用一次
   ══════════════════════════════════════════════════════════════════════ */

void Chassis_StateMachine_Run(const SensorData_t *sensor,
                              uint8_t has_vision, const VisionData_t *vis,
                              int32_t enc_fl, int32_t enc_fr,
                              int32_t enc_bl, int32_t enc_br)
{
    (void)enc_fl; (void)enc_fr; (void)enc_bl; (void)enc_br; // Phase 2+ 使用

    static ChassisState_t prev_state = STATE_COUNT; // 初始值≠任何合法状态，首次触发 entry

    // ── 状态切换：调用新状态的 entry ──
    if (g_chassis_state != prev_state) {
        if (g_chassis_state < STATE_COUNT) {
            state_table[g_chassis_state].entry();
        }
        g_state_ticks = 0;
        prev_state    = g_chassis_state;
    }

    // ── 运行当前状态 ──
    const StateDescriptor_t *desc = &state_table[g_chassis_state];
    ChassisState_t next = desc->run(sensor, has_vision, vis);
    g_state_ticks++;

    // ── 检查状态切换请求 ──
    if (next != g_chassis_state && next < STATE_COUNT) {
        g_chassis_state = next;
    }
}
