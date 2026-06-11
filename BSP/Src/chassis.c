#include "chassis.h"
#include "pid.h"
#include "cmsis_os.h"
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
uint16_t           g_red_count      = 0;
uint16_t           g_green_count    = 0;
int32_t            g_enc_total      = 0;
int32_t            g_turn_enc       = 0;
float              g_avg_speed_mps  = 0.0f;

/* ══════════════════════════════════════════════════════════════════════
   Phase 2 文件级持久状态（entry 函数复位，run 函数使用）
   ══════════════════════════════════════════════════════════════════════ */

// LINE_FOLLOW 持久状态
static uint8_t  gl_lock         = 0;
static uint8_t  gl_last_x       = 0;
static uint8_t  gl_unlock_vote  = 0;
static uint8_t  gl_no_class_cnt = 0;
static float    last_line_err   = 0.0f;

// OBSTACLE 持久状态
static ObsSubStage_t ob_stage      = OB_STOP;
static int32_t       ob_start_enc  = 0;
static uint32_t      ob_ticks      = 0;
static int32_t       ob_mem_dist   = 0;
static int32_t       ob_turn_start = 0;  // 转弯编码器起点

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

    // ── W 命令：选轮（所有状态可用） ──
    if (cmd_buf[0] == 'W') {
        cur_wheel = (cmd_buf[1] == 'A') ? 4 : (cmd_buf[1] - '0');
        // 仅在 DEBUG 下锁死非选中轮（避免干扰差速转向）
        if (g_chassis_state == STATE_DEBUG) {
            for (int i = 0; i < 4; i++) {
                if (cur_wheel < 4 && i != (int)cur_wheel) {
                    get_wheel_pid(i)->Target = 0.0f;
                }
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

static void entry_line_follow(void) {
    Chassis_MoveForward(LINE_BASE_SPEED);
    // 排空 K210 队列，扔掉避障/倒车期间囤的旧帧
    extern osMessageQId Queue_VisionDataHandle;
    while (osMessageGet(Queue_VisionDataHandle, 0).status == osEventMessage) {}
    // 重置绿锁
    gl_lock         = 0;
    gl_last_x       = 0;
    gl_unlock_vote  = 0;
    gl_no_class_cnt = 0;
    last_line_err   = 0.0f;
}

// ═══ Phase 1 STUB entries — 其余安全停住，Phase 2~4 逐步替换 ═══
static void entry_obstacle(void) {
    Chassis_Stop();
    ob_stage      = OB_STOP;
    ob_start_enc  = 0;
    ob_ticks      = 0;
    ob_mem_dist   = 0;
    ob_turn_start = 0;
}
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

/* ── 2. LINE_FOLLOW — 红外巡线 + 绿草莓锁 + 红草莓检测 ── */
static ChassisState_t run_line_follow(const SensorData_t *s,
                                       uint8_t hv, const VisionData_t *v) {
    /* ── 1. 红外位置计算（加权平均） ──
       PE0(最左)~PE7(最右), 0=黑线(在线), 1=白(离线) */
    float ir_sum   = 0.0f;
    int   black_cnt = 0;
    for (int i = 0; i < 8; i++) {
        if (s->ir[i] == 0) {
            ir_sum += (float)i;
            black_cnt++;
        }
    }

    float line_err;
    if (black_cnt > 0) {
        float center = ir_sum / (float)black_cnt;
        line_err = center - 3.5f;       // 中心在传感器 3 和 4 之间
        last_line_err = line_err;
    } else {
        line_err = last_line_err;       // 全线丢失，沿用上次误差
    }

    /* ── 2. P 控制器 → 差速转向 ── */
#if FORCE_STRAIGHT
    Chassis_MoveForward(LINE_BASE_SPEED);  // 空转测试：忽略红外，纯直行
#else
    float turn_corr = line_err * LINE_KP;
    float speed = (black_cnt == 0) ? 0.15f : LINE_BASE_SPEED; // 丢线降速
    Chassis_DiffTurn(speed, turn_corr);
#endif

    /* ── 3. 视觉检测 ── */
    if (hv && v != NULL) {

        /* ── 3a. 红草莓检测 → BREAKAWAY ── */
        if (v->class_id == 1) {
            uint8_t trigger = 0;
            // 条件 A: Area≥10% 且 X 在侧边（草莓在赛道旁，非正前方）
            if (v->area >= 10 && (v->x < 80 || v->x > 240)) {
                trigger = 1;
            }
            // 条件 B: Area≥60% 强制触发（草莓怼脸兜底）
            if (v->area >= 60) {
                trigger = 1;
            }
            if (trigger) {
                // 抑制区检查：采摘后 20cm 内不重复触发
                if (!g_skip_strawberry || g_enc_total >= g_skip_until_enc) {
                    g_red_count++;
                    return STATE_BREAKAWAY;
                }
            }
        }

        /* ── 3b. 绿草莓 5 步动态视野锁 ── */
        if (v->class_id == 0 && v->area >= 10) {
            gl_no_class_cnt = 0;  // 有效绿帧，复位丢锁计数

            if (gl_lock == 0 && v->x > 200) {
                // ② 捕获上锁：X>200 首次出现
                gl_lock   = 1;
                gl_last_x = v->x;
                gl_unlock_vote = 0;
                g_green_count++;
            } else if (gl_lock == 1) {
                // ③ 持续跟踪，更新 Last_X
                // ④ X 显著左移（草莓被超过）→ 累计解锁投票
                if (v->x + 40 < gl_last_x) {   // X < Last_X - 40
                    gl_unlock_vote++;
                    if (gl_unlock_vote >= 3) {
                        gl_lock = 0;
                        gl_unlock_vote = 0;
                    }
                } else {
                    gl_unlock_vote = 0;         // 不连续则重置
                }
                gl_last_x = v->x;
            }
            // ① Area<10% 已在上面过滤，不进入此处
        } else if (gl_lock == 1) {
            // ⑤ 锁住但连续 N 帧无 Class=0 → 强制解锁兜底
            gl_no_class_cnt++;
            if (gl_no_class_cnt >= 10) {        // 10帧=100ms
                gl_lock = 0;
                gl_unlock_vote = 0;
                gl_no_class_cnt = 0;
            }
        }
    } else if (gl_lock == 1) {
        // 无 K210 帧也累计丢锁
        gl_no_class_cnt++;
        if (gl_no_class_cnt >= 10) {
            gl_lock = 0;
            gl_unlock_vote = 0;
            gl_no_class_cnt = 0;
        }
    }

    return STATE_LINE_FOLLOW;
}

/* ── 3. OBSTACLE — 10 阶段避障绕行 ──
   流程: 急停→退10cm→右转90°→横行25cm(记忆)→左转90°(朝前)
         →直行30cm→左转90°(朝左)→回线Ncm(记忆距离)→右转45°(斜朝线)→直行找线
   每个子阶段切换间刹停四轮，检测实际速度全部归零后再起步 */

static ChassisState_t run_obstacle(const SensorData_t *s,
                                    uint8_t hv, const VisionData_t *v) {
    (void)hv; (void)v;

    int32_t dist_traveled;
    int     black_cnt;

    // 子阶段切换时刹停四轮，速度全部 <0.03 m/s 才放行
    static ObsSubStage_t prev_ob_stage = OB_DONE;
    static uint8_t       brake_ticks  = 0;

    if (ob_stage != prev_ob_stage) {
        brake_ticks   = 0;
        prev_ob_stage = ob_stage;
    }

    if (brake_ticks < OB_BRAKE_TIMEOUT) {
        Chassis_Stop();
        brake_ticks++;
        // 四轮全部接近静止 → 立即退出刹车，不等满 1s
        float spd = pid_FL.Actual; if (spd < 0) spd = -spd;
        if (spd < 0.03f) {
            spd = pid_FR.Actual; if (spd < 0) spd = -spd;
            if (spd < 0.03f) {
                spd = pid_BL.Actual; if (spd < 0) spd = -spd;
                if (spd < 0.03f) {
                    spd = pid_BR.Actual; if (spd < 0) spd = -spd;
                    if (spd < 0.03f) {
                        brake_ticks = OB_BRAKE_TIMEOUT;  // 强制放行
                    }
                }
            }
        }
        return STATE_OBSTACLE;
    }

    switch (ob_stage) {

    case OB_STOP:
        // Chassis_Stop() 已在 entry 调用，等待 PID 斜坡归零
        ob_ticks++;
        if (ob_ticks >= 30) {               // 300ms 确保停稳
            ob_stage = OB_BACK;
            ob_start_enc = g_enc_total;
            ob_ticks = 0;
        }
        break;

    case OB_BACK:
        Chassis_MoveBackward(OB_BACK_SPEED);
        dist_traveled = ob_start_enc - g_enc_total; // 后退：enc 减小
        if (dist_traveled >= (int32_t)(OB_BACK_CM * ENC_PER_CM)) {
            ob_stage = OB_TURN_RIGHT;
            ob_ticks = 0;
        }
        break;

    case OB_TURN_RIGHT:
        if (ob_ticks == 0) ob_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, 1);      // CW → 车头朝右
        ob_ticks++;
        if (g_turn_enc - ob_turn_start >= TURN90_TOTAL_ENC
            || ob_ticks >= OB_TURN_MS / 10) {       // 编码器到位 / 超时兜底
            ob_stage = OB_CROSS;
            ob_start_enc = g_enc_total;
            ob_ticks = 0;
        }
        break;

    case OB_CROSS:
        Chassis_MoveForward(OB_FWD_SPEED);          // 横行绕过障碍物
        dist_traveled = g_enc_total - ob_start_enc;
        ob_mem_dist = dist_traveled;                 // 实时记忆距离
        if (dist_traveled >= (int32_t)(OB_CROSS_CM * ENC_PER_CM)) {
            ob_stage = OB_TURN_LEFT1;
            ob_ticks = 0;
        }
        break;

    case OB_TURN_LEFT1:
        if (ob_ticks == 0) ob_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, -1);     // CCW → 车头恢复朝前 ↑
        ob_ticks++;
        if (g_turn_enc - ob_turn_start >= TURN90_TOTAL_ENC
            || ob_ticks >= OB_TURN_MS / 10) {
            ob_stage = OB_FORWARD;
            ob_start_enc = g_enc_total;
            ob_ticks = 0;
        }
        break;

    case OB_FORWARD:
        Chassis_MoveForward(OB_FWD_SPEED);          // 纵向超越障碍物
        dist_traveled = g_enc_total - ob_start_enc;
        if (dist_traveled >= (int32_t)(OB_FORWARD_CM * ENC_PER_CM)) {
            ob_stage = OB_TURN_LEFT2;
            ob_ticks = 0;
        }
        break;

    case OB_TURN_LEFT2:
        if (ob_ticks == 0) ob_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, -1);     // CCW → 车头朝左，正对黑线
        ob_ticks++;
        if (g_turn_enc - ob_turn_start >= TURN90_TOTAL_ENC
            || ob_ticks >= OB_TURN_MS / 10) {
            ob_stage = OB_RETURN;
            ob_start_enc = g_enc_total;
            ob_ticks = 0;
        }
        break;

    case OB_RETURN:
        Chassis_MoveForward(OB_FWD_SPEED);          // 用记忆距离原路回线
        dist_traveled = g_enc_total - ob_start_enc;
        if (dist_traveled >= ob_mem_dist) {
            ob_stage = OB_TURN_45;
            ob_ticks = 0;
        }
        break;

    case OB_TURN_45:
        if (ob_ticks == 0) ob_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, 1);      // CW → 斜向45°朝线
        ob_ticks++;
        if (g_turn_enc - ob_turn_start >= TURN45_TOTAL_ENC
            || ob_ticks >= OB_TURN_MS / 10) {
            ob_stage = OB_FORWARD_TO_LINE;
            ob_start_enc = g_enc_total;
            ob_ticks = 0;
        }
        break;

    case OB_FORWARD_TO_LINE:
        Chassis_MoveForward(OB_FWD_SPEED);          // 直行斜插上线
        ob_ticks++;
        black_cnt = 0;
        for (int i = 0; i < 8; i++) {
            if (s->ir[i] == 0) black_cnt++;
        }
        dist_traveled = g_enc_total - ob_start_enc;
        // 找到线 或 走了 50cm 还没找到 → 放弃兜底
        if (black_cnt >= 2 || dist_traveled >= (int32_t)(OB_FWD_TO_LINE_CM * ENC_PER_CM)) {
            ob_stage = OB_DONE;
        }
        break;

    case OB_DONE:
    default:
        ob_stage = OB_STOP;                         // 复位以备下次避障
        return STATE_LINE_FOLLOW;
    }

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
    g_red_count       = 0;
    g_green_count     = 0;
    g_enc_total       = 0;
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
