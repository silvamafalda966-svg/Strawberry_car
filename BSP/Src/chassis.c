#include "chassis.h"
#include "pid.h"
#include "cmsis_os.h"
#include "tim.h"
#include <stdlib.h>
#include <string.h>

/* ══════════════════════════════════════════════════════════════════════
   Global Variables
   ══════════════════════════════════════════════════════════════════════ */
volatile uint8_t   g_emergency_stop  = 0;
ChassisState_t     g_chassis_state   = STATE_DEBUG;
uint32_t           g_state_ticks     = 0;
uint16_t           g_red_count       = 0;
uint16_t           g_green_count     = 0;
int32_t            g_enc_total       = 0;
int32_t            g_turn_enc        = 0;
float              g_avg_speed_mps   = 0.0f;
uint32_t           g_sys_tick        = 0;     // freertos.c 每周期 +1

/* ══════════════════════════════════════════════════════════════════════
   Phase 2 文件级持久状态（保持巡线+避障不变）
   ══════════════════════════════════════════════════════════════════════ */

// 岔路口抑制（出岔后 3 秒内不再触发）
static uint32_t g_suppress_until_tick = 0;

// LINE_FOLLOW 持久状态
static float    last_line_err   = 0.0f;


// OBSTACLE 持久状态
static ObsSubStage_t ob_stage      = OB_STOP;
static int32_t       ob_start_enc  = 0;
static uint32_t      ob_ticks      = 0;
static int32_t       ob_mem_dist   = 0;
static int32_t       ob_turn_start = 0;

/* ══════════════════════════════════════════════════════════════════════
   Phase 2 文件级持久状态（岔路采摘）
   ══════════════════════════════════════════════════════════════════════ */

// 全局判定结果（供 OLED 读取）
BranchDecision_t g_branch_decision = {0, 0, 0, 0, 0, 0};
uint8_t          g_branch_zone     = 0;     // 区域编号递增

// LINE_FOLLOW 岔路检测计数器
static uint8_t  junction_cnt    = 0;        // 连续全黑帧计数

// BRANCH_ENTER 持久状态
static int32_t  be_fwd_start    = 0;        // 前进补偿起始编码器
static int32_t  be_turn_start   = 0;        // 转弯起始编码器
static uint32_t be_turn_ticks   = 0;        // 转弯超时计数

// BRANCH_FOLLOW 持久状态
static int32_t  bf_fwd_start     = 0;       // 前进起始编码器（用于倒车距离记录）
static int32_t  bf_rev_start     = 0;       // 倒车微调起始编码器
static uint8_t  bf_stop_cnt      = 0;       // 超声波停车确认计数
static uint8_t  bf_phase         = 0;       // 0=前进 1=倒车微调到36cm
static float    bf_last_line_err = 0.0f;    // 分支线丢线保持（独立，不污染主赛道）

// BRANCH_JUDGE 持久状态
static uint32_t bj_tick         = 0;        // 判定采集计数
static uint8_t  bj_best_red_area   = 0;     // 窗口内最大红草莓面积
static uint8_t  bj_best_green_area = 0;     // 窗口内最大绿草莓面积
static uint8_t  bj_red_cnt      = 0;        // 窗口内红草莓峰值
static uint8_t  bj_green_cnt    = 0;        // 窗口内绿草莓峰值

// BRANCH_REVERSE 持久状态
static int32_t  br_rev_start    = 0;        // 倒车起始编码器
static int32_t  br_comp_start   = 0;        // 倒车补偿起始编码器
static uint8_t  br_junction_cnt = 0;        // 倒车时岔路检测消抖
// Zone Demo 持久状态
uint8_t  g_zone_active      = 0;            // 1=Zone Demo 模式中
uint8_t  g_zone_id          = 0;            // 目标 Zone 编号 1~N
int32_t  g_zone_target_enc  = 0;            // 目标岔路编码器值
uint8_t  g_zone_done        = 0;            // 1=已完成判定

// ZONE_RETURN 持久状态
static int32_t  zr_turn_start    = 0;       // 180掉头起始 g_turn_enc
static int32_t  zr_fwd_start     = 0;       // Phase2 前进15cm起始 enc
static uint8_t  zr_phase         = 0;       // 0=掉头 1=前进 2=右转 3=倒车
static float    zr_last_line_err = 0.0f;    // 巡线丢线保持
static uint8_t  zr_junction_cnt  = 0;       // 岔路检测消抖


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
   Helper: wheel pointer lookup
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

/* ══════════════════════════════════════════════════════════════════════
   VOFA+ 命令解析
   MD→DEBUG  M0→IDLE  M1→LINE_FOLLOW ... M9→FINISH
   ══════════════════════════════════════════════════════════════════════ */

static uint8_t cur_wheel = 4; // 0=FL,1=FR,2=BL,3=BR,4=ALL

void Chassis_VOFA_Parse(const uint8_t *cmd_buf) {
    if (cmd_buf[0] == '\0') return;

    if (cmd_buf[0] == 'M') {
        // 手动切模式时取消 Zone Demo
        g_zone_active = 0;
        if (cmd_buf[1] == 'D') {
            g_chassis_state = STATE_DEBUG;
        } else if (cmd_buf[1] >= '0' && cmd_buf[1] <= '9') {
            // M0=IDLE(1) M1=LINE_FOLLOW(2) M2=OBSTACLE(3) M3=FINISH(4) M4=BRANCH_ENTER(5) M5=BRANCH_FOLLOW(6) M6=BRANCH_JUDGE(7) M7=BRANCH_REVERSE(8) M8=ZONE_RETURN(9)
            int m_idx = cmd_buf[1] - '0';
            if (cmd_buf[2] >= '0' && cmd_buf[2] <= '9') {
                m_idx = m_idx * 10 + (cmd_buf[2] - '0');
            }
            if (m_idx >= 0 && m_idx <= 8) {
                g_chassis_state = (ChassisState_t)(m_idx + 1);
            }
        }
        return;
    }

    if (cmd_buf[0] == 'W') {
        cur_wheel = (cmd_buf[1] == 'A') ? 4 : (cmd_buf[1] - '0');
        if (g_chassis_state == STATE_DEBUG) {
            for (int i = 0; i < 4; i++) {
                if (cur_wheel < 4 && i != (int)cur_wheel) {
                    get_wheel_pid(i)->Target = 0.0f;
                }
            }
        }
        return;
    }

    // F/L 命令：全局参数，不依赖 DEBUG 状态
    if (cmd_buf[0] == 'F') {
        extern float g_br_feedforward;
        g_br_feedforward = atof((const char *)&cmd_buf[1]);
        return;
    }
    if (cmd_buf[0] == 'L') {
        extern float g_speed_filt_alpha;
        float v = atof((const char *)&cmd_buf[1]);
        if (v >= 0.0f && v <= 0.99f) g_speed_filt_alpha = v;
        return;
    }

    if (cmd_buf[0] == 'T' && g_chassis_state != STATE_DEBUG) return;

    // Z: Zone Demo 命令 Z:<id>,<encoder>
    if (cmd_buf[0] == 'Z') {
        const char *p = (const char *)&cmd_buf[1];
        char *end;
        long id = strtol(p, &end, 10);
        if (end > p && *end == ',') {
            long enc = strtol(end + 1, NULL, 10);
            if (id >= 1 && id <= 99 && enc > 0) {
                Chassis_Stop();  // 先停车，再切巡线
                g_zone_active     = 1;
                g_zone_id         = (uint8_t)id;
                g_zone_target_enc = (int32_t)enc;
                g_zone_done       = 0;
                // 🔧 Day 21 修复：清除残留抑制 + 重置消抖计数
                //   - g_suppress_until_tick 可能被上次 BRANCH_REVERSE 设了值
                //   - junction_cnt 清空确保从 0 开始积累
                g_suppress_until_tick = 0;
                junction_cnt = 0;
                g_chassis_state   = STATE_LINE_FOLLOW;
            }
        }
        return;
    }

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

static void entry_debug(void)           { /* 保持 PID 状态，不操作 */ }

static void entry_idle(void)            { Chassis_Stop(); }

static void entry_line_follow(void) {
    Chassis_MoveForward(LINE_BASE_SPEED);

    last_line_err = 0.0f;

    junction_cnt  = 0;       // 重置岔路检测
    // 注意：不重置 g_suppress_until_tick！
    // BRANCH_REVERSE→LINE_FOLLOW 需要 suppress 防止重触发同一岔路
    // Z: 命令通过显式赋值来清零，见 Chassis_VOFA_Parse
}

static void entry_obstacle(void) {
    Chassis_Stop();
    ob_stage      = OB_STOP;
    ob_start_enc  = 0;
    ob_ticks      = 0;
    ob_mem_dist   = 0;
    ob_turn_start = 0;
}

static void entry_finish(void)          { Chassis_Stop(); }

/* ── BRANCH_ENTER: 前进补偿传感器偏移 → 原地转弯 ── */
static void entry_branch_enter(void) {
    // Phase 0: 先慢速前进，让轮轴对齐十字标记
    Chassis_MoveForward(0.15f);
    be_fwd_start  = g_enc_total;
    be_turn_start = 0;
    be_turn_ticks = 0;
    g_branch_zone++;                        // 进入新区域
}

/* ── BRANCH_FOLLOW: 沿分支线慢速前进，超声<36cm倒车微调 ── */
static void entry_branch_follow(void) {
    Chassis_MoveForward(BRANCH_FOLLOW_SPEED);
    bf_fwd_start = g_enc_total;
    bf_rev_start = 0;
    bf_stop_cnt  = 0;
    bf_phase     = 0;
}

/* ── BRANCH_JUDGE: 停车，重置采集窗口 ── */
static void entry_branch_judge(void) {
    Chassis_Stop();
    bj_tick           = 0;
    bj_best_red_area  = 0;
    bj_best_green_area = 0;
    bj_red_cnt        = 0;
    bj_green_cnt      = 0;
    memset(&g_branch_decision, 0, sizeof(g_branch_decision));
}

/* ── BRANCH_REVERSE: 直线倒车 ── */
static void entry_branch_reverse(void) {
    Chassis_MoveBackward(BRANCH_REVERSE_SPEED);
    br_rev_start    = g_enc_total;
    br_junction_cnt = 0;
}

/* ══════════════════════════════════════════════════════════════════════
   Run 函数 — 每周期调用，返回下一状态（或同状态继续）
   ══════════════════════════════════════════════════════════════════════ */

/* ── 0. DEBUG ── */
static ChassisState_t run_debug(const SensorData_t *s,
                                uint8_t hv, const VisionFrame_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_DEBUG;
}

/* ── 1. IDLE ── */
static ChassisState_t run_idle(const SensorData_t *s,
                                uint8_t hv, const VisionFrame_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_IDLE;
}

/* ── 2. LINE_FOLLOW — 红外巡线 + v5 侧边触发 ── */
static ChassisState_t run_line_follow(const SensorData_t *s,
                                       uint8_t hv, const VisionFrame_t *v) {
    /* ── 1. 红外位置计算（加权平均） ── */
    float ir_sum   = 0.0f;
    int   black_cnt = 0;
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        if (s->ir[i] == 0) { ir_sum += (float)i; black_cnt++; }
    }

    float line_err;
    if (black_cnt > 0) {
        line_err = ir_sum / (float)black_cnt - IR_CENTER_OFFSET;
        last_line_err = line_err;
    } else {
        // 丢线时衰减 last_line_err，防止永久转圈
        // 0.95^100 ≈ 0.006，约 1 秒后趋近直行
        last_line_err *= 0.95f;
        line_err = last_line_err;
    }

    /* ── 2. P 控制器 → 差速转向 ── */
#if FORCE_STRAIGHT
    Chassis_MoveForward(LINE_BASE_SPEED);
#else
    float turn_corr = line_err * LINE_KP;
    float speed = (black_cnt == 0) ? 0.15f : LINE_BASE_SPEED;
    Chassis_DiffTurn(speed, turn_corr);
#endif

    /* ── 3. 岔路口检测（6路全黑 = 十字标记，出岔后抑制期内不触发） ── */
    if (g_sys_tick < g_suppress_until_tick) {
        junction_cnt = 0;  // 抑制期内清零
    } else if (black_cnt >= IR_SENSOR_COUNT) {
        junction_cnt++;
        if (junction_cnt >= JUNCTION_CONFIRM_FRAMES) {
            junction_cnt = 0;
            // Zone Demo 模式：只匹配目标岔路
            if (g_zone_active) {
                int32_t diff = g_enc_total - g_zone_target_enc;
                if (diff < 0) diff = -diff;
                if (diff < ZONE_ENC_TOLERANCE) {
                    return STATE_BRANCH_ENTER;  // 匹配！拐入
                }
                // 不匹配 → 短抑制 500ms（50tick），只够越过胶带宽度
                // 用 500ms 而非 3s：避免误伤后续的目标岔路
                g_suppress_until_tick = g_sys_tick + 50;  // 500ms @ 100Hz
            } else {
                return STATE_BRANCH_ENTER;  // 普通模式：任何岔路都拐
            }
        }
    } else {
        junction_cnt = 0;
    }

    return STATE_LINE_FOLLOW;
}

/* ── 3. OBSTACLE — 10 阶段避障绕行（保持不变） ── */
static ChassisState_t run_obstacle(const SensorData_t *s,
                                    uint8_t hv, const VisionFrame_t *v) {
    (void)hv; (void)v;

    int32_t dist_traveled;
    int     black_cnt;

    static ObsSubStage_t prev_ob_stage = OB_DONE;
    static uint8_t       brake_ticks   = 0;

    if (ob_stage != prev_ob_stage) {
        brake_ticks   = 0;
        prev_ob_stage = ob_stage;
    }

    if (brake_ticks < OB_BRAKE_TIMEOUT) {
        Chassis_Stop();
        brake_ticks++;
        float spd = pid_FL.Actual; if (spd < 0) spd = -spd;
        if (spd < 0.03f) {
            spd = pid_FR.Actual; if (spd < 0) spd = -spd;
            if (spd < 0.03f) {
                spd = pid_BL.Actual; if (spd < 0) spd = -spd;
                if (spd < 0.03f) {
                    spd = pid_BR.Actual; if (spd < 0) spd = -spd;
                    if (spd < 0.03f) brake_ticks = OB_BRAKE_TIMEOUT;
                }
            }
        }
        return STATE_OBSTACLE;
    }

    switch (ob_stage) {

    case OB_STOP:
        ob_ticks++;
        if (ob_ticks >= 30) {
            ob_stage = OB_BACK;
            ob_start_enc = g_enc_total;
            ob_ticks = 0;
        }
        break;

    case OB_BACK:
        Chassis_MoveBackward(OB_BACK_SPEED);
        dist_traveled = ob_start_enc - g_enc_total;
        if (dist_traveled >= (int32_t)(OB_BACK_CM * ENC_PER_CM)) {
            ob_stage = OB_TURN_RIGHT; ob_ticks = 0;
        }
        break;

    case OB_TURN_RIGHT:
        if (ob_ticks == 0) ob_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, 1);
        ob_ticks++;
        if (g_turn_enc - ob_turn_start >= TURN90_TOTAL_ENC || ob_ticks >= OB_TURN_MS / 10) {
            ob_stage = OB_CROSS; ob_start_enc = g_enc_total; ob_ticks = 0;
        }
        break;

    case OB_CROSS:
        Chassis_MoveForward(OB_FWD_SPEED);
        dist_traveled = g_enc_total - ob_start_enc;
        ob_mem_dist = dist_traveled;
        if (dist_traveled >= (int32_t)(OB_CROSS_CM * ENC_PER_CM)) {
            ob_stage = OB_TURN_LEFT1; ob_ticks = 0;
        }
        break;

    case OB_TURN_LEFT1:
        if (ob_ticks == 0) ob_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, -1);
        ob_ticks++;
        if (g_turn_enc - ob_turn_start >= TURN90_TOTAL_ENC || ob_ticks >= OB_TURN_MS / 10) {
            ob_stage = OB_FORWARD; ob_start_enc = g_enc_total; ob_ticks = 0;
        }
        break;

    case OB_FORWARD:
        Chassis_MoveForward(OB_FWD_SPEED);
        dist_traveled = g_enc_total - ob_start_enc;
        if (dist_traveled >= (int32_t)(OB_FORWARD_CM * ENC_PER_CM)) {
            ob_stage = OB_TURN_LEFT2; ob_ticks = 0;
        }
        break;

    case OB_TURN_LEFT2:
        if (ob_ticks == 0) ob_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, -1);
        ob_ticks++;
        if (g_turn_enc - ob_turn_start >= TURN90_TOTAL_ENC || ob_ticks >= OB_TURN_MS / 10) {
            ob_stage = OB_RETURN; ob_start_enc = g_enc_total; ob_ticks = 0;
        }
        break;

    case OB_RETURN:
        Chassis_MoveForward(OB_FWD_SPEED);
        dist_traveled = g_enc_total - ob_start_enc;
        if (dist_traveled >= ob_mem_dist) {
            ob_stage = OB_TURN_45; ob_ticks = 0;
        }
        break;

    case OB_TURN_45:
        if (ob_ticks == 0) ob_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, 1);
        ob_ticks++;
        if (g_turn_enc - ob_turn_start >= TURN45_TOTAL_ENC || ob_ticks >= OB_TURN_MS / 10) {
            ob_stage = OB_FORWARD_TO_LINE; ob_start_enc = g_enc_total; ob_ticks = 0;
        }
        break;

    case OB_FORWARD_TO_LINE:
        Chassis_MoveForward(OB_FWD_SPEED);
        ob_ticks++;
        black_cnt = 0;
        for (int i = 0; i < IR_SENSOR_COUNT; i++) if (s->ir[i] == 0) black_cnt++;
        dist_traveled = g_enc_total - ob_start_enc;
        if (black_cnt >= 2 || dist_traveled >= (int32_t)(OB_FWD_TO_LINE_CM * ENC_PER_CM)) {
            ob_stage = OB_DONE;
        }
        break;

    case OB_DONE:
    default:
        ob_stage = OB_STOP;
        return STATE_LINE_FOLLOW;
    }

    return STATE_OBSTACLE;
}


/* ── 4. FINISH [STUB] ── */
static ChassisState_t run_finish(const SensorData_t *s,
                                  uint8_t hv, const VisionFrame_t *v) {
    (void)s; (void)hv; (void)v;
    return STATE_FINISH;
}

/* ── 5. BRANCH_ENTER — 前进补偿 → 原地转弯 90° 进入分支线 ── */
static ChassisState_t run_branch_enter(const SensorData_t *s,
                                        uint8_t hv, const VisionFrame_t *v) {
    (void)s; (void)hv; (void)v;

    // Phase 0: 前进补偿传感器→轮轴偏移距离
    if (be_turn_start == 0) {
        int32_t fwd_dist = g_enc_total - be_fwd_start;
        if (fwd_dist < (int32_t)(BRANCH_FWD_BEFORE_TURN_CM * ENC_PER_CM)) {
            return STATE_BRANCH_ENTER;  // 还没走够，继续前进
        }
        // 到位，开始转弯
        be_turn_start = g_turn_enc;
        Chassis_TurnInPlace(OB_TURN_SPEED, JUNCTION_SIDE);
    }

    // Phase 1: 原地转弯 90°
    be_turn_ticks++;

    if ((g_turn_enc - be_turn_start) >= TURN90_TOTAL_ENC
        || be_turn_ticks >= (OB_TURN_MS / 10)) {
        Chassis_Stop();
        return STATE_BRANCH_FOLLOW;
    }

    return STATE_BRANCH_ENTER;
}

/* ── 6. BRANCH_FOLLOW — 沿分支线慢速巡线，超声<36cm倒车微调到36cm ── */
static ChassisState_t run_branch_follow(const SensorData_t *s,
                                         uint8_t hv, const VisionFrame_t *v) {
    (void)hv; (void)v;

    // Phase 0: 沿分支线前进直到超声 < 36cm
    if (bf_phase == 0) {
        float ir_sum = 0.0f;
        int   black_cnt = 0;
        for (int i = 0; i < IR_SENSOR_COUNT; i++) {
            if (s->ir[i] == 0) { ir_sum += (float)i; black_cnt++; }
        }

        float line_err;
        if (black_cnt > 0) {
            line_err = ir_sum / (float)black_cnt - IR_CENTER_OFFSET;
            bf_last_line_err = line_err;
        } else {
            bf_last_line_err *= 0.95f;
            line_err = bf_last_line_err;
        }
        float turn_corr = line_err * BRANCH_LINE_KP;
        float speed = (black_cnt == 0) ? 0.10f : BRANCH_FOLLOW_SPEED;
        Chassis_DiffTurn(speed, turn_corr);

        if (s->distance_cm < BRANCH_STOP_DIST_CM) {
            bf_stop_cnt++;
            if (bf_stop_cnt >= BRANCH_STOP_CONFIRM) {
                Chassis_Stop();
                bf_rev_start = g_enc_total;
                bf_phase = 1;
            }
        } else {
            bf_stop_cnt = 0;
        }

        int32_t fwd_dist = g_enc_total - bf_fwd_start;
        if (fwd_dist >= (int32_t)(BRANCH_REVERSE_MAX_CM * ENC_PER_CM)) {
            Chassis_Stop();
            return STATE_BRANCH_REVERSE;
        }
    }
    // Phase 1: 倒车微调，直到超声距离回到 36cm
    else {
        Chassis_MoveBackward(0.10f);
        if (s->distance_cm >= BRANCH_STOP_DIST_CM) {
            bf_stop_cnt++;
            if (bf_stop_cnt >= BRANCH_STOP_CONFIRM) {
                bf_stop_cnt = 0;
                bf_phase = 0;
                Chassis_Stop();
                return STATE_BRANCH_JUDGE;
            }
        } else {
            bf_stop_cnt = 0;
        }

        // 安全超时：倒车距离不超过前进距离
        int32_t rev_dist = bf_rev_start - g_enc_total;
        if (rev_dist >= (int32_t)(BRANCH_REVERSE_MAX_CM * ENC_PER_CM)) {
            Chassis_Stop();
            bf_phase = 0;
            return STATE_BRANCH_JUDGE;  // 超时也判定，不浪费这次岔路
        }
    }

    return STATE_BRANCH_FOLLOW;
}

/* ── 7. BRANCH_JUDGE — 停车采集 K210 数据，多目标排序决策 ── */
static ChassisState_t run_branch_judge(const SensorData_t *s,
                                        uint8_t hv, const VisionFrame_t *v) {
    (void)s;

    if (hv && v != NULL && v->count > 0) {
        for (int i = 0; i < v->count; i++) {
            uint8_t cls  = v->t[i].cls;
            uint8_t area = v->t[i].area;

            if (area < BRANCH_AREA_MIN) continue;

            if (cls == 0) {
                bj_red_cnt++;
                if (area > bj_best_red_area) bj_best_red_area = area;
            } else if (cls == 1) {
                bj_green_cnt++;
                if (area > bj_best_green_area) bj_best_green_area = area;
            }
        }
    }

    bj_tick++;

    if (bj_tick >= BRANCH_JUDGE_TICKS) {
        g_branch_decision.total_red   = (bj_red_cnt > 255) ? 255 : (uint8_t)bj_red_cnt;
        g_branch_decision.total_green = (bj_green_cnt > 255) ? 255 : (uint8_t)bj_green_cnt;

        if (bj_best_red_area >= BRANCH_AREA_MIN) {
            g_branch_decision.detected = 1;
            g_branch_decision.cls      = 0;
            g_branch_decision.area     = bj_best_red_area;
            g_branch_decision.pick     = 1;
        } else if (bj_best_green_area >= BRANCH_AREA_MIN) {
            g_branch_decision.detected = 1;
            g_branch_decision.cls      = 1;
            g_branch_decision.area     = bj_best_green_area;
            g_branch_decision.pick     = 0;
        } else {
            g_branch_decision.detected = 0;
            g_branch_decision.pick     = 0;
        }

        if (g_branch_decision.pick) {
            g_red_count++;
        } else if (g_branch_decision.detected && g_branch_decision.cls == 1) {
            g_green_count++;
        }

        return STATE_BRANCH_REVERSE;
    }

    return STATE_BRANCH_JUDGE;
}

/* ── 8. BRANCH_REVERSE — 直线倒车返回主赛道 ── */
static ChassisState_t run_branch_reverse(const SensorData_t *s,
                                          uint8_t hv, const VisionFrame_t *v) {
    (void)hv; (void)v;

    static uint8_t  br_phase      = 0;
    static int32_t  br_turn_start = 0;
    static uint32_t br_turn_ticks = 0;

    if (g_state_ticks == 0) {
        br_phase      = 0;
        br_comp_start = 0;
        br_turn_start = 0;
        br_turn_ticks = 0;
    }

    // Phase 0: 倒车 → 检测十字标记
    if (br_phase == 0) {
        int black_cnt = 0;
        for (int i = 0; i < IR_SENSOR_COUNT; i++) {
            if (s->ir[i] == 0) black_cnt++;
        }

        if (black_cnt >= IR_SENSOR_COUNT) {
            br_junction_cnt++;
            if (br_junction_cnt >= JUNCTION_CONFIRM_FRAMES) {
                // 检测到十字 → 刹车 → 进入前进补偿阶段
                Chassis_Stop();
                br_comp_start = g_enc_total;
                br_phase = 1;
                br_junction_cnt = 0;
            }
        } else {
            br_junction_cnt = 0;
        }

        int32_t rev_dist = br_rev_start - g_enc_total;
        if (rev_dist >= (int32_t)(BRANCH_REVERSE_MAX_CM * ENC_PER_CM)) {
            Chassis_Stop();
            g_suppress_until_tick = g_sys_tick + JUNCTION_SUPPRESS_MS / 10;
            junction_cnt = 0;
            br_junction_cnt = 0;
            br_phase = 0;
            if (g_zone_active) {
                return STATE_ZONE_RETURN;
            }
            return STATE_LINE_FOLLOW;
        }
    }
    // Phase 1: 前进 13cm，把几何中心拉回十字
    else if (br_phase == 1) {
        Chassis_MoveForward(0.15f);  // 每帧调用，实际只在首帧生效
        int32_t comp_dist = g_enc_total - br_comp_start;
        if (comp_dist >= (int32_t)(BRANCH_FWD_BEFORE_TURN_CM * ENC_PER_CM)) {
            Chassis_Stop();
            br_phase = 2;
            br_turn_start = g_turn_enc;
            br_turn_ticks = 0;
        }
    }
    // Phase 2: 原地转弯 90° 回正
    else {
        Chassis_TurnInPlace(OB_TURN_SPEED, -JUNCTION_SIDE);
        br_turn_ticks++;

        if ((g_turn_enc - br_turn_start) >= TURN90_TOTAL_ENC
            || br_turn_ticks >= (OB_TURN_MS / 10)) {
            Chassis_Stop();
            g_suppress_until_tick = g_sys_tick + JUNCTION_SUPPRESS_MS / 10;
            junction_cnt = 0;
            br_junction_cnt = 0;
            br_phase = 0;
            if (g_zone_active) {
                return STATE_ZONE_RETURN;
            }
            return STATE_LINE_FOLLOW;
        }
    }

    return STATE_BRANCH_REVERSE;
}



/* ── 9. ZONE_RETURN: 掉头180→前进到岔路→右转15cm→倒车回START ── */
static void entry_zone_return(void) {
    Chassis_Stop();
    zr_phase        = 0;
    zr_turn_start   = 0;
    zr_fwd_start    = 0;
    zr_junction_cnt = 0;
    zr_last_line_err = 0.0f;
}

static ChassisState_t run_zone_return(const SensorData_t *s,
                                       uint8_t hv, const VisionFrame_t *v) {
    (void)hv; (void)v;

    /* Phase 0: 原地180度右转，车头从朝草莓转成朝主赛道 */
    if (zr_phase == 0) {
        if (zr_turn_start == 0) {
            zr_turn_start = g_turn_enc;
            Chassis_TurnInPlace(OB_TURN_SPEED, JUNCTION_SIDE);
        }
        if ((g_turn_enc - zr_turn_start) >= TURN180_TOTAL_ENC) {
            Chassis_Stop();
            zr_phase = 1;
        }
        return STATE_ZONE_RETURN;
    }

    /* Phase 1: 沿分支线前进，回到主赛道岔路口 */
    if (zr_phase == 1) {
        float ir_sum = 0.0f;
        int   black_cnt = 0;
        for (int i = 0; i < IR_SENSOR_COUNT; i++) {
            if (s->ir[i] == 0) { ir_sum += (float)i; black_cnt++; }
        }

        float line_err;
        if (black_cnt > 0) {
            line_err = ir_sum / (float)black_cnt - IR_CENTER_OFFSET;
            zr_last_line_err = line_err;
        } else {
            zr_last_line_err *= 0.95f;
            line_err = zr_last_line_err;
        }
        float turn_corr = line_err * BRANCH_LINE_KP;
        float speed = (black_cnt == 0) ? 0.10f : BRANCH_FOLLOW_SPEED;
        Chassis_DiffTurn(speed, turn_corr);

        if (black_cnt >= IR_SENSOR_COUNT) {
            zr_junction_cnt++;
            if (zr_junction_cnt >= JUNCTION_CONFIRM_FRAMES) {
                zr_junction_cnt = 0;
                Chassis_Stop();
                zr_phase = 2;
                zr_turn_start = 0;
                zr_fwd_start  = 0;
            }
        } else {
            zr_junction_cnt = 0;
        }
        // 超时保护（10秒未到达岔路口）
        if (g_state_ticks > 1000) {
            Chassis_Stop();
            g_zone_active = 0;
            g_zone_done   = 1;
            return STATE_IDLE;
        }
        return STATE_ZONE_RETURN;
    }

    /* Phase 2: 右转90度 + 前进15cm，车头朝终点 */
    if (zr_phase == 2) {
        if (zr_turn_start == 0) {
            zr_turn_start = g_turn_enc;
            Chassis_TurnInPlace(OB_TURN_SPEED, JUNCTION_SIDE);
        } else if ((g_turn_enc - zr_turn_start) >= TURN90_TOTAL_ENC) {
            if (zr_fwd_start == 0) {
                zr_fwd_start = g_enc_total;
                Chassis_MoveForward(0.15f);
            } else if ((g_enc_total - zr_fwd_start) >= (int32_t)(15 * ENC_PER_CM)) {
                Chassis_Stop();
                zr_phase = 3;
            }
        }
        return STATE_ZONE_RETURN;
    }

    /* Phase 3: IR巡线倒车回START，停车: 4路全黑 AND enc<500 */
    int black_cnt = 0;
    float ir_sum = 0.0f;
    for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        if (s->ir[i] == 0) { ir_sum += (float)i; black_cnt++; }
    }

    if (black_cnt == 4 && g_enc_total < ZONE_START_ENC_THRESHOLD) {
        Chassis_Stop();
        g_zone_active = 0;
        g_zone_done   = 1;
        return STATE_IDLE;
    }

    // 超时保护（30秒未到 START 则强制停车）
    if (g_state_ticks > (ZONE_RETURN_MAX_MS / 10)) {
        Chassis_Stop();
        g_zone_active = 0;
        g_zone_done   = 1;
        return STATE_IDLE;
    }

    float line_err;
    if (black_cnt > 0) {
        line_err = ir_sum / (float)black_cnt - IR_CENTER_OFFSET;
        zr_last_line_err = line_err;
    } else {
        zr_last_line_err *= 0.95f;
        line_err = zr_last_line_err;
    }
    float turn_corr = line_err * LINE_KP;
    float speed = (black_cnt == 0) ? 0.08f : ZONE_RETURN_SPEED;
    Chassis_DiffTurn(-speed, -turn_corr);

    return STATE_ZONE_RETURN;
}

/* ══════════════════════════════════════════════════════════════════════
   状态描述符表（15 状态，含 Phase 4 岔路采摘）
   ══════════════════════════════════════════════════════════════════════ */

static const StateDescriptor_t state_table[STATE_COUNT] = {
    [STATE_DEBUG]           = { "DEBUG",           entry_debug,           run_debug,           1 },
    [STATE_IDLE]            = { "IDLE",            entry_idle,            run_idle,            0 },
    [STATE_LINE_FOLLOW]     = { "LINE_FOLLOW",     entry_line_follow,     run_line_follow,     0 },
    [STATE_OBSTACLE]        = { "OBSTACLE",        entry_obstacle,        run_obstacle,        0 },
    [STATE_FINISH]          = { "FINISH",          entry_finish,          run_finish,          0 },
    [STATE_BRANCH_ENTER]    = { "BRANCH_ENTER",    entry_branch_enter,    run_branch_enter,    0 },
    [STATE_BRANCH_FOLLOW]   = { "BRANCH_FOLLOW",   entry_branch_follow,   run_branch_follow,   0 },
    [STATE_BRANCH_JUDGE]    = { "BRANCH_JUDGE",    entry_branch_judge,    run_branch_judge,    0 },
    [STATE_BRANCH_REVERSE]  = { "BRANCH_REVERSE",  entry_branch_reverse,  run_branch_reverse,  0 },
    [STATE_ZONE_RETURN]    = { "ZONE_RETURN",    entry_zone_return,    run_zone_return,      0 },
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
    g_chassis_state       = STATE_DEBUG;
    g_state_ticks         = 0;
    g_emergency_stop      = 0;
    g_red_count           = 0;
    g_green_count         = 0;
    g_enc_total           = 0;
    g_branch_zone         = 0;
    memset(&g_branch_decision, 0, sizeof(g_branch_decision));
    g_zone_active         = 0;
    g_zone_id             = 0;
    g_zone_target_enc     = 0;
    g_zone_done           = 0;
}

/* ══════════════════════════════════════════════════════════════════════
   Chassis_StateMachine_Run — 每 10ms 主循环调用一次
   ══════════════════════════════════════════════════════════════════════ */

void Chassis_StateMachine_Run(const SensorData_t *sensor,
                              uint8_t has_vision, const VisionFrame_t *vis,
                              int32_t enc_fl, int32_t enc_fr,
                              int32_t enc_bl, int32_t enc_br)
{
    (void)enc_fl; (void)enc_fr; (void)enc_bl; (void)enc_br;

    static ChassisState_t prev_state = STATE_COUNT;

    if (g_chassis_state != prev_state) {
        if (g_chassis_state < STATE_COUNT) {
            state_table[g_chassis_state].entry();
        }
        g_state_ticks = 0;
        prev_state    = g_chassis_state;
    }

    const StateDescriptor_t *desc = &state_table[g_chassis_state];
    ChassisState_t next = desc->run(sensor, has_vision, vis);
    g_state_ticks++;

    if (next != g_chassis_state && next < STATE_COUNT) {
        g_chassis_state = next;
    }
}
