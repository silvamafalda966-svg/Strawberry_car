#ifndef __CHASSIS_H
#define __CHASSIS_H

#include "main.h"

/* ────────── 状态枚举（v2 岔路方案 + Zone Demo） ────────── */
typedef enum {
    STATE_DEBUG = 0,        // VOFA+ 手动调参
    STATE_IDLE,             // 空闲等待
    STATE_LINE_FOLLOW,      // 红外巡线行驶（含岔路口检测）
    STATE_OBSTACLE,         // 超声波避障绕行
    STATE_FINISH,           // 任务完成
    STATE_BRANCH_ENTER,     // 岔路检测 → 原地转弯进入分支线
    STATE_BRANCH_FOLLOW,    // 沿分支线前进，超声波监测距离
    STATE_BRANCH_JUDGE,     // 停车，K210 判定成熟度+大小
    STATE_BRANCH_REVERSE,   // 直线倒车返回主赛道
    STATE_ZONE_RETURN,      // Zone Demo 返程：掉头180°→巡线倒退→START停车
    STATE_COUNT
} ChassisState_t;

/* ────────── TCRT5000 红外参数 ────────── */
// 6 路等间距排列，总宽 10cm，DO 口数字输出
// DO=0 表示黑线，DO=1 表示白底（LM393 比较器）
#define IR_SENSOR_COUNT      6           // PE0~PE5 有效传感器数
#define IR_CENTER_OFFSET     2.5f        // (IR_SENSOR_COUNT-1)/2 = 2.5

/* ────────── 传感器数据结构体（对齐 1 字节） ────────── */
#pragma pack(1)
typedef struct {
    uint8_t  ir[8];         // PE0~PE5 = TCRT5000 DO(0=黑 1=白), PE6~PE7 = 保留(1)
    float    distance_cm;   // HC-SR04 超声波测距
} SensorData_t;

/* ── K210 视觉帧（v5：定长 24B 多目标批量帧） ── */
typedef struct {
    uint8_t cls;            // 0=红草莓(成熟/ripe), 1=绿草莓(未熟/unripe)
    uint8_t x;              // 水平坐标 0~255（K210 320→255 压缩）
    uint8_t y;              // 垂直坐标 0~240
    uint8_t area;           // 面积百分比 0~100
} VisionTarget_t;

typedef struct {
    uint8_t         count;      // 有效草莓数 0~5
    VisionTarget_t  t[5];       // 最多 5 个 Target，无效槽全 0
} VisionFrame_t;
#pragma pack()

/* ────────── 状态描述符表项 ────────── */
typedef struct {
    const char    *name;
    void         (*entry)(void);
    ChassisState_t (*run)(const SensorData_t *sensor,
                          uint8_t has_vision, const VisionFrame_t *vis);
    uint8_t        allow_voafa_t;   // 1=DEBUG之外也允许T命令
} StateDescriptor_t;

/* ────────── 全局变量声明 ────────── */
extern volatile uint8_t   g_emergency_stop;
extern ChassisState_t     g_chassis_state;
extern uint32_t           g_state_ticks;
extern uint16_t           g_red_count;       // 红草莓累计计数
extern uint16_t           g_green_count;     // 绿草莓累计计数
extern int32_t            g_enc_total;       // 四轮编码器累积均值（有符号，前+后-）
extern float              g_avg_speed_mps;   // 四轮平均速度 (m/s)，Task_OLED 读取
extern int32_t            g_turn_enc;        // 四轮编码器绝对值累加（用于精算转角）
extern uint32_t           g_sys_tick;        // 全局滴答（每 10ms +1），freertos.c 递增

/* ────────── 避障子阶段枚举（右绕 + 斜向回线） ────────── */
typedef enum {
    OB_STOP = 0,       // 刹车停顿 300ms
    OB_BACK,           // 直线倒车 5cm
    OB_TURN_RIGHT,     // 原地右转 90°
    OB_FWD_RIGHT,      // 前进 25cm 绕过障碍
    OB_TURN_LEFT135,   // 原地左转 135° 斜向对线
    OB_SEEK,           // 直线前进，IR 找黑线
    OB_DONE
} ObsSubStage_t;

/* ═══════════════════════════════════════════════════════════════
   🔧 赛道实测校准参数（7月8日上赛道第一件事：测这三个！）
   ═══════════════════════════════════════════════════════════════ */

// [1] 直线里程校准 — 巡线跑 2m 直道，录 enc 值，除以 5414
//     理论: 2707 enc/cm，赛道实际有打滑
//     例: enc=6500 → 因子=6500/5414=1.20 → 改下面这行
#define ENC_PER_CM          2789   // 赛道实测: 284500enc/102cm=2789

// [2] 原地90°转弯 — 赛道地面原地右转 90°，car-studio 录 g_turn_enc
//     当前为地板实测值，赛道上必须重测！
#define TURN90_TOTAL_ENC    199700 // 赛道实测: 209700×90/94.5
#define TURN96_TOTAL_ENC    213000 // 赛道实测: TURN90 × 96/90 ≈ 213000
#define TURN45_TOTAL_ENC    (TURN90_TOTAL_ENC / 2)

// [3] 原地180°掉头 — 赛道地面原地右转 180°，car-studio 录 g_turn_enc
//     测法: MD→WA T0.15→W0 T0.15→W2 T0.15→W1 T-0.15→W3 T-0.15→转够180°→T0
#define TURN180_TOTAL_ENC   399400 // 赛道实测: =TURN90×2

// [4] 岔路编码器匹配容差 — Zone 匹配时允许的编码器偏差
//     25cm × ENC_PER_CM ≈ 68000
#define ZONE_ENC_TOLERANCE  68000  // 约 ±25cm，赛道实测后根据精度调整

/* ═══════════════════════════════════════════════════════════════ */

/* ────────── Phase 2 可调参数（巡线 + 避障） ────────── */
#define LINE_KP             0.06f
#define LINE_BASE_SPEED     0.3f
#define OB_BACK_SPEED       0.2f
#define OB_FWD_SPEED        0.3f
#define OB_SEEK_SPEED       0.15f   // 斜向找线慢速，防冲过头
#define OB_TURN_SPEED       0.25f   // 原 0.15，提升到 0.25 克服 FR 倒车死区
#define OB_TURN_MS          2000
#define OB_BACK_CM          5.0f    // 倒车距离 (cm)
#define OB_FWD_RIGHT_CM     25.0f   // 右绕前进距离 (cm)
#define OB_SEEK_MAX_CM      80.0f   // 找线最大距离 (cm)，超时兜底
#define OB_BRAKE_TIMEOUT    50      // 阶段间刹车停顿 (tick)
#define ENABLE_ULTRASONIC   1
#define FORCE_STRAIGHT      0

/* ────────── Phase 2 岔路采摘参数 ────────── */
// 岔路口检测
#define JUNCTION_CONFIRM_FRAMES  3      // 连续 N 帧 6 路全黑 = 岔路确认
#define JUNCTION_SIDE           -1      // 🔧 草莓区内侧：左转进入分支 (-1=左转, 1=右转)
#define JUNCTION_SUPPRESS_MS     2000   // 出岔路后抑制重触发 (ms)
#define BRANCH_FWD_BEFORE_TURN_CM 13.0f  // 检测到岔路后前进距离：传感器→几何中心 (cm)

// 分支线巡线
#define BRANCH_FOLLOW_SPEED      0.2f   // 分支线巡线速度 (m/s)
#define BRANCH_LINE_KP           0.04f  // 分支线 P 控制器增益

// 超声波停车
#define BRANCH_STOP_DIST_CM      23.0f  // 超声波 < 此值停车 (cm)
#define BRANCH_STOP_CONFIRM      5      // 连续 N 帧满足距离才停车

// 判定
#define BRANCH_JUDGE_TICKS       300    // 停车后采集 N 帧 K210 数据（3s）
#define BRANCH_AREA_MIN          2      // 面积 < 此值视为噪声/误检（v2新模型框较小）

// 倒车返回
#define BRANCH_REVERSE_SPEED     0.25f  // 倒车速度 (m/s)
#define BRANCH_REVERSE_MAX_CM    100.0f // 倒车最大距离（安全超时）

/* ── Zone Demo 返程参数 ── */
#define ZONE_RETURN_SPEED          0.2f   // 返程倒退巡线速度 (m/s)
#define ZONE_START_ENC_THRESHOLD   500    // 编码器 < 此值认为回到 START (约 2cm)
#define ZONE_RETURN_MAX_MS         30000  // 返程最长时限 30 秒（安全超时）

/* ── 岔路判定结果（供 OLED 显示） ── */
typedef struct {
    uint8_t  detected;      // 0=无草莓, 1=有有效目标
    uint8_t  cls;           // 0=红(ripe), 1=绿(unripe)
    uint8_t  area;          // 选中目标的面积
    uint8_t  total_red;     // 视野中红草莓总数
    uint8_t  total_green;   // 视野中绿草莓总数
    uint8_t  pick;          // 1=采摘, 0=跳过(绿/太小)
    uint16_t frames_in;     // 调试：窗口内收到的帧总数
    uint16_t frames_ok;     // 调试：其中 count>0 的帧数
    uint8_t  best_red;      // 调试：最佳红面积
    uint8_t  best_green;    // 调试：最佳绿面积
} BranchDecision_t;

extern BranchDecision_t g_branch_decision;
extern uint8_t          g_branch_zone;     // 当前区域编号（每进岔路+1）
extern float            g_mpu6050_yaw;     // MPU6050 陀螺仪积分航向角 (deg)
extern uint8_t          g_mpu6050_alive;   // 1=陀螺仪 I2C 通信正常

/* ── Zone Demo 任务状态 ── */
extern uint8_t  g_zone_active;       // 1=Zone Demo 模式中
extern uint8_t  g_zone_id;           // 目标 Zone 编号（1~N）
extern int32_t  g_zone_target_enc;   // 目标岔路的预期编码器值
extern uint8_t  g_zone_done;         // 1=已完成判定，准备返程

/* ────────── Chassis_Move API（只设 Target，不碰 PWM） ────────── */
void Chassis_MoveForward(float speed_mps);
void Chassis_MoveBackward(float speed_mps);
void Chassis_Stop(void);
void Chassis_DiffTurn(float base_speed, float turn_diff);
void Chassis_TurnInPlace(float speed_mps, int8_t dir);
void Chassis_SetTargets(float fl, float fr, float bl, float br);

/* ────────── 状态机调度 ────────── */
void Chassis_Init(void);
void Chassis_StateMachine_Run(const SensorData_t *sensor,
                              uint8_t has_vision, const VisionFrame_t *vis,
                              int32_t enc_fl, int32_t enc_fr,
                              int32_t enc_bl, int32_t enc_br);
void Chassis_VOFA_Parse(const uint8_t *cmd_buf);

/* ────────── 舵机 Y 轴伺服（供 VISUAL_APPROACH 调用） ────────── */
void Servo_Y_Servo(float y_deviation_px);

#endif /* __CHASSIS_H */
