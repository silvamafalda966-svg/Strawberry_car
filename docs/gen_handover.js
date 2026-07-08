const fs = require("fs");
const {
  Document, Packer, Paragraph, TextRun, Table, TableRow, TableCell,
  Header, Footer, AlignmentType, HeadingLevel,
  BorderStyle, WidthType, ShadingType,
  PageNumber, PageBreak, LevelFormat
} = require("docx");

// ── reusable helpers ──
const border = { style: BorderStyle.SINGLE, size: 1, color: "999999" };
const borders = { top: border, bottom: border, left: border, right: border };
const cellMargins = { top: 60, bottom: 60, left: 100, right: 100 };

function headerCell(text, width) {
  return new TableCell({
    borders,
    width: { size: width, type: WidthType.DXA },
    shading: { fill: "1F4E79", type: ShadingType.CLEAR },
    margins: cellMargins,
    verticalAlign: "center",
    children: [new Paragraph({
      alignment: AlignmentType.CENTER,
      children: [new TextRun({ text, bold: true, font: "Microsoft YaHei", size: 20, color: "FFFFFF" })]
    })]
  });
}

function cell(text, width, opts = {}) {
  const runs = [];
  if (typeof text === "string") {
    runs.push(new TextRun({ text, font: "Microsoft YaHei", size: 20, ...opts }));
  } else {
    text.forEach(t => runs.push(new TextRun({ font: "Microsoft YaHei", size: 20, ...t })));
  }
  return new TableCell({
    borders,
    width: { size: width, type: WidthType.DXA },
    margins: cellMargins,
    children: [new Paragraph({ children: runs })]
  });
}

function monoCell(text, width) {
  return new TableCell({
    borders,
    width: { size: width, type: WidthType.DXA },
    shading: { fill: "F5F5F5", type: ShadingType.CLEAR },
    margins: cellMargins,
    children: [new Paragraph({
      children: [new TextRun({ text, font: "Consolas", size: 18 })]
    })]
  });
}

function heading1(text) {
  return new Paragraph({
    heading: HeadingLevel.HEADING_1,
    spacing: { before: 360, after: 200 },
    children: [new TextRun({ text, font: "Microsoft YaHei", size: 32, bold: true, color: "1F4E79" })]
  });
}
function heading2(text) {
  return new Paragraph({
    heading: HeadingLevel.HEADING_2,
    spacing: { before: 280, after: 160 },
    children: [new TextRun({ text, font: "Microsoft YaHei", size: 26, bold: true, color: "2E75B6" })]
  });
}
function para(text, opts = {}) {
  return new Paragraph({
    spacing: { after: 120 },
    children: [new TextRun({ text, font: "Microsoft YaHei", size: 20, ...opts })]
  });
}
function bullet(text, ref = "bullets") {
  return new Paragraph({
    numbering: { reference: ref, level: 0 },
    spacing: { after: 80 },
    children: [new TextRun({ text, font: "Microsoft YaHei", size: 20 })]
  });
}

// content width for A4 with 1" margins = 11906 - 2880 = 9026
const CW = 9026;

const doc = new Document({
  styles: {
    default: { document: { run: { font: "Microsoft YaHei", size: 20 } } },
    paragraphStyles: [
      { id: "Heading1", name: "Heading 1", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { size: 32, bold: true, font: "Microsoft YaHei", color: "1F4E79" },
        paragraph: { spacing: { before: 360, after: 200 }, outlineLevel: 0 } },
      { id: "Heading2", name: "Heading 2", basedOn: "Normal", next: "Normal", quickFormat: true,
        run: { size: 26, bold: true, font: "Microsoft YaHei", color: "2E75B6" },
        paragraph: { spacing: { before: 280, after: 160 }, outlineLevel: 1 } },
    ]
  },
  numbering: {
    config: [
      { reference: "bullets",
        levels: [{ level: 0, format: LevelFormat.BULLET, text: "•", alignment: AlignmentType.LEFT,
          style: { paragraph: { indent: { left: 720, hanging: 360 } } } }] },
      { reference: "numbers",
        levels: [{ level: 0, format: LevelFormat.DECIMAL, text: "%1.", alignment: AlignmentType.LEFT,
          style: { paragraph: { indent: { left: 720, hanging: 360 } } } }] },
    ]
  },
  sections: [
    // ── Cover Page ──
    {
      properties: {
        page: {
          size: { width: 11906, height: 16838 },
          margin: { top: 1440, right: 1440, bottom: 1440, left: 1440 }
        }
      },
      children: [
        new Paragraph({ spacing: { before: 3600 } }),
        new Paragraph({
          alignment: AlignmentType.CENTER,
          spacing: { after: 200 },
          children: [new TextRun({ text: "草莓采摘机器人", font: "Microsoft YaHei", size: 52, bold: true, color: "1F4E79" })]
        }),
        new Paragraph({
          alignment: AlignmentType.CENTER,
          spacing: { after: 600 },
          children: [new TextRun({ text: "视觉模块对接交付清单", font: "Microsoft YaHei", size: 44, bold: true, color: "2E75B6" })]
        }),
        new Paragraph({ alignment: AlignmentType.CENTER, spacing: { after: 120 },
          children: [new TextRun({ text: "K210 视觉模组  →  STM32F407 底盘主控", font: "Microsoft YaHei", size: 24, color: "666666" })] }),
        new Paragraph({ spacing: { before: 1200 } }),
        // info table
        new Table({
          width: { size: 5400, type: WidthType.DXA },
          columnWidths: [1800, 3600],
          rows: [
            new TableRow({ children: [
              cell("文档版本", 1800), cell("v1.0", 3600)
            ]}),
            new TableRow({ children: [
              cell("日期", 1800), cell("2026-06-26", 3600)
            ]}),
            new TableRow({ children: [
              cell("底盘负责人", 1800), cell("（你）", 3600)
            ]}),
            new TableRow({ children: [
              cell("视觉负责人", 1800), cell("（视觉队友）", 3600)
            ]}),
            new TableRow({ children: [
              cell("交付截止", 1800), cell("2026-06-28（2 天后）", 3600)
            ]}),
          ]
        }),
        new Paragraph({ spacing: { before: 1200 } }),
        new Paragraph({
          alignment: AlignmentType.CENTER,
          children: [new TextRun({ text: "⚠ 请在交付前逐项核对，确保联调一次成功", font: "Microsoft YaHei", size: 22, color: "CC0000", bold: true })]
        }),
      ]
    },

    // ── Body ──
    {
      properties: {
        page: {
          size: { width: 11906, height: 16838 },
          margin: { top: 1440, right: 1440, bottom: 1440, left: 1440 }
        }
      },
      headers: {
        default: new Header({
          children: [new Paragraph({
            alignment: AlignmentType.RIGHT,
            border: { bottom: { style: BorderStyle.SINGLE, size: 4, color: "CCCCCC", space: 4 } },
            children: [new TextRun({ text: "草莓采摘机器人 — 视觉模块对接交付清单", font: "Microsoft YaHei", size: 16, color: "999999" })]
          })]
        })
      },
      footers: {
        default: new Footer({
          children: [new Paragraph({
            alignment: AlignmentType.CENTER,
            children: [new TextRun({ text: "第 ", font: "Microsoft YaHei", size: 16, color: "999999" }),
                       new TextRun({ children: [PageNumber.CURRENT], font: "Microsoft YaHei", size: 16, color: "999999" }),
                       new TextRun({ text: " 页", font: "Microsoft YaHei", size: 16, color: "999999" })]
          })]
        })
      },
      children: [

// ═══════════════════════════════════════════
// 1. HARDWARE CONNECTION
// ═══════════════════════════════════════════
heading1("一、硬件连接规范"),

heading2("1.1 串口引脚"),
para("K210 与 STM32F407VET6 之间通过 USART2 连接，引脚定义如下："),

new Table({
  width: { size: CW, type: WidthType.DXA },
  columnWidths: [1400, 1400, 1400, 1600, 1600, 1626],
  rows: [
    new TableRow({ children: [
      headerCell("信号", 1400), headerCell("K210 引脚", 1400), headerCell("STM32 引脚", 1400),
      headerCell("GPIO", 1600), headerCell("功能", 1600), headerCell("备注", 1626)
    ]}),
    new TableRow({ children: [
      cell("TX → RX", 1400), cell("TX", 1400), cell("PD6", 1400), cell("USART2_RX", 1600), cell("K210 发送", 1600), cell("主数据通道", 1626)
    ]}),
    new TableRow({ children: [
      cell("RX ← TX", 1400), cell("RX", 1400), cell("PD5", 1400), cell("USART2_TX", 1600), cell("K210 接收", 1600), cell("预留，可发配置", 1626)
    ]}),
    new TableRow({ children: [
      cell("GND", 1400), cell("GND", 1400), cell("GND", 1400), cell("—", 1600), cell("共地", 1600), cell("必须共地！", 1626)
    ]}),
  ]
}),
para(""),

heading2("1.2 电气特性"),
bullet("电平标准：3.3V TTL（K210 与 STM32 均为 3.3V，无需电平转换）"),
bullet("接线方式：杜邦线直连，建议线长 ≤ 20cm"),
bullet("接插件建议：PH2.0 3P 端子（TX / RX / GND），防反插设计"),
para(""),

// ═══════════════════════════════════════════
// 2. COMMUNICATION PROTOCOL
// ═══════════════════════════════════════════
heading1("二、通信协议规范（重要！）"),

heading2("2.1 串口参数"),
new Table({
  width: { size: CW, type: WidthType.DXA },
  columnWidths: [2256, 2256, 2256, 2258],
  rows: [
    new TableRow({ children: [
      headerCell("参数", 2256), headerCell("值", 2256), headerCell("参数", 2256), headerCell("值", 2258)
    ]}),
    new TableRow({ children: [
      cell("波特率", 2256), cell("115200", 2256), cell("数据位", 2256), cell("8", 2258)
    ]}),
    new TableRow({ children: [
      cell("停止位", 2256), cell("1", 2256), cell("校验位", 2256), cell("无 (None)", 2258)
    ]}),
    new TableRow({ children: [
      cell("流控", 2256), cell("无", 2256), cell("帧间隔", 2256), cell("≥ 1ms（推荐 10ms）", 2258)
    ]}),
  ]
}),
para(""),

heading2("2.2 帧格式（定长 24 字节）"),
para("每帧固定 24 字节，STM32 端使用 DMA + IDLE 中断接收。帧结构如下："),

new Table({
  width: { size: CW, type: WidthType.DXA },
  columnWidths: [900, 1300, 1200, 5626],
  rows: [
    new TableRow({ children: [
      headerCell("字节", 900), headerCell("字段", 1300), headerCell("类型", 1200), headerCell("说明", 5626)
    ]}),
    new TableRow({ children: [
      monoCell("0", 900), monoCell("Header", 1300), monoCell("uint8", 1200),
      cell("帧头，固定值 0xAA", 5626)
    ]}),
    new TableRow({ children: [
      monoCell("1", 900), monoCell("Count", 1300), monoCell("uint8", 1200),
      cell("有效草莓目标数，范围 0~5。0 表示当前画面无草莓", 5626)
    ]}),
    new TableRow({ children: [
      monoCell("2~5", 900), monoCell("Target[0]", 1300), monoCell("4×uint8", 1200),
      cell("第 1 个目标：cls(1B) + x(1B) + y(1B) + area(1B)", 5626)
    ]}),
    new TableRow({ children: [
      monoCell("6~9", 900), monoCell("Target[1]", 1300), monoCell("4×uint8", 1200),
      cell("第 2 个目标（格式同上）", 5626)
    ]}),
    new TableRow({ children: [
      monoCell("10~13", 900), monoCell("Target[2]", 1300), monoCell("4×uint8", 1200),
      cell("第 3 个目标（格式同上）", 5626)
    ]}),
    new TableRow({ children: [
      monoCell("14~17", 900), monoCell("Target[3]", 1300), monoCell("4×uint8", 1200),
      cell("第 4 个目标（格式同上）", 5626)
    ]}),
    new TableRow({ children: [
      monoCell("18~21", 900), monoCell("Target[4]", 1300), monoCell("4×uint8", 1200),
      cell("第 5 个目标（格式同上）。无效槽位须填 0x00", 5626)
    ]}),
    new TableRow({ children: [
      monoCell("22", 900), monoCell("Checksum", 1300), monoCell("uint8", 1200),
      cell("XOR 校验：Byte1 ^ Byte2 ^ ... ^ Byte21。STM32 校验失败会丢弃该帧", 5626)
    ]}),
    new TableRow({ children: [
      monoCell("23", 900), monoCell("Tail", 1300), monoCell("uint8", 1200),
      cell("帧尾，固定值 0x55", 5626)
    ]}),
  ]
}),
para(""),

heading2("2.3 Target 字段详解"),
para("每个草莓目标占用 4 字节，各字段含义："),

new Table({
  width: { size: CW, type: WidthType.DXA },
  columnWidths: [900, 1300, 1600, 5226],
  rows: [
    new TableRow({ children: [
      headerCell("字段", 900), headerCell("类型", 1300), headerCell("范围", 1600), headerCell("说明", 5226)
    ]}),
    new TableRow({ children: [
      monoCell("cls", 900), monoCell("uint8", 1300), monoCell("0 或 1", 1600),
      cell("草莓类别 — 0 = 绿色（未熟，不摘），1 = 红色（成熟，需采摘）", 5226)
    ]}),
    new TableRow({ children: [
      monoCell("x", 900), monoCell("uint8", 1300), monoCell("0 ~ 255", 1600),
      cell("草莓中心水平坐标。K210 原始图像 320×240，建议压缩映射到 0~255。0=最左，255=最右；画面中心 = 128", 5226)
    ]}),
    new TableRow({ children: [
      monoCell("y", 900), monoCell("uint8", 1300), monoCell("0 ~ 240", 1600),
      cell("草莓中心垂直坐标。原始分辨率无需压缩。0=顶部，240=底部。数值越大越靠近小车", 5226)
    ]}),
    new TableRow({ children: [
      monoCell("area", 900), monoCell("uint8", 1300), monoCell("0 ~ 100", 1600),
      cell("草莓在画面中的面积百分比（%）。用于判断接近程度：area ≥ 20 认为小车已抵达采摘位", 5226)
    ]}),
  ]
}),
para(""),

heading2("2.4 校验规则（必须实现）"),
bullet("XOR 校验范围：Byte 1（Count）到 Byte 21（Target[4].area），共 21 字节"),
bullet("帧头 0xAA（Byte 0）和帧尾 0x55（Byte 23）不参与 XOR 计算"),
bullet("校验失败 → STM32 静默丢弃该帧，K210 不需要重传（下一帧自动覆盖）"),
bullet("示例伪代码：x = 0; for i in 1..21: x ^= frame[i];  // x 应等于 frame[22]"),
para(""),

heading2("2.5 发送频率与时机"),
bullet("推荐发送频率：50~100 Hz（每 10~20ms 一帧），与 STM32 PID 控制周期（10ms）匹配"),
bullet("发送策略：持续发送，无需等待 STM32 应答（单向流式传输）"),
bullet("无草莓时：发送 Count=0 的空帧（5 个 Target 槽位全部填 0），不可停止发送"),
bullet("DMA 注意事项：帧间隔建议 ≥ 1ms，避免两帧粘连导致 IDLE 中断误判帧长"),
para(""),

// ═══════════════════════════════════════════
// 3. STATE MACHINE INTEGRATION
// ═══════════════════════════════════════════
heading1("三、视觉相关状态机流程"),

para("STM32 状态机中与 K210 视觉相关的状态共 5 个。视觉同学需要理解这些状态的触发条件和对 K210 数据的期望："),

heading2("3.1 状态流转图"),
new Table({
  width: { size: CW, type: WidthType.DXA },
  columnWidths: [1600, 1600, 2200, 3626],
  rows: [
    new TableRow({ children: [
      headerCell("状态", 1600), headerCell("进入条件", 1600), headerCell("K210 数据用途", 2200), headerCell("退出条件", 3626)
    ]}),
    new TableRow({ children: [
      cell("LINE_FOLLOW\n（巡线）", 1600),
      cell("默认行驶状态", 1600),
      cell("监控草莓位置，判断是否触发脱离", 2200),
      cell("红外检测到左侧或右侧有植株 → BREAKAWAY", 3626)
    ]}),
    new TableRow({ children: [
      cell("BREAKAWAY\n（脱离）", 1600),
      cell("LINE_FOLLOW 中检测到植株", 1600),
      cell("视觉伺服转向：将草莓 X 坐标对准画面中心", 2200),
      cell("X 对中 → VISUAL_APPROACH；1s 无目标 → BLIND_RETURN", 3626)
    ]}),
    new TableRow({ children: [
      cell("VISUAL_APPROACH\n（接近）", 1600),
      cell("BREAKAWAY 对中完成", 1600),
      cell("① X 伺服保持对中；② 用 Y+area 控制前进减速直至停止", 2200),
      cell("area≥20 且 X 对中稳定 10 帧 → SCANNING；>200ms 无目标 → BLIND_RETURN", 3626)
    ]}),
    new TableRow({ children: [
      cell("SCANNING\n（扫描）", 1600),
      cell("VISUAL_APPROACH 抵达采摘位", 1600),
      cell("静止采集 50 帧，统计红/绿草莓数量（cls=0/1 计数）", 2200),
      cell("采集满 50 帧 → DOCKING", 3626)
    ]}),
    new TableRow({ children: [
      cell("DOCKING\n（对接）", 1600),
      cell("SCANNING 完成", 1600),
      cell("蜂鸣器上报，等待采摘完成确认", 2200),
      cell("延时结束 → BLIND_RETURN", 3626)
    ]}),
    new TableRow({ children: [
      cell("BLIND_RETURN\n（盲返）", 1600),
      cell("DOCKING 完成 / 视觉丢失目标超时", 1600),
      cell("无需 K210 数据，依靠编码器里程计倒退回赛道并找线", 2200),
      cell("红外重新检测到黑线 → LINE_FOLLOW", 3626)
    ]}),
  ]
}),
para(""),

heading2("3.2 状态机对视觉数据的依赖"),
para("⚠ 以下 4 个状态需要 K210 提供数据，其余状态不读取视觉帧："),
bullet("LINE_FOLLOW：仅在判断植株位置时参考 K210 目标位置（X 坐标），辅助红外触发"),
bullet("BREAKAWAY：核心依赖 K210 实时 X 坐标进行视觉伺服转向"),
bullet("VISUAL_APPROACH：核心依赖 K210 实时 X/Y/area 进行双轴伺服靠近"),
bullet("SCANNING：核心依赖 K210 返回的 cls 字段统计红绿草莓数"),
bullet("其他状态（DEBUG/IDLE/OBSTACLE/GREEN_FILTER/BLIND_RETURN/FINISH）不读取 K210 数据"),
para(""),

// ═══════════════════════════════════════════
// 4. PARAMETERS
// ═══════════════════════════════════════════
heading1("四、对接参数速查表"),

heading2("4.1 触发阈值（K210 不需要修改，仅需了解）"),
para("以下参数定义在 STM32 chassis.h 中，视觉同学需要知道 STM32 端如何解读 K210 数据："),

new Table({
  width: { size: CW, type: WidthType.DXA },
  columnWidths: [2800, 1200, 2200, 2826],
  rows: [
    new TableRow({ children: [
      headerCell("参数", 2800), headerCell("值", 1200), headerCell("含义", 2200), headerCell("说明", 2826)
    ]}),
    new TableRow({ children: [
      monoCell("PLANT_X_LEFT", 2800), monoCell("65", 1200),
      cell("植株左侧触发阈值", 2200), cell("x < 65 → 植株在左侧，触发脱离", 2826)
    ]}),
    new TableRow({ children: [
      monoCell("PLANT_X_RIGHT", 2800), monoCell("190", 1200),
      cell("植株右侧触发阈值", 2200), cell("x > 190 → 植株在右侧，触发脱离", 2826)
    ]}),
    new TableRow({ children: [
      monoCell("TRIGGER_CONFIRM", 2800), monoCell("3", 1200),
      cell("连续确认帧数", 2200), cell("连续 3 帧满足条件才触发，防抖动", 2826)
    ]}),
    new TableRow({ children: [
      monoCell("X_CENTER_PX", 2800), monoCell("128", 1200),
      cell("画面水平中心", 2200), cell("K210 输出 x∈[0,255]，中心=128。STM32 已适配该映射", 2826)
    ]}),
    new TableRow({ children: [
      monoCell("X_DEADBAND_PX", 2800), monoCell("15", 1200),
      cell("X 对中死区", 2200), cell("|x - center| < 15 认为已对中，停止转向", 2826)
    ]}),
    new TableRow({ children: [
      monoCell("APPROACH_AREA_STOP", 2800), monoCell("20", 1200),
      cell("接近停止面积", 2200), cell("area ≥ 20% 认为已抵达采摘距离，可停车", 2826)
    ]}),
    new TableRow({ children: [
      monoCell("APPROACH_CENTER_OK", 2800), monoCell("10", 1200),
      cell("对中稳定帧数", 2200), cell("连续 10 帧 X 对中 + area 达标 → 进入扫描", 2826)
    ]}),
    new TableRow({ children: [
      monoCell("SCAN_FRAMES", 2800), monoCell("50", 1200),
      cell("扫描采样帧数", 2200), cell("静止采集 50 帧（500ms @100Hz），统计红绿草莓", 2826)
    ]}),
  ]
}),
para(""),

heading2("4.2 X 坐标映射规范（强制要求）"),
para("为避免 uint8_t 溢出和双方坐标系不一致，统一规定如下：", { bold: true, color: "CC0000" }),
bullet("K210 侧：必须将原始 X 坐标（0~319）线性映射到 0~255 后发送"),
bullet("映射公式：x = (rawX * 255 + 159) / 319  或  x = rawX * 255 / 319（整数运算均可）"),
bullet("STM32 侧：已同步修改 X_CENTER_PX=128、PLANT_X_LEFT=65、PLANT_X_RIGHT=190"),
bullet("画面中心 = 128，左侧触发 = <65，右侧触发 = >190"),
bullet("不得直接发送 0~319 原始坐标，否则 x 字段溢出（uint8_t 上限 255）导致数据错误"),
para(""),

// ═══════════════════════════════════════════
// 5. DELIVERABLES
// ═══════════════════════════════════════════
heading1("五、交付物清单"),

new Table({
  width: { size: CW, type: WidthType.DXA },
  columnWidths: [600, 3000, 1600, 1600, 2226],
  rows: [
    new TableRow({ children: [
      headerCell("序号", 600), headerCell("交付物", 3000), headerCell("数量", 1600), headerCell("验收标准", 1600), headerCell("备注", 2226)
    ]}),
    new TableRow({ children: [
      cell("1", 600, { alignment: AlignmentType.CENTER }),
      cell("K210 开发板（含摄像头）", 3000),
      cell("1 套", 1600),
      cell("上电正常运行，摄像头出图", 1600),
      cell("型号：Sipeed MAIX Bit 或同等，OV2640/OV5640 摄像头", 2226)
    ]}),
    new TableRow({ children: [
      cell("2", 600, { alignment: AlignmentType.CENTER }),
      cell("USART 通信线（3P 杜邦线）", 3000),
      cell("1 组", 1600),
      cell("TX/RX/GND，长度 ≥ 15cm", 1600),
      cell("建议 PH2.0 3P 端子，颜色区分", 2226)
    ]}),
    new TableRow({ children: [
      cell("3", 600, { alignment: AlignmentType.CENTER }),
      cell("K210 固件（.bin / .kfpkg）", 3000),
      cell("1 份", 1600),
      cell("烧录后可正常运行目标检测 + 串口发送", 1600),
      cell("附烧录工具和步骤说明", 2226)
    ]}),
    new TableRow({ children: [
      cell("4", 600, { alignment: AlignmentType.CENTER }),
      cell("目标检测模型文件（.kmodel）", 3000),
      cell("1 份", 1600),
      cell("能识别红/绿草莓并输出 cls/x/y/w/h", 1600),
      cell("如内嵌在固件中则无需单独交付", 2226)
    ]}),
    new TableRow({ children: [
      cell("5", 600, { alignment: AlignmentType.CENTER }),
      cell("通信协议实现代码", 3000),
      cell("1 份", 1600),
      cell("严格按照本文档 24B 帧格式组帧并发送", 1600),
      cell("包含 XOR 校验和帧头帧尾", 2226)
    ]}),
    new TableRow({ children: [
      cell("6", 600, { alignment: AlignmentType.CENTER }),
      cell("K210 源码", 3000),
      cell("1 份", 1600),
      cell("可编译、可修改", 1600),
      cell("Git 仓库或压缩包，含 README", 2226)
    ]}),
    new TableRow({ children: [
      cell("7", 600, { alignment: AlignmentType.CENTER }),
      cell("引脚定义文档", 3000),
      cell("1 份", 1600),
      cell("明确标注 TX/RX/GND 对应 K210 物理引脚编号", 1600),
      cell("拍照标注更直观", 2226)
    ]}),
    new TableRow({ children: [
      cell("8", 600, { alignment: AlignmentType.CENTER }),
      cell("供电方案", 3000),
      cell("1 份", 1600),
      cell("说明供电电压/电流、供电方式", 1600),
      cell("独立电池 or 从小车取电？需确认", 2226)
    ]}),
  ]
}),
para(""),

// ═══════════════════════════════════════════
// 6. ACCEPTANCE TEST
// ═══════════════════════════════════════════
heading1("六、验收测试用例"),

heading2("6.1 通信链路测试"),
para("目标：验证 K210 → STM32 串口通信链路正常"),

new Table({
  width: { size: CW, type: WidthType.DXA },
  columnWidths: [600, 3200, 2200, 3026],
  rows: [
    new TableRow({ children: [
      headerCell("步骤", 600), headerCell("操作", 3200), headerCell("预期结果", 2200), headerCell("验证方法", 3026)
    ]}),
    new TableRow({ children: [
      cell("1", 600, { alignment: AlignmentType.CENTER }),
      cell("K210 上电，运行目标检测固件", 3200),
      cell("K210 正常启动，摄像头出图", 2200),
      cell("观察 K210 屏幕或串口调试输出", 3026)
    ]}),
    new TableRow({ children: [
      cell("2", 600, { alignment: AlignmentType.CENTER }),
      cell("连接 K210 TX → STM32 PD6 (USART2_RX)", 3200),
      cell("物理连接可靠", 2200),
      cell("万用表测通断", 3026)
    ]}),
    new TableRow({ children: [
      cell("3", 600, { alignment: AlignmentType.CENTER }),
      cell("STM32 上电，VOFA+ 打开 USART1 查看日志", 3200),
      cell("STM32 收到 K210 帧（VOFA+ 日志可见帧数据）", 2200),
      cell("暂时可在 freertos.c 中加 printf 打印帧头", 3026)
    ]}),
  ]
}),
para(""),

heading2("6.2 帧格式验收测试"),
para("目标：验证每帧 24 字节格式完全符合协议规范"),

new Table({
  width: { size: CW, type: WidthType.DXA },
  columnWidths: [600, 3200, 2200, 3026],
  rows: [
    new TableRow({ children: [
      headerCell("用例", 600), headerCell("测试内容", 3200), headerCell("验收标准", 2200), headerCell("方法", 3026)
    ]}),
    new TableRow({ children: [
      cell("1", 600, { alignment: AlignmentType.CENTER }),
      cell("帧头帧尾：Byte0=0xAA, Byte23=0x55", 3200),
      cell("逻辑分析仪 / USB 转串口抓取，确认帧头帧尾正确", 2200),
      cell("PC 端串口助手 Hex 显示模式", 3026)
    ]}),
    new TableRow({ children: [
      cell("2", 600, { alignment: AlignmentType.CENTER }),
      cell("帧长固定 24 字节", 3200),
      cell("每帧严格 24 字节，无多余字节，无缺字节", 2200),
      cell("串口助手统计接收字节数", 3026)
    ]}),
    new TableRow({ children: [
      cell("3", 600, { alignment: AlignmentType.CENTER }),
      cell("XOR 校验正确", 3200),
      cell("校验值 = Byte1~Byte21 的 XOR 结果", 2200),
      cell("手动计算校验值对比", 3026)
    ]}),
    new TableRow({ children: [
      cell("4", 600, { alignment: AlignmentType.CENTER }),
      cell("无草莓场景：Count=0", 3200),
      cell("Count=0，5 个 Target 槽位全 0，持续发送", 2200),
      cell("遮挡摄像头模拟无目标场景", 3026)
    ]}),
    new TableRow({ children: [
      cell("5", 600, { alignment: AlignmentType.CENTER }),
      cell("单草莓场景：Count=1", 3200),
      cell("Count=1，Target[0] 填充有效数据，Target[1~4] 全 0", 2200),
      cell("放一个草莓模型在摄像头前", 3026)
    ]}),
    new TableRow({ children: [
      cell("6", 600, { alignment: AlignmentType.CENTER }),
      cell("多草莓场景：Count=2~5", 3200),
      cell("Count 等于实际检测数，对应 Target 槽位有效，其余全 0", 2200),
      cell("放多个草莓模型", 3026)
    ]}),
    new TableRow({ children: [
      cell("7", 600, { alignment: AlignmentType.CENTER }),
      cell("cls 字段：红色=1，绿色=0", 3200),
      cell("红色草莓模型 → cls=1，绿色 → cls=0", 2200),
      cell("分别测试红/绿模型", 3026)
    ]}),
    new TableRow({ children: [
      cell("8", 600, { alignment: AlignmentType.CENTER }),
      cell("X 坐标范围 0~255", 3200),
      cell("草莓在最左 → x≈0，最右 → x≈255", 2200),
      cell("移动草莓从画面左边缘到右边缘", 3026)
    ]}),
    new TableRow({ children: [
      cell("9", 600, { alignment: AlignmentType.CENTER }),
      cell("area 随距离变化", 3200),
      cell("草莓越近 area 越大，0~100 范围", 2200),
      cell("从远到近移动草莓，记录 area 变化", 3026)
    ]}),
  ]
}),
para(""),

heading2("6.3 系统联调测试"),
para("目标：K210 + STM32 完整系统联合调试"),

new Table({
  width: { size: CW, type: WidthType.DXA },
  columnWidths: [600, 3200, 2200, 3026],
  rows: [
    new TableRow({ children: [
      headerCell("用例", 600), headerCell("测试内容", 3200), headerCell("验收标准", 2200), headerCell("方法", 3026)
    ]}),
    new TableRow({ children: [
      cell("1", 600, { alignment: AlignmentType.CENTER }),
      cell("视觉触发脱离：草莓模型放在巡线小车左侧", 3200),
      cell("STM32 状态从 LINE_FOLLOW → BREAKAWAY", 2200),
      cell("模拟赛道场景，观察 OLED 状态显示", 3026)
    ]}),
    new TableRow({ children: [
      cell("2", 600, { alignment: AlignmentType.CENTER }),
      cell("视觉伺服对中：BREAKAWAY 中小车转向对准草莓", 3200),
      cell("小车转动使草莓 X 坐标趋近画面中心", 2200),
      cell("观察 OLED + VOFA+ 实时数据", 3026)
    ]}),
    new TableRow({ children: [
      cell("3", 600, { alignment: AlignmentType.CENTER }),
      cell("接近停车：小车前进至 area≥20 自动停止", 3200),
      cell("VISUAL_APPROACH → SCANNING 状态切换", 2200),
      cell("小车从远处驶向草莓，验证自动停车", 3026)
    ]}),
    new TableRow({ children: [
      cell("4", 600, { alignment: AlignmentType.CENTER }),
      cell("扫描统计：SCANNING 中正确计数红绿草莓", 3200),
      cell("OLED 显示的红/绿计数值与实际一致", 2200),
      cell("分别测试只有红草莓、只有绿草莓、混合场景", 3026)
    ]}),
    new TableRow({ children: [
      cell("5", 600, { alignment: AlignmentType.CENTER }),
      cell("视觉丢失超时保护：遮挡摄像头", 3200),
      cell("1s 内无目标 → 自动转入 BLIND_RETURN", 2200),
      cell("BREAKAWAY 或 VISUAL_APPROACH 中突然遮挡摄像头", 3026)
    ]}),
    new TableRow({ children: [
      cell("6", 600, { alignment: AlignmentType.CENTER }),
      cell("盲返找线：BLIND_RETURN 后重新检测到黑线", 3200),
      cell("BLIND_RETURN → LINE_FOLLOW，恢复正常巡线", 2200),
      cell("观察完整采摘→返回→巡线循环", 3026)
    ]}),
    new TableRow({ children: [
      cell("7", 600, { alignment: AlignmentType.CENTER }),
      cell("连续多株采摘：赛道放置 3 株以上草莓", 3200),
      cell("完成采摘→返回→巡线→采摘循环，无卡死", 2200),
      cell("完整赛道实测，记录每株采摘时间", 3026)
    ]}),
  ]
}),
para(""),

// ═══════════════════════════════════════════
// 7. IMPORTANT NOTES
// ═══════════════════════════════════════════
heading1("七、重要注意事项"),

heading2("7.1 通信层面"),
bullet("共地必须：K210 与 STM32 的 GND 必须连接，否则串口电平无法正确识别"),
bullet("不要发不定长帧：24 字节定长是硬性要求。不定长会导致 DMA+IDLE 接收异常"),
bullet("无效槽位填零：Count=1 时，Target[1]~[4] 的 16 个字节必须全部填 0x00，不能留随机值"),
bullet("不要停发：即使画面无草莓，也要持续发送 Count=0 的空帧。STM32 端依赖帧率判断 K210 是否在线"),
bullet("波特率不要改：115200 是双方代码中写死的值，修改需同步改 STM32 usart.c"),

heading2("7.2 坐标系统"),
bullet("坐标系确认：K210 画面 X 轴从左到右递增，Y 轴从上到下递增。与 STM32 端理解一致"),
bullet("X 范围对齐：K210 输出 X 坐标必须映射到 0~255（公式见 4.2 节），不得发送 0~319 原始坐标"),
bullet("X 方向确认：草莓在摄像头左侧 → x 小（< 65 触发左转），右侧 → x 大（> 190 触发右转）"),
bullet("Y 方向确认：草莓越远 → y 越小（靠近画面顶部），越近 → y 越大（靠近画面底部）"),

heading2("7.3 交付前自查"),
bullet("K210 独立上电，摄像头出图正常"),
bullet("用 USB 转串口模块抓取 K210 TX 引脚输出，确认帧格式正确（推荐逻辑分析仪或串口助手 Hex 显示）"),
bullet("准备至少 2 个草莓模型（红色 + 绿色），用于联调测试"),
bullet("K210 源码和固件备份到团队 Git 仓库"),
bullet("引脚接线照片 + 标注，发给底盘负责人"),
bullet("供电方案确认：K210 独立供电 or 从小车电池取电？如从小车取电需确认电压范围和接口"),

heading2("7.4 交付时现场联调流程（建议）"),
para("建议交付当天按以下流程进行，预计 1~2 小时："),
bullet("第 1 步（15min）：硬件连接 — 接好 TX/RX/GND，万用表确认通断，上电确认 K210 和 STM32 均正常工作"),
bullet("第 2 步（15min）：通信验证 — PC 端串口助手抓取 K210 帧，确认 24B 格式、校验、帧率正确"),
bullet("第 3 步（30min）：STM32 联调 — 连接 K210 → STM32，VOFA+ 观察视觉帧解析结果，逐状态测试"),
bullet("第 4 步（30min）：完整流程测试 — 模拟赛道 + 草莓，跑通 LINE_FOLLOW → BREAKAWAY → APPROACH → SCANNING → DOCKING → RETURN 全流程"),
bullet("第 5 步（15min）：问题记录 — 记录所有异常情况和参数调整，更新本文档到 v1.1"),
para(""),

// ═══════════════════════════════════════════
// 8. CONTACT & SIGN-OFF
// ═══════════════════════════════════════════
heading1("八、签收确认"),

new Table({
  width: { size: CW, type: WidthType.DXA },
  columnWidths: [2256, 2256, 2256, 2258],
  rows: [
    new TableRow({ children: [
      headerCell("角色", 2256), headerCell("姓名", 2256), headerCell("日期", 2256), headerCell("签字", 2258)
    ]}),
    new TableRow({ children: [
      cell("交付方（视觉）", 2256), cell("", 2256), cell("", 2256), cell("", 2258)
    ]}),
    new TableRow({ children: [
      cell("接收方（底盘）", 2256), cell("", 2256), cell("", 2256), cell("", 2258)
    ]}),
  ]
}),
para(""),
para("以上所有项目验收通过后，双方签字确认，视觉模块正式交付底盘团队集成。", { italics: true, color: "666666" }),
para("交付后如需修改协议或参数，请同步更新本文档并通知对方。", { italics: true, color: "666666" }),

      ] // end children
    }   // end body section
  ]     // end sections
});

// ── generate ──
const outPath = "F:/New_Folder/Strawberry_car/docs/视觉模块对接交付清单.docx";
Packer.toBuffer(doc).then(buffer => {
  fs.writeFileSync(outPath, buffer);
  console.log("✅ document saved: " + outPath);
});
