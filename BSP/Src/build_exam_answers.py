#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
生成现代通信网期末习题答案 DOCX
三、填空 - 答案在对应位置用下划线标出
四、术语释义 - 跳过不做
五、简答 - 给出答案
"""

from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml.ns import qn
from docx.oxml import OxmlElement
import copy

doc = Document()

# 设置默认字体
style = doc.styles['Normal']
font = style.font
font.name = '宋体'
font.size = Pt(11)
style.element.rPr.rFonts.set(qn('w:eastAsia'), '宋体')

# ====== 辅助函数 ======
def add_heading_styled(text, level=1):
    h = doc.add_heading(text, level=level)
    for run in h.runs:
        run.font.name = '宋体'
        run._element.rPr.rFonts.set(qn('w:eastAsia'), '宋体')
    return h

def add_run_with_font(paragraph, text, bold=False, underline=False, size=None, color=None):
    """添加一个带格式的run"""
    run = paragraph.add_run(text)
    run.font.name = '宋体'
    run._element.rPr.rFonts.set(qn('w:eastAsia'), '宋体')
    if bold:
        run.bold = True
    if underline:
        run.underline = True
    if size:
        run.font.size = size
    if color:
        run.font.color.rgb = color
    return run

def fill_blank_question(doc, num, question_parts):
    """
    添加一道填空题，question_parts 是一个列表，
    每项是 (text, is_answer) 的元组，is_answer=True 的文字会加下划线
    """
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(6)
    p.paragraph_format.space_before = Pt(3)

    # 题号
    add_run_with_font(p, f"{num}. ", bold=True)

    for text, is_answer in question_parts:
        add_run_with_font(p, text, underline=is_answer)

    return p

def add_simple_answer(doc, num, answer_text):
    """添加简单的回答段落"""
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(4)
    add_run_with_font(p, f"{num}. ", bold=True)
    add_run_with_font(p, answer_text)
    return p

# ====================================================
#                     文档正文开始
# ====================================================

title = doc.add_heading('现代通信网期末习题 答案', level=0)
for run in title.runs:
    run.font.name = '宋体'
    run._element.rPr.rFonts.set(qn('w:eastAsia'), '宋体')

doc.add_paragraph()

# ====================================================
#  三、填空
# ====================================================
add_heading_styled('三、填空', level=1)

# 1
fill_blank_question(doc, 1, [
    ("通信网的基本构成要素(三要素)是: ", False),
    ("终端节点", True),
    ("、", False),
    ("交换节点", True),
    ("和", False),
    ("传输系统", True),
    ("。", False),
])

# 2
fill_blank_question(doc, 2, [
    ("现代通信网是将一定数量的", False),
    ("节点", True),
    ("和连接这些节点的", False),
    ("传输系统", True),
    ("有机地组织在一起,按约定的", False),
    ("协议", True),
    ("或", False),
    ("信令", True),
    ("完成任意用户间信息交换的通信体系。", False),
])

# 3
fill_blank_question(doc, 3, [
    ("通信网的垂直划分是从功能的角度看,一个完整的现代通信网可以分为: ", False),
    ("业务网", True),
    ("、", False),
    ("传送网", True),
    ("和", False),
    ("支撑网", True),
    ("。", False),
])

# 4
fill_blank_question(doc, 4, [
    ("通信网的水平划分是从地域的角度看,可分为", False),
    ("广域网(WAN)", True),
    ("、", False),
    ("城域网(MAN)", True),
    ("和", False),
    ("局域网(LAN)", True),
    ("。", False),
])

# 5
fill_blank_question(doc, 5, [
    ("面向连接型的网络中,两个通信节点间典型的一次数据交换过程包含三个阶段: ", False),
    ("连接建立", True),
    ("、", False),
    ("数据传输", True),
    ("和", False),
    ("连接释放", True),
    ("。", False),
])

# 6
fill_blank_question(doc, 6, [
    ("通信网的发展趋势是全IP化、", False),
    ("宽带化", True),
    ("和", False),
    ("智能化", True),
    ("。", False),
])

# 7
fill_blank_question(doc, 7, [
    ("被ISO称为开放系统互连OSI的参考模型共有7层,依次是: ", False),
    ("物理层", True),
    ("、", False),
    ("数据链路层", True),
    ("、", False),
    ("网络层", True),
    ("、", False),
    ("传输层", True),
    ("、", False),
    ("会话层", True),
    ("、", False),
    ("表示层", True),
    ("、", False),
    ("应用层", True),
    ("。", False),
])

# 8
fill_blank_question(doc, 8, [
    ("TCP/IP 体系结构的模型共有五层,分别是: ", False),
    ("应用层", True),
    ("、", False),
    ("传输层", True),
    ("、", False),
    ("IP层(网络层)", True),
    ("、", False),
    ("数据链路层", True),
    ("、", False),
    ("物理层", True),
    ("。", False),
])

# 9 (已答)
fill_blank_question(doc, 9, [
    ("根据信道中传输信号的不同,通信系统可分为", False),
    ("模拟通信系统", True),
    ("和", False),
    ("数字通信系统", True),
    ("。", False),
])

# 10 (已答)
fill_blank_question(doc, 10, [
    ("将数字基带信号直接送到信道传输的方式称为", False),
    ("基带传输", True),
    (",将数字信号经过调制后送到信道传输方式称为", False),
    ("频带传输", True),
    ("。", False),
])

# 11
fill_blank_question(doc, 11, [
    ("支撑网包括", False),
    ("信令网", True),
    ("、", False),
    ("同步网", True),
    ("和", False),
    ("管理网", True),
    ("。", False),
])

# 12 (已答)
fill_blank_question(doc, 12, [
    ("数据通信用户设备按功能可分成: ", False),
    ("数据终端设备(DTE)", True),
    ("和", False),
    ("数据电路终接设备(DCE)", True),
    ("。", False),
])

# 13 (已答)
fill_blank_question(doc, 13, [
    ("FTTC是指", False),
    ("光纤到路边", True),
    ("、FTTB是指", False),
    ("光纤到大楼", True),
    ("、FTTO是指", False),
    ("光纤到办公室", True),
    ("、FTTH是指", False),
    ("光纤到住户", True),
    ("。", False),
])

# 14 (已答)
fill_blank_question(doc, 14, [
    ("面向电路型的传送网,常用的传输类型有", False),
    ("PDH", True),
    ("、", False),
    ("SDH", True),
    ("和", False),
    ("OTN", True),
    ("。", False),
])

# 15 (已答)
fill_blank_question(doc, 15, [
    ("在星型网的拓扑结构中,N个节点完全互连至少需要", False),
    ("N-1", True),
    ("条传输线路。", False),
])

# 16
fill_blank_question(doc, 16, [
    ("数据通信的工作方式,按照其方向性可以分成三种,分别是: ", False),
    ("单工", True),
    ("、", False),
    ("半双工", True),
    ("和", False),
    ("全双工", True),
    ("。", False),
])

# 17 (已答)
fill_blank_question(doc, 17, [
    ("话音信号的主要能量集中在", False),
    ("300", True),
    ("和", False),
    ("3400", True),
    ("Hz范围,因此话音信号的有效带宽是", False),
    ("3100", True),
    ("Hz。", False),
])

# 18
fill_blank_question(doc, 18, [
    ("香农公式表明,信道的", False),
    ("带宽", True),
    ("越大,信道的极限传输速率", False),
    ("越高", True),
    ("。", False),
])

# 19
fill_blank_question(doc, 19, [
    ("双绞线带宽一般在", False),
    ("1MHz", True),
    ("范围之内,传输距离约为", False),
    ("2~4km", True),
    ("。", False),
])

# 20
fill_blank_question(doc, 20, [
    ("按照信号在传输介质上的复用方式不同,传输系统可分为: ", False),
    ("频分复用(FDM)", True),
    ("、", False),
    ("时分复用(TDM)", True),
    ("、", False),
    ("波分复用(WDM)", True),
    ("和", False),
    ("码分复用(CDM)", True),
    ("四类传输系统。", False),
])

# 21 (已答)
fill_blank_question(doc, 21, [
    ("光纤系统的工作频率分布在", False),
    ("10¹⁴~10¹⁵Hz", True),
    ("范围内,其工作波长为", False),
    ("850nm", True),
    ("、", False),
    ("1310nm", True),
    ("和", False),
    ("1550nm", True),
    ("三种。", False),
])

# 22 (已答)
fill_blank_question(doc, 22, [
    ("我国现阶段长距离通信采用的光纤为", False),
    ("单模光纤", True),
    (",工作波长采用", False),
    ("1550nm", True),
    ("。", False),
])

# 23
fill_blank_question(doc, 23, [
    ("ITU-T标准电话通信的FDM体制策略是:每路话音信号占带宽", False),
    ("4kHz", True),
    ("，", False),
    ("12", True),
    ("路话音信号构成一个群,每路占用带宽中的一个", False),
    ("4kHz", True),
    ("子信道。", False),
])

# 24 (已答)
fill_blank_question(doc, 24, [
    ("光信号在光纤中传输,每公里正常衰减约为", False),
    ("0.2dB", True),
    ("。采用", False),
    ("掺铒光纤放大器(EDFA)", True),
    (",可以将输入信号在1550nm窗口增益", False),
    ("+25dB", True),
    ("以上。", False),
])

# 25 (已答)
fill_blank_question(doc, 25, [
    ("G.652光纤是零色散波长在", False),
    ("1310nm", True),
    ("的单模光纤;数字光纤通信系统光端机框图组成包括: ", False),
    ("光发射机", True),
    ("和", False),
    ("光接收机", True),
    ("两部分。", False),
])

# 26 (已答)
fill_blank_question(doc, 26, [
    ("目前实用的光纤有三个低损耗窗口,其中", False),
    ("0.85μm", True),
    ("处损耗最大,", False),
    ("1.55μm", True),
    ("处损耗最小。", False),
])

# 27 (已答)
fill_blank_question(doc, 27, [
    ("阻抗为50欧姆的同轴细缆网线,单网段最大传输距离限制为", False),
    ("185m", True),
    (",无屏蔽双绞网线的最大传输距离为", False),
    ("100m", True),
    ("。", False),
])

# 28
fill_blank_question(doc, 28, [
    ("PDH系统中,ITU-T标准的PCM30/32系统,每帧", False),
    ("32", True),
    ("时隙,每帧时长为", False),
    ("125μs", True),
    (",一次群的传输速率为", False),
    ("2.048Mbps", True),
    ("。", False),
])

# 29 (已答)
fill_blank_question(doc, 29, [
    ("PCM系统中,TSO时隙是用来传送", False),
    ("帧同步码", True),
    ("，TS16是用来传送", False),
    ("信令", True),
    ("的,帧定位信号是在", False),
    ("偶帧", True),
    ("的TSO时隙传送。", False),
])

# 30 (已答)
fill_blank_question(doc, 30, [
    ("在SDH中,STM-1的速率为", False),
    ("155.520", True),
    ("Mbps,STM-4的速率为", False),
    ("622.080", True),
    ("Mbps,STM-16的速率为", False),
    ("2488.320", True),
    ("Mbps。", False),
])

# 31
fill_blank_question(doc, 31, [
    ("SDH的基本网络单元包括: ", False),
    ("TM(终端复用器)", True),
    ("、", False),
    ("ADM(分插复用器)", True),
    ("、", False),
    ("DXC(数字交叉连接设备)", True),
    ("、", False),
    ("REG(再生中继器)", True),
    ("。", False),
])

# 32 (已答)
fill_blank_question(doc, 32, [
    ("STM意思是", False),
    ("同步传送模块", True),
    (",STM-1可以收容", False),
    ("63", True),
    ("个2.048Mb/s的支路业务。", False),
])

# 33 (已答)
fill_blank_question(doc, 33, [
    ("SDH帧结构分为", False),
    ("段开销(SOH)", True),
    ("、", False),
    ("信息净负荷(PAYLOAD)", True),
    ("、", False),
    ("管理单元指针(AUPTR)", True),
    ("三部分。", False),
])

# 34 (已答)
fill_blank_question(doc, 34, [
    ("SDH帧结构为矩形块状帧结构,帧结构字节的传输是从", False),
    ("左到右", True),
    ("、", False),
    ("从上到下", True),
    ("进行的,每秒传", False),
    ("8000", True),
    ("帧。", False),
])

# 35
fill_blank_question(doc, 35, [
    ("STM-1帧,每帧有", False),
    ("9", True),
    ("行,每行有", False),
    ("270", True),
    ("列(字节),其中净载荷在每行中占", False),
    ("261", True),
    ("列。", False),
])

# 36
fill_blank_question(doc, 36, [
    ("我国的SDH传送网主要分为四个层面: ", False),
    ("省际干线层", True),
    ("、", False),
    ("省内干线层", True),
    ("、", False),
    ("本地传送层", True),
    ("和", False),
    ("接入层", True),
    ("。", False),
])

# 37 (已答)
fill_blank_question(doc, 37, [
    ("流量控制负责管理和协调两个通信实体之间的数据传输速率,分", False),
    ("开环控制", True),
    ("和", False),
    ("闭环控制", True),
    ("两种控制方式。", False),
])

# 38
fill_blank_question(doc, 38, [
    ("滑动窗口协议分为", False),
    ("后退N帧协议(GBN)", True),
    ("和", False),
    ("选择性重传协议(SR)", True),
    ("两类。", False),
])

# 39
fill_blank_question(doc, 39, [
    ("数据单元传输中的差错分为两类: ", False),
    ("单比特差错", True),
    ("和", False),
    ("突发差错", True),
    (",其中", False),
    ("突发差错", True),
    ("意味着数据单元中存在两个或以上的比特被改变。", False),
])

# 40
fill_blank_question(doc, 40, [
    ("局域网将数据链路层分为两个子层,分别是", False),
    ("MAC(媒体接入控制)", True),
    ("和", False),
    ("LLC(逻辑链路控制)", True),
    ("。", False),
])

# 41
fill_blank_question(doc, 41, [
    ("以太网对设备的识别使用", False),
    ("MAC地址", True),
    (",该地址为", False),
    ("48", True),
    ("位bit,前", False),
    ("3", True),
    ("字节表示网络硬件制造商。", False),
])

# 42 (已答)
fill_blank_question(doc, 42, [
    ("以太网的MAC地址分为三种类型,分别是", False),
    ("单播地址", True),
    ("、", False),
    ("多播地址", True),
    ("和", False),
    ("广播地址", True),
    ("。", False),
])

# 43
fill_blank_question(doc, 43, [
    ("一般来说,称以太网的端到端往返时间为", False),
    ("争用期", True),
    (",对于10Base-T局域网来说,这个时间是", False),
    ("51.2μs", True),
    (",可以发", False),
    ("64", True),
    ("字节。", False),
])

# 44 (已答)
fill_blank_question(doc, 44, [
    ("交换式以太网主要是涉及三个方面的技术: ", False),
    ("透明网桥协议", True),
    ("、", False),
    ("生成树协议", True),
    ("和", False),
    ("虚拟局域网(VLAN)", True),
    ("。", False),
])

# 45 (已答)
fill_blank_question(doc, 45, [
    ("交换机在各端口之间转发数据的方式有三种: ", False),
    ("直通方式", True),
    ("、", False),
    ("存储转发方式", True),
    ("和", False),
    ("无碎片方式", True),
    ("。", False),
])

# 46
fill_blank_question(doc, 46, [
    ("网络层在", False),
    ("数据链路层", True),
    ("之上,可以为任意主机之间的通信提供", False),
    ("不可靠的", True),
    ("、", False),
    ("无连接的", True),
    ("数据报服务。", False),
])

# 47
fill_blank_question(doc, 47, [
    ("PSTN网(公共交换电话网)的构成三要素是", False),
    ("用户终端设备", True),
    ("、", False),
    ("交换设备", True),
    ("和", False),
    ("传输系统", True),
    (",其中", False),
    ("交换设备", True),
    ("是构成通信网的核心要素。", False),
])

# 48
fill_blank_question(doc, 48, [
    ("数字程控交换机的硬件结构划分为", False),
    ("话路子系统", True),
    ("和", False),
    ("控制子系统", True),
    ("两部分,其中,语音信息传输属于", False),
    ("话路子系统", True),
    ("。", False),
])

# 49
fill_blank_question(doc, 49, [
    ("程控交换机的软件包括", False),
    ("操作系统", True),
    ("、", False),
    ("数据库系统", True),
    ("和", False),
    ("应用软件系统", True),
    ("。", False),
])

# 50
fill_blank_question(doc, 50, [
    ("二级长途电话网中,高平面是", False),
    ("DC1(省级交换中心)", True),
    (",以网状网相连,低一级平面是", False),
    ("DC2(地区中心)", True),
    ("平面,以", False),
    ("星型网", True),
    ("相连。", False),
])

# 51
fill_blank_question(doc, 51, [
    ("电话网的本地网中,交换局有两类,分别是", False),
    ("端局(DL)", True),
    ("和", False),
    ("汇接局(Tm)", True),
    (",其中", False),
    ("汇接局(Tm)", True),
    ("处在高层。", False),
])

# 52
fill_blank_question(doc, 52, [
    ("传统电话网电话业务的路由,按照呼损可分成", False),
    ("直达路由", True),
    ("和", False),
    ("迂回路由", True),
    ("路由。", False),
])

# 53
fill_blank_question(doc, 53, [
    ("No.7信令的工作方式包括: ", False),
    ("直连方式", True),
    ("、", False),
    ("准直连方式", True),
    ("和", False),
    ("非直连方式", True),
    ("。", False),
])

# 54
fill_blank_question(doc, 54, [
    ("No.7信令系统中,用户级可分成", False),
    ("TUP(电话用户部分)", True),
    ("和", False),
    ("ISUP(ISDN用户部分)", True),
    ("。", False),
])

# ====================================================
#  四、术语释义 —— 跳过
# ====================================================
doc.add_page_break()
add_heading_styled('四、术语释义', level=1)
p = doc.add_paragraph()
add_run_with_font(p, "（此大题不做，已跳过）", bold=False, color=RGBColor(128, 128, 128))

# ====================================================
#  五、简答
# ====================================================
doc.add_page_break()
add_heading_styled('五、简答', level=1)

# 1
add_simple_answer(doc, 1, "面向连接网络和无连接网络的主要区别是什么？")
p = doc.add_paragraph()
add_run_with_font(p, "答：面向连接的网络在通信前需要建立连接，通信过程中资源被预留，数据沿固定路径按序传输，通信结束后释放连接（如虚电路方式）。无连接的网络不需要建立连接，每个分组携带完整的目的地址独立选路，可能走不同路径，不保证按序到达（如数据报方式）。面向连接适用于对服务质量要求高的业务，无连接适用于突发性较强的数据业务。")

# 2
add_simple_answer(doc, 2, "一个通信协议主要包含哪些内容？")
p = doc.add_paragraph()
add_run_with_font(p, "答：一个通信协议主要包含三方面内容：\n1) 语法：定义协议的数据格式，即用户数据与控制信息的结构和格式；\n2) 语义：进行协调和错误处理的控制信息，即需要发出的控制信息，以及完成的动作与做出的响应；\n3) 时序：包括同步和顺序控制，即详细说明事件进行的顺序。")

# 3
add_simple_answer(doc, 3, "TCP/IP模型的第三层为IP层，其主要功能有哪些？")
p = doc.add_paragraph()
add_run_with_font(p, "答：IP层（网络层）的主要功能是完成分组在网中任意主机（终端）间的传递，具体包括：\n1) 路由选择：为分组选择合适的传输路径；\n2) 分组转发：将分组从输入接口转发到输出接口；\n3) 异构网络互连：实现不同类型物理网络的互连互通。")

# 4
add_simple_answer(doc, 4, "构成SDH/SONET传送网的主要网元设备有哪些？他们在网络中的作用是什么？")
p = doc.add_paragraph()
add_run_with_font(p, "答：SDH/SONET传送网的主要网元设备包括：\n1) 终端复用器(TM)：为传统接口用户提供到SDH网络的接入，将多个PDH低阶支路信号复用成一个STM-1或STM-4，也可完成电信号到光信号的转换；\n2) 分插复用器(ADM)：便于组建环网，提高光网络生存性，负责在STM-N中插入或提取低阶支路信号，利用内部时隙交换功能实现两个STM-N之间不同VC的连接，是目前SDH网中应用最广泛的网络单元；\n3) 数字交叉连接设备(DXC)：具有多个STM-N信号端口，可提供任意两端口速率之间的交叉连接；\n4) 再生中继器(REG)：主要通过光/电整形放大以延长光传输距离。")

# 5
add_simple_answer(doc, 5, "分析SDH/SONET传送网的主要优缺点。")
p = doc.add_paragraph()
add_run_with_font(p, "答：SDH/SONET传送网的主要优点：\n1) 大颗粒的带宽复用、交叉和配置；\n2) 多种客户信号封装和透明传输；\n3) 强大的开销和维护管理能力；\n4) 组网和保护能力增强。\n\n主要缺点：\n面向话音业务优化设计，采用严格的TDM技术方案，对于突发性很强的数据业务，带宽利用率不高。")

# 6
add_simple_answer(doc, 6, "SDH的帧结构有哪几部分组成，各起什么作用？")
p = doc.add_paragraph()
add_run_with_font(p, "答：SDH的帧结构由三部分组成：\n1) 段开销(SOH)：用于SDH传输网的运行、维护、管理和指配(OAM&P)；\n2) STM净负荷(PAYLOAD)：用于存放各种业务信息，也包含少量的通道开销POH；\n3) 管理单元指针(AU-PTR)：用于指示STM净负荷中的第一个字节在STM-N帧内的起始位置。\n\nSDH帧格式是以字节为单位的块状结构，STM-1由9行、270列字节组成，STM-N有9行、270×N列组成。")

# 7
add_simple_answer(doc, 7, "什么是PTN，有什么技术特点？")
p = doc.add_paragraph()
add_run_with_font(p, "答：PTN（Packet Transport Network，分组传送网）是新一代基于分组的、面向连接的多业务统一传送技术。其技术特点包括：\n1) 融合了现有光传送网络架构，主要针对分组业务的流量突发特性和统计复用的传送需求而设计；\n2) 既保留了传统SDH传送网络的部分技术优势，又引入了分组技术的基本特征；\n3) 具备高效的多业务适配能力和灵活的标签转发机制；\n4) 实现了基于分组交换的多业务传送平台，满足未来多业务承载的统一需求。")

# 8
add_simple_answer(doc, 8, "试分析为什么PTN传送网是未来传送网络的发展趋势？")
p = doc.add_paragraph()
add_run_with_font(p, "答：PTN是未来传送网络发展趋势的原因：\n1) PTN充分考虑支持运营级以太网业务，同时兼具传统的TDM业务能力；\n2) PTN遵循下一代网络的体系架构；\n3) PTN体系结构中，下层可支持各种传输介质，兼容OTN、SDH、PDH等各种传统网络；\n4) PTN体系结构中，上层可支持多种客户业务，如Ethernet、ATM、T-MPLS等；\n5) 具备高效的多业务适配能力和灵活的标签转发机制，实现了基于分组交换的多业务传送平台。")

# 9
add_simple_answer(doc, 9, "简述局域网的基本特点。")
p = doc.add_paragraph()
add_run_with_font(p, "答：计算机局域网主要有以下特点：\n1) 局域网覆盖有限的地理范围，适用于机关、公司、校园、军营、工厂等有限范围内的计算机、终端与各类信息处理设备连网的需求；\n2) 局域网的数据传输速率较高，数据传输误码率较低(<10⁻⁸)；\n3) 局域网一般属于一个单位或园区所有，易于建立、维护和扩展。")

# 10
add_simple_answer(doc, 10, "简述共享式以太网的特点。")
p = doc.add_paragraph()
add_run_with_font(p, "答：共享式以太网的特点是所有站点共享同一传输介质（总线或集线器），任一时刻只能有一个站点发送数据。采用CSMA/CD协议进行介质访问控制，如果有两个以上站点同时发送数据会产生冲突。所有站点处于同一个冲突域中，共享带宽。")

# 11
add_simple_answer(doc, 11, "交换机式以太网中，交换机收到一个MAC帧，将执行何种操作？")
p = doc.add_paragraph()
add_run_with_font(p, "答：交换机收到一个MAC帧时，可能出现三种情况：\n1) MAC转发表中无对应项：交换机将帧从除了入端口之外的所有端口进行泛洪；\n2) MAC转发表中有对应项且目的端口≠入端口：交换机将帧从对应的端口转发出去；\n3) MAC转发表中有对应项目但目的端口=入端口：交换机丢弃该帧（过滤）。\n\n交换机的工作原理可描述为\"基于源MAC学习，基于目的MAC转发\"，主要包括基于源地址的自学习过程和基于目的地址的转发/过滤过程。")

# 12
add_simple_answer(doc, 12, "试解释冲突域、广播域的概念。")
p = doc.add_paragraph()
add_run_with_font(p, "答：在共享介质型局域网中，单播帧通达的网络范围，称为冲突域；广播帧通达的网络范围，称为一个广播域。冲突域内的站点共享传输介质，同一时刻只能有一个站点发送数据，否则会发生冲突；广播域内的所有站点都能收到广播帧。")

# 13
add_simple_answer(doc, 13, "比较集线器和以太网交换机的区别。")
p = doc.add_paragraph()
add_run_with_font(p, "答：集线器和以太网交换机都是数据传输的枢纽，主要区别如下：\n1) 工作层次：集线器工作在物理层，仅对信号进行放大和转发；交换机工作在数据链路层，能识别MAC地址；\n2) 冲突域：集线器的所有端口在一个冲突域中，所有端口共享带宽；交换机的每一个端口都是一个独立的冲突域，每个端口独享带宽；\n3) 转发方式：集线器将信号从所有端口广播发送；交换机根据MAC地址表选择目标端口转发，不会向所有端口发送；\n4) 双工方式：集线器只能半双工通信；交换机支持全双工通信，可同时发送和接收数据。")

# 14
add_simple_answer(doc, 14, "什么是VLAN？划分VLAN之后能解决二层网络中的哪些问题？")
p = doc.add_paragraph()
add_run_with_font(p, "答：虚拟局域网VLAN是将网络从逻辑上划分出一个个与地理位置无关的子集，每个子集构成一个VLAN。划分VLAN之后，可以解决二层网络中的三个问题：\n1) 隔离了广播域：将广播限制在VLAN内，减少广播风暴的影响范围；\n2) 增加了安全性：不同VLAN之间不能直接通信，需经过三层设备转发；\n3) 用户管理方便：用户可以按逻辑分组而非物理位置分组，便于网络管理。")

# 15
add_simple_answer(doc, 15, "简述TCP/IP体系结构的特点。")
p = doc.add_paragraph()
add_run_with_font(p, "答：TCP/IP体系结构的主要特点：\n1) 简单性：相比OSI七层模型，TCP/IP采用更精简的层次结构，便于实现；\n2) 开放性：TCP/IP协议标准是公开的，任何厂商都可以实现，促进了互联网的发展；\n3) 异构网络互连：TCP/IP明确了异构网络之间应基于网络层(IP层)实现互连的思想，这是其最重要的贡献。Internet的成功主要归功于TCP/IP协议的简单性和开放性。")

# 16
add_simple_answer(doc, 16, "简述网络层实现的基本功能。")
p = doc.add_paragraph()
add_run_with_font(p, "答：网络层实现的基本功能包括：\n1) 路由选择：根据网络拓扑和路由协议，为分组选择合适的传输路径；\n2) 分组转发：将分组从接收端口转发到相应的输出端口；\n3) 拥塞控制：当网络负载过重时采取相应措施避免或缓解拥塞；\n4) 异构网络互连：实现不同类型网络之间的互连互通。")

# 17
add_simple_answer(doc, 17, "简述BGP协议的工作过程。")
p = doc.add_paragraph()
add_run_with_font(p, "答：（参考资料中本题标记为\"略\"，以下为基于教材的简要回答）\nBGP（边界网关协议）是不同自治系统(AS)之间交换路由信息的外部网关协议。工作过程：首先BGP路由器建立TCP连接形成BGP对等体关系，然后通过UPDATE消息交换路由信息，宣告可达的网络前缀及其路径属性。BGP采用路径向量算法，每条路由携带完整的AS路径信息，避免了路由环路。BGP支持路由策略控制，可根据AS路径长度等因素选择最优路由。")

# 18
add_simple_answer(doc, 18, "简述OSPF路由协议的工作过程。")
p = doc.add_paragraph()
add_run_with_font(p, "答：OSPF路由协议的工作过程：\n1) 运行OSPF协议的路由器创建自己的链路状态信息，寻找路由域内的邻居路由器；\n2) 将自己的链路状态信息洪泛到路由域内的其他所有路由器；\n3) 通过洪泛，路由域内的每台路由器最终会构建一个内容相同的本地链路状态数据库LSDB；\n4) 利用LSDB，每台路由器采用最短路径算法(Dijkstra算法)独立地计算去往目的网络的最短路径；\n5) 一旦网络结构稳定，则停发链路状态信息，仅周期性地发短小的Hello消息来监控网络变化。")

# 19
add_simple_answer(doc, 19, "什么是信令转接点，有哪几种类型？")
p = doc.add_paragraph()
add_run_with_font(p, "答：信令转接点(STP)是No.7信令网中负责转接信令消息的节点，它将信令消息从一条信令链路转接到另一条信令链路。STP分为两种类型：\n1) 独立型STP：专门用于转接信令消息的设备；\n2) 综合式STP：与交换局合设，既完成交换功能又完成信令转接功能。\n\n我国No.7信令网分为三级：高级信令转接点(HSTP)、低级信令转接点(LSTP)、信令点(SP)。")

# 20
add_simple_answer(doc, 20, "简述我国的三级信令网结构。")
p = doc.add_paragraph()
add_run_with_font(p, "答：我国No.7信令网采用了三级结构，从上到下分别为：\n1) 高级信令转接点(HSTP)：信令网中最高的一级，负责转接它所汇接的LSTP和SP之间的信令消息。设在各省、自治区及直辖市，成对设置，一对HSTP管辖的区域称为主信令区。HSTP采用独立型STP。\n2) 低级信令转接点(LSTP)：信令网的第二级，负责转接它所汇接的第三级SP的信令消息。设在各个地、市，成对设置，一对LSTP管辖的区域称为分信令区。LSTP可以采用独立型STP，也可采用综合式STP。\n3) 信令点(SP)：信令网的第三级，是传送各种信令消息的源点或目的地点，由各种交换局和特种服务中心（业务控制点、网管中心等）组成。")

# 21
add_simple_answer(doc, 21, "请比较静态复用和动态复用的不同与各自的优缺点。")
p = doc.add_paragraph()
add_run_with_font(p, "答：静态复用（又称同步复用）是指将分割后的子信道（频带或时隙）静态地分给每个用户专用。优点是一旦为某用户分配了信道，该用户的服务质量便不会受网络中其他用户的影响。缺点是为了保证用户所需带宽，必须按信息最大速率分配信道资源。\n\n动态复用（又称统计复用）是指在给用户分配资源时采用动态分配（即按需分配），只有在用户有信息传送时才给它分配资源。优点是可以获得较高的信道利用率，由于每个终端的数据使用一个自己独有的\"标记\"，可以把传送的信道按照需要动态地分配给每个终端用户，因此提高了传送信道的利用率。缺点是在用户过多时可能出现服务质量下降。")

# 22
add_simple_answer(doc, 22, "光传送网为什么要引入一个光信道层，它在OTN中起什么作用？")
p = doc.add_paragraph()
add_run_with_font(p, "答：OTN的分层结构为光信道层(OCh)、光复用段层(OMS)和光传输段层(OTS)。\n\nOTN引入光信道层(OCh)的主要原因是：OTN很重要的一个设计目标就是要将类似SDH/SONet网络中基于单波长的OMAP(运行、管理、维护和指配)功能引入到基于多波长复用技术的光网络中，OCh就是为实现这一目标而引入的。\n\n光信道层的作用：负责为来自电复用段层的各种类型的客户信息选择路由、分配波长，为灵活的网络选路安排光信道连接，处理光信道开销，提供光信道层的检测、管理功能，在故障发生时支持端到端的光信道(以波长为基本交换单元)连接，在网络发生故障时执行重选路由或进行保护切换。")

# 23
add_simple_answer(doc, 23, "简述路由器的功能。")
p = doc.add_paragraph()
add_run_with_font(p, "答：路由器是工作在网络层的核心设备，主要功能包括：\n1) 网络互连：连接不同类型的网络，实现异构网络的互通；\n2) 路由选择：根据路由表和路由协议选择最佳传输路径；\n3) 分组转发：将IP分组从一个接口转发到另一个接口；\n4) 网络隔离：隔离广播域，防止广播风暴蔓延；\n5) 访问控制：通过ACL等机制实现流量过滤和安全控制。")

# 24
add_simple_answer(doc, 24, "简述本地网的交换中心及职能。")
p = doc.add_paragraph()
add_run_with_font(p, "答：本地网内可设置端局(DL)和汇接局(Tm)两类交换中心。\n\n1) 端局的职能：端局通过用户线与用户相连，它的职能是负责疏通本局用户的去话和来话话务。\n\n2) 汇接局的职能：汇接局与所管辖的端局相连，以疏通这些端局间的话务；汇接局还与其他汇接局相连，疏通不同汇接区间端局的话务；根据需要汇接局还可与长途交换中心相连，用来疏通本汇接区的长途转话话务。")

# 25
add_simple_answer(doc, 25, "目前在我国智能网中已开放的常见业务有哪些？")
p = doc.add_paragraph()
add_run_with_font(p, "答：常见的智能网业务有：\n1) 被叫集中付费业务（如800业务）；\n2) 虚拟专用网业务(VPN)；\n3) 号码携带业务；\n4) 通用个人通信业务(UPT)；\n5) 统一帐号业务；\n6) 广域集中交换机业务(Centrex)。")

# ====================================================
#  保存
# ====================================================
output_path = r"C:\Users\w1895\Desktop\现代通信网期末习题_答案.docx"
doc.save(output_path)
print(f"答案文档已保存至: {output_path}")
