# 宽窄视场探测传感器-----★★★

Status: draft
Authority: historical source requirement draft for `docs/space_based_infrared_sensor/design.md`

## 功能概述

宽窄视场探测传感器模型是一个基于红外探测技术的传感器模型，用于模拟红外传感器在复杂环境下的目标扫描、探测功能。其主要任务包括：

a）基于红外辐射特性探测敌方目标、计算目标的方位角和俯仰角、生成目标跟踪记录，以及根据气象条件调整探测性能；

b）通过与仿真平台交互，获取目标位置、姿态及环境数据，结合红外探测算法，输出目标的探测结果。

满足：

1）模拟卫星对目标的探测与跟踪能力。

a）能够生成各探测设备可观测范围；

b）能够生成目标的可探测性与动态探测结果，如距离或角度信息；

c）能够对高速动态目标的多个观测误差来源进行模拟。

2）模拟宽视场扫描探测器和窄视场跟踪探测的能力。

a）能够模拟扫描探测器在可视范围内，对空域进行扫描的功能；

b）能够模拟窄视场跟踪传感器对目标的持续跟踪能力；

## 工作原理

a）初始化与参数配置：模型通过结构体存储传感器属性参数（如大气透过率、探测阈值、方位精度等）。在首次调用探测方法时，模型会实例化探测算法对象，并根据当前单位位置（经纬度）查询气象影响比例，动态调整探测阈值以反映环境对红外信号的衰减。

b）气象影响处理：模型通过获取环境参数（海浪等级、天气类型、温度、湿度、能见度），并从气象影响列表中查询红外探测的衰减比例。

衰减比例通过查表获取，气象影响列表设计参考下表。

表1 气象影响列表

<table>
<colgroup>
<col style="width: 15%" />
<col style="width: 40%" />
<col style="width: 43%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>气象参数</strong></th>
<th style="text-align: center;"><strong>红外探测衰减比例</strong></th>
<th style="text-align: center;"><strong>影响描述</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;">海浪等级</td>
<td style="text-align: left;">低等级：5%<br />
中等级：10%<br />
高等级：15%</td>
<td style="text-align: left;">海浪等级越高，大气中的水汽含量和颗粒物可能越多，对红外辐射的吸收和散射作用越强，导致探测性能下降。</td>
</tr>
<tr>
<td style="text-align: center;">天气类型</td>
<td style="text-align: left;">晴天：0%<br />
多云：5%<br />
雨天：15%<br />
雾天：20%</td>
<td style="text-align: left;">不同天气类型下，大气中的水汽、云层、颗粒物等含量不同，对红外辐射的衰减作用各异。雨天和雾天由于水汽含量高，衰减比例较大。</td>
</tr>
<tr>
<td style="text-align: center;">温度</td>
<td style="text-align: left;">每升高10℃，衰减比例减少2%（在合理范围内）</td>
<td style="text-align: left;">温度升高可能导致大气中的水汽蒸发，减少对红外辐射的吸收和散减作用，但过高温度也可能引发其他复杂效应（此样例为简化处理）。</td>
</tr>
<tr>
<td style="text-align: center;">湿度</td>
<td style="text-align: left;">每增加20%，衰减比例增加5%</td>
<td style="text-align: left;">湿度增加意味着大气中水汽含量增多，对红外辐射的吸收和散射作用增强，导致探测性能下降。</td>
</tr>
<tr>
<td style="text-align: center;">能见度</td>
<td style="text-align: left;">能见度10km以上：0%<br />
能见度5-10km：5%<br />
能见度1-5km：10%<br />
能见度1km以下：20%</td>
<td style="text-align: left;">能见度越低，大气中的颗粒物（如尘埃、烟雾等）可能越多，对红外辐射的散射作用越强，导致探测性能下降。</td>
</tr>
</tbody>
</table>

基于加权叠加和修正的算法公式计算综合的衰减比例：

<img src="红外模型1205-V3.0-media/media/image1.png" style="width:3.18647in;height:0.75316in" />

其中：

Atotal：综合衰减比例（0 ≤Atotal≤ 1，1表示完全衰减）。

Ai：第i个气象参数的独立衰减比例（如海浪、天气类型等）。

wi：第i个参数的权重系数（0 ≤wi≤1，∑wi=1）。

kj：第j对参数交互项的修正系数。

Ap⋅Aq：参数p和q的交互衰减项（如湿度与能见度的联合影响）。

C：常数修正项（如温度基准调整）。

c）目标探测逻辑：模型通过探测相关函数执行探测任务：首先，检查传感器是否开机。若开机，模型获取父单位的位置和姿态信息，转换为经纬高坐标（LLA）。获取探测半径内的所有目标信息，仅处理敌方目标。通过坐标变换对每个目标计算其相对于传感器的方位角和俯仰角，根据目标和传感器位置计算视线向量（LOS），通过坐标系变换（ECR 到局部 FOV 坐标系）将视线向量转换为局部坐标，进而计算俯仰角（基于反正弦函数）和方位角（基于反正切函数），并查询目标的红外辐射强度。

红外辐射强度通过查表获取，表设计如下所示。

表2红外辐射强度表

| 序号 | 方位角 | 俯仰角 | 红外辐射强度 |
|:----:|:------:|:------:|:------------:|
|  1   |        |        |              |
|  2   |        |        |              |
|  3   |        |        |              |

d）数据处理与输出：探测算法处理输入数据（包括传感器状态、目标列表及时间步），生成输出数据，包含探测到的目标数量及详细信息（如目标ID、方位角、俯仰角、距离等）。这些数据被输出给数据处理器，生成跟踪记录。若检测到新目标，模型会记录目标ID并触发关键事件通知。

## 相关算法

### **探测阈值调整算法公式**

``` math
T = \mu + k \times \sigma
```

a）参数说明：

$`\text{T}`$：调整后的探测阈值，量纲为原始DN值。

$`\mu`$：局部背景灰度/幅度均值，量纲为原始DN值。

![](红外模型1205-V3.0-media/media/image2.wmf)：局部背景灰度/幅度标准差，量纲为原始DN值。

$`k`$：调整系数，无量纲，其值选取参考表如下：

| k值 |       虚警率P_fa        |          适用场景          |
|:---:|:-----------------------:|:--------------------------:|
|  3  |         0.0013          |  高灵敏度，允许一定误报。  |
|  4  | 3.17$`\times 10^{- 6}`$ |    标准，如宽视场探测。    |
|  5  | 2.87$`\times 10^{- 7}`$ |          低虚警。          |
|  6  | 9.9$`\times 10^{- 10}`$ | 极低虚警，用于窄视场探测。 |

b）算法描述：

该公式是实际工程中最常用的红外设备探测阈值调整公式，可以根据常用经验背景DN值调整$`\mu`$和![](红外模型1205-V3.0-media/media/image2.wmf)，调整参数模拟复杂背景；并调整虚警率，进而调整系数k模拟不同用途的红外探测设备。

### **方位角和俯仰角计算算法公式**

俯仰角：$`\text{El} = \text{RADTODEG}\left( - arcsin\left( \frac{\text{XLOS}_{z}}{\text{LOSRange}} \right) \right)`$

方位角：$`Az = \text{RADTODEG}\left( \arctan 2\left( \text{XLOS}_{y},\text{XLOS}_{x} \right) \right)`$

a）参数说明：

$`\text{El}`$：俯仰角（El），单位：度，范围 \[-90, 90\]。

$`\text{Az}`$：方位角（Az），单位：度，范围 \[0, 360\]。

$`\text{XLOS}_{z},\text{XLOS}_{y},\text{XLOS}_{x}`$：视线向量在局部 FOV 坐标系中的分量（$`\text{XLOS}_{z}`$, $`\text{XLOS}_{y}`$ $`\text{XLOS}_{x}`$），单位：米。

$`\text{LOSRange}`$：视线向量模长（LOSvect.Norm()），单位：米，计算为 $`\sqrt{\text{LOS}_{x}^{2} + \text{LOS}_{y}^{2} + \text{LOS}_{z}^{2}}`$。

$`\text{RADTODEG}`$：弧度转角度转换函数，定义为 $`\text{RADTODEG}(x) = x \times \frac{180}{\pi}`$。

b）算法描述：

（1）视线向量计算：根据目标位置（Target.position\_）和传感器位置（Senserunit-\>GetUnitInfo()-\>position\_）计算视线向量（LOSvect）：

``` math
\text{LOS}_{x} = - \left( \text{Target.position}_{x} - \text{Senserunit.position}_{x} \right)
```

``` math
\text{LOS}_{y} = - \left( \text{Target.position}_{y} - \text{Senserunit.position}_{y} \right)
```

``` math
\text{LOS}_{z} = - \left( \text{Target.position}_{z} - \text{Senserunit.position}_{z} \right)
```

（2）坐标变换：获取 ECR 到局部 FOV 坐标系的变换矩阵，并通过矩阵乘法将视线向量转换为局部坐标。

（3）俯仰角计算：使用反正弦函数计算俯仰角，负号确保俯仰角方向符合坐标系定义。

（4）方位角计算：使用反正切函数计算方位角，考虑象限以确保角度范围正确。

### 目标可探测性与动态探测结果生成

目标可探测性与动态探测结果生成主要计算目标的动态探测结果（位置、距离、角度）和红外可探测性（基于信噪比SNR判断）。

（1）目标红外可探测性计算（核心：SNR模型）

红外探测的本质是“目标辐射信号能否超过探测器噪声阈值”，通过计算信噪比（SNR） 判断可探测性（通常SNR≥5视为可探测）。

1）算法逻辑

a）目标红外辐射建模：计算目标在红外波段（3-5μm或8-14μm）的辐射通量；

b）大气衰减建模：通过大气透过率修正辐射通量（考虑水汽、CO₂吸收）；

c）探测器响应建模：计算探测器接收到的信号功率，再结合探测器噪声（光子噪声、热噪声、读出噪声）计算SNR；

d）可探测性判断：若SNR≥阈值SNR<sub>th</sub>，则目标可探测。

2）核心公式

a）目标自身辐射通量（斯特藩-玻尔兹曼定律，简化为黑体辐射），假设目标为朗伯表面，辐射亮度各向同性，则从特定方向观察目标，目标的辐射功率大约为：

``` math
P_{tar} = \frac{\sigma \bullet T_{tar}^{4}}{\pi} \bullet A_{tar} \bullet \omega
```

其中：$`\sigma = 5.76 \bullet 10^{- 8}W/(m^{2} \bullet k^{4})`$为斯特藩-玻尔兹曼常数，$`T_{tar}`$为目标表面温度（高超声速目标≈1000-2000K），$`A_{tar}`$为从特定方向观察目标的投影面积（m²），ω为接收立体角（![](红外模型1205-V3.0-media/media/image3.wmf)，d为探测器光敏面直径，r为探测器与目标距离）。

b）大气衰减后的辐射通量（朗伯-比尔定律）：

``` math
\Phi_{atm} = \Phi_{tar} \bullet \tau(\lambda,d)
```

其中：$`\tau(\lambda,d)`$为大气透过率（依赖红外波段$`\lambda`$和目标-卫星距离d，通过LOWTRAN/MODTRAN简化模型计算，如3-5μm波段d=1000km时<img src="红外模型1205-V3.0-media/media/image4.png" style="height:0.139in" />≈0.6）。

c）探测器接收功率：

``` math
P_{sig} = \frac{\Phi_{atm} \bullet A_{\det} \bullet \eta_{\det}}{d^{2}}
```

其中：A<sub>det</sub>为探测器光敏面面积（m²），$`\eta_{\det}`$为探测器量子效率（0.5-0.8）。

d）探测器噪声功率（总噪声为各噪声的均方根）：

光子噪声：$`N_{ph} = \sqrt{2 \bullet q \bullet P_{sig} \bullet t_{int}}`$（$`q = 1.6 \bullet 10^{- 19}`$C为电子电荷，$`t_{int}`$为积分时间）；

热噪声：$`N_{th} = \sqrt{\frac{{4k}_{B}T_{\det}B}{R_{\det}}}`$（$`k_{B} = 1.38*10^{- 23}`$J/K为玻尔兹曼常数，$`T_{\det}`$为探测器工作温度，B为带宽，$`R_{\det}`$为探测器内阻）；

读出噪声：N<sub>read</sub>（固定值，如10-50 e⁻/帧）；

总噪声：$`N_{total} = \sqrt{N_{ph}^{2} + N_{th}^{2} + N_{read}^{2}}`$。

e）信噪比与可探测性判断：

``` math
SNR = \frac{P_{sig} \bullet t_{int}}{N_{total}}，若SNR \geq {SNR}_{th} \Longrightarrow 可探测
```

### 宽窄视场联合探测算法 

红外传感器根据其探测方式可分为两种主要工作模式：宽视场模式和窄视场模式，宽视场模式用来实现卫星对大面积区域的扫描探测，窄视场模式用来实现卫星对小面积目标的凝视跟踪。

1.  宽视场模式（WFOV）

宽视场模式的扫描探测是指通过光学机械装置（如扫描镜或旋转棱镜）或电子扫描方式，使红外传感器在较大的视场内移动，逐步成像整个区域。适合执行目标搜索与预警任务，使其时间分辨率相对较低。

2.  窄视场模式（NFOV）

窄视场模式的凝视跟踪指红外传感器固定视场角，持续观测一个区域，不进行机械或光学扫描。具有高时间分辨率和高空间分辨率，适合对目标进行持续跟踪和成像识别。

3.  宽窄视场联合探测

现代天基红外预警卫星采用宽视场（WFOV）+窄视场（NFOV）双模式协同工作，实现“先发现、后盯死”的全程跟踪。核心算法为宽→窄视场无缝移交（Cueing & Handover），全流程星上自主完成，从目标出现到建立精密跟踪不超过5秒。

联合探测总体流程

> 1\. 宽视场TBD检测 → 2. LOS重建 → 3. 运动预测（Cueing）→ 4. ATP快速指向 → 5. 窄视场快速捕获。

联合探测核心算法

宽视场先跟踪后检测（Track-Before-Detect, TBD）

> **管道滤波能量累积：**
>
> S_pipeline(x, y) = Σ(i=0 to L-1) I(x, y, t-iΔt)
>
> 参数:
>
> L: 管道长度 (典型值3~5帧)
>
> I(x,y,t): 时刻t的像元(x,y)强度
>
> Δt: 帧间隔 (典型0.1秒)
>
> 检测准则：
>
> S_pipeline(x,y) \> μ_bg + k·σ_bg
>
> 参数：
>
> μ_bg: 背景均值
>
> σ_bg: 背景标准差
>
> k: 阈值系数 (4~6倍标准差)
>
> **动态规划TBD：**
>
> 在速度空间(vx, vy)网格上，沿假设轨迹累积能量：
>
> E(vx, vy) = Σ(k=1 to N) I(x0 + vx·kΔt, y0 + vy·kΔt, tk)
>
> 参数：
>
> vₓ, vᵧ：假设目标在焦平面上的速度分量（像素/帧）
>
> N：参与能量累积的最大帧数
>
> x₀, y₀：轨迹起始像元坐标
>
> tₖ：第k帧的时间戳

视线重建与误差协方差 (LOS Reconstruction)

> **从图像元到视线转换：**
>
> 像元归一化坐标：
>
> u = (x - x_center) / W
>
> v = (y - y_center) / H
>
> 参数：
>
> x, y：目标在探测器阵列上的像素坐标
>
> x_center, y_center：光学轴在探测器上的中心像素坐标
>
> W, H：宽视场模式下实际使用的图像宽度和高度
>
> 计算目标视线角度：
>
> α_az = u · FOV_az , α_el = v · FOV_el
>
> 参数：
>
> α_az, α_el：目标相对于光学轴的方位角和俯仰角（弧度）
>
> FOV_az, FOV_el：宽视场在方位和俯仰方向的总视场角（典型10°×10°）
>
> 计算目标视线向量：
>
> v_LOS = \[cos(α_el)·cos(α_az), cos(α_el)·sin(α_az), sin(α_el)\]^T
>
> **测量误差协方差矩阵：**
>
> 角度测量噪声：
>
> σ_angle = √\[(σ_pixel · p / f)^2 + σ_cal^2\]
>
> 参数：
>
> σ_pixel: 像元定位误差 (典型0.3像元)
>
> p: 像元物理尺寸 (15~30 μm)
>
> f: 焦距 (典型300 mm)
>
> σ_cal: 标定残差 (典型50 μrad)
>
> 测量误差协方差
>
> P1 = diag(σ_α_az^2, σ_α_el^2, σ_R^2)
>
> 参数：
>
> σ_az, σ_el：方位、俯仰角测量标准差（典型80~150 μrad）
>
> σ_R：距离方向不确定性（宽视场无测距能力，通常设为100~500 km）

Cueing预测与协方差传播

> **运动模型状态预测：**
>
> 匀速模型(CV)状态向量：
>
> x = \[x, y, z, vx, vy, vz\]^T
>
> 状态转移矩阵Φ：
>
> x y z vx vy vz
>
> x \[ 1 0 0 Δt 0 0 \]
>
> y \[ 0 1 0 0 Δt 0 \]
>
> z \[ 0 0 1 0 0 Δt \]
>
> vx \[ 0 0 0 1 0 0 \]
>
> vy \[ 0 0 0 0 1 0 \]
>
> vz \[ 0 0 0 0 0 1 \]
>
> 匀加速模型(CA)状态向量：
>
> x_CA = \[x, y, z, vx, vy, vz, ax, ay, az\]^T
>
> 状态转移矩阵Φ_CA：
>
> Φ_CA = \[ I3 Δt·I3 (1/2)Δt^2·I3 \]
>
> \[ 03 I3 Δt·I3 \]
>
> \[ 03 03 I3 \]
>
> 其中I3为3×3单位矩阵，03为3×3零矩阵
>
> **协方差传播：**
>
> 预测协方差：
>
> P(t + Δt) = Φ · P1 · Φ^T + Q
>
> 其中：
>
> P1: 初始测量协方差 (来自LOS重建)
>
> Φ: 状态转移矩阵
>
> Q: 过程噪声协方差矩阵
>
> 得到3σ不确定性椭球，通过对P进行特征值分解得到长、中、短半轴。
>
> 过程噪声Q (连续白噪声离散化)：
>
> Q = σ_a^2 · \[ (Δt^4/4)·I3 (Δt^3/2)·I3 \]
>
> \[ (Δt^3/2)·I3 Δt^2·I3 \]
>
> 参数：
>
> σ_a: 机动加速度标准差 (高超音速目标: 10~50 m/s²)

ATP快速姿态机动（Acquisition, Tracking, Pointing）

> 将控制镜头转动的ATP快速姿态机动过程，看做一个固定时延+高斯误差的过程。
>
> 设置时延T_slew，单位秒，如2秒；
>
> 设置指向误差ε_pointing，如N（0,75μrad）。

窄视场快速捕获(Reacquisition)

> 由于ATP指向误差和预测不确定性，目标可能偏离窄视场中心，需要快速捕获算法。
>
> 搜索区域：
>
> R_search = 1.5 × r_3σ (3σ椭圆半径的1.5倍)
>
> 通过对宽视场目标灰度建立模板，进行模板匹配，实现窄视场快速捕获。
>
> NCC(x,y) = Σ\[T(i,j)-T̄\]·\[I(x+i,y+j)-Ī_x,y\] / √{Σ\[T(i,j)-T̄\]²·Σ\[I(x+i,y+j)-Ī_x,y\]²}
>
> 参数：
>
> T(i,j)：从宽视场最后1~2帧提取的目标模板（通常15×15~31×31像素）
>
> T̄：模板灰度均值
>
> I(x+i,y+j) ：窄视场当前帧在(x,y)位置的子图像
>
> Ī：搜索子区灰度均值
>
> 匹配准则：
>
> (x\*, y\*) = argmax NCC(x,y)
>
> 置信度：
>
> C_template = NCC(x\*, y\*)
>
> 要求：C_template \> 0.7

## 数据要求

### 输入数据

输入数据如下表所示。

表 3 传感器配置参数

| **序号** | **名称** | **数据类型** | **单位** | **取值范围** | **说明** |
|:--:|:--:|:--:|:--:|:--:|:--:|
| 1 | wide_field_fov | tuple | degree | (1-20, 1-20) | 宽视场视场角 (方位角×俯仰角) |
| 2 | narrow_field_fov | tuple | degree | (0.1-5, 0.1-5) | 窄视场视场角 (方位角×俯仰角) |
| 3 | pixel_width | int | pixel | 256-2048 | 图像宽度 |
| 4 | pixel_height | int | pixel | 256-2048 | 图像高度 |
| 5 | pixel_size | float | μm | 10-50 | 像元物理尺寸 |
| 6 | focal_length | float | mm | 100-1000 | 焦距 |
| 7 | quantum_efficiency | float | \- | 0.5-0.95 | 量子效率 |
| 8 | read_noise | float | e⁻ | 5-100 | 读出噪声 (电子数) |
| 9 | dark_current | float | e⁻/s | 10-500 | 暗电流 |
| 10 | background_radiance | float | W/(m²·sr·μm) | 0.01-0.1 | 背景辐射亮度 |

表 4 目标参数

| **序号** | **名称** | **数据类型** | **单位** | **取值范围** | **说明** |
|:--:|:--:|:--:|:--:|:--:|:--:|
| 1 | target_id | str | \- | \- | 目标唯一标识符 |
| 2 | initial_position | ndarray | m | \- | 初始位置 (x, y, z) ECI坐标系 |
| 3 | initial_velocity | ndarray | m/s | \- | 初始速度 (vx, vy, vz) |
| 4 | target_type | str | \- | missile/aircraft/satellite | 目标类型 |
| 5 | temperature | float | K | 300-3000 | 目标温度 |
| 6 | emissivity | float | \- | 0.1-1.0 | 发射率 |
| 7 | effective_area | float | m² | 0.01-10 | 有效辐射面积 |

表 5 环境参数

| **序号** | **名称** | **数据类型** | **单位** | **取值范围** | **说明** |
|:--:|:--:|:--:|:--:|:--:|:--:|
| 1 | temperature | float | K | 200-320 | 大气温度 |
| 2 | pressure | float | Pa | 50000-110000 | 大气压强 |
| 3 | humidity | float | % | 0-100 | 相对湿度 |
| 4 | visibility | float | km | 0.1-50 | 能见度 |
| 5 | cloud_coverage | float | \- | 0-1.0 | 云覆盖率 (0=晴天, 1=全覆盖) |
| 6 | background_temperature | float | K | 200-320 | 背景温度 |

### 中间数据

表 6 TBD (Track-Before-Detect) 参数

| **序号** | **名称** | **数据类型** | **单位** | **取值范围** | **说明** |
|:--:|:--:|:--:|:--:|:--:|:--:|
| 1 | pipeline_length | int | frame | 3-7 | 管道滤波帧数 |
| 2 | k_threshold | float | σ | 3-6 | 检测阈值 (标准差倍数) |
| 3 | max_velocity | float | pixel/frame | 5-20 | 最大速度搜索范围 |
| 4 | velocity_bins | int | \- | 11-51 | 速度网格离散化数量 |
| 5 | dp_alpha | float | \- | 0.5-1.0 | DP-TBD折扣因子 |
| 6 | nms_threshold | float | \- | 0.3-0.8 | 非极大值抑制阈值 |

表 7 LOS (Line-of-Sight) 重建参数

| **序号** | **名称** | **数据类型** | **单位** | **取值范围** | **说明** |
|:--:|:--:|:--:|:--:|:--:|:--:|
| 1 | sensor_quality | str | \- | standard/high/premium | 传感器质量等级 |
| 2 | pixel_accuracy | float | pixel | 0.1-1.0 | 像元定位精度 |
| 3 | calibration_residual | float | μrad | 10-200 | 标定残差 |
| 4 | range_estimate | float | m | 100e3-5000e3 | 距离估计值 |

表 8 Cueing 预测参数

<table>
<colgroup>
<col style="width: 6%" />
<col style="width: 27%" />
<col style="width: 11%" />
<col style="width: 12%" />
<col style="width: 24%" />
<col style="width: 17%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>序号</strong></th>
<th style="text-align: center;"><strong>名称</strong></th>
<th style="text-align: center;"><strong>数据类型</strong></th>
<th style="text-align: center;"><strong>单位</strong></th>
<th style="text-align: center;"><strong>取值范围</strong></th>
<th style="text-align: center;"><strong>说明</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;">1</td>
<td style="text-align: center;">motion_model</td>
<td style="text-align: center;">str</td>
<td style="text-align: center;">-</td>
<td style="text-align: center;">CV/CA</td>
<td style="text-align: center;">运动模型 (匀速/匀加速)</td>
</tr>
<tr>
<td style="text-align: center;">2</td>
<td style="text-align: center;">target_type</td>
<td style="text-align: center;">str</td>
<td style="text-align: center;">-</td>
<td style="text-align: center;"><p>ballistic/</p>
<p>hypersonic/aircraft</p></td>
<td style="text-align: center;">目标类型</td>
</tr>
<tr>
<td style="text-align: center;">3</td>
<td style="text-align: center;">prediction_time_dt</td>
<td style="text-align: center;">float</td>
<td style="text-align: center;">s</td>
<td style="text-align: center;">0.5-3.0</td>
<td style="text-align: center;">预测时间间隔</td>
</tr>
<tr>
<td style="text-align: center;">4</td>
<td style="text-align: center;">process_noise_sigma</td>
<td style="text-align: center;">float</td>
<td style="text-align: center;">m/s²</td>
<td style="text-align: center;">10-100</td>
<td style="text-align: center;">过程噪声标准差</td>
</tr>
<tr>
<td style="text-align: center;">5</td>
<td style="text-align: center;">confidence_level</td>
<td style="text-align: center;">float</td>
<td style="text-align: center;">-</td>
<td style="text-align: center;">0.95-0.999</td>
<td style="text-align: center;">置信度 (3σ=99.7%)</td>
</tr>
</tbody>
</table>

表 9 ATP (Attitude Pointing) 控制参数

| **序号** |  **名称**  | **数据类型** | **单位** | **取值范围** |   **说明**   |
|:--------:|:----------:|:------------:|:--------:|:------------:|:------------:|
|    1     |   T_slew   |    float     |    s     |      \-      |   转向时延   |
|    2     | ε_pointing |    float     |   rad    |      \-      | 角度指向误差 |

表 10 窄视场捕获参数

<table style="width:100%;">
<colgroup>
<col style="width: 6%" />
<col style="width: 31%" />
<col style="width: 9%" />
<col style="width: 12%" />
<col style="width: 29%" />
<col style="width: 11%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;"><strong>序号</strong></th>
<th style="text-align: center;"><strong>名称</strong></th>
<th style="text-align: center;"><strong>数据类型</strong></th>
<th style="text-align: center;"><strong>单位</strong></th>
<th style="text-align: center;"><strong>取值范围</strong></th>
<th style="text-align: center;"><strong>说明</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;">1</td>
<td style="text-align: center;">acquisition_mode</td>
<td style="text-align: center;">str</td>
<td style="text-align: center;">-</td>
<td style="text-align: center;"><p>TEMPLATE/</p>
<p>CENTROID/HYBRID</p></td>
<td style="text-align: center;">捕获模式</td>
</tr>
<tr>
<td style="text-align: center;">2</td>
<td style="text-align: center;">template_size</td>
<td style="text-align: center;">int</td>
<td style="text-align: center;">pixel</td>
<td style="text-align: center;">16-64</td>
<td style="text-align: center;">模板尺寸</td>
</tr>
<tr>
<td style="text-align: center;">3</td>
<td style="text-align: center;">search_radius_multiplier</td>
<td style="text-align: center;">float</td>
<td style="text-align: center;">-</td>
<td style="text-align: center;">1.0-3.0</td>
<td style="text-align: center;">搜索半径系数 (×3σ)</td>
</tr>
<tr>
<td style="text-align: center;">4</td>
<td style="text-align: center;">ncc_threshold</td>
<td style="text-align: center;">float</td>
<td style="text-align: center;">-</td>
<td style="text-align: center;">0.5-0.9</td>
<td style="text-align: center;">归一化互相关阈值</td>
</tr>
<tr>
<td style="text-align: center;">5</td>
<td style="text-align: center;">k_narrow</td>
<td style="text-align: center;">float</td>
<td style="text-align: center;">σ</td>
<td style="text-align: center;">5-7</td>
<td style="text-align: center;">窄视场检测阈值</td>
</tr>
<tr>
<td style="text-align: center;">6</td>
<td style="text-align: center;">centroid_window_size</td>
<td style="text-align: center;">int</td>
<td style="text-align: center;">pixel</td>
<td style="text-align: center;">20-100</td>
<td style="text-align: center;">质心窗口尺寸</td>
</tr>
<tr>
<td style="text-align: center;">7</td>
<td style="text-align: center;">acquisition_timeout</td>
<td style="text-align: center;">float</td>
<td style="text-align: center;">ms</td>
<td style="text-align: center;">100-500</td>
<td style="text-align: center;">捕获超时时间</td>
</tr>
</tbody>
</table>

### 输出数据

表 11 检测结果

| **序号** | **名称** | **数据类型** | **单位** | **说明** |
|:--:|:--:|:--:|:--:|:--:|
| 1 | detection_pixel | tuple | pixel | 检测到的目标像元坐标 (x, y) |
| 2 | accumulated_energy | float | \- | TBD累积能量 |
| 3 | detection_confidence | float | \- | 检测置信度 (0-1) |
| 4 | snr | float | dB | 信噪比 |
| 5 | detection_time | float | s | 检测时刻 |

表 12 视线重建结果

| **序号** |       **名称**        | **数据类型** | **单位** |        **说明**        |
|:--------:|:---------------------:|:------------:|:--------:|:----------------------:|
|    1     |      azimuth_rad      |    float     |   rad    |   方位角 (ECI坐标系)   |
|    2     |     elevation_rad     |    float     |   rad    |   俯仰角 (ECI坐标系)   |
|    3     |      los_vector       |   ndarray    |    \-    | 归一化视线向量 \[3×1\] |
|    4     |     covariance_P1     |   ndarray    | rad²/m²  | 测量协方差矩阵 \[3×3\] |
|    5     | angular_accuracy_urad |    float     |   μrad   |      角度测量精度      |

表 13 预测结果

| **序号** | **名称** | **数据类型** | **单位** | **说明** |
|:--:|:--:|:--:|:--:|:--:|
| 1 | predicted_position | ndarray | m | 预测位置 \[x, y, z\] |
| 2 | predicted_velocity | ndarray | m/s | 预测速度 \[vx, vy, vz\] |
| 3 | predicted_covariance | ndarray | m²/(m/s)² | 预测协方差矩阵 \[6×6\] |
| 4 | uncertainty_ellipse_3sigma | dict | \- | 3σ不确定性椭圆参数 |
| 5 | semi_major_m | float | m | 长半轴 |
| 6 | semi_minor_m | float | m | 短半轴 |
| 7 | orientation_deg | float | deg | 方向角 |
| 8 | prediction_confidence | float | \- | 预测置信度 |

表 14 指向结果

| **序号** | **名称** | **数据类型** | **单位** | **说明** |
|:--:|:--:|:--:|:--:|:--:|
| 1 | target_azimuth_deg | float | deg | 目标方位角 |
| 2 | target_elevation_deg | float | deg | 目标俯仰角 |
| 3 | slew_time | float | s | 实际机动时间 |
| 4 | pointing_error_deg | float | deg | 指向误差 |
| 5 | is_stable | bool | \- | 是否达到稳定 |
| 6 | angular_velocity | ndarray | rad/s | 当前角速度 \[ωx, ωy, ωz\] |

表 15 捕获结果

| **序号** | **名称** | **数据类型** | **单位** | **说明** |
|:--:|:--:|:--:|:--:|:--:|
| 1 | acquisition_success | bool | \- | 是否捕获成功 |
| 2 | acquired_pixel | tuple | pixel | 捕获到的目标像元 (x, y) |
| 3 | acquisition_confidence | float | \- | 捕获置信度 |
| 4 | acquisition_method | str | \- | 使用的方法 (TEMPLATE/CENTROID/HYBRID) |
| 5 | pixel_error | float | pixel | 与预测位置的偏差 |
| 6 | acquisition_time_ms | float | ms | 捕获耗时 |

表 16 跟踪结果

| **序号** |     **名称**      | **数据类型** | **单位** |        **说明**         |
|:--------:|:-----------------:|:------------:|:--------:|:-----------------------:|
|    1     |     target_id     |    string    |    \-    |     跟踪目标实体id      |
|    2     | position_estimate |   ndarray    |    m     |  位置估计 \[x, y, z\]   |
|    3     | velocity_estimate |   ndarray    |   m/s    | 速度估计 \[vx, vy, vz\] |

## 工作流程

<img src="红外模型1205-V3.0-media/media/image5.png" style="width:1.77917in;height:9.00764in" />

图 1红外探测传感器模型工作流程图

红外探测传感器模型工作流程描述如下：

a\) 检查传感器是否开机：通过属性判断传感器是否处于开机状态，若关闭则直接返回成功。

b\) 获取父单位位置和姿态：获取传感器的位置（ECR 坐标）和姿态（航向、俯仰、滚转）。

c\) 转换为经纬高坐标：调用坐标系转换算法将 ECR 坐标转换为经纬高（LLA）坐标。

d\) 获取气象影响比例：查询环境参数，并计算红外探测衰减比例。

e\) 调整探测阈值：根据宽窄视场模式调整探测阈值。

f\) 获取探测半径内目标：获取探测半径内的所有单位信息。

g\) 遍历目标，判断敌方目标：对每个目标，检查其关系，仅处理敌方目标。

h\) 计算方位角和俯仰角：计算目标相对于传感器的方位角和俯仰角。

i\) 查询目标红外辐射强度：查询目标的红外辐射强度。

j\) 设置输入数据：构造输入数据结构体，包含传感器状态、目标列表等。

k\) 执行探测算法：执行宽窄视场联合探测算法。

l\) 获取输出数据：获取探测结果（目标ID、方位角、距离等）。

m\) 生成跟踪记录：将探测结果送往数据处理器，生成跟踪记录。

n\) 检查是否为新目标：检查目标是否为首次探测，若是则记录目标ID并触发事件通知。

o\) 更新仿真时间：仿真一个周期完成，更新模型内仿真时间。

p\) 返回成功：返回探测完成。

## 模拟卫星对目标的探测与跟踪能力

卫星对目标的探测与跟踪能力围绕“探测范围生成→目标可探测性与动态探测结果计算→观测误差模拟”三大核心模块展开，各模块通过坐标统一、数据交互形成闭环仿真链路。系统算法整体链路如下：

![](红外模型1205-V3.0-media/media/image6.emf)

图 1 卫星对目标的探测与跟踪能力算法流程

### 探测设备可观测范围生成

探测设备可观测范围生成主要计算宽视场扫描探测器（WFOV）和窄视场跟踪探测器（NFOV）的空间覆盖范围，并排除地球遮挡、大气边界外的不可观测区域。通过卫星轨道与姿态计算、探测器视场边界计算、地球遮挡与大气边界过滤三部分计算探测设备可观测范围。

（1）卫星轨道与姿态计算

卫星的位置/姿态直接决定探测器视场的空间位置，需先通过轨道模型计算卫星状态。

1）算法逻辑

a）输入卫星轨道六要素（半长轴、偏心率、轨道倾角、升交点赤经、近地点幅角、真近点角）和历元时间；

b）计算任意时刻卫星的位置和速度；

c）通过卫星姿态控制系统模型，输出姿态四元数，转换为姿态旋转矩阵。

2）核心公式

先计算轨道平均角速度$`n = \sqrt{\frac{\mu}{a^{3}}}\left( \mu = 3.986 \times 10^{14}\frac{m^{3}}{s^{2}} \right)`$，再通过真近点角<img src="红外模型1205-V3.0-media/media/image7.png" style="width:0.11917in;height:0.19691in" />计算轨道平面内位置：$`r = \frac{a\left( 1 - e^{2} \right)}{1 + e\cos v}`$，$`{\overset{⃑}{r}}_{orb} = \left\lbrack r\cos{v,r\sin v},0 \right\rbrack^{T}`$，最后通过轨道旋转矩阵M<sub>orb</sub>（含倾角$`i`$、升交点赤经$`\Omega`$、近地点幅角$`\omega`$)转换到J2000系：

``` math
{\overset{⃑}{r}}_{orb}(t) = M_{orb} \bullet {\overset{⃑}{r}}_{orb}
```

姿态旋转矩阵（四元数转矩阵）：

<img src="红外模型1205-V3.0-media/media/image8.png" style="width:4.1503in;height:0.69305in" />

（2）探测器视场边界计算

根据探测器类型（宽/窄视场），计算视场在空间中的覆盖范围，输出视场边界的方向向量。

1）算法逻辑

a）定义探测器视场参数：WFOV为圆锥/矩形视场（半顶角$`\theta_{W}`$，如10°-30°），NFOV为小角度圆锥视场（半顶角$`\theta_{N}`$，如0.1°-1°）；

b）在探测器坐标系中生成视场边界采样点（如圆锥视场取方位角$`\phi \in \left\lbrack 0,2\pi) \right.\`$、俯仰角$`\phi \in \left\lbrack 0,\theta_{\frac{W}{N}}) \right.\`$的均匀采样点）；

c）通过姿态旋转矩阵M<sub>att</sub>将探测器系下的边界点转换到J2000系，得到视场边界的空间方向向量$`\overset{⃑}{u}FOV(t)`$。

2）核心公式

探测器坐标系下圆锥视场边界点向量：

对任意采样方位角$`\phi`$、俯仰角$`\theta`$，边界点向量为：

``` math
{\overset{⃑}{u}}_{\det} = \left\lbrack \sin{\theta\cos{\phi,\sin{\theta\sin{\phi,\cos\theta}}}} \right\rbrack^{T}
```

视场边界向量：

``` math
\overset{⃑}{u}FOV(t) = M_{att}(t) \bullet {\overset{⃑}{u}}_{\det}
```

（3）地球遮挡与大气边界过滤

地球遮挡与大气边界过滤主要计算排除卫星视场中被地球遮挡或超出大气红外探测有效范围的区域。

1）算法逻辑

a）计算地球遮挡角：卫星到地心的距离为$`R_{sat} = \left\| {\overset{⃑}{r}}_{sat}(t) \right\|`$，地球半径R<sub>E</sub> = 6371km，遮挡角$`\varnothing_{occ} = arc\sin\left( \frac{R_{E}}{R_{sat}} \right)`$；

b）对任意视场边界点$`{\overset{⃑}{u}}_{FOV}`$，计算卫星-地心-边界点的夹角$`\varnothing = arc\cos\left( \frac{{\overset{⃑}{r}}_{sat} \bullet \left( R_{sat}{\overset{⃑}{u}}_{FOV} \right)}{R_{sat}^{2}} \right)`$：若$`\varnothing < \varnothing_{occ}`$，则该点被地球遮挡，排除；

c）大气边界过滤：设定大气顶层高度H<sub>atm</sub>（如100km），对视场中目标高度h \< H<sub>atm</sub>的区域，考虑大气红外吸收（通过透过率阈值过滤）。

2）核心公式

地球遮挡判断条件：

若$`arc\cos{\left( \frac{{\overset{⃑}{r}}_{sat} \bullet \left( R_{sat}{\overset{⃑}{u}}_{FOV} \right)}{R_{sat}^{2}} \right) < arc\sin\left( \frac{R_{E}}{R_{sat}} \right)} \rightarrow`$遮挡，不可观测

### 目标可探测性与动态探测结果生成

目标可探测性与动态探测结果生成主要计算目标的动态探测结果（位置、距离、角度）和红外可探测性（基于信噪比SNR判断）。

（1）目标动态探测结果计算

目标包括弹道导弹、高超声速滑翔体等高速目标，基于窄视场探测的角度结果，计算Kalman增益，通过修正获得目标的探测位置和速度。

1）算法逻辑

EKF在窄视场成功捕获目标后立即启动，并以50–200 Hz的帧率执行以下6步闭环，直至目标消失或被拦截：

1\. 状态预测：基于匀加速（CA）运动模型，将上一时刻的最优状态估计外推至当前帧时刻，同时考虑目标可能的机动。

2\. 量测获取：接收窄视场红外传感器输出的高精度方位/俯仰角测量（典型精度15–25 μrad）。

3\. 理论量测计算：根据预测状态和卫星实时位置，计算在当前预测状态下传感器应观测到的理论视线角度。

4\. 新息生成：计算实测角度与理论角度之差（新息），作为状态估计误差的直接体现。

5\. 最优融合更新：通过卡尔曼增益对预测状态和新息进行加权融合，得到当前时刻的最优状态估计，同时更新状态协方差矩阵。

6\. 航迹维持与输出：执行波门关联判定本帧量测是否属于当前航迹；将成熟航迹的最优状态及协方差周期性下发火控系统，用于拦截导引。

2）核心公式

a）状态空间模型：

状态向量：

xk = \[x, y, z, vx, vy, vz, ax,ay,az\]^T

地心惯性系位置(m)、速度(m/s)、加速度(m/s²)，包含加速度项以适应高超音速目标的剧烈机动。

状态预测方程：

xk+1 = F · xk + wk

参数：

F: 状态转移矩阵 (同Cueing的Φ)

wk ~ N(0, Q): 过程噪声；

依据运动学方程进行时间外推。

观测方程(非线性)：

zk = h(xk) + vk

角度测量：

h(x) = \[arctan(y/x), arctan(z/√(x²+y²))\]^T

其中：vk ~ N(0, R)为测量噪声。

窄视场实时输出的绝对方位角与俯仰角，唯一外部观测输入，仅含角度信息。

b）EKF递推公式：

协方差预测：

x̂k\|k-1 = F · x̂k-1\|k-1

Pk\|k-1 = F · Pk-1\|k-1 · F^T + Q

Q为过程噪声协方差,描述目标机动不确定性，防止滤波器过度自信。

更新步骤：

计算观测雅可比矩阵：

Hk = ∂h/∂x \|x̂k\|k-1

仅前三列（位置）非零，每帧实时解析计算 \| 将非线性观测方程在预测点线性化。

新息(Innovation)：

yk = zk - h(x̂k\|k-1)

量化预测状态与实际观测的偏差。

新息协方差：

Sk = Hk · Pk\|k-1 · Hk^T + R

综合预测与量测不确定性在角度域的表现。

Kalman增益：

Kk = Pk\|k-1 · Hk^T · Sk^-1

确定预测与量测的最优加权比例。

状态更新：

x̂k\|k = x̂k\|k-1 + Kk · yk

Pk\|k = (I - Kk·Hk) · Pk\|k-1

生成当前时刻最优状态估计。

c）波门关联(Gating)：

Mahalanobis距离：

d² = yk^T · Sk^-1 · yk

3σ波门判据：

d² \< χ²(n, 0.997)

防止量测-航迹错配，保障饱和攻击下的航迹纯度。

（2）目标红外可探测性计算（核心：SNR模型）

红外探测的本质是“目标辐射信号能否超过探测器噪声阈值”，通过计算信噪比（SNR） 判断可探测性（通常SNR≥5视为可探测）。

1）算法逻辑

a）目标红外辐射建模：计算目标在红外波段（3-5μm或8-14μm）的辐射通量；

b）大气衰减建模：通过大气透过率修正辐射通量（考虑水汽、CO₂吸收）；

c）探测器响应建模：计算探测器接收到的信号功率，再结合探测器噪声（光子噪声、热噪声、读出噪声）计算SNR；

d）可探测性判断：若SNR≥阈值SNR<sub>th</sub>，则目标可探测。

2）核心公式

a）目标自身辐射通量（斯特藩-玻尔兹曼定律，简化为黑体辐射）：

``` math
\Phi_{tar} = \varepsilon \bullet \sigma \bullet T_{tar}^{4} \bullet A_{tar} \bullet \cos\theta_{ops}
```

其中：$`\varepsilon`$为目标发射率（0.1-0.9，金属目标≈0.3），$`\sigma = 5.76 \bullet 10^{- 8}W/(m^{2} \bullet k^{4})`$为斯特藩-玻尔兹曼常数，$`T_{tar}`$为目标表面温度（高超声速目标≈1000-2000K），$`A_{tar}`$为目标辐射面积（m²），$`\theta_{obs}`$为观测角（目标法线与探测器视线的夹角）。

b）大气衰减后的辐射通量（朗伯-比尔定律）：

``` math
\Phi_{atm} = \Phi_{tar} \bullet \tau(\lambda,d)
```

其中：$`\tau(\lambda,d)`$为大气透过率（依赖红外波段$`\lambda`$和目标-卫星距离d，通过LOWTRAN/MODTRAN简化模型计算，如3-5μm波段d=1000km时<img src="红外模型1205-V3.0-media/media/image4.png" style="height:0.139in" />≈0.6）。

c）探测器接收功率：

``` math
P_{sig} = \frac{\Phi_{atm} \bullet A_{\det} \bullet \eta_{\det}}{d^{2}}
```

其中：A<sub>det</sub>为探测器光敏面面积（m²），$`\eta_{\det}`$为探测器量子效率（0.5-0.8）。

d）探测器噪声功率（总噪声为各噪声的均方根）：

光子噪声：$`N_{ph} = \sqrt{2 \bullet q \bullet P_{sig} \bullet t_{int}}`$（$`q = 1.6 \bullet 10^{- 19}`$C为电子电荷，$`t_{int}`$为积分时间）；

热噪声：$`N_{th} = \sqrt{\frac{{4k}_{B}T_{\det}B}{R_{\det}}}`$（$`k_{B} = 1.38*10^{- 23}`$J/K为玻尔兹曼常数，$`T_{\det}`$为探测器工作温度，B为带宽，$`R_{\det}`$为探测器内阻）；

读出噪声：N<sub>read</sub>（固定值，如10-50 e⁻/帧）；

总噪声：$`N_{total} = \sqrt{N_{ph}^{2} + N_{th}^{2} + N_{read}^{2}}`$。

e）信噪比与可探测性判断：

``` math
SNR = \frac{P_{sig} \bullet t_{int}}{N_{total}}，若SNR \geq {SNR}_{th} \Longrightarrow 可探测
```

### 高速动态目标观测误差模拟

高速动态目标观测误差源分为5类。

<table style="width:98%;">
<colgroup>
<col style="width: 16%" />
<col style="width: 19%" />
<col style="width: 17%" />
<col style="width: 45%" />
</colgroup>
<thead>
<tr>
<th style="text-align: center;">误差类型</th>
<th style="text-align: center;">物理成因</th>
<th style="text-align: center;">建模方法</th>
<th style="text-align: center;">核心公式</th>
</tr>
</thead>
<tbody>
<tr>
<td style="text-align: center;">卫星轨道误差</td>
<td style="text-align: center;">轨道预报摄动（太阳辐射压、大气阻力）</td>
<td style="text-align: center;">高斯分布+协方差传播</td>
<td style="text-align: center;"><p><span class="math display"><em>Δ</em><em>r⃗</em><sub><em>s</em><em>a</em><em>t</em></sub> ∼ <em>Ν</em>(0, <em>Σ</em><sub><em>o</em><em>r</em><em>b</em></sub>)</span></p>
<p><span class="math display">(<em>Σ</em><sub><em>o</em><em>r</em><em>b</em></sub><em>为</em><em>轨</em><em>道</em><em>误</em><em>差</em><em>协</em><em>方</em><em>差</em><em>矩</em><em>阵</em>）</span></p></td>
</tr>
<tr>
<td style="text-align: center;">卫星姿态误差</td>
<td style="text-align: center;">姿态传感器噪声（陀螺漂移、星敏误差）</td>
<td style="text-align: center;">高斯分布+一阶马尔可夫过程</td>
<td style="text-align: center;"><p><span class="math display"><em>Δ</em><em>α</em>，<em>Δ</em><em>β</em> ∼ <em>Ν</em>(0, <em>σ</em><sub><em>a</em><em>t</em><em>t</em></sub><sup>2</sup>)</span></p>
<p><span class="math display">(<em>σ</em><sub><em>a</em><em>t</em><em>t</em></sub><sup>2</sup><em>为</em><em>姿</em><em>态</em><em>角</em><em>误</em><em>差</em><em>标</em><em>准</em><em>差</em>, <em>如</em>0.01<sup>∘</sup>）</span></p></td>
</tr>
<tr>
<td style="text-align: center;">探测器视场误差</td>
<td style="text-align: center;">像元错位、光学畸变</td>
<td style="text-align: center;">随机偏移+系统偏差</td>
<td style="text-align: center;"><p><span class="math inline"><em>Δ</em><em>θ</em><sub><em>F</em><em>O</em><em>V</em></sub> = <em>Δ</em><em>θ</em><sub><em>r</em><em>a</em><em>n</em><em>d</em></sub></span>+<span class="math inline"><em>Δ</em><em>θ</em><sub><em>s</em><em>y</em><em>s</em></sub></span></p>
<p><span class="math display">(<em>Δ</em><em>θ</em><sub><em>r</em><em>a</em><em>n</em><em>d</em></sub><em>为</em><em>随</em><em>机</em><em>误</em><em>差</em>，<em>Δ</em><em>θ</em><sub><em>s</em><em>y</em><em>s</em></sub><em>为</em><em>系</em><em>统</em><em>偏</em><em>差</em>）</span></p></td>
</tr>
<tr>
<td style="text-align: center;">大气折射误差</td>
<td style="text-align: center;">大气密度梯度导致的光线偏折</td>
<td style="text-align: center;">标准大气模型（US Standard Atmosphere）</td>
<td style="text-align: center;">红外波段折射角<span class="math inline">$\Delta\theta_{refr} = \frac{1.5 \times 10^{- 6}}{d\ \cos\beta}$</span>(<span class="math inline"><em>β</em><em>为</em><em>目</em><em>标</em><em>俯</em><em>仰</em><em>角</em>）</span></td>
</tr>
<tr>
<td style="text-align: center;">动态滞后误差</td>
<td style="text-align: center;">探测器响应延迟（高速目标运动快）</td>
<td style="text-align: center;">一阶系统滞后模型</td>
<td style="text-align: center;"><p><span class="math display">$$\Delta\theta_{lag} = \frac{\omega_{tar}}{2\pi f_{det}}$$</span></p>
<p><span class="math display">(<em>ω</em><sub><em>t</em><em>a</em><em>r</em></sub><em>为</em><em>目</em><em>标</em><em>角</em><em>速</em><em>度</em>，<em>f</em><sub><em>d</em><em>e</em><em>t</em></sub><em>为</em><em>探</em><em>测</em><em>器</em><em>带</em><em>宽</em>)</span></p></td>
</tr>
</tbody>
</table>

将各类误差按“加法合成”（角度误差）或“乘法合成”（距离误差）叠加到无误差观测值上，最终输出带误差的探测参数：

带误差的方位角：$`\alpha_{meas} = \alpha_{true} + \Delta\alpha_{orb} + \Delta\alpha_{att} + \Delta\alpha_{refr} + \Delta\alpha_{lag}`$

带误差的俯仰角：$`\beta_{meas} = \beta_{true} + \Delta\beta_{orb} + \Delta\beta_{att} + \Delta\beta_{refr} + \Delta\beta_{lag}`$

带误差的距离：$`d_{meas} = d_{true} \bullet （1 + \mathrm{\Delta}d_{rand}）`$（$`\mathrm{\Delta}d_{rand}`$为距离随机误差，如0.1%）

## 模拟宽视场扫描探测器和窄视场跟踪探测的能力

### 宽视场扫描探测器

（1）红外信号处理与目标检测

1）辐射模型：

基于普朗克定律，目标与背景的红外辐射强度差异通过信杂比（SCR）量化：

``` math
SCR = \frac{P_{target} - P_{background}}{\sigma_{background}}
```

其中，$`P_{target}`$和$`P_{background}`$分别为目标与背景的辐射功率，\
``` math
\sigma_{background}
```
为背景噪声标准差。

2）信号传递模型：

像平面上的能量幅度关系通过辐射对比度表示：

``` math
C = \frac{L_{target}.\tau_{atm}.\Omega.\eta}{L_{background}.\tau_{atm}.\Omega.\eta + N_{dark} + N_{shot}}
```

式中，$`L_{target}`$和$`L_{background}`$为目标与背景的光谱辐射亮度，$`\tau_{atm}`$为大气透过率，<img src="红外模型1205-V3.0-media/media/image9.png" style="width:0.123in;height:0.14909in" />为光学系统立体角，$`\eta`$为探测器量子效率，$`N_{dark}`$和$`N_{shot}`$为暗电流与散粒噪声。

（2）空域扫描与几何投影

1）视场覆盖算法：

卫星宽视场（如50°×50°）通过逐行扫描或凝视模式覆盖目标区域，结合卫星轨道参数（如高度、倾角）计算扫描带宽度W：

``` math
W = 2 \bullet R_{earth} \bullet \sin\left( \frac{\theta}{2} \right)
```

其中，$`R_{earth}`$为地球半径，$`\theta`$为视场角。

2）几何投影模型：

目标在像平面的坐标 (u, v)通过透视变换计算：

``` math
\left\{ \begin{array}{r}
u = f.\frac{x}{z} + u_{0} \\
v = f.\frac{x}{z} + v_{0}
\end{array} \right.\ 
```

式中 (x, y, z)为目标在卫星坐标系下的坐标，<img src="红外模型1205-V3.0-media/media/image10.png" style="width:0.1123in;height:0.17647in" />为焦距，(u<sub>0</sub>, v<sub>0</sub>)为像平面中心坐标。

3）多目标检测与聚类

自适应阈值分割：

采用Otsu算法或局部阈值法动态确定检测阈值T，分割目标与背景：

``` math
T = {argmax}_{T}\left\lbrack \sigma_{B}^{2}(T) \right\rbrack
```

其中，$`\sigma_{B}^{2}(T)`$为类间方差。

4）聚类分析：

利用DBSCAN或Mean Shift算法对检测到的目标点进行聚类，形成初始航迹。

### 窄视场跟踪传感器

（1）目标状态估计与滤波

1）状态空间模型：

目标状态向量$`X_{k} = \left\lbrack x,\dot{x},y,\dot{y},z,\dot{z} \right\rbrack^{T}`$，状态转移方程为：$`X_{k} = F \bullet X_{k - 1} + w_{k - 1}`$

其中，F为状态转移矩阵，w为过程噪声（服从零均值高斯分布）。

2）容积卡尔曼滤波（CKF）：

预测步骤：

``` math
{\widehat{X}}_{k}^{-} = F \bullet {\widehat{X}}_{k - 1}P_{k}^{-} = F \bullet P_{k - 1} \bullet F^{T} + Q
```

更新步骤：

``` math
y_{k} = Z_{k} - H \bullet {\widehat{X}}_{k}^{-}S_{k} = H \bullet P_{k}^{-} \bullet H^{T} + RK_{k} = P_{k}^{-} \bullet H^{T} \bullet S_{k}^{- 1}
```

``` math
{\widehat{X}}_{k} = {\widehat{X}}_{k}^{-} + K_{k} \bullet y_{k}P_{k} = \left( I - K_{k} \bullet H \right) \bullet P_{k}^{-}
```

式中，H为观测矩阵，Q和R为过程与测量噪声协方差矩阵。

（2）高超声速目标跟踪优化

1）自适应偏差补偿：

采用变分贝叶斯推断估计测量偏差b<sub>k</sub>，构建增广状态向，$`x_{k}^{a} = \left\lbrack x_{k}^{T},b_{k}^{T} \right\rbrack^{T}`$通过边缘化处理降低计算复杂度。

2）轨迹平滑与约束：

结合容积粒子滤波（CPF）与加速度约束，剔除不合理轨迹段：

``` math
\widehat{X_{k}} = \sum_{i = 1}^{N}{w_{i}^{(k)}.x_{i}^{(k)}}
```

其中，N为粒子数，$`w_{i}^{(k)}`$为粒子权重，通过观测视线-速度矢量双重约束优化权重分布。
