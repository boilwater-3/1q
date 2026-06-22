# 1.1.4.4 SAR雷达组件
## 1.1.4.4.1 接口层
### 1.1.4.4.1.1 仿真流程总控模块
#### 1.1.4.4.1.1.1 功能
串联SAR仿真全链路，依次完成参数输入、波形生成、场景构建、原始回波生成、可选脉冲压缩、聚焦算法处理、频谱计算和结果输出，是控制SAR仿真运转的顶层调度单元。实施审批阶段默认聚焦算法为 RDA；GBP/BP 仅作为小场景高精度扩展算法，不作为 Phase 1 强制交付项。
为了解决平台宏观推演时间尺度（例如 $\ge 0.05$ 秒步长）与雷达微观信号脉冲重复周期（毫秒级）的跨尺度冲突，总控模块采用**"快慢时间双尺度异步解耦"机制**。每个宏观步长内按 PRF 累积脉冲回波并写入**环形脉冲缓冲区（Pulse Ring Buffer）**；SAR 成像处理器按合成孔径时间窗口从缓冲区异步读取连续 $N_{aperture}$ 个脉冲，**不与宏观步长强同步**，自动跨步长拼接。PRF 由方位向采样定理（避免多普勒模糊）独立决定，宏观步长由任务推演精度独立决定，二者解耦。脉冲缓冲区容量按 $2 \times N_{aperture}$ 配置，支持读写并发与覆盖保护。此外，为优化内存，雷达图像等大矩阵统一采用共享内存池或指针引用进行传递，以实现原地（In-place）处理，避免频繁申请和释放大块内存造成的性能开销。

#### 1.1.4.4.1.1.2 算法输入
输入模块：用户交互（控制台）/ 基础计算层各模块
- `start_frequency` (unsigned long / Hz): 线性调频信号起始频率
- `bandwidth` (unsigned long / Hz): 线性调频信号带宽
- `btproduct` (unsigned int / —): 时宽带宽积（BT积）
- `prf` (unsigned int / Hz): 脉冲重复频率，用于确定每个宏观步长内的脉冲数
- `altitude` (double / m): SAR平台飞行高度
- `platform_speed` (double / m/s): 平台飞行速度，用于计算方位向采样
- `beamwidth` (float / rad): 天线方位波束宽度
- `area_azimuth_len` (double / m): 仿真场景方位向长度
- `area_range` (double / m): 仿真场景距离向长度
- `pc_enable` (char (y/n)): 是否启用脉冲压缩
- `norm_enable` (char (y/n)): 是否开启图像幅度预归一化（保留绝对RCS时设为 'n'）
- `phase_reference_mode` (unsigned int): 相位参考模式（0=算法原生, 1=场景中心统一基准）
- `trajectory_fidelity` (unsigned int): 平台轨迹保真度（1=L1, 2=L2, 3=L3，默认=1）

#### 1.1.4.4.1.1.3 算法输出
输出模块：数据访问层（write_data）
- `time_vector` (matrix): 时间向量矩阵
- `chirp` (matrix): 发射线性调频波形
- `match` (matrix): 匹配滤波器时域响应
- `pc_waveform` (matrix): 单路脉冲压缩结果
- `scene` (matrix): 仿真目标场景矩阵
- `radar_image` (matrix): 原始雷达回波图像
- `pc_image` (matrix): 脉冲压缩后雷达图像
- `sar_image` (matrix): 聚焦SAR图像
- `sar_image_fft` (matrix): SAR图像二维频谱
- `chirp_fft` (matrix): 发射波形频谱
- `match_fft` (matrix): 匹配滤波器频谱

#### 1.1.4.4.1.1.4 算法流程
1. 流程启动，输入仿真配置参数（起始频率、带宽、BT积、场景尺寸、平台高度、波束宽度、速度、PRF、处理与归一化开关），进入接口层总控逻辑。
2. 计算当前宏观时间步长 $\Delta T_{macro}$ 内的脉冲发射数。实现时必须保留分数脉冲累积量 $\epsilon_{pulse}$，避免长期 `floor` 截断造成 PRF 漂移：
   $$ N_{burst} = \left\lfloor \Delta T_{macro} \cdot prf + \epsilon_{pulse} \right\rfloor $$
   $$ \epsilon'_{pulse} = \Delta T_{macro} \cdot prf + \epsilon_{pulse} - N_{burst} $$
3. 执行基础数据准备：
   - 调用线性调频波形生成模块，生成时间向量与发射波形；
   - 调用匹配滤波器构造模块，生成匹配滤波器；
   - 调用单路脉冲压缩验证，输出压缩脉冲分辨率用于质量校验。
4. 执行场景与成像处理（异步流水线）：
   - 构建仿真场景矩阵并植入目标；
   - 根据 $N_{burst}$ 计算各脉冲发射位置，并调用SAR场景成像仿真模块生成原始回波；
   - 将生成的 $N_{burst}$ 个脉冲回波**写入环形脉冲缓冲区**（容量 $2 \times N_{aperture}$）；
   - SAR 聚焦处理器异步从缓冲区读取 $N_{aperture}$ 个连续脉冲，跨宏观步长自动拼接；
   - 根据 `pc_enable` 判定是否执行全图脉冲压缩，若启用，则调用全图脉冲压缩处理模块。
5. 判定是否执行图像幅度归一化（依据 `norm_enable` 开关）：
   - 若启用，则在脉冲压缩或原始图像基础上进行最大值归一化，消除列间能量差异；
   - 若不启用，则保留绝对物理RCS幅度以支持精确的雷达目标特征识别。
6. 执行聚焦与频谱分析：
   - 调用已审批的聚焦算法生成SAR图像。Phase 1 默认为 RDA；自动选择模式仅允许在算法能力和测试覆盖齐备后启用；
   - 调用二维FFT生成频谱图用于质量评估。
7. 流程结束，统一输出时域/频域波形、场景、原始图像、压缩图像、聚焦图像与频谱结果。

---

## 1.1.4.4.2 核心处理层
### 1.1.4.4.2.1 SAR场景成像仿真模块
#### 1.1.4.4.2.1.1 功能
模拟SAR平台沿方位向逐列扫描、天线向场景发射线性调频信号并接收目标回波的过程，生成包含目标双曲线响应特征的原始回波矩阵。这里的三种路径仅表示原始回波生成保真度，不等同于后续聚焦算法枚举。

#### 1.1.4.4.2.1.2 算法输入
输入模块：仿真流程总控模块
- `scene` (matrix（复数）): 仿真目标场景矩阵（rows×cols）
- `chirp` (matrix（复数）): 发射线性调频波形
- `variables.beamwidth` (float / rad): 天线方位波束宽度
- `variables.altitude` (double / m): 平台飞行高度
- `variables.platform_speed` (double / m/s): 平台飞行速度
- `variables.prf` (unsigned int / Hz): 脉冲重复频率
- `variables.chirp_samples` (unsigned long / —): 脉冲采样点数
- `variables.signal_distance` (double / m): 脉冲覆盖距离
- `variables.bandwidth` (unsigned long / Hz): 信号带宽
- `echo_model` (unsigned int): 原始回波生成模型选择（1=近似能量投影, 2=时延门植入, 3=频域相位延迟）

#### 1.1.4.4.2.1.3 算法输出
输出模块：仿真流程总控模块
- `radar_image` (matrix（复数）): 原始雷达回波图像（rows×cols）

#### 1.1.4.4.2.1.4 算法流程
1. 流程启动，输入场景矩阵、发射波形、平台速度、PRF及其他参数，进入原始回波生成模型分派逻辑。
2. 执行模型选择判定：
   - 若选择模型1（近似能量投影），计算波束覆盖宽度并在波束覆盖区累加回波，仅用于快速功能冒烟和可视化；
   - 若选择模型2（时延门植入），按几何斜距计算双程传播时延和距离门并植入回波；
   - 若选择模型3（频域相位延迟），逐列执行FFT、频域相位补偿及IFFT还原。
3. 执行统一结果汇聚：
   - 将三种模型路径结果统一写入原始回波矩阵 `radar_image`；
   - 完成成像阶段输出，供后续压缩与聚焦使用。
4. 流程结束，输出原始雷达图像及本轮成像元数据。

#### 1.1.4.4.2.1.5 数学公式
##### 模型1 — 近似能量投影模型（RIA）
方位覆盖半径（米）及对应的列数：
$$ R_{az} = H \cdot \tan\left(\frac{\theta_{bw}}{2}\right) $$
$$ N_{beam} = N_s \cdot \frac{R_{az}}{d_{sig}} $$
各方位位置的回波叠加：
$$ I_{radar}[i][r] += S_{scene}[i+b][k], \quad r = \lfloor\sqrt{b^2 + k^2}\rfloor $$
其中 $b \in [-N_{beam}, N_{beam}]$，$k$ 为距离向索引，$r < N_{rows}$。
（变量说明：$H$：平台高度 [m]；$\theta_{bw}$：天线方位波束宽度 [rad]；$N_s$：脉冲采样点数；$d_{sig}$：信号覆盖距离 [m]）

##### 模型2 — 相干时延门模型
每列中心斜距与双程时延：
$$ R_i = \left\|p_{platform}(t_i) - p_{target}\right\|_2 $$
$$ \tau_i = \frac{2R_i}{c} $$
距离门索引：
$$ n_i = \operatorname{round}(\tau_i f_s) $$
回波能量植入并按双程雷达方程幅度衰减：
$$ I_{radar}[i][n_i:n_i+N_s] += \frac{\sqrt{\sigma}\, s_{chirp}}{R_i^2} \cdot e^{-j\frac{4\pi R_i}{\lambda}} $$

##### 模型3 — 频域相位延迟模型
各列回波相位调制（在频域施加传播时延相位）：
$$ S[i][f_j] = \frac{S[i][f_j]}{R_i^2} \cdot e^{-j 2\pi f_j \tau_i} \cdot e^{-j\frac{4\pi R_i}{\lambda}} $$
其中传播时延：
$$ \tau_i = \frac{2R_i}{c}, \quad \Delta r = \frac{c}{2f_s} $$
频率采样间隔（理论修正）：
$$ \Delta f = \frac{f_s}{N_{rows}} $$
（变量说明：$c=3\times 10^8$ m/s（光速）；$B$：信号带宽 [Hz]；$f_s$：雷达采样率 [Hz]；$\frac{1}{r_i^4}$：距离四次方幅度衰减因子）

---

### 1.1.4.4.2.2 全图脉冲压缩处理模块
#### 1.1.4.4.2.2.1 功能
对原始雷达图像的每一列回波独立实施频域脉冲压缩，利用匹配滤波器核在频域相乘后经逆傅里叶变换还原为时域，显著提升距离向分辨率。

#### 1.1.4.4.2.2.2 算法输入
输入模块：仿真流程总控模块 / 基础计算层匹配滤波器构造模块
- `radar_image` (matrix（复数）): 原始雷达回波图像（rows×cols）
- `match` (matrix（复数）): 匹配滤波器时域响应（长度 kernel_length）

#### 1.1.4.4.2.2.3 算法输出
输出模块：仿真流程总控模块
- `pc_image` (matrix（复数）): 脉冲压缩后雷达图像（rows×cols）

#### 1.1.4.4.2.2.4 算法流程
1. 流程启动，输入原始雷达图像与匹配滤波器，进入逐列压缩逻辑。
2. 执行预处理：
   - 对匹配滤波器进行零填充并计算频域滤波核。
3. 执行列级循环处理：
   - 逐列读取回波数据并零填充；
   - 逐列FFT并与滤波核相乘；
   - 逐列IFFT恢复时域压缩回波；
   - 将压缩结果写回输出图像对应列。
4. 判定循环结束条件，若仍有未处理列则继续循环，否则进入结果输出。
5. 流程结束，输出全图脉冲压缩结果 `pc_image`。

#### 1.1.4.4.2.2.5 数学公式
滤波核零填充与FFT：
$$ H(f) = \text{FFT}\{h(t), L_f\} $$
其中 $h(t)$ 为匹配滤波器时域响应，补零后长度为 $L_f = N_{rows} + N_k$。
各列零填充与FFT：
$$ X_{col}(f) = \text{FFT}\{x_{col}(t), L_f\} $$
频域相乘：
$$ Y_{col}(f) = X_{col}(f) \cdot H(f) $$
逆傅里叶变换还原：
$$ y_{col}(t) = \text{IFFT}\{Y_{col}(f)\}|_{0:N_{rows}} $$
（变量说明：$N_k$：匹配滤波器长度；$L_f$：补零后滤波长度）

---

### 1.1.4.4.2.3 聚焦处理与频谱分析模块
#### 1.1.4.4.2.3.1 功能
对脉冲压缩图像实施已审批的聚焦算法处理，消除距离-方位耦合造成的散焦，生成目标点响应聚焦后的复图像；随后对聚焦图像实施频率中心化的二维傅里叶变换，生成空间频谱用于成像质量评估。Phase 1 默认使用 RDA，GBP/BP 为后续扩展路径。

鉴于GBP算法 $O(N_{pixel} \times N_{pulse})$ 的极端计算复杂度，本模块内置**自适应聚焦策略**：根据场景规模与可用算力自动选择聚焦算法，防止在大场景或算力受限平台上造成仿真主线程的严重阻塞。

/*
 * 【未进行设计需求，不再扩展】
 * 自适应聚焦策略（Auto / 自动选择）不属于原始设计需求范围，已冻结不再扩展。
 * 以下策略表仅作为历史设计记录保留，不作为实施依据；正式运行统一使用显式指定算法。
 */
##### 聚焦算法自适应选择策略

| 条件 | 选择算法 | 说明 |
|------|----------|------|
| Phase 1 或用户强制指定 `algorithm=1` | RDA（Range-Doppler Algorithm） | 审批默认算法，频域快速算法，复杂度 $O(N \log N)$ |
| 场景像素数 $N_{pixel} \le 10^6$ 且并行线程数 $\ge 4$ 且用户强制指定 `algorithm=4` | GBP（全局后向投影） | 小场景高精度扩展算法，Phase 1 不默认启用 |
| 用户强制指定 `algorithm=5` | BP（Backprojection） | 非直线轨迹扩展算法，需配合运动补偿验收 |
| 用户强制指定 `algorithm=0` | 自动选择 | 仅在 RDA/GBP/BP 均有回归测试和性能基准后允许用于正式运行 |

算法编号约定（用于配置与日志）：`0=Auto, 1=RDA, 2=CSA, 3=OmegaK, 4=GBP, 5=BP`。注：`module_design.md` v2.2 已将公共层 `FocusingAlgorithm` 枚举废弃，公共对外改用 `SarPolicyConfig` 布尔开关；此处编号仅作内部 `RecommendedFocusingAlgorithm` 与日志参考。当自动选择逻辑改变用户指定算法时，总控模块应在日志中输出结构化警告，记录请求算法、实际算法、降级原因和场景规模。

#### 1.1.4.4.2.3.2 算法输入
输入模块：仿真流程总控模块
- `radar_image` (matrix（复数）): 脉冲压缩后雷达图像，作为聚焦输入
- `algorithm` (unsigned int): 聚焦算法选择（0=Auto, 1=RDA, 2=CSA, 3=OmegaK, 4=GBP, 5=BP）
- `scene_pixels` (unsigned long): 场景总像素数，用于自动决策
- `num_threads` (int): 可用并行线程数，用于自动决策

#### 1.1.4.4.2.3.3 算法输出
输出模块：仿真流程总控模块
- `sar_image` (matrix（复数）): 聚焦SAR图像（rows×cols）
- `sar_image_fft` (matrix（复数）): SAR聚焦图像的二维空间频谱
- `selected_algorithm` (unsigned int): 实际执行的聚焦算法（0=Auto, 1=RDA, 2=CSA, 3=OmegaK, 4=GBP, 5=BP）
- `phase_reference_applied` (bool): 是否执行了相位重参考
- `image_entropy` (double / —): 图像熵指标，数值越小表示聚焦质量越高
- `radiometric_accuracy_db` (double / dB): 辐射精度（实测RCS与理论RCS偏差），用于评估绝对RCS保真度。未启用辐射定标模块时该值仅作相对精度参考。

#### 1.1.4.4.2.3.4 算法流程
1. 流程启动，输入用于聚焦的雷达图像，初始化聚焦图像矩阵。
2. 执行聚焦算法自适应选择判定：
   - 若 `algorithm=1`，强制使用 RDA；
   - 若 `algorithm=4`，强制使用 GBP；
   - 若 `algorithm=5`，强制使用 BP；
   - 若 `algorithm=0`（自动），根据 $N_{pixel}$、轨迹保真度和可用线程数按策略表判定。
3. 若选择GBP聚焦（开启多线程/GPU并行化加速）：
   - 遍历输出图像各像素；
   - 对每个像素遍历全部方位采样位置并计算距离门索引；
   - 累加对应距离门复回波完成相干叠加。
4. 若选择RDA聚焦：
   - 对脉冲压缩图像执行方位向FFT，转至距离-多普勒域；
   - 执行距离单元徙动校正（RCMC）；
   - 执行方位向匹配滤波与IFFT，完成聚焦。
5. 执行频谱分析：
   - 对聚焦图像进行频率中心化预处理；
   - 执行二维FFT并生成空间频谱图。
6. 执行相位重参考（仅在以下条件满足时）：
   - `phase_reference_mode=1`；或
   - 算法发生切换（GBP↔RDA）；或
   - 跨合成孔径图像拼接。
   - 在 RDA 输出或跨孔径拼接时施加补偿相位 $\phi_{ref} = -4\pi R_0/\lambda$，使图像相位基准对齐到场景中心零多普勒时刻的双程延迟相位。
7. 流程结束，输出聚焦图像 `sar_image` 和频谱图 `sar_image_fft`。

#### 1.1.4.4.2.3.5 数学公式
##### GBP相干积累聚焦
对输出图像中每个像素 $(j_0, k_0)$，遍历全部方位采样位置 $l$，计算该像素至各方位位置的距离门索引，并将对应回波值相干叠加：
$$ I_{SAR}[j_0][k_0] = \sum_{l=0}^{N_{cols}-1} I_{rad}[l]\left[r_{lk_0j_0}\right] $$
其中距离门索引由几何距离决定：
$$ r_{lk_0j_0} = \sqrt{(l - j_0)^2 + k_0^2}, \quad r_{lk_0j_0} < N_{rows} $$
（变量说明：$j_0$：输出图像方位向像素索引；$k_0$：输出图像距离向像素索引；$N_{cols}$：图像方位向列数；$N_{rows}$：图像距离向行数）

##### GBP图像二维频谱（频率中心化）
对图像进行棋盘格相位预调制以实现频率中心化，同时预归一化消除FFT变换尺度因子：
$$ I_{SAR}[i][j] = I_{SAR}[i][j] \cdot (-1)^{i+j} \cdot \frac{1}{N_{rows} \cdot N_{cols}} $$
二维离散傅里叶变换：
$$ F[u][v] = \sum_{i=0}^{N_{cols}-1}\sum_{j=0}^{N_{rows}-1} I_{SAR}[i][j] \cdot e^{-j 2\pi \left(\frac{ui}{N_{cols}} + \frac{vj}{N_{rows}}\right)} $$
（变量说明：$u, v$：方位向和距离向空间频率索引；$(-1)^{i+j}$：频率中心化移位因子）

##### RDA聚焦算法（Phase 1 默认）
RDA 为 Phase 1 审批默认聚焦算法。其输入必须是距离向已压缩的回波矩阵，且平台轨迹默认满足 L1 匀速直线条带模式；L2/L3 轨迹进入 RDA 前必须先声明运动误差处理策略。

距离-多普勒域转换（方位向FFT）：
$$ I_{RD}[r][f_a] = \text{FFT}_{azimuth}\{I_{pc}[r][a]\} $$

距离单元徙动校正（RCMC）— sinc插值：
$$ I_{RD,RCMC}[r][f_a] = I_{RD}[r + \Delta r_{CM}(f_a)][f_a] $$
其中徙动量：
$$ \Delta r_{CM}(f_a) = \frac{\lambda^2 r_0 f_a^2}{8 v^2} $$

方位向匹配滤波：
$$ H_{az}(f_a) = \exp\left(j \frac{\pi f_a^2}{K_a}\right), \quad K_a = \frac{2v^2}{\lambda r_0} $$

方位向IFFT还原聚焦图像：
$$ I_{SAR}[r][a] = \text{IFFT}_{azimuth}\{I_{RD,RCMC}[r][f_a] \cdot H_{az}(f_a)\} $$
（变量说明：$f_a$：方位向多普勒频率 [Hz]；$r_0$：场景中心斜距 [m]；$v$：平台速度 [m/s]；$\lambda$：波长 [m]；$K_a$：多普勒调频率 [Hz/s]）

##### 相位重参考（统一相位基准）
当 `phase_reference_mode=1` 或算法切换 / 跨孔径拼接时，对聚焦图像施加补偿相位，使相位基准对齐到场景中心 $R_0$ 处的零多普勒双程延迟相位：
$$ I'_{SAR}[i][j] = I_{SAR}[i][j] \cdot e^{-j \frac{4\pi R_0}{\lambda}} $$
（变量说明：$R_0$：场景中心斜距 [m]；$\lambda$：波长 [m]；$e^{-j 4\pi R_0/\lambda}$：双程传播延迟对应的相位补偿因子）

该步骤只能统一全局相位基准，不能替代运动补偿、距离徙动校正或空间变化相位误差校正。若不同算法输出存在像素级相位斜坡或残余散焦，必须通过 RCMC、运动补偿或自聚焦模块处理，不能仅靠常数相位重参考掩盖。

##### 图像熵（聚焦质量指标）
将图像幅度归一化为概率分布，计算信息熵。聚焦良好的图像能量集中，熵值较低：
$$ p_{i,j} = \frac{|I_{SAR}[i][j]|^2}{\sum_{i,j} |I_{SAR}[i][j]|^2} $$
$$ H = -\sum_{i,j} p_{i,j} \log_2 p_{i,j} $$

##### 辐射精度（绝对RCS保真度）
对已知RCS的标定目标，比较仿真成像提取的RCS与理论值的偏差：
$$ \sigma_{measured} = \frac{|I_{SAR}[i_{target}][j_{target}]|^2 \cdot R^4}{K_{cal}} $$
$$ \Delta\sigma_{dB} = 10 \log_{10}\left(\frac{\sigma_{measured}}{\sigma_{theory}}\right) \quad [\text{dB}] $$
（变量说明：$K_{cal}$：辐射定标常数；$R$：目标斜距 [m]；$\sigma_{theory}$：理论RCS值 [m²]）

---

## 1.1.4.4.3 基础计算层
### 1.1.4.4.3.1 线性调频波形生成模块
#### 1.1.4.4.3.1.1 功能
以起始频率、带宽和时宽带宽积为参数，推导全部派生仿真参数，生成复数线性调频（LFM/Chirp）波形和对应的时间向量，是SAR仿真信号处理链路的源头。

#### 1.1.4.4.3.1.2 算法输入
输入模块：用户交互（控制台）
- `start_frequency` (unsigned long / Hz): 发射信号起始频率
- `bandwidth` (unsigned long / Hz): 发射信号带宽 $B$
- `btproduct` (unsigned int / —): 时宽带宽积 $BT$

#### 1.1.4.4.3.1.3 算法输出
输出模块：仿真流程总控模块 / 全局仿真参数结构 variables
- `time_vector` (matrix（复数）): 等间隔时间向量，长度 $N_s$
- `chirp` (matrix（复数）): LFM复数波形，长度 $N_s$
- `variables.chirp_samples` (unsigned long / —): 脉冲采样点数 $N_s$
- `variables.signal_distance` (double / m): 脉冲覆盖距离 $d_{sig}$
- `variables.start_frequency` (unsigned long / Hz): 起始频率
- `variables.bandwidth` (unsigned long / Hz): 信号带宽
- `variables.btproduct` (unsigned int / —): 时宽带宽积

#### 1.1.4.4.3.1.4 算法流程
1. 流程启动，输入起始频率、带宽与BT积，进入参数校验逻辑。
2. 执行输入约束检查：若关键参数非法（非正值或超范围）则返回错误并终止。
3. 执行派生参数计算：
   - 计算脉冲时宽、调频斜率、采样率、采样点数与信号覆盖距离；
   - 生成等间隔时间向量。
4. 执行波形构造：
   - 按采样点循环生成LFM复数波形；
   - 写入全局变量结构供后续模块复用。
5. 流程结束，输出 `time_vector` 与 `chirp`。

#### 1.1.4.4.3.1.5 数学公式
派生仿真参数推导：
$$ T = \frac{BT}{B} $$
$$ K = \frac{B}{T} = \frac{B^2}{BT} $$
$$ f_s = 2B $$
$$ \Delta t = \frac{1}{f_s} = \frac{1}{2B} $$
$$ N_s = T \cdot f_s = BT $$
$$ d_{sig} = \frac{T \cdot c}{2} $$
（变量说明：$T$ [s]：脉冲时宽；$K$ [Hz/s]：调频斜率；$f_s$ [Hz]：奈奎斯特采样频率；$\Delta t$ [s]：采样间隔；$N_s$：脉冲采样点数；$d_{sig}$ [m]：按双程传播定义的无模糊距离窗口；$c=3\times 10^8$ m/s）

时间向量生成：
$$ t_i = i \cdot \Delta t, \quad i = 0, 1, \dots, N_s - 1 $$
LFM复数波形生成：
$$ s(t_i) = e^{j 2\pi \left(f_0 t_i + \frac{K}{2} t_i^2\right)} $$
（变量说明：$f_0$：起始频率 [Hz]；$K$：调频斜率 [Hz/s]）

---

### 1.1.4.4.3.2 匹配滤波器构造模块
#### 1.1.4.4.3.2.1 功能
对发射Chirp波形构造匹配滤波器，供脉冲压缩使用。审批实现必须在“时域匹配滤波器”和“频域匹配核”之间二选一作为内部主表示，避免在构造和压缩阶段重复取共轭。

#### 1.1.4.4.3.2.2 算法输入
输入模块：线性调频波形生成模块
- `chirp` (matrix（复数，长度 $N_s$）): 发射LFM复数波形

#### 1.1.4.4.3.2.3 算法输出
输出模块：仿真流程总控模块 / 全图脉冲压缩处理模块 / 脉冲压缩验证模块
- `match` (matrix（复数，长度 $N_s$）): 匹配滤波器时域响应

#### 1.1.4.4.3.2.4 算法流程
1. 流程启动，输入发射波形 `chirp`，进入频域匹配滤波器构造逻辑。
2. 构造时域匹配滤波器：
   - 对发射波形执行时间反转；
   - 对反转后的复数样本取共轭；
   - 根据统一 FFT/IFFT 归一化约定决定是否附加尺度因子。
3. 如实现选择频域主表示，则直接计算 $H[k] = S^*[k]$ 并记录“已共轭”状态，不再对 $H[k]$ 重复共轭。
4. 流程结束，输出匹配滤波器供单路与全图压缩复用。

#### 1.1.4.4.3.2.5 数学公式
时域匹配滤波器定义：
$$ h[n] = s^*[N_s - 1 - n], \quad n = 0, 1, \dots, N_s - 1 $$
频域匹配核定义：
$$ H[k] = S^*[k] $$
线性卷积实现时，发射回波和匹配滤波器均补零至 $L_f \ge N_x + N_s - 1$。FFT/IFFT 的 $1/N$ 尺度因子由数值库约定统一处理，不在算法公式中重复写入。
（变量说明：$(\cdot)^*$：复共轭；$N_s$：采样点数；$N_x$：待压缩回波长度）

---

### 1.1.4.4.3.3 脉冲压缩验证与图像归一化模块
#### 1.1.4.4.3.3.1 功能
对发射波形与匹配滤波器进行单路脉冲压缩，在压缩波形上搜索峰值和半功率点以计算3 dB距离分辨率，同时输出**峰值旁瓣比（PSLR）** 与 **积分旁瓣比（ISLR）** 等成像质量指标；并根据配置对雷达图像进行幅度归一化预处理。幅度归一化作为可配置选项（`norm_enable`），若关闭则完全保留图像原始物理RCS反射率，为下游目标关联识别提供物理依据。

#### 1.1.4.4.3.3.2 算法输入
输入模块：线性调频波形生成模块 / 匹配滤波器构造模块 / SAR场景成像仿真模块
- `chirp` (matrix（复数，长度 $N_s$）): 发射LFM波形
- `match` (matrix（复数，长度 $N_s$）): 匹配滤波器
- `radar_image` (matrix（复数）): 待归一化雷达图像
- `variables.chirp_samples` (unsigned long / —): 脉冲采样点数 $N_s$
- `variables.signal_distance` (double / m): 脉冲覆盖距离 $d_{sig}$
- `norm_enable` (char (y/n)): 是否启用幅度归一化预处理
- `slr_method` (unsigned int): 旁瓣比主瓣区域判定方法（0=3dB, 1=IRW, 2=20dB）

#### 1.1.4.4.3.3.3 算法输出
输出模块：仿真流程总控模块
- `pc_waveform` (matrix（复数，长度 $N_s$）): 单路脉冲压缩波形
- `resolution` (float / m): 压缩脉冲3 dB距离分辨率
- `pslr` (float / dB): 峰值旁瓣比（Peak Side Lobe Ratio）
- `islr` (float / dB): 积分旁瓣比（Integrated Side Lobe Ratio）
- `slr_method` (unsigned int): 主瓣区域判定方法（0=3dB, 1=IRW, 2=20dB）
- `radar_image`（原地修改）: 幅度处理后的雷达图像

#### 1.1.4.4.3.3.4 算法流程
1. 流程启动，输入 `chirp`、`match` 与原始图像，进入验证与预处理逻辑。
2. 执行单路压缩验证：
   - 进行频域乘法与逆变换得到压缩脉冲；
   - 定位峰值并向两侧搜索半功率点；
   - 计算3 dB距离分辨率并输出质量指标。
3. 读取 `norm_enable` 状态：
   - 若为 `y`：遍历图像所有像素并按复数幅度最大值归一化，消除列间动态范围差异，方便可视化；
   - 若为 `n`：跳过归一化逻辑，保留图像的真实雷达反射系数物理量纲，避免破坏RCS绝对值分布。
4. 流程结束，输出压缩验证结果与归一化后的雷达图像。

#### 1.1.4.4.3.3.5 数学公式
##### 单路脉冲压缩（频域卷积）
补零以抑制圆卷积混叠，设补零长度 $L \ge N_x + N_s - 1$：
$$ Y(f) = \text{FFT}\{x(t), L\} \cdot \text{FFT}\{h(t), L\} $$
取前 $N_s$ 点：
$$ y_{pc}(t) = \text{IFFT}\{Y(f)\}|_{0:N_s} $$

##### 压缩脉冲3 dB分辨率估算
定位压缩波形幅值峰值位置及峰值幅度：
$$ n_{peak} = \text{argmax}_n |y_{pc}(t_n)|, \quad A_{max} = |y_{pc}(t_{n_{peak}})| $$
向两侧扫描半功率（-3 dB）点：
$$ n_{low} = \max \{ n \le n_{peak} : |y_{pc}(t_n)| \le \frac{A_{max}}{\sqrt{2}} \} $$
$$ n_{high} = \min \{ n \ge n_{peak} : |y_{pc}(t_n)| \le \frac{A_{max}}{\sqrt{2}} \} $$
换算为距离分辨率：
$$ \delta r = (n_{high} - n_{low}) \cdot \frac{c}{2f_s} $$

##### 峰值旁瓣比（PSLR）计算
定位主瓣峰值与最强副瓣峰值：
$$ A_{main} = |y_{pc}(t_{n_{peak}})| $$
$$ n_{sll} = \text{argmax}_{n \notin [n_{low}, n_{high}]} |y_{pc}(t_n)| $$
$$ A_{sll} = |y_{pc}(t_{n_{sll}})| $$
$$ \text{PSLR} = 20 \log_{10}\left(\frac{A_{sll}}{A_{main}}\right) \quad [\text{dB}] $$

##### 积分旁瓣比（ISLR）计算
主瓣能量与旁瓣能量之比：
$$ E_{main} = \sum_{n=n_{low}}^{n_{high}} |y_{pc}(t_n)|^2 $$
$$ E_{side} = \sum_{n=0}^{n_{low}-1} |y_{pc}(t_n)|^2 + \sum_{n=n_{high}+1}^{N_s-1} |y_{pc}(t_n)|^2 $$
$$ \text{ISLR} = 10 \log_{10}\left(\frac{E_{side}}{E_{main}}\right) \quad [\text{dB}] $$

##### 主瓣区域自适应判定（IRW 法，推荐用于加窗场景）
当 `slr_method=1` 时，使用 IRW（Integrated Resolution Width）法自适应确定主瓣区域宽度，避免加窗后 3dB 准则失效的问题：
$$ W_{IRW} = \frac{\left(\sum_{n=0}^{N_s-1} |y_{pc}(t_n)|^2\right)^2}{\sum_{n=0}^{N_s-1} |y_{pc}(t_n)|^4} $$
主瓣边界：
$$ n_{low} = n_{peak} - \lceil W_{IRW}/2 \rceil, \quad n_{high} = n_{peak} + \lceil W_{IRW}/2 \rceil $$

当 `slr_method=2` 时，使用 20dB 准则：
$$ n_{low} = \max\{n \le n_{peak} : |y_{pc}(t_n)| \le 0.1 \cdot A_{max}\}, \quad n_{high} = \text{对称右侧} $$

##### 图像幅度归一化（当 `norm_enable` 开启时执行）
$$ I'[i][j] = \frac{I[i][j]}{\max_{i,j} |I[i][j]|} $$

---

### 1.1.4.4.3.4 辐射定标模块
/*
 * 【未进行设计需求，不再扩展】
 * 辐射定标模块不属于原始设计需求范围，已冻结不再扩展。
 * 以下内容仅作为历史设计记录保留，不作为实施依据。
 */
#### 1.1.4.4.3.4.1 功能
利用已知 RCS 的标定目标（角反射器、主动定标器等）反推 SAR 系统的辐射定标常数 $K_{cal}$，建立图像像素值与目标 RCS 的映射关系，为后续辐射精度评估与绝对 RCS 反演提供桥梁。未定标时 $K_{cal}=1$，仅作相对辐射精度评估。

#### 1.1.4.4.3.4.2 算法输入
输入模块：仿真流程总控模块 / 标定目标配置
- `cal_target_rcs` (double / m²): 标定目标已知 RCS 值
- `cal_target_position` (FzVector / m): 标定目标位置（ECEF 坐标）
- `cal_image_peak` (double / —): 标定目标在 SAR 图像中的峰值像素幅度值
- `platform_position` (FzVector / m): 标定时刻平台位置
- `wavelength_m` (double / m): 雷达工作波长
- `antenna_gain_dbi` (double / dBi): 天线增益（含双向）
- `tx_power_w` (double / W): 发射功率

#### 1.1.4.4.3.4.3 算法输出
输出模块：仿真流程总控模块 / SAR 聚焦与频谱分析模块 / 成像质量评估模块
- `K_cal` (double / —): 辐射定标常数
- `cal_residual_error_db` (double / dB): 定标残差（实测 RCS 与理论 RCS 的偏差），用于评估定标质量
- `cal_valid` (bool): 定标是否有效（标定目标信噪比充足、位置准确）

#### 1.1.4.4.3.4.4 算法流程
1. 流程启动，输入标定目标参数与 SAR 系统参数，进入定标常数计算逻辑。
2. 执行定标有效性检查：
   - 检查标定目标峰值信噪比是否满足阈值（SNR $\ge$ 20 dB）；
   - 检查标定目标位置是否处于图像聚焦良好区域（远离图像边缘 $\ge 10\%$ 像素）。
3. 执行定标常数计算（基于 SAR 方程反推）：
   $$ K_{cal} = \frac{P_t G^2 \lambda^2 \sigma_{cal}}{(4\pi)^3 R_{cal}^4 \cdot |I_{cal}|^2} $$
4. 执行定标残差评估：
   $$ \Delta_{cal} = 10\log_{10}\left(\frac{|I_{cal}|^2 K_{cal} (4\pi)^3 R_{cal}^4}{P_t G^2 \lambda^2 \sigma_{cal}}\right) \quad [\text{dB}] $$
5. 流程结束，输出定标常数与残差。

#### 1.1.4.4.3.4.5 数学公式
##### SAR 辐射方程（定标基础）
$$ P_r = \frac{P_t G^2 \lambda^2 \sigma}{(4\pi)^3 R^4} $$

##### 定标常数反推
$$ K_{cal} = \frac{P_t G^2 \lambda^2 \sigma_{cal}}{(4\pi)^3 R_{cal}^4 \cdot |I_{cal}|^2} $$

##### 绝对 RCS 反演（应用定标后）
$$ \sigma_{measured} = K_{cal} \cdot |I_{SAR}[i][j]|^2 \cdot R_{ij}^4 $$

##### 辐射精度评估
$$ \Delta\sigma_{dB} = 10 \log_{10}\left(\frac{\sigma_{measured}}{\sigma_{theory}}\right) \quad [\text{dB}] $$

##### 多点定标加权融合（使用 N 个标定目标时）
$$ K_{cal,fused} = \frac{\sum_{n=1}^{N} w_n \cdot K_{cal,n}}{\sum_{n=1}^{N} w_n}, \quad w_n = \frac{1}{\Delta_{cal,n}^2} $$
（变量说明：$P_t$：发射功率 [W]；$G$：天线增益（双向，线性值）；$\lambda$：波长 [m]；$\sigma_{cal}$：标定目标 RCS [m²]；$R_{cal}$：标定目标斜距 [m]；$|I_{cal}|$：标定目标图像峰值幅度；$K_{cal,n}$：第 $n$ 个标定目标反推的定标常数；$\Delta_{cal,n}$：第 $n$ 个标定目标的定标残差 [dB]；$w_n$：定标融合权重）

---

## 1.1.4.4.A SAR 算法审批清单

> 审批范围说明：1.1.4.5 机动行为组件与 1.1.4.6 侦察行为组件属于上层任务/行为系统，不作为 SAR 雷达组件 Phase 1 的实施范围。SAR Phase 1 仅审批 1.1.4.4 中的信号、回波、脉冲压缩、RDA 聚焦、基础质量指标和脉冲缓冲区闭环。机动行为、多源融合、传感器激活策略应另立模块边界和验收计划。

| 审批项 | Phase 1 结论 | 必须满足的条件 |
|---|---|---|
| LFM 波形 | 通过后可实施 | 明确 $T=BT/B$、$K=B/T$、$f_s \ge 2B$，记录 FFT/IFFT 归一化约定 |
| 匹配滤波 | 通过后可实施 | 统一采用 `h[n]=s^*[N-1-n]` 或 `H[k]=S^*[k]`，禁止重复共轭 |
| 距离向脉冲压缩 | 通过后可实施 | 使用线性卷积补零长度 $L_f \ge N_x+N_s-1$，分辨率按 $c/(2f_s)$ 换算 |
| 原始回波生成 | 条件通过 | Phase 1 以点目标和时延门/相位延迟模型为准，近似能量投影仅用于冒烟测试 |
| RDA 聚焦 | 通过后可实施 | 限定 L1 匀速直线条带模式，必须验证 RCMC、方位匹配滤波和峰值定位 |
| Phase 1 运行尺寸 | 冻结通过 | 当前平台上限为 1024×1024；超过该尺寸继续门禁，后续阶段单独调整 |
| FFT/编译兼容 | 冻结通过 | SAR engine 使用 C++11 + Eigen 3.3.9 当前平台编译门；Windows/VS2015 不作为 Phase 1 强制审批门 |
| Trace/Replay | 摘要级通过 | Phase 1 不保存 focused complex image 全矩阵；全图 replay 后置审批 |
| GBP/BP | 暂缓 | 作为 Phase 2/3 扩展，不得作为 Phase 1 默认算法或自动降级目标 |
| CSA/Omega-K | **未进行设计需求，不再扩展** | 不属于原始设计需求范围，已冻结不再扩展；既有相关代码与文档仅作探索性参考，不作为实施依据 |
| 自动选择 | **未进行设计需求，不再扩展** | 不属于原始设计需求范围，已冻结不再扩展；既有相关代码与文档仅作探索性参考，不作为实施依据 |
| 相位重参考 | 条件通过 | 仅用于全局相位基准对齐，不得替代运动补偿、RCMC 或自聚焦 |
| 辐射定标 | **未进行设计需求，不再扩展** | 不属于原始设计需求范围，已冻结不再扩展；既有相关代码与文档仅作探索性参考，不作为实施依据 |

---

# 1.1.4.5 机动行为组件
## 1.1.4.5.1 接口层
### 1.1.4.5.1.1 任务参数解析与机动指令封装
#### 1.1.4.5.1.1.1 功能
解析外部系统下达的侦察任务参数，初始化行为决策层的任务区域、目标列表和约束条件；在每步仿真中接收当前平台状态与传感器探测反馈，触发行为决策并将决策结果封装为飞机机动组件标准控制指令格式输出。

#### 1.1.4.5.1.1.2 算法输入
输入模块：外部仿真系统
- `missionArea` (WayPointInfo[] / deg): 巡逻区域多边形顶点序列（经纬高）
- `priorityTargets` (TargetInfo[]): 优先侦察目标列表（目标标识、位置、类型）
- `altRange[2]` (double / m): 飞行高度约束范围 [最低, 最高]
- `speedRange[2]` (double / m/s): 飞行速度约束范围
- `missionDuration` (double / s): 任务总时限
- `platformState` (PlatformState): 当前平台位置、姿态、速度、剩余燃油
- `sensorResults` (SensorResultSet): 当前步各传感器探测结果汇集

#### 1.1.4.5.1.1.3 算法输出
输出模块：飞机机动组件（ControlCmd结构）
- `maneuverCmd.moveType` (enum MoveType): 机动类型（方向/点对点/航路点）
- `maneuverCmd.desirePoint[]` (WayPointInfo[] / deg): 目标航路点序列
- `maneuverCmd.cmdAlt` (double / m): 目标高度
- `maneuverCmd.cmdSpeed` (double / m/s): 目标速度
- `maneuverCmd.cmdHeading` (double / deg): 目标航向（规避机动时有效）
- `maneuverCmd.durationTime` (double / s): 指令持续时间
- `maneuverCmd.priority` (int): 机动指令优先级

#### 1.1.4.5.1.1.4 数学公式
任务剩余时间约束校验：
$$ t_{remain} = t_{mission} - t_{elapsed}, \quad t_{remain} > 0 $$
巡逻区域边界中心点计算（用于初始巡逻起点定位）：
$$ \lambda = \frac{1}{N}\sum_{i=1}^N \lambda_i, \quad \phi = \frac{1}{N}\sum_{i=1}^N \phi_i $$
（变量说明：$\lambda_i, \phi_i$：第 $i$ 个多边形顶点的经度和纬度 [度]；$N$：多边形顶点总数；$\lambda, \phi$：区域中心经度和纬度 [度]）

#### 1.1.4.5.1.1.5 算法流程
1. 接收外部系统任务指令，读取并解析 `missionArea` 多边形几何顶点数据，校验合法性。
2. 提取机动边界约束（高度、速度限制），初始化运行环境上下文。
3. 在仿真步长循环中，周期性采集 `platformState`（当前经纬高、速度、剩余油量）与传感器探测结果。
4. 调用行为状态机，计算下一步的行为状态转移。
5. 将选定的行为模式转化为具体的转向角、航路点序列、目标高度及目标速度等，封装为 `ControlCmd` 发送至机动控制组件。

---

## 1.1.4.5.2 行为决策层
### 1.1.4.5.2.1 行为状态机转换
#### 1.1.4.5.2.1.1 功能
以有限状态机（FSM）为驱动，根据传感器探测结果、威胁感知状态和平台资源约束，完成当前行为状态的转移条件检测与状态切换，确保平台行为对任务态势变化的实时正确响应。

#### 1.1.4.5.2.1.2 算法输入
输入模块：行为状态机模块
- `currentState` (enum BehaviorState): 当前行为状态（PATROL/DWELL/TRACKING/EVADE/RETURN）
- `detectionResults` (DetectResult[]): 当前步传感器有效探测结果列表
- `threatDetected` (bool): 是否检测到威胁
- `threatPosition` (FzVector): 威胁地理位置（ECEF坐标，m）
- `fuelRemaining` (double / kg): 当前剩余燃油量
- `fuelThreshold` (double / kg): 返航燃油阈值
- `timeRemaining` (double / s): 任务剩余时间
- `platformPos` (FzVector / m): 当前平台位置（ECEF坐标）

#### 1.1.4.5.2.1.3 算法输出
输出模块：行为状态机模块
- `nextState` (enum BehaviorState): 转移后的目标行为状态
- `activeTarget` (TargetInfo*): 当前跟踪目标信息（TRACKING状态时有效）
- `evadeDirection` (FzVector): 规避方向向量（EVADE状态时有效）

#### 1.1.4.5.2.1.4 数学公式
##### 威胁接近距离计算
$$ d_{threat} = \| r_{platform} - r_{threat} \|_2 $$

##### 带滞回防抖区间的状态转移优先级判断
为了避免系统在临界阈值点附近（如飞入/飞出威胁范围的边界）发生反复无常的破坏性高频状态跳变，引入转移滞回距离 $\Delta d_{hysteresis}$，状态转移优先级（自上而下匹配，取首个成立条件）公式定义为：
$$ 
S_{next} = 
\begin{cases} 
\text{EVADE} & \text{if } (S_{current} \neq \text{EVADE} \land d_{threat} < d_{safe}) \lor (S_{current} == \text{EVADE} \land d_{threat} < d_{safe} + \Delta d_{hysteresis}) \\
\text{RETURN} & \text{if } W_{fuel} \le W_{threshold} \lor t_{remain} \le 0 \\
\text{TRACKING}& \text{if } N_{detected} > 0 \land S_{current} \neq \text{EVADE} \\
\text{DWELL} & \text{if } P_{target} \in \text{FOV} \land S_{current} \in \{\text{PATROL}, \text{TRACKING}\} \\
\text{PATROL} & \text{otherwise}
\end{cases}
$$
（变量说明：$d_{safe}$：威胁安全距离阈值 [m]；$\Delta d_{hysteresis}$：防抖滞回缓冲距离 [m]，推荐配置为 $2000\text{ m}$；$W_{threshold}$：返航燃油阈值 [kg]；$N_{detected}$：当前步有效探测目标数；$P_{target} \in \text{FOV}$：目标位于传感器视场内判定）

#### 1.1.4.5.2.1.5 算法流程
1. 读取当前的生存参数：燃油、剩余时间和与已知威胁的实时距离。
2. 依据带有滞回判断的转移公式判定是否强制进入规避（EVADE）或返航（RETURN）状态。
3. 若无紧急态势，评估普通任务级状态切换（如从搜索巡逻巡航 PATROL 进入对目标的跟踪聚焦 TRACKING 状态）。
4. 发生转移时更新 `currentState` 并导出对应的行为目标参数（如规避航向或跟踪目标引用）。

---

### 1.1.4.5.2.2 机动优先级决策
#### 1.1.4.5.2.2.1 功能
在多个行为状态触发条件并发时，按预设优先级权重从候选行为集合中选取最高优先级的行为状态，调用路径规划层生成对应机动参数。

#### 1.1.4.5.2.2.2 算法输入
输入模块：机动优先级决策模块
- `candidateStates` (BehaviorState[]): 当前步所有触发的行为状态候选列表
- `stateContext` (BehaviorContext): 各状态对应的上下文参数

#### 1.1.4.5.2.2.3 算法输出
输出模块：机动优先级决策模块
- `selectedState` (BehaviorState): 本步选定的执行行为状态
- `motionParams` (MotionPlanParam): 行为对应的路径规划请求参数

#### 1.1.4.5.2.2.4 数学公式
最高优先级行为选取：
$$ s^* = \text{argmax}_{s \in S_{active}} w(s) $$
预设优先级权重映射：
$$ 
w(s) = 
\begin{cases}
100 & s = \text{EVADE} \\
80  & s = \text{RETURN} \\
60  & s = \text{TRACKING} \\
40  & s = \text{DWELL} \\
20  & s = \text{PATROL}
\end{cases}
$$
（变量说明：$S_{active}$：当前所有触发条件成立的行为状态候选集合；$w(s)$：对应行为的预设权重数值）

---

## 1.1.4.5.3 路径规划层
### 1.1.4.5.3.1 区域覆盖路径规划
#### 1.1.4.5.3.1.1 功能
依据任务巡逻区域多边形和当前传感器有效侦察幅宽，采用往返条带（梳形）扫描策略生成覆盖任务区域全部有效位置所需的最短路径航路点序列，支持单程与循环两种扫描模式。

#### 1.1.4.5.3.1.2 算法输入
输入模块：区域覆盖路径规划模块
- `areaBoundary` (WayPointInfo[] / deg): 巡逻区域多边形顶点序列
- `swathWidth` (double / m): 当前传感器有效侦察幅宽
- `flyAlt` (double / m): 巡逻飞行高度
- `flySpeed` (double / m/s): 巡逻飞行速度
- `overlapRatio` (double): 条带重叠率（推荐0.1~0.2）
- `loopMode` (bool): 是否启用循环覆盖模式

#### 1.1.4.5.3.1.3 算法输出
输出模块：区域覆盖路径规划模块
- `patrolWaypoints` (WayPointInfo[] / deg): 梳形覆盖路径航路点序列（经纬高+速度）
- `estCoverage` (double / %): 预估区域覆盖率
- `stripCount` (int): 总条带数量

#### 1.1.4.5.3.1.4 数学公式
条带间距（含重叠率保障）：
$$ d_{strip} = (1 - \alpha_{overlap}) \cdot W_{swath} $$
总条带数量：
$$ N_{strips} = \frac{L_{area}}{d_{strip}} $$
预估覆盖率（矩形等效区域）：
$$ C_{est} = \min\left(1.0, \frac{N_{strips} \cdot W_{swath}}{L_{area}}\right) \times 100\% $$
（变量说明：$\alpha_{overlap}$：条带重叠率；$W_{swath}$：传感器侦察幅宽 [m]；$L_{area}$：垂直条带方向的区域跨度 [m]）

#### 1.1.4.5.3.1.5 算法流程
1. 确定多边形包围盒，并沿着长轴确定条带扫描的基准方向。
2. 依据条带间距计算公式，对规划区域切片，生成每条扫描线的起止节点。
3. 按照往返条带的端点进行Z字形（梳形）串联，输出生成航路点数组。
4. 如果设置了循环模式，在最后一个航路点添加返回首点的连接弧线。

---

### 1.1.4.5.3.2 目标观测位置计算
#### 1.1.4.5.3.2.1 功能
针对行为决策层指定的优先观测目标，计算使目标处于当前传感器最优探测范围内的飞行位置与接近航向，生成驻留或跟踪机动所需的目标航路点和速度参数。

#### 1.1.4.5.3.2.2 算法输入
输入模块：目标观测位置计算模块
- `targetLon`, `targetLat` (double / deg): 目标经纬度
- `targetAlt` (double / m): 目标海拔高度
- `optimalRange` (double / m): 传感器最优探测斜距
- `platformAlt` (double / m): 当前飞行高度
- `approachAzimuth` (double / deg): 接近方位角（默认取平台至目标当前方位）

#### 1.1.4.5.3.2.3 算法输出
输出模块：目标观测位置计算模块
- `observeWaypoint` (WayPointInfo / deg): 最优观测位置（经纬高）
- `observeHeading` (double / deg): 最优观测接近航向
- `observeSpeed` (double / m/s): 推荐观测飞行速度

#### 1.1.4.5.3.2.4 数学公式
最优观测斜距转水平距离：
$$ d_{horizontal} = \sqrt{R_{opt}^2 - \Delta h^2}, \quad \Delta h = h_{platform} - h_{target} $$
最优观测位置经纬度偏移计算（基于球面近似模型）：
$$ \phi_{obs} = \phi_{target} + \frac{d_{horizontal} \cdot \cos\psi_{approach}}{R_E} $$
$$ \lambda_{obs} = \lambda_{target} + \frac{d_{horizontal} \cdot \sin\psi_{approach}}{R_E \cdot \cos\phi_{target}} $$
（变量说明：$R_{opt}$：传感器最优探测斜距 [m]；$\Delta h$：高度差 [m]；$\psi_{approach}$：接近方位角 [rad]；$R_E = 6378137\text{ m}$：地球参考半径；$\phi_{target}, \lambda_{target}$：目标纬度和经度 [rad]）

---

# 1.1.4.6 侦察行为组件
## 1.1.4.6.1 接口层
### 1.1.4.6.1.1 侦察任务参数解析与传感器指令封装
#### 1.1.4.6.1.1.1 功能
解析外部系统下达的侦察任务参数，初始化任务调度层的任务目标列表、传感器约束条件和侦察覆盖网格；在每步仿真中接收各传感器探测结果并触发任务调度决策，将传感器激活决策封装为各设备组件可直接接收的控制消息输出。

#### 1.1.4.6.1.1.2 算法输入
输入模块：外部仿真系统
- `taskArea` (WayPointInfo[] / deg): 任务区域多边形顶点序列
- `priorityTargets` (TargetInfo[]): 优先侦察目标列表
- `sensorAvailable` (int): 可用传感器类型标志位
- `gridResolution` (double / deg): 覆盖评估网格角分辨率
- `taskDuration` (double / s): 任务总时限
- `elintResults` (ElintResult[]): 电子侦察当前步探测结果
- `opticalResults` (OpticalResult[]): 光学传感器当前步探测结果
- `sarResults` (SARResult[]): SAR雷达当前步探测结果

#### 1.1.4.6.1.1.3 算法输出
输出模块：各传感器设备组件
- `elintActivate` (bool): 电子侦察设备是否激活
- `elintMode` (int): 电子侦察工作模式
- `opticalActivate` (bool): 光学传感器是否激活
- `opticalMode` (int): 光学传感器工作模式
- `sarActivate` (bool): SAR雷达是否激活

##### 数学公式
覆盖评估网格初始化（按角分辨率划分任务区域）：
$$ N_{grid} = \text{round}\left(\frac{\Delta\lambda_{area}}{\delta\lambda}\right) \times \text{round}\left(\frac{\Delta\phi_{area}}{\delta\phi}\right) $$
（变量说明：$\Delta\lambda_{area}, \Delta\phi_{area}$：任务区域的经纬度跨度 [度]；$\delta\lambda, \delta\phi$：配置的网格经纬向采样角分辨率）

#### 1.1.4.6.1.1.4 算法流程
1. 初始化阶段根据任务区域经纬度跨度及分辨率，划分生成覆盖评估网格矩阵。
2. 周期性接收下层的多源探测原始数据（如电子侦察截获信号、光学信噪比、SAR成像帧）。
3. 调用任务调度层，运行传感器激活控制决策。
4. 将布尔决策控制指令输出给对应的物理探测设备驱动（控制高压开启、天线扫描或云台旋转等）。

---

## 1.1.4.6.2 任务调度层
### 1.1.4.6.2.1 传感器激活决策
#### 1.1.4.6.2.1.1 功能
根据当前任务阶段、目标类型特征和环境条件，按优先规则集决策本步应激活的传感器组合及工作模式，实现多手段协同侦察。

#### 1.1.4.6.2.1.2 算法输入
输入模块：传感器激活决策模块
- `missionPhase` (enum MissionPhase): 任务阶段（SEARCH/DETECT/TRACK/CONFIRM）
- `detectedTargetTypes` (int): 当前已探测目标类型标志位
- `envCondition.dayNight` (int): 昼夜类型（0=昼，1=夜，2=晨昏）
- `envCondition.cloudCover` (double): 云量（0~1）
- `envCondition.jamLevel` (int): 电磁干扰等级（0~3）
- `uncoveredRatio` (double / %): 任务区域当前未覆盖比例
- `sarThreshold` (double / %): 触发SAR激活的未覆盖率阈值

#### 1.1.4.6.2.1.3 算法输出
- `elintActivate` (bool): 电子侦察设备激活标志
- `elintMode` (int): 电子侦察工作模式
- `opticalActivate` (bool): 光学传感器激活标志
- `opticalMode` (int): 光学传感器工作模式
- `sarActivate` (bool): SAR雷达激活标志

#### 1.1.4.6.2.1.4 数学公式
##### 电子侦察激活判定
$$ A_{ELINT} = \text{true if } (P_{phase} == \text{SEARCH} \lor T_{type} \in \{\text{RADAR}, \text{COMM}\}) \text{ else false} $$

##### 光学传感器激活判定及模式选择
$$ A_{OPT} = \text{true if } (P_{phase} \in \{\text{DETECT}, \text{TRACK}\} \lor T_{type} == \text{OPTICAL}) \text{ else false} $$
工作模式选择逻辑：
$$ 
M_{OPT} = 
\begin{cases} 
\text{IR} & \text{if } \text{dayNight} == \text{NIGHT} \\
\text{FUSION} & \text{if } \text{cloudCover} > 0.5 \\
\text{VIS} & \text{otherwise}
\end{cases}
$$

##### SAR雷达激活判定
$$ A_{SAR} = \text{true if } (P_{phase} == \text{CONFIRM} \lor U_{coverage} > \theta_{SAR}) \text{ else false} $$
（变量说明：$P_{phase}$：当前阶段；$T_{type}$：目标电磁与特征响应；$U_{coverage}$：未覆盖比例；$\theta_{SAR}$：触发阈值）

#### 1.1.4.6.2.1.5 算法流程
1. 获取环境参数，检查是否处于高干扰或重度多云天气。
2. 评估当前侦察进度，若在大范围搜索模式且未覆盖率极高，激活电子侦察与SAR大测宽模式。
3. 若侦察进入确认阶段或获取高优先级航迹，激活光学可见光/红外传感器进行精确跟踪。
4. 综合各项逻辑标志，给出多手段联合工作的最终硬激活矩阵。

---

### 1.1.4.6.2.2 任务进度状态更新
#### 1.1.4.6.2.2.1 功能
在每步仿真结束后，更新侦察区域覆盖网格状态矩阵和目标探测状态表，统计区域覆盖率和目标探测完成率，提供闭环反馈。

#### 1.1.4.6.2.2.2 算法输入
- `platformPos` (WayPointInfo / deg): 平台当前经纬高
- `swathWidths[3]` (double / m): 三类传感器当前有效侦察幅宽
- `elintResults` (ElintResult[]), `opticalResults` (OpticalResult[]), `sarResults` (SARResult[])

#### 1.1.4.6.2.2.3 算法输出
- `gridStatus` (bool[][]): 覆盖网格状态矩阵（已覆盖为 true）
- `coveredRatio` (double / %): 当前区域总覆盖率
- `targetDetected` (bool[]): 优先目标探测完成状态数组
- `detectedCount` (int): 已成功探测优先目标数量

#### 1.1.4.6.2.2.4 数学公式
##### 传感器覆盖格点更新判断（以光学传感器为例）
$$ \text{grid}[i][j] = \text{true if } \| p_{grid}(i,j) - p_{platform} \|_2 \le R_{sensor} $$
传感器投影在地面上所产生的等效截面半径计算：
$$ R_{sensor} = \frac{H_{platform} \cdot \tan\left(\frac{\theta_{FOV}}{2}\right)}{\cos\theta_{offnadir}} $$
##### 区域覆盖率统计
$$ C_{coverage} = \frac{N_{covered}}{N_{grid}} \times 100\% $$
（变量说明：$p_{grid}(i,j)$：网格物理坐标；$H_{platform}$：高度；$\theta_{FOV}$：视场角；$\theta_{offnadir}$：下视偏角；$N_{covered}$：置为 true 的格点总数）

---

## 1.1.4.6.3 覆盖评估与结果汇总层
### 1.1.4.6.3.1 多源探测关联融合
#### 1.1.4.6.3.1.1 功能
对电子侦察、光学传感器和SAR三类传感器的独立探测结果进行关联汇聚，计算综合探测置信度，生成融合目标态势条目列表。

#### 1.1.4.6.3.1.2 算法输入
- `elintResults` (ElintResult[]), `opticalResults` (OpticalResult[]), `sarResults` (SARResult[])
- `positionGate` (double / m): 关联检测基准门限
- `sensorWeights[3]` (double): 三类传感器置信度融合权重

#### 1.1.4.6.3.1.3 算法输出
- `fusedTargets` (FusedTargetInfo[]): 融合后目标态势列表
- `fusedTargets[i].confidence` (double): 综合探测置信度（0~1）

#### 1.1.4.6.3.1.4 数学公式
##### 多传感器异构误差动态关联门限判断
为了适应异构传感器误差特性（如电子侦察在大气传输和侧向中误差呈狭长椭圆形分布，而SAR测量距离精准），使用加权关联距离模型取代固定距离关联门限：
$$ d_{match}(T_i, T_j) = \sqrt{\left(\frac{x_i - x_j}{\sigma_x}\right)^2 + \left(\frac{y_i - y_j}{\sigma_y}\right)^2} $$
位置关联判定：
$$ \text{match}(T_i, T_j) = \text{true if } d_{match}(T_i, T_j) \le \chi_{gate} \text{ else false} $$
（变量说明：$\sigma_x, \sigma_y$：当前融合过程中，测量传感器在当地地理坐标系下的经、纬方向测定位姿协方差标准差；$\chi_{gate}$：无量纲卡方检验判定门限，通常取值范围为 2.0~3.0）

##### 综合探测置信度加权融合
$$ C_{fused} = \sum_{k \in \{\text{ELINT}, \text{OPT}, \text{SAR}\}} w_k \cdot D_k \cdot Q_k $$
（变量说明：$D_k \in \{0, 1\}$：传感器检测成功判决值；$Q_k \in [0, 1]$：传感器接收信噪比等参数映射的探测质量；$w_k$：权重分配系数，$\sum w_k = 1$）

---

### 1.1.4.6.3.2 覆盖完整度评估与报告生成
#### 1.1.4.6.3.2.1 功能
评估当前侦察区域网格覆盖完整度，识别侦察盲区地理范围，计算优先目标探测完成率，并将融合目标态势与覆盖统计结果整理为标准化综合侦察结果报告输出。

#### 1.1.4.6.3.2.2 算法输入
- `gridStatus` (bool[][]): 覆盖网格状态矩阵
- `gridResolution` (double / deg): 网格分辨率
- `fusedTargets` (FusedTargetInfo[]): 当前步融合目标态势列表
- `priorityTargets` (TargetInfo[]): 优先侦察目标列表
- `confThreshold` (double): 判定目标有效探测的置信度阈值

#### 1.1.4.6.3.2.3 算法输出
- `coverageReport.totalRatio` (double / %): 任务区域总覆盖率
- `coverageReport.blindZones` (WayPointInfo[][]/deg): 侦察盲区区域边界坐标列表
- `reconReport.detectedTargets` (FusedTargetInfo[]): 有效探测目标汇总列表
- `reconReport.priorityComplete` (double / %): 优先目标探测完成率

#### 1.1.4.6.3.2.4 数学公式
区域总覆盖率统计：
$$ C_{total} = \frac{N_{covered}}{N_{grid}} \times 100\% $$
优先目标探测完成率：
$$ R_{priority} = \frac{|T_i \in T_{priority} : C_{fused}(T_i) \ge \theta_{conf}|}{|T_{priority}|} \times 100\% $$
（变量说明：$T_{priority}$：优先目标总集合；$C_{fused}$：各融合目标的综合置信度；$\theta_{conf}$：置信度门限阈值）

# 1.1.5 数据要求

## 1.1.5.1 输入数据
| 序号 | 名称 | 数据类型 | 单位 | 取值范围/说明 |
|---|---|---|---|---|
| 1 | 平台经度 | double | 度 | -180 ~ 180 |
| 2 | 平台纬度 | double | 度 | -90 ~ 90 |
| 3 | 平台高度 | double | m | 0 ~ 20000 |
| 4 | 平台速度 | double | m/s | 50 ~ 800 |
| 5 | 平台航向/俯仰/横滚 | double | 度 | 0~360 / -90~90 / -180~180 |
| 6 | 电子侦察装备参数 | ESMPARAM | — | 包含接收频段、灵敏度、天线参数 |
| 7 | 光学传感器装备参数 | OPTPARAM | — | 红外与可见光双通道物理参数 |
| 8 | SAR雷达装备参数 | SARPARAM | — | **【更新】** 包含发射频率、带宽、天线、PRF等参数 |
| 9 | 场景与辐射源目标列表 | RadiatorTarget[] | — | 包含目标位置、主瓣频率、脉宽等 |
| 10| 侦察任务参数 | ReconMission | — | 巡逻区域多边形、优先目标、任务时限 |
| 11| 仿真时间步长 | double | s | $>0$ （平台宏观仿真步长，如 0.05s） |

## 1.1.5.2 中间数据
主要包括：飞机当前ECEF位置/速度/加速度向量、经纬高与ECEF坐标双向转换中间量、飞行包线速度-高度约束判断中间值、机动栈任务队列与超时判断标志、各航路段插值路径点序列、目标-传感器空间布局计算中间量（观测坐标系下各目标的方位角/俯仰角/距离）、电子侦察链路预算中间量（自由空间损耗/等效接收功率/干扰功率叠加值）、光学探测通道辐射亮度与焦平面接收功率、SAR波形匹配滤波器与脉冲压缩验证结果、多源探测关联中间矩阵（基于测位协方差的关联门限评估矩阵）、各传感器激活/关闭决策标志、区域覆盖网格状态矩阵与覆盖率统计量、各行为状态机的当前状态与转移条件判断标志、防抖判断延迟计数，以及动态威胁绕障重规划的绕障中间点坐标。

## 1.1.5.3 输出数据
- 平台当前位置与运动特征指标（经纬高、速度、航向、剩余燃油）。
- 探测目标情况列表（辐射源频段/信噪比，光学距离/方位/信噪比）。
- **SAR成像结果**：SAR聚焦复图像阵列（当 `norm_enable` 关闭时保留原始物理RCS，开启时为归一化图像）及检测目标参数。
- 多源融合结果、区域覆盖率、优先目标探测完成率、侦察盲区边界点、以及当前的行为状态和传感器状态。

---

# 1.1.6 约束与简化假定
## 1.1.6.1 功能级建模简化假定
1. **平台动力学**：采用功能级运动模型（航向/俯仰修正与速度积分），不做六自由度气动力学仿真。
2. **电子侦察设备**：采用链路预算与频段匹配判决模型，不做信号级解调、参数测量或脉冲描述字（PDW）级仿真。
3. **光学传感器**：采用目标辐射亮度模型和焦平面接收功率评估，以信噪比阈值进行探测判决，不做光子级探测器响应与焦平面读出噪声仿真。
4. **SAR成像**：Phase 1 采用线性调频波形、点目标回波模型和 RDA 聚焦算法进行条带模式成像。GBP/BP、CSA、Omega-K、自动选择、运动补偿、自聚焦、辐射定标均属于后续阶段能力。**平台轨迹模型 Phase 1 限定 L1 理想匀速直线**；L2/L3 需在后续阶段显式声明误差模型、运动补偿策略和验收指标。L4 需完整 6DOF 轨迹与 IMU/GPS 误差建模，不在本方案范围内。不做全极化SAR、干涉SAR（InSAR）或动目标指示（GMTI）仿真。

##### SAR 平台轨迹保真度分级

| 等级 | 轨迹模型 | 适用场景 | 约束标记 |
|------|----------|----------|----------|
| **L1** | 理想匀速直线 | 条带模式直线段 | 默认 |
| **L2** | 匀速直线 + 速度扰动（高斯噪声） | 气流扰动场景 | L1 的超集 |
| **L3** | 折线/航路点序列 | 聚束模式转弯、斜视模式 | 需配合时变 PRF |
| **L4** | 完整 6DOF 轨迹 | 高级模式/真实数据回放 | **不在本方案范围** |

**默认 L1 假设**。L2/L3 需在场景配置中显式声明 `trajectory_fidelity` 参数。L4 需引入运动补偿模块（Motion Compensation），属于未来扩展范畴。
5. **行为决策**：机动行为与侦察行为均采用有限状态机（FSM）驱动，加入转移滞回防抖逻辑，按预设优先级规则进行状态转移，不支持基于强化学习或深度神经网络的自主决策。
6. **路径规划**：区域覆盖采用梳形往返扫描路径，威胁绕障采用几何替换重规划，不进行全局最优路径搜索或动态规划。
7. **多传感器融合**：采用多源置信度加权融合与异构传感器测量协方差动态门限判断，不做贝叶斯网络或D-S证据理论融合。
8. **时间尺度解耦（快慢时间异步映射）**：明确定义平台运动和任务推演为慢时间步长（$\ge 0.05$秒），SAR成像仿真的发波和回波处理为快时间毫秒级脉冲。**PRF 与宏观步长解耦**：PRF 由方位向采样定理（避免多普勒模糊）独立决定，宏观步长由任务推演精度独立决定，二者无强约束关系。**异步环形缓冲区机制**：每个宏观步长内的脉冲回波写入容量为 $2 \times N_{aperture}$ 的环形缓冲区，SAR 聚焦处理器按合成孔径时间窗口异步读取，可自然跨宏观步长拼接。实现时必须保留分数脉冲累积量并检查 `pulse_id` 连续性，禁止简单 `floor(dt * prf)` 长期截断。合成孔径时间 $T_{sa} = \frac{\lambda R_0}{v \cdot \delta_{az}}$ 由物理参数（波长 $\lambda$、斜距 $R_0$、速度 $v$、方位分辨率 $\delta_{az}$）决定，与宏观步长无直接耦合。
9. **绝对RCS量纲隔离策略**：系统的图像幅度预归一化（1.1.4.4.3.3.4）仅作为显示或外接特定人眼读图模块的临时处理，对于目标态势关联和AI目标识别等需要提取物理属性的下游模块，雷达图像内存中严禁执行任何破坏物理量纲的归一化，严格保留浮点格式的原始复数幅度值。
10. **并行算力与自适应降级策略**：Phase 1 不启用自动算法切换，默认使用 RDA 频域算法（复杂度 $O(N \log N)$）。GBP算法复杂度为 $O(N_{pixel} \times N_{pulse})$，仅作为小场景高精度扩展能力。后续若启用 Auto 模式，必须在日志中输出请求算法、实际算法、切换原因、场景规模和线程数。实现层可使用 OpenMP 多核并行；GPU CUDA 加速属于 Phase 5 性能优化，不作为早期验收前提。

## 1.1.6.2 模型能力边界参数表
- 飞行速度范围：50 m/s ~ 800 m/s
- 飞行高度范围：0 m ~ 20000 m
- 最大过载：2g ~ 12g
- 电子侦察频段：0.1 GHz ~ 40 GHz
- 电子侦察灵敏度：-120 dBm ~ -60 dBm
- 测向精度：0.5° ~ 5° (RMS)
- 光学口径：10 mm ~ 500 mm
- SAR工作频率：1 GHz ~ 18 GHz
- SAR信号带宽：10 MHz ~ 500 MHz
- SAR方位分辨率：0.1 m ~ 10 m
- 区域覆盖面积：$\le 10000\text{ km}^2$
- 最大跟踪目标数：$\le 200$ 个
- 燃油容量：1000 kg ~ 50000 kg
- 最小仿真步长：$\ge 0.05\text{ s}$
- **脉冲重复频率(PRF)范围**：500 Hz ~ 5000 Hz **【新增】** 用于控制宏观步长内的脉冲发射数。
- **状态转移滞回距离($\Delta d_{hysteresis}$)**：$\ge 2000\text{ m}$ **【新增】** 规避决策模块防抖缓冲。
- **动态位置关联检验判定门限 ($\chi_{gate}$)**：2.0 ~ 3.0 **【新增】**。
