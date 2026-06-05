# SAR 仿真模块设计方案

## 背景与目标

在现有 1Q 仿真模型库（机载雷达、电子侦察雷达等）基础上，新增 **合成孔径雷达 (SAR) 成像仿真** 能力，实现从回波仿真到图像形成的完整处理链路。

### 设计目标

1. 支持条带模式 (Stripmap) SAR 成像仿真
2. 兼容现有 1Q 公共基础设施（几何变换、大气模型、数值求解器、RCS 计算等）
3. 提供可扩展的算法框架，后续可扩展聚束 (Spotlight)、扫描 (Scan)、滑动聚束 (Sliding Spotlight) 等模式
4. 保证工程级精度与鲁棒性
5. **快慢时间异步解耦**：PRF 与平台宏观步长独立设计，通过环形脉冲缓冲区实现跨步长拼接
6. **聚焦算法受控选择**：Phase 1 固定 RDA；后续在 GBP/BP 等扩展算法完成回归测试和性能基准后，再启用自动选择
7. **相位基准统一**：支持跨算法 / 跨合成孔径图像的相位重参考，避免下游处理伪影
8. **辐射定标闭环**：通过标定目标反推定标常数，支持绝对 RCS 反演与辐射精度评估
9. **轨迹保真度分级**：L1~L3 三级保真度适配不同场景需求

---

## 目录结构

```text
include/1q/sar/
├── sar_types.h                        公共类型定义（枚举、常量、配置结构体）
├── signal/
│   ├── lfm_waveform.h                 LFM 波形生成
│   ├── pulse_compression.h            脉冲压缩处理（含主瓣自适应判定）
│   ├── matched_filter.h               匹配滤波器构造
│   └── antenna_pattern.h              天线方向图建模
├── geometry/
│   ├── platform_dynamics.h            平台轨迹与姿态建模（含 L1~L3 保真度）
│   ├── slant_range_model.h            斜距模型（含弯曲轨迹）
│   └── doppler_model.h                多普勒历程建模
├── echo/
│   ├── raw_echo_generator.h           原始回波数据生成
│   ├── pulse_ring_buffer.h            异步环形脉冲缓冲区
│   └── clutter_model.h               杂波与分布式目标建模
├── imaging/
│   ├── imaging_algorithm.h            成像算法公共接口
│   ├── imaging_selector.h             聚焦算法自适应选择器
│   ├── rda_processor.h                Range-Doppler Algorithm
│   ├── csa_processor.h                Chirp Scaling Algorithm
│   ├── omega_k_processor.h            ω-K (Wavenumber) Algorithm
│   ├── gbp_processor.h                Global Backprojection（全局后向投影）
│   ├── bp_processor.h                 Backprojection（后向投影）
│   └── phase_reference.h              相位重参考（统一相位基准）
├── calibration/
│   ├── motion_compensation.h          运动补偿
│   ├── autofocus.h                    自聚焦算法
│   └── radiometric_cal.h              辐射定标（含 SAR 方程反推）
└── output/
    ├── image_formation.h              成像结果输出
    └── sar_metrics.h                  成像质量评估指标

src/sar/
├── signal/
│   ├── lfm_waveform.cpp
│   ├── pulse_compression.cpp
│   ├── matched_filter.cpp
│   └── antenna_pattern.cpp
├── geometry/
│   ├── platform_dynamics.cpp
│   ├── slant_range_model.cpp
│   └── doppler_model.cpp
├── echo/
│   ├── raw_echo_generator.cpp
│   ├── pulse_ring_buffer.cpp
│   └── clutter_model.cpp
├── imaging/
│   ├── imaging_selector.cpp
│   ├── rda_processor.cpp
│   ├── csa_processor.cpp
│   ├── omega_k_processor.cpp
│   ├── gbp_processor.cpp
│   ├── bp_processor.cpp
│   └── phase_reference.cpp
├── calibration/
│   ├── motion_compensation.cpp
│   ├── autofocus.cpp
│   └── radiometric_cal.cpp
└── output/
    ├── image_formation.cpp
    └── sar_metrics.cpp
```

---

## 模块设计

### 0. 公共类型 (`sar/sar_types.h`)

> 审批口径：SAR 模块落地时应沿用 1Q 现有模块的公开 API 形态，即公开头文件使用 `include/1q/sar/...`、导出类使用 `ONEQ_API`、会话入口采用 `SarSession::Step()` / `SarSession::StepWithResult()`。下列 `oneq::sar` 命名空间仅表示初稿逻辑归属；实施前需统一为仓库实际约定的 `sar::{config,session,...}` 或由项目统一确认命名空间策略。

```cpp
namespace oneq::sar {

// 聚焦算法选择。枚举值进入配置、日志和回放后不得随意重排。
enum class FocusingAlgorithm {
    kAuto    = 0,  // 自适应选择
    kRDA     = 1,  // Range-Doppler Algorithm，Phase 1 默认交付
    kCSA     = 2,  // Chirp Scaling Algorithm，扩展算法
    kOmegaK  = 3,  // Omega-K Algorithm，扩展算法
    kGBP     = 4,  // Global Backprojection，小场景高精度扩展算法
    kBP      = 5,  // Backprojection，非直线轨迹扩展算法
};

// 相位参考模式
enum class PhaseReferenceMode {
    kNative  = 0,  // 算法原生相位
    kCenter  = 1,  // 场景中心统一基准
};

// 主瓣区域判定方法（PSLR/ISLR）
enum class MainlobeEstimationMethod {
    k3dB   = 0,
    kIRW   = 1,  // Integrated Resolution Width（加窗推荐）
    k20dB  = 2,
};

// 平台轨迹保真度
enum class TrajectoryFidelity {
    kL1_Uniform  = 1,  // 理想匀速直线
    kL2_Perturbed = 2, // 匀速 + 高斯扰动
    kL3_Waypoint = 3,  // 折线/航路点序列
};

// SAR 仿真全局配置
struct SarSimulationConfig {
    int range_samples;                 // 距离向采样点数
    int azimuth_samples;               // 方位向采样点数
    double prf_hz;                     // 脉冲重复频率
    double range_sample_rate_hz;       // 距离向采样率
    double macro_step_s;               // 平台宏观步长
    FocusingAlgorithm focus_algo;      // 聚焦算法
    PhaseReferenceMode phase_ref;      // 相位参考模式
    MainlobeEstimationMethod slr_method; // 旁瓣比主瓣判定方法
    TrajectoryFidelity trajectory;     // 轨迹保真度
    bool apply_autofocus;              // 是否启用自聚焦
    bool apply_motion_comp;            // 是否启用运动补偿
    bool apply_multilook;              // 是否启用多视处理
    bool preserve_absolute_rcs;        // 是否保留绝对 RCS（禁止归一化）
    int ring_buffer_capacity;          // 脉冲环形缓冲区容量（脉冲数）
};

} // namespace oneq::sar
```

### 0.1 对外会话契约（实施前置）

SAR 模块不能只交付算法类，还必须补齐与现有雷达模块一致的外部接入契约：

- `include/1q/sar/sar.hpp`：模块统一入口。
- `include/1q/sar/config/SarSessionConfig.h`：四域或等价公开配置聚合，至少包含硬件、任务、策略、环境。
- `include/1q/sar/session/SarCycleInput.h`：单周期输入，包含平台状态、目标/场景输入、时间步长和运行期指令。
- `include/1q/sar/session/SarCycleResult.h`：单周期聚合结果，包含执行状态、聚焦图像摘要、质量指标和错误/降级原因。
- `include/1q/sar/session/SarSession.h`：`Step()` / `StepWithResult()` 高层门面，采用 PIMPL 和 `ONEQ_API`。
- `include/1q/sar/config/SarRuntimeConfigPatch.h`：运行期可变配置补丁，不能直接暴露内部算法对象。

未补齐上述接口前，算法实现只能视为内部原型，不能进入 1Q 公共模块审批。

---

### 1. 信号层 (`sar/signal/`)

#### 1.1 LFM 波形生成 (`lfm_waveform.h`)

```cpp
namespace oneq::sar::signal {

struct LfmWaveformParams {
    double pulse_duration_s;      // 脉冲宽度 (s)
    double bandwidth_hz;          // 信号带宽 (Hz)
    double carrier_freq_hz;       // 载频 (Hz)
    double sample_rate_hz;        // 采样率 (Hz)
    double chirp_slope;           // 调频斜率 K = B/T (Hz/s)
};

class LfmWaveform {
public:
    explicit LfmWaveform(const LfmWaveformParams& params);

    // 生成时域 LFM 信号
    std::vector<std::complex<double>> generate() const;

    // 生成频域匹配滤波器
    std::vector<std::complex<double>> matchedFilter() const;

    // 获取波形参数
    const LfmWaveformParams& params() const;

private:
    LfmWaveformParams params_;
};

} // namespace oneq::sar::signal
```

#### 1.2 脉冲压缩 (`pulse_compression.h`)

```cpp
namespace oneq::sar::signal {

enum class WindowType {
    kNone,
    kHamming,
    kHanning,
    kBlackman,
    kKaiser,
};

struct PulseCompressionResult {
    std::vector<std::complex<double>> compressed_signal;
    double range_resolution_m;                  // 距离分辨率
    double peak_side_lobe_ratio_db;             // 峰值副瓣比
    double integrated_side_lobe_ratio_db;       // 积分副瓣比
    MainlobeEstimationMethod slr_method_used;   // 实际使用的主瓣判定方法
};

class PulseCompressor {
public:
    PulseCompressor(double sample_rate_hz,
                    WindowType window = WindowType::kNone,
                    MainlobeEstimationMethod slr_method = MainlobeEstimationMethod::kIRW);

    // 频域匹配滤波脉冲压缩
    PulseCompressionResult compress(
        const std::vector<std::complex<double>>& echo,
        const std::vector<std::complex<double>>& reference) const;

    // 二维脉冲压缩（距离向 + 方位向）
    Eigen::MatrixXcd compress2d(
        const Eigen::MatrixXcd& echo_matrix,
        const Eigen::VectorXcd& range_reference,
        const Eigen::VectorXcd& azimuth_reference) const;

    // 计算 IRW（Integrated Resolution Width）
    // 用于自适应主瓣判定，避免加窗后 3dB 准则失效
    static double computeIRW(const std::vector<std::complex<double>>& signal);

private:
    double sample_rate_;
    WindowType window_type_;
    MainlobeEstimationMethod slr_method_;

    Eigen::VectorXd generateWindow(int size) const;

    // 根据 slr_method_ 判定主瓣边界
    std::pair<int, int> estimateMainlobeRegion(
        const std::vector<std::complex<double>>& compressed) const;
};

} // namespace oneq::sar::signal
```

#### 1.3 匹配滤波器构造 (`matched_filter.h`)

```cpp
namespace oneq::sar::signal {

class MatchedFilterBuilder {
public:
    // 从发射 LFM 波形构造匹配滤波器
    // 流程: FFT → 复共轭归一化 → IFFT
    static std::vector<std::complex<double>> buildFromChirp(
        const std::vector<std::complex<double>>& chirp);

    // 生成频域匹配核（补零后长度 = chirp_samples + kernel_length）
    static std::vector<std::complex<double>> buildFrequencyKernel(
        const std::vector<std::complex<double>>& chirp,
        int filter_length);

    // 时域匹配滤波器响应
    static std::vector<std::complex<double>> timeDomainResponse(
        const std::vector<std::complex<double>>& chirp);
};

} // namespace oneq::sar::signal
```

#### 1.4 天线方向图 (`antenna_pattern.h`)

```cpp
namespace oneq::sar::signal {

struct AntennaParams {
    double azimuth_beamwidth_deg;  // 方位向波束宽度
    double elevation_beamwidth_deg; // 俯仰向波束宽度
    double azimuth_tilt_deg;       // 方位向波束指向偏移
    double gain_dbi;               // 天线增益
};

class AntennaPattern {
public:
    explicit AntennaPattern(const AntennaParams& params);

    // 计算给定角度处的增益 (dBi)
    double gain(double azimuth_angle_rad, double elevation_angle_rad) const;

    // 生成方位向方向图
    std::vector<double> azimuthPattern(double angle_range_rad, int num_points) const;

private:
    AntennaParams params_;
    // sinc 函数建模（后续可扩展为实测方向图）
    double sincPattern(double angle_rad, double beamwidth_rad) const;
};

} // namespace oneq::sar::signal
```

---

### 2. 几何层 (`sar/geometry/`)

#### 2.1 平台轨迹建模 (`platform_dynamics.h`)

```cpp
namespace oneq::sar::geometry {

struct TrajectoryPoint {
    double time_s;
    Eigen::Vector3d position;       // ECEF 或本地坐标系位置
    Eigen::Vector3d velocity;       // 速度矢量
    Eigen::Vector3d acceleration;   // 加度矢量
    Eigen::Quaterniond attitude;    // 姿态四元数
};

class PlatformDynamics {
public:
    // L1: 理想匀速直线运动（默认）
    static PlatformDynamics createUniform(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& velocity,
        double duration_s,
        double prf_hz);

    // L2: 匀速直线 + 速度扰动（高斯噪声，模拟气流扰动）
    static PlatformDynamics createPerturbedUniform(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& velocity,
        double duration_s,
        double prf_hz,
        double velocity_noise_std_mps);

    // L3: 折线 / 航路点序列（聚束模式转弯、斜视模式）
    static PlatformDynamics createWaypointSequence(
        const std::vector<TrajectoryPoint>& waypoints,
        double prf_hz);

    // L4: 从外部轨迹文件加载（IMU/GPS 数据，超出本方案范围）

    // 获取指定时刻的平台状态（含插值）
    TrajectoryPoint stateAt(double time_s) const;

    // 获取所有采样时刻
    const std::vector<TrajectoryPoint>& trajectory() const;

    // 平台运动方向单位矢量
    Eigen::Vector3d velocityDirection() const;

    // 当前轨迹保真度
    TrajectoryFidelity fidelity() const;

private:
    std::vector<TrajectoryPoint> points_;
    TrajectoryFidelity fidelity_;
};

} // namespace oneq::sar::geometry
```

#### 2.2 斜距模型 (`slant_range_model.h`)

```cpp
namespace oneq::sar::geometry {

class SlantRangeModel {
public:
    SlantRangeModel(const PlatformDynamics& platform,
                    const Eigen::Vector3d& target_position);

    // 精确斜距（考虑弯曲轨迹）
    double exactRange(double time_s) const;

    // 二次近似斜距 R(t) ≈ R0 + v_r*t + 0.5*a_r*t^2
    double quadraticApprox(double time_s) const;

    // 最近斜距 R0
    double closestRange() const;

    // 斜距变化率
    double rangeRate(double time_s) const;

private:
    const PlatformDynamics& platform_;
    Eigen::Vector3d target_;
    double R0_;
};

} // namespace oneq::sar::geometry
```

#### 2.3 多普勒历程 (`doppler_model.h`)

```cpp
namespace oneq::sar::geometry {

struct DopplerParams {
    double fd_central;          // 多普勒中心频率
    double fd_rate;             // 多普勒调频率 (Hz/s)
    double synthetic_aperture_time_s; // 合成孔径时间
    double doppler_bandwidth_hz;     // 多普勒带宽
};

class DopplerModel {
public:
    DopplerModel(const PlatformDynamics& platform,
                 const SlantRangeModel& slant_range,
                 double wavelength_m);

    // 计算多普勒参数
    DopplerParams compute() const;

    // 计算任意时刻的多普勒频率
    double frequencyAt(double time_s) const;

    // 方位向分辨率
    double azimuthResolution() const;

private:
    const PlatformDynamics& platform_;
    const SlantRangeModel& slant_range_;
    double wavelength_;
};

} // namespace oneq::sar::geometry
```

---

### 3. 回波仿真层 (`sar/echo/`)

#### 3.1 原始回波生成 (`raw_echo_generator.h`)

```cpp
namespace oneq::sar::echo {

struct EchoConfig {
    int range_samples;           // 距离向采样点数
    int azimuth_samples;         // 方位向采样点数
    double prf_hz;               // 脉冲重复频率
    double range_sample_rate_hz; // 距离向采样率
};

struct PointTarget {
    Eigen::Vector3d position;    // 目标三维坐标
    double rcs_dbsm;             // 目标 RCS (dBsm)
};

class RawEchoGenerator {
public:
    RawEchoGenerator(const signal::LfmWaveform& waveform,
                     const geometry::PlatformDynamics& platform,
                     const EchoConfig& config);

    // 生成单点目标回波
    Eigen::MatrixXcd generateEcho(const PointTarget& target) const;

    // 生成多点目标回波（叠加）
    Eigen::MatrixXcd generateEcho(const std::vector<PointTarget>& targets) const;

    // 生成分布式目标回波（面目标场景）
    Eigen::MatrixXcd generateEcho(const SceneDescription& scene) const;

    // 添加噪声
    static Eigen::MatrixXcd addNoise(const Eigen::MatrixXcd& echo,
                                     double snr_db);

private:
    const signal::LfmWaveform& waveform_;
    const geometry::PlatformDynamics& platform_;
    EchoConfig config_;
};

} // namespace oneq::sar::echo
```

#### 3.2 异步环形脉冲缓冲区 (`pulse_ring_buffer.h`)

```cpp
namespace oneq::sar::echo {

// 环形脉冲缓冲区，实现快慢时间异步解耦
// PRF 由方位向采样定理独立决定，宏观步长由任务推演独立决定
// SAR 聚焦处理器可跨宏观步长从缓冲区读取连续 N_aperture 个脉冲
class PulseRingBuffer {
public:
    explicit PulseRingBuffer(int capacity_pulses);

    // 写入一个脉冲回波（线程安全）
    // 容量不足时覆盖最旧数据，标记溢出
    bool push(int64_t pulse_id, const Eigen::VectorXcd& echo);

    // 读取最近 N 个连续脉冲（从最新位置向前）
    // 不足时返回 false
    bool popLatestN(int n, std::vector<Eigen::VectorXcd>* echoes_out,
                    std::vector<int64_t>* pulse_ids_out) const;

    // 读取指定 ID 范围的脉冲
    bool popRange(int64_t start_id, int64_t end_id,
                  std::vector<Eigen::VectorXcd>* echoes_out) const;

    // 当前缓冲区状态
    int size() const;
    int capacity() const;
    bool isOverflow() const;
    void clear();

private:
    struct Slot {
        int64_t pulse_id;
        Eigen::VectorXcd echo;
    };

    mutable std::mutex mtx_;
    std::vector<Slot> slots_;
    size_t head_ = 0;
    size_t tail_ = 0;
    bool overflow_ = false;
};

} // namespace oneq::sar::echo
```

#### 3.3 杂波建模 (`clutter_model.h`)

```cpp
namespace oneq::sar::echo {

class ClutterModel {
public:
    // 地面杂波散射系数模型
    static double gammaModel(double incidence_angle_rad,
                             double frequency_hz,
                             const std::string& terrain_type);

    // 海杂波模型
    static double seaClutter(double incidence_angle_rad,
                             double wind_speed_ms,
                             double frequency_hz);

    // 生成杂波场景
    static SceneDescription generateClutterScene(
        const geometry::PlatformDynamics& platform,
        double scene_width_m,
        double scene_height_m,
        double range_resolution_m,
        double azimuth_resolution_m,
        const std::string& terrain_type);
};

} // namespace oneq::sar::echo
```

---

### 4. 成像算法层 (`sar/imaging/`)

#### 4.1 公共接口 (`imaging_algorithm.h`)

```cpp
namespace oneq::sar::imaging {

struct ImagingConfig {
    bool apply_autofocus = false;     // 是否启用自聚焦
    bool apply_motion_comp = false;   // 是否启用运动补偿
    bool apply_multilook = false;     // 是否启用多视处理
    int multilook_azimuth = 1;        // 方位向视数
    int multilook_range = 1;          // 距离向视数
    signal::WindowType range_window = signal::WindowType::kHamming;
    signal::WindowType azimuth_window = signal::WindowType::kHamming;
    PhaseReferenceMode phase_ref = PhaseReferenceMode::kCenter;
};

struct ImagingResult {
    Eigen::MatrixXd amplitude_db;            // 幅度图像 (dB)
    Eigen::MatrixXd phase;                   // 相位图像
    Eigen::MatrixXcd complex_image;          // 复图像
    double range_resolution_m;
    double azimuth_resolution_m;
    double peak_side_lobe_ratio_db;
    double integrated_side_lobe_ratio_db;
    double image_entropy;                    // 图像熵
    double radiometric_accuracy_db = NAN;    // 辐射精度（dB），未定标时为 NaN
    FocusingAlgorithm selected_algorithm;    // 实际使用的算法
    bool phase_reference_applied;            // 是否执行了相位重参考
};

class ImagingAlgorithm {
public:
    virtual ~ImagingAlgorithm() = default;

    // 执行成像处理
    virtual ImagingResult process(
        const Eigen::MatrixXcd& raw_echo,
        const EchoConfig& echo_config,
        const signal::LfmWaveform& waveform,
        const geometry::PlatformDynamics& platform,
        const geometry::SlantRangeModel& slant_range,
        const ImagingConfig& config) const = 0;

    // 算法名称
    virtual std::string name() const = 0;
};

} // namespace oneq::sar::imaging
```

#### 4.2 聚焦算法自适应选择器 (`imaging_selector.h`)

```cpp
namespace oneq::sar::imaging {

class ImagingSelector {
public:
    // 根据场景规模、轨迹保真度与算力自动选择聚焦算法。
    // Phase 1 正式运行只允许 kRDA 或显式用户指定算法；
    // kAuto 需在 RDA/GBP/BP 均有回归测试和性能基准后启用。
    static FocusingAlgorithm selectAuto(
        int64_t scene_pixels,
        int num_threads,
        FocusingAlgorithm user_requested);

    // 工厂方法创建对应算法实例
    static std::unique_ptr<ImagingAlgorithm> create(FocusingAlgorithm algo);

    // 评估当前选择是否需要降级警告
    static bool shouldLogDegradationWarning(
        FocusingAlgorithm user_requested,
        FocusingAlgorithm actually_used);
};

} // namespace oneq::sar::imaging
```

#### 4.3 Range-Doppler Algorithm (`rda_processor.h`)

```cpp
namespace oneq::sar::imaging {

// 经典 RDA 处理流程，Phase 1 审批默认算法:
// 1. 距离向 FFT
// 2. 距离向匹配滤波
// 3. 距离向 IFFT
// 4. 方位向 FFT（转到距离-多普勒域）
// 5. RCMC（距离单元徙动校正）—  sinc 插值
// 6. 方位向匹配滤波
// 7. 方位向 IFFT
class RdaProcessor : public ImagingAlgorithm {
public:
    ImagingResult process(...) const override;
    std::string name() const override { return "Range-Doppler Algorithm"; }

private:
    // RCMC 核心：sinc 插值校正距离徙动
    Eigen::MatrixXcd rcmc(const Eigen::MatrixXcd& rd_domain,
                           const geometry::DopplerModel& doppler,
                           double range_sample_rate) const;
};

} // namespace oneq::sar::imaging
```

#### 4.4 Chirp Scaling Algorithm (`csa_processor.h`)

```cpp
namespace oneq::sar::imaging {

// CSA 处理流程:
// 1. 距离向 FFT
// 2. Chirp Scaling 操作（相位相乘，统一徙动曲线）
// 3. 方位向 FFT
// 4. 二级相位相乘（SRC + RCMC 合并）
// 5. 方位向 IFFT
// 6. 距离向 IFFT
class CsaProcessor : public ImagingAlgorithm {
public:
    ImagingResult process(...) const override;
    std::string name() const override { return "Chirp Scaling Algorithm"; }
};

} // namespace oneq::sar::imaging
```

#### 4.5 ω-K Algorithm (`omega_k_processor.h`)

```cpp
namespace oneq::sar::imaging {

// ω-K (Wavenumber) 算法:
// 1. 二维 FFT
// 2. Stolt 插值（频率轴映射）
// 3. 二维相位相乘
// 4. 二维 IFFT
class OmegaKProcessor : public ImagingAlgorithm {
public:
    ImagingResult process(...) const override;
    std::string name() const override { return "Omega-K Algorithm"; }
};

} // namespace oneq::sar::imaging
```

#### 4.6 Global Backprojection (`gbp_processor.h`)

```cpp
namespace oneq::sar::imaging {

// 全局后向投影（Global Backprojection, GBP）
// 对每个输出像素 (j0, k0)，遍历全部方位采样位置 l：
//   I_SAR[j0][k0] = Σ_l I_rad[l][ r_{l,k0,j0} ]
// 其中 r_{l,k0,j0} = √((l - j0)² + k0²)
//
// 复杂度 O(N_pixel × N_pulse)，仅作为小场景高精度扩展算法。
// Phase 1 不以 GBP 作为默认聚焦路径。
class GbpProcessor : public ImagingAlgorithm {
public:
    ImagingResult process(...) const override;
    std::string name() const override { return "Global Backprojection"; }

    // 并行配置
    void setNumThreads(int n) { num_threads_ = n; }
    void setUseGpu(bool enable) { use_gpu_ = enable; }

private:
    int num_threads_ = 1;
    bool use_gpu_ = false;

    // 单脉冲回波距离门索引计算
    int computeRangeGateIndex(double azimuth_pos, double range_pos,
                              int num_azimuth, int num_range) const;
};

} // namespace oneq::sar::imaging
```

#### 4.7 Backprojection (`bp_processor.h`)

```cpp
namespace oneq::sar::imaging {

// 标准后向投影（Backprojection, BP）
// 与 GBP 的区别：BP 通常按方位位置遍历累加到像素，
//                GBP 按像素遍历累加脉冲。
// 适用场景：聚束模式、非直线轨迹。需要运动补偿和轨迹误差验收后启用。
class BpProcessor : public ImagingAlgorithm {
public:
    ImagingResult process(...) const override;
    std::string name() const override { return "Backprojection"; }

    // 快速 BP 变体（后续扩展）
    // void processFbp(...) const;
};

} // namespace oneq::sar::imaging
```

#### 4.8 相位重参考 (`phase_reference.h`)

```cpp
namespace oneq::sar::imaging {

// 相位重参考：解决 GBP 与 RDA 等不同算法输出的相位基准不一致问题
// 当 phase_ref = kCenter 时，统一以场景中心 R0 零多普勒双程延迟为基准
// 当 phase_ref = kNative 时，保持各算法原生相位
class PhaseReference {
public:
    // 对聚焦图像施加补偿相位
    //   I'_SAR = I_SAR * exp(-j * 4π R0 / λ)
    static Eigen::MatrixXcd applyCenterReference(
        const Eigen::MatrixXcd& image,
        double scene_center_range_m,
        double wavelength_m);

    // 检查是否需要相位重参考
    static bool needsReference(
        PhaseReferenceMode mode,
        FocusingAlgorithm algo_current,
        FocusingAlgorithm algo_previous = FocusingAlgorithm::kAuto);
};

} // namespace oneq::sar::imaging
```

---

### 5. 校准与补偿层 (`sar/calibration/`)

#### 5.1 运动补偿 (`motion_compensation.h`)

```cpp
namespace oneq::sar::calibration {

struct MotionCompensationParams {
    bool enable_high_order = true;   // 高阶运动补偿
    double filter_cutoff_hz = 10.0;  // IMU 数据低通滤波截止频率
};

class MotionCompensator {
public:
    explicit MotionCompensator(const MotionCompensationParams& params = {});

    // 一阶运动补偿（包络校正 + 相位校正）
    Eigen::MatrixXcd compensate(
        const Eigen::MatrixXcd& raw_echo,
        const geometry::PlatformDynamics& ideal_trajectory,
        const geometry::PlatformDynamics& actual_trajectory) const;

    // 二阶运动补偿（基于图像域的残余误差校正）
    Eigen::MatrixXcd compensateHighOrder(
        const Eigen::MatrixXcd& partially_focused,
        const geometry::PlatformDynamics& residual_error) const;

private:
    MotionCompensationParams params_;
};

} // namespace oneq::sar::calibration
```

#### 5.2 自聚焦 (`autofocus.h`)

```cpp
namespace oneq::sar::calibration {

enum class AutofocusMethod {
    kPhaseGradient,    // PGA (Phase Gradient Autofocus)
    kMapDrift,         // Map Drift
    kContrastOptimization, // 对比度最优
};

class Autofocus {
public:
    explicit Autofocus(AutofocusMethod method = AutofocusMethod::kPhaseGradient);

    // 估计方位向相位误差
    Eigen::VectorXd estimatePhaseError(
        const Eigen::MatrixXcd& focused_image,
        int azimuth_bin) const;

    // 应用相位校正
    Eigen::MatrixXcd applyCorrection(
        const Eigen::MatrixXcd& image,
        const Eigen::VectorXd& phase_correction) const;

private:
    AutofocusMethod method_;
};

} // namespace oneq::sar::calibration
```

#### 5.3 辐射定标 (`radiometric_cal.h`)

```cpp
namespace oneq::sar::calibration {

struct CalTargetInfo {
    Eigen::Vector3d position;     // 标定目标位置
    double known_rcs_dbsm;        // 已知 RCS (dBsm)
    double image_peak;            // 图像峰值像素幅度
};

struct CalResult {
    double k_cal;                 // 辐射定标常数
    double residual_error_db;     // 定标残差
    bool valid;                   // 定标是否有效
    int num_targets_used;         // 使用的标定目标数
};

class RadiometricCalibrator {
public:
    // 基于 SAR 方程反推定标常数
    //   K_cal = (P_t * G^2 * λ^2 * σ_cal) / ((4π)^3 * R^4 * |I_cal|^2)
    static CalResult calibrateSingle(
        const CalTargetInfo& target,
        double tx_power_w,
        double antenna_gain_linear,
        double wavelength_m,
        double slant_range_m);

    // 多点定标加权融合（按残差平方倒数加权）
    //   K_cal_fused = Σ w_n * K_cal,n / Σ w_n, w_n = 1/Δ_cal,n^2
    static CalResult calibrateMultiTarget(
        const std::vector<CalTargetInfo>& targets,
        double tx_power_w,
        double antenna_gain_linear,
        double wavelength_m);

    // 绝对 RCS 反演
    static double invertRCS(
        const Eigen::MatrixXcd& sar_image,
        int row, int col,
        double slant_range_m,
        const CalResult& cal);

    // 辐射精度评估
    static double evaluateRadiometricAccuracy(
        double measured_rcs,
        double theoretical_rcs);
};

} // namespace oneq::sar::calibration
```

---

### 6. 输出层 (`sar/output/`)

#### 6.1 成像结果输出 (`image_formation.h`)

```cpp
namespace oneq::sar::output {

class ImageFormatter {
public:
    // 输出为 HDF5 格式（工程级标准）
    static void toHdf5(const imaging::ImagingResult& result,
                       const std::string& filepath);

    // 输出为 GeoTIFF（带地理坐标）
    static void toGeoTiff(const imaging::ImagingResult& result,
                          const std::string& filepath,
                          double origin_lat,
                          double origin_lon);

    // 输出为简单二进制格式（调试用）
    static void toBinary(const imaging::ImagingResult& result,
                         const std::string& filepath);
};

} // namespace oneq::sar::output
```

#### 6.2 成像质量评估 (`sar_metrics.h`)

```cpp
namespace oneq::sar::output {

struct SarMetrics {
    double range_resolution_3db_m;     // -3dB 距离分辨率
    double azimuth_resolution_3db_m;   // -3dB 方位分辨率
    double peak_side_lobe_db;          // 峰值副瓣比
    double integrated_side_lobe_db;    // 积分副瓣比
    double image_entropy;              // 图像熵（聚焦质量指标）
    double image_contrast;             // 图像对比度
    double radiometric_accuracy_db;    // 辐射精度（dB）
    MainlobeEstimationMethod slr_method_used; // 实际使用的主瓣判定方法
    bool phase_reference_applied;      // 是否执行了相位重参考
};

class MetricsCalculator {
public:
    // 计算点目标响应指标
    static SarMetrics computePointTargetMetrics(
        const Eigen::MatrixXcd& complex_image,
        int target_row,
        int target_col,
        double range_pixel_spacing_m,
        double azimuth_pixel_spading_m,
        MainlobeEstimationMethod slr_method = MainlobeEstimationMethod::kIRW);

    // 计算全图指标
    static SarMetrics computeImageMetrics(
        const Eigen::MatrixXd& amplitude_db,
        double range_pixel_spacing_m,
        double azimuth_pixel_spacing_m);
};

} // namespace oneq::sar::output
```

---

## 与现有 1Q 模块的复用关系

| SAR 模块需求 | 复用 1Q 现有模块 |
|-------------|-----------------|
| 坐标变换、斜距计算 | `common/geometry/` |
| 大气传播延迟修正 | `common/atmosphere/` |
| FFT/IFFT、数值积分 | `common/numerics/` |
| 目标 RCS 计算 | `common/rcs/` |
| 日志输出 | `common/logging/` |
| 时序管理 | `common/timing/` |
| 运行时基础设施 | `common/runtime/` |
| 验证辅助 | `common/validation/` |

---

## 算法实现优先级

| 阶段 | 内容 | 预计工作量 |
|------|------|-----------|
| **Phase 0** | `SarSession`/`SarCycleInput`/`SarCycleResult`/配置补丁/trace-replay 契约 + CMake 安装清单 | 公共 API 冻结 |
| **Phase 1** | LFM 波形 + 匹配滤波器 + 距离向脉冲压缩（含 IRW） + 点目标原始回波仿真 + RDA 成像 + 脉冲环形缓冲区 | 最小可审批闭环 |
| **Phase 2** | 成像选择器受控启用 + 相位重参考 + GBP 小场景实现 + 质量指标闭环 | 成像算法扩展 |
| **Phase 3** | CSA + Omega-K + BP + 运动补偿 + 自聚焦 + 杂波建模 | 高级算法增强 |
| **Phase 4** | 辐射定标 + 聚束/扫描模式 + 多视处理 + L2/L3 轨迹保真度 + HDF5/GeoTIFF 输出 | 完整功能集 |
| **Phase 5** | OpenMP/GPU 加速 + 实时处理 + 与现有雷达模块联合仿真 | 性能优化 |

---

## 参考文献

1. **I.G. Cumming, F.H. Wong**, *Digital Processing of Synthetic Aperture Radar Data: Algorithms and Implementation*, Artech House, 2005.
2. **C. Oliver, S. Quegan**, *Understanding Synthetic Aperture Radar Images*, SciTech Publishing, 1998.
3. **J.C. Curlander, R.N. McDonough**, *Synthetic Aperture Radar: Systems and Signal Processing*, Wiley, 1991.
4. **R. Bamler, P. Hartl**, "Synthetic Aperture Radar Interferometry", *Inverse Problems*, 1998.

---

## 验证策略

### 单元测试

- LFM 波形生成 → 验证采样点数、调频斜率、瞬时频率单调性和能量归一化约定
- 匹配滤波器 → 验证 `h[n] = s*[N-1-n]` 与频域核 `H[k] = S*[k]` 不重复共轭
- 脉冲压缩 → 验证点目标响应的距离分辨率、副瓣比（含 3dB/IRW/20dB 三种主瓣判定）
- 斜距模型 → 对比解析解与数值解
- 多普勒参数 → 验证中心频率、调频率计算
- 脉冲环形缓冲区 → 验证并发读写、覆盖、连续 pulse_id 检查、跨步长拼接和分数脉冲累积
- 成像选择器 → Phase 1 仅验证显式 RDA；Auto 模式在扩展算法测试齐备前不得进入正式验收
- 辐射定标 → 验证 SAR 方程反推与多点加权融合
- 相位重参考 → 验证跨算法输出相位基准对齐

### 集成测试

- Phase 1 点目标成像 → 用固定点目标阵列验证 RDA 的距离/方位聚焦、PSLR/ISLR、图像熵和峰值位置误差
- 外部工具对比 → 先使用仓库内固定参数参考数据或 MATLAB/Python 参考脚本；公开真实数据集对比进入 Phase 4 后验收
- 算法对比 → Phase 2 后在同一场景下对比 RDA、GBP；CSA/Omega-K/BP 各自进入对应阶段后再纳入矩阵
- GBP→RDA 自动切换 → 仅在 Auto 模式审批启用后验证相位重参考生效、图像无伪影
- 轨迹 L1/L2/L3 对比 → 验证机动场景下的成像一致性
- 辐射精度端到端 → 已知 RCS 目标的实测反演误差 < 1 dB

### 性能基准

- Phase 1 正式冻结 1024×1024 为当前平台运行时上限；超过该尺寸继续门禁并进入后续阶段审批
- Phase 1 验证 1024×1024 FFT、内部 RDA、真实点目标管线和 public Session 的处理时间与峰值内存
- 2048×2048、4096×4096 属于后续阶段扩容基准，不作为 Phase 1 审批项
- 内存占用峰值
- 与固定参考实现的数值对比：波形和脉冲压缩可使用严格复数误差；成像结果以峰值位置、分辨率、PSLR/ISLR、熵和归一化幅相误差验收，不使用笼统的全图 `< 1e-10` 作为唯一标准
- GBP vs RDA 自适应切换的边界响应时间（Auto 模式启用后）
- 环形缓冲区在跨步长拼接时的吞吐量

### Phase 1 已冻结兼容与 Replay 决策

- SAR engine 必须在当前验证平台以 C++11 + Eigen 3.3.9 编译通过；Windows/VS2015 不作为 Phase 1 强制审批门
- Phase 1 trace/replay 仅记录 public 摘要，不序列化 focused complex image 全矩阵
- 全图复数矩阵 replay、外部 artifact 引用和图像产品格式进入后续阶段单独审批

---

*文档版本: v2.0*
*创建日期: 2026-06-04*
*更新日期: 2026-06-04*
*变更摘要: 对齐 sar_construction_scheme_complete.md 第二轮改造，新增异步环形脉冲缓冲区、聚焦算法自适应选择器、GBP 与 BP 分离、相位重参考、辐射定标细化、轨迹保真度分级（v1.0 → v2.0）*
