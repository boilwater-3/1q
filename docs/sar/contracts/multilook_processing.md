# SAR 多视(Multilook)降斑后处理工程契约

Date: 2026-06-24
状态: **已实现。聚焦后图像域非相干多视 10 测试全绿,纯后处理零侵入,全量 SAR 测试零回归。**

> **实现记录(2026-06-24)**:多视已实现。新增 `src/sar/imaging/SarMultilook.{h,cpp}` +
> `tests/unit/sar_multilook_test.cpp`,10 个测试覆盖契约 §4-6 全部不变量与拒绝路径:
> 1. `SingleLookDegeneratesToAmplitudeImage` — looks=(1,1) 退化为逐像素幅度图(尺寸不变)。
> 2. `OutputSizeShrinksByLookCount` — looks=(N,M) 输出 = (⌊rows/N⌋×⌊cols/M⌋) 降采样。
> 3. `SpeckleStddevDecreasesBySqrtENL` — 4×4 视(ENL=16)变异系数降至单视的 <50%(1/√16≈0.25)。
> 4. `PowerAverageProducesFiniteOutput` — 幅度/功率平均均收敛。
> 5. `PointTargetPeakPreserved` — 点目标多视后峰值可定位。
> 6. `Deterministic` + 4 个拒绝路径。
>
> **语义冻结**:`looks` = 每视像素数(降采样步长),非"视数"。looks=1 每像素自成一视(不降采样);
> looks=N 每 N 像素块平均成 1 像素。这保证单视退化不变量。
>
> **零侵入**:多视是独立后处理工具(消费任意 ComplexMatrix),不改聚焦器/输出层/session/配置。
前置评估: `multilook_value_assessment.md`(阶段 A 价值评估——图像域多视实现,RD 域/raw 域冻结)

> **本契约范围(据阶段 A 评估决策)**:
> - **仅实现聚焦后图像域非相干多视**(路径 α)。
> - **不实现** RD 域多视(侵入 RDA 黑盒,仅 RDA 适用,不通用——评估 §3.3)。
> - **不实现** raw history 域多视(与图像域等价但贵 N 倍——评估 §3.2)。
> - **零侵入**:多视是独立后处理工具,不改聚焦器/输出层/session/配置。

## 1. 目标

提供 SAR 聚焦图像的**非相干多视降斑**后处理:对聚焦复图像在方位/距离向分块(视),
每视取幅度(或功率)后平均,以 N 倍分辨率为代价换取相干斑标准差降至 1/√N。

**物理本质**:SAR 相干成像产生乘性斑噪声(speckle)。多视把全孔径分成 N 个子孔径(视),
各视独立成像后非相干叠加——斑噪声(随机相位)因非相干而部分抵消,真实散射(相干)保留。
等效视数 ENL 越大,斑噪声方差越小,但空间分辨率越低。

## 2. 作用对象与通用性

多视作用于**任意聚焦复图像**(`signal::ComplexMatrix`),与聚焦算法解耦:

| 聚焦器 | 图像结构 | 图像体字段 | 多视适用 |
|---|---|---|---|
| L1-RDA | `FocusedSarImage` | `.image`(`SarRda.h:62`) | ✅ |
| GBP/BP | `FocusedGbpImage` | `.image`(`SarGbp.h:45`) | ✅ |
| Omega-K | `FocusedOmegaKImage` | `.image`(`SarOmegaKFocusing.h:49`) | ✅ |
| ScanSAR | `FocusedScanSarSubswath` | `.image`(`SarScanSarFocusing.h:60`) | ✅ |

四种结构的图像体均为 `signal::ComplexMatrix`(`SarFft.h:22`,行主序 POD),
多视直接消费,无需感知聚焦器内部细节。

## 3. 算法契约(冻结)

### 3.1 分块策略

对聚焦图像 `image`(rows × cols),`looks` 是**每视包含的像素数(降采样步长)**:
沿方位(行)向每 `azimuth_looks` 个像素为一视,沿距离(列)向每 `range_looks` 个像素为一视。

```
输出行数 = floor(rows / azimuth_looks)
输出列数 = floor(cols / range_looks)
每输出像素 = 一个 azimuth_looks × range_looks 块的平均
```

- 分块为**相邻不重叠**子块(非滑动窗口)——标准多视的子孔径划分。
- 不足整除的边缘行/列被丢弃(向下取整),保证每视等大、统计一致。
- 输出图像 = 输入的降采样图(每视块 → 1 像素),尺寸 = (⌊rows/azimuth_looks⌋, ⌊cols/range_looks⌋)。

> 语义说明:`looks` 是降采样步长(每视像素数),不是"视数"。looks=1 表示每像素自成一视
> (不降采样);looks=N 表示每 N 像素块平均成 1 像素(N 倍降采样,等效视数 ENL=N²)。
> 这保证了单视退化不变量(looks=1 → 输出 = 输入逐像素幅度图,尺寸不变)。

### 3.2 非相干平均类型(冻结二选一)

```
幅度平均(amplitude):output_pixel = mean( |sub_block| )           // |·| 取模
功率平均(power):     output_pixel = sqrt( mean( |sub_block|² ) )  // |·|² 取功率,再开方回幅度域
```

- **幅度平均**(默认):对幅度 `|z|` 直接平均。简单,对点目标峰值保持较好。
- **功率平均**:对功率 `|z|²` 平均后开方。理论上对相干斑抑制略优(功率域均匀分布方差更小)。

两者均输出**幅度图**(实数,无相位)——多视降斑的产物是检测后的幅度/强度图,非复图像。
这是物理正确的:非相干叠加后相位无意义。

### 3.3 配置(冻结)

```cpp
struct MultilookConfig {
  std::size_t azimuth_looks{1U};   // 方位每视像素数(降采样步长),≥1
  std::size_t range_looks{1U};     // 距离每视像素数(降采样步长),≥1
  enum class AverageType { kAmplitude, kPower } average_type{AverageType::kAmplitude};
};
```

- `azimuth_looks=1` 且 `range_looks=1`:单视退化(每像素自成一视,输出 = 输入幅度图,尺寸不变)。
- `azimuth_looks>1` 或 `range_looks>1`:多视降斑,输出尺寸按 ⌊rows/looks⌋ 缩小,分辨率下降。

### 3.4 输出

多视产物是**实数幅度图**(非复图像)。输出形态:

```cpp
// 实数矩阵(幅度图),行主序。
struct RealMatrix {
  std::size_t rows{0U};
  std::size_t cols{0U};
  std::vector<double> values{};
};
```

> 用独立的 `RealMatrix` 而非复用 `ComplexMatrix`——多视产物是幅度图(实数),语义上与复图像
> 不同。这与聚束/扫描聚焦器输出复图像的约定区分开。

## 4. 接口与不变量

### 4.1 接口(冻结)

```cpp
// src/sar/imaging/SarMultilook.h(新建)
namespace sar::imaging {

struct MultilookConfig { /* §3.3 */ };
struct RealMatrix { /* §3.4 */ };

/**
 * @brief 聚焦后图像域非相干多视降斑。
 *
 * 对聚焦复图像按 azimuth_looks × range_looks 分块,每视取幅度/功率平均,输出实数幅度图。
 * 算法复杂度 O(rows×cols),无 FFT。与聚焦算法解耦(消费任意 ComplexMatrix)。
 *
 * 不变量:
 * 1. 单视退化:looks=(1,1) 时,输出幅度图 = 输入图像的逐像素 |·|(尺寸不变)。
 * 2. 分辨率下降:looks=(N,M) 时,输出尺寸 = (rows/N, cols/M) 向下取整。
 * 3. 降斑增强:looks 越大,均匀区域幅度方差越小(1/√ENL 统计)。
 * 4. 确定性 + 无 NaN/Inf。
 */
bool ApplyMultilook(const MultilookConfig& config, const signal::ComplexMatrix& focused_image,
                    RealMatrix* output);
}  // namespace sar::imaging
```

### 4.2 不变量(冻结)

1. **单视退化**:looks=(1,1) → 输出 = 输入的逐像素幅度图(`output(i,j) = |input(i,j)|`),尺寸不变。
2. **尺寸收缩**:looks=(N,M) → 输出尺寸 = (⌊rows/N⌋, ⌊cols/M⌋),丢弃边缘余数(降采样)。
3. **降斑单调性**:固定输入,azimuth_looks 或 range_looks 增大 → 均匀区域幅度标准差下降(1/√ENL)。
4. **峰值保持**:点目标在多视后仍可定位(峰值像素为该视块的最大幅度来源)。
5. 确定性(同输入同输出)+ 全有限(无 NaN/Inf)。

### 4.3 拒绝路径(冻结)

- `output == nullptr` → false。
- `focused_image` 空(rows=0 或 cols=0 或 values 尺寸不符)→ false。
- `azimuth_looks == 0` 或 `range_looks == 0` → false。
- `azimuth_looks > rows` 或 `range_looks > cols`(视数超过图像尺寸,无法分块)→ false。

## 5. 实现指南(非冻结)

### 5.1 文件组织

```
src/sar/imaging/SarMultilook.h     // MultilookConfig + RealMatrix + ApplyMultilook 声明
src/sar/imaging/SarMultilook.cpp   // 实现
src/sar/SarSources.cmake           // 注册 SarMultilook.cpp 到 SAR_ENGINE_SOURCES
tests/unit/sar_multilook_test.cpp  // 测试
```

### 5.2 实现要点

- 分块:对每个输出像素 (i, j),对应输入块 `[i*block_rows, (i+1)*block_rows) ×
  [j*block_cols, (j+1)*block_cols)`,其中 `block_rows = rows/azimuth_looks`,
  `block_cols = cols/range_looks`(整数除法,自动丢弃余数)。
- 幅度平均:`output(i,j) = mean(|input(r,c)| for r,c in block)`。
- 功率平均:`output(i,j) = sqrt(mean(|input(r,c)|² for r,c in block))`。
- 无 FFT、无插值、无相位——纯空间域统计平均。

## 6. 验收门

1. **单视退化**:looks=(1,1) 输出 = 输入逐像素幅度图(逐元素验证)。
2. **尺寸收缩**:looks=(N,M) 输出尺寸正确(⌊rows/N⌋ × ⌊cols/M⌋)。
3. **降斑效果量化**:构造含斑噪声的合成图像(随机复数),多视后均匀区域幅度标准差
   按 1/√ENL 下降(looks=4 标准差 ≈ looks=1 的 1/2)。
4. **幅度 vs 功率平均**:两者均收敛(功率平均方差略小)。
5. **峰值保持**:点目标场景多视后峰值位置可定位。
6. **拒绝路径**:空指针/空图像/零视数/视数超尺寸均结构化拒绝。
7. **全量回归**:现有 SAR 测试零回归(多视是新文件,不改任何现有代码)。
8. **门验证**:Eigen 3.3.9/C++11、sar_ci(含 frozen_sources)、sar_performance 通过。

## 7. 非目标(据阶段 A 评估)

- **不实现** RD 域多视(侵入 RDA,仅 RDA 适用,不通用——评估 §3.3)。
- **不实现** raw history 域多视(与图像域等价但贵 N 倍——评估 §3.2)。
- **不进** session 数据链路(独立后处理工具,与聚束/扫描一致,走自由函数)。
- **不改**任何聚焦器/输出层/配置结构(零侵入)。
- **不实现**滑动窗口多视(本契约用非重叠分块,标准子孔径划分)。
- **不实现**自适应多视/加权多视(后置)。
- **不输出**复图像(多视产物是幅度图,非相干叠加后相位无意义)。
- **不引入** GeoTIFF(独立子项)。
- 不重开任何冻结项。

## 8. 实现难度评估

| 维度 | 评估 |
|---|---|
| 算法 | 🟢 极低(分块 + 幅度/功率平均,O(N) 无 FFT) |
| 聚焦器改动 | 🟢 零(消费任意 ComplexMatrix) |
| session/配置改动 | 🟢 零(独立后处理工具) |
| 测试 | 🟢 低(退化/尺寸/降斑量化/拒绝,6-8 用例) |
| 预计人天 | **3-5**(纯后处理,据阶段 A 评估 §6) |
| 对比 phase4 原估 | phase4 §7 原估 6-10 人天(含 RD 域/raw 域路径探索);阶段 A 收窄到图像域,降至 3-5 |

**难度极低的关键**:多视是纯空间域统计平均,无信号处理复杂度。输入(聚焦图像)全部就绪,
算法是一个双循环求均值。主要工作在契约边界冻结 + 测试覆盖,非算法实现。

## 9. 与阶段 A 评估的一致性

本契约严格遵循 `multilook_value_assessment.md` §4 决策:
- 路径 α(图像域多视)→ §3 本算法契约。
- 路径 β(raw 域)冻结 → §7 非目标。
- 路径 γ(RD 域)冻结 → §7 非目标。
- 零侵入(不改聚焦器/session/配置)→ §7 非目标。
- 独立后处理工具(走自由函数)→ §4.1 接口。

无任何超出阶段 A 决策范围的内容。
