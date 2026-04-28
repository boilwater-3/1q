# 坐标工具破坏性重构设计方案

## 背景

当前仓库已经出现两套坐标转换公共面：

- `include/1q/coordinate/coordinate.h` / `src/common/coordinate/coordinate.cpp`
  - 新增的独立坐标工具面，命名空间为 `oneq::coordinate`。
  - 同一个头文件同时定义 LLA/ECEF/ENU/NED/NUE 坐标结构体、位置转换函数、局部坐标轴重排函数。
- `include/1q/foundation/coordinate_transform.h` / `src/common/geometry/CoordinateTransform.cpp`
  - 现有三模块输入适配层实际使用的公共面，命名空间为 `oneq::foundation`。
  - 同样定义 LLA/ECEF/ENU/NUE 结构体和 WGS84 位置转换函数。

库内还有一层 `src/common/geometry/CoordinateConversion.{h,cpp}`，负责把 ECEF/ENU/NED 速度转换到模块局部坐标，并调用 `GeometryTransform.{h,cpp}` 完成姿态旋转。也就是说，当前位置、速度、姿态转换已经存在，但边界混在 `foundation` 和 `common/geometry` 中，`coordinate.h` 又重新引入了一套重复实现。

这次重构按破坏性收敛处理：不保留旧头文件作为兼容入口，不保留同一能力的双路径实现。

## 当前问题

1. `coordinate.h` 不是单一职责。
   它同时承载坐标结构定义、位置转换、局部轴重排，后续再加入速度和姿态转换会继续变成大杂烩。

2. 公共坐标类型重复。
   `oneq::coordinate::{LlaDegM,EcefM,EnuM,NedM,NueM}` 与 `oneq::foundation::{LlaCoordinateDegM,EcefCoordinateM,EnuCoordinateM,NueCoordinateM}` 表达同一概念，但字段命名、命名空间、覆盖范围不一致。

3. 位置转换公式重复。
   `src/common/coordinate/coordinate.cpp` 与 `src/common/geometry/CoordinateTransform.cpp` 都实现了 WGS84 LLA/ECEF/ENU/NUE 换算。

4. 速度转换藏在内部 geometry 文件。
   `TryConvertEcefVelocityToEnu`、`ToEnuFromNed`、`TryConvertVelocityToLocal` 本质上属于坐标系速度转换，但目前跟外部输入适配和局部 frame reference 绑定在一起。

5. 姿态转换语义不独立。
   `BuildRotationMatrix`、`RotateVectorToLocalFrame`、多 frame 姿态复合逻辑散落在 `GeometryTransform`、AR `RadarOrientationUtils` 和模块适配器里，缺少“不同坐标系下姿态角转换”的公共分类。

## 目标设计

将 `include/1q/coordinate/` 作为坐标工具的唯一公共包，按职责拆成四类文件：

```text
include/1q/coordinate/
|-- types.h                    坐标、速度、姿态值类型
|-- position_transform.h        不同坐标系位置之间的转换
|-- velocity_transform.h        不同坐标系速度之间的转换
`-- attitude_transform.h        不同坐标系姿态角/旋转之间的转换

src/common/coordinate/
|-- PositionTransform.cpp
|-- VelocityTransform.cpp
`-- AttitudeTransform.cpp
```

删除或停止安装 `include/1q/coordinate/coordinate.h` 和 `include/1q/foundation/coordinate_transform.h`，调用点改为按职责包含具体头文件。

## 公共类型

`types.h` 只放值类型，不放转换函数。

推荐命名：

```cpp
namespace oneq {
namespace coordinate {

struct LlaPositionDegM {
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
};

struct EcefPositionM {
  double x_m{0.0};
  double y_m{0.0};
  double z_m{0.0};
};

struct EnuPositionM {
  double east_m{0.0};
  double north_m{0.0};
  double up_m{0.0};
};

struct NedPositionM {
  double north_m{0.0};
  double east_m{0.0};
  double down_m{0.0};
};

struct NuePositionM {
  double north_m{0.0};
  double up_m{0.0};
  double east_m{0.0};
};

struct EcefVelocityMps {
  double x_mps{0.0};
  double y_mps{0.0};
  double z_mps{0.0};
};

struct EnuVelocityMps {
  double east_mps{0.0};
  double north_mps{0.0};
  double up_mps{0.0};
};

struct NedVelocityMps {
  double north_mps{0.0};
  double east_mps{0.0};
  double down_mps{0.0};
};

struct NueVelocityMps {
  double north_mps{0.0};
  double up_mps{0.0};
  double east_mps{0.0};
};

struct EulerAnglesDeg {
  double yaw_deg{0.0};
  double pitch_deg{0.0};
  double roll_deg{0.0};
};

struct LocalFrameReference {
  LlaPositionDegM origin_lla{};
  EulerAnglesDeg frame_attitude_deg{};
};

}  // namespace coordinate
}  // namespace oneq
```

命名原则：

- 位置类型显式带 `Position`，速度类型显式带 `Velocity`，避免用同一个 `Vector3f` 同时表达位置、速度和方向。
- 局部坐标字段使用语义名 `east/north/up/down`，不使用 `x/y/z` 承载隐含 frame 语义。
- 姿态角统一放在 coordinate 包内，后续模块公共输入若需要坐标姿态转换，应引用 `oneq::coordinate::EulerAnglesDeg`。

## 位置转换

`position_transform.h` 只处理位置点之间的转换。

建议 API：

```cpp
ONEQ_API bool IsValid(const LlaPositionDegM& lla);
ONEQ_API bool IsFinite(const EcefPositionM& ecef);
ONEQ_API bool IsFinite(const EnuPositionM& enu);
ONEQ_API bool IsFinite(const NedPositionM& ned);
ONEQ_API bool IsFinite(const NuePositionM& nue);

ONEQ_API bool TryLlaToEcef(const LlaPositionDegM& lla, EcefPositionM* ecef);
ONEQ_API bool TryEcefToLla(const EcefPositionM& ecef, LlaPositionDegM* lla);

ONEQ_API bool TryEcefToEnu(const EcefPositionM& ecef,
                           const LlaPositionDegM& origin_lla,
                           EnuPositionM* enu);
ONEQ_API bool TryLlaToEnu(const LlaPositionDegM& lla,
                          const LlaPositionDegM& origin_lla,
                          EnuPositionM* enu);

ONEQ_API NedPositionM ToNed(const EnuPositionM& enu);
ONEQ_API EnuPositionM ToEnu(const NedPositionM& ned);
ONEQ_API NuePositionM ToNue(const EnuPositionM& enu);
ONEQ_API EnuPositionM ToEnu(const NuePositionM& nue);
ONEQ_API NuePositionM ToNue(const NedPositionM& ned);
ONEQ_API NedPositionM ToNed(const NuePositionM& nue);
```

设计约束：

- LLA/ECEF 只处理椭球位置点。
- ECEF 到 ENU/NED/NUE 的位置转换必须使用参考原点，因为位置转换需要平移。
- ENU/NED/NUE 之间是局部 frame 内的轴重排，不需要参考原点。

## 速度转换

`velocity_transform.h` 只处理自由向量速度之间的转换。

建议 API：

```cpp
ONEQ_API bool IsFinite(const EcefVelocityMps& velocity);
ONEQ_API bool IsFinite(const EnuVelocityMps& velocity);
ONEQ_API bool IsFinite(const NedVelocityMps& velocity);
ONEQ_API bool IsFinite(const NueVelocityMps& velocity);

ONEQ_API bool TryEcefToEnuVelocity(const EcefVelocityMps& ecef_velocity,
                                   const LlaPositionDegM& origin_lla,
                                   EnuVelocityMps* enu_velocity);
ONEQ_API bool TryEcefToNedVelocity(const EcefVelocityMps& ecef_velocity,
                                   const LlaPositionDegM& origin_lla,
                                   NedVelocityMps* ned_velocity);
ONEQ_API bool TryEcefToNueVelocity(const EcefVelocityMps& ecef_velocity,
                                   const LlaPositionDegM& origin_lla,
                                   NueVelocityMps* nue_velocity);

ONEQ_API NedVelocityMps ToNedVelocity(const EnuVelocityMps& enu_velocity);
ONEQ_API EnuVelocityMps ToEnuVelocity(const NedVelocityMps& ned_velocity);
ONEQ_API NueVelocityMps ToNueVelocity(const EnuVelocityMps& enu_velocity);
ONEQ_API EnuVelocityMps ToEnuVelocity(const NueVelocityMps& nue_velocity);
ONEQ_API NueVelocityMps ToNueVelocity(const NedVelocityMps& ned_velocity);
ONEQ_API NedVelocityMps ToNedVelocity(const NueVelocityMps& nue_velocity);
```

设计约束：

- 速度是自由向量，不执行位置平移。
- ECEF 速度转局部速度需要参考原点的纬经度来确定旋转矩阵。
- 不提供 `LlaVelocity`。经纬高变化率不是普通三维速度，若将来需要，应以 `GeodeticRate` 单独建模，不能塞进本次坐标速度工具。

## 姿态转换

`attitude_transform.h` 处理 frame 姿态、旋转矩阵和姿态复合。

建议 API：

```cpp
struct RotationMatrix3d {
  double m00{1.0};
  double m01{0.0};
  double m02{0.0};
  double m10{0.0};
  double m11{1.0};
  double m12{0.0};
  double m20{0.0};
  double m21{0.0};
  double m22{1.0};
};

ONEQ_API bool IsFinite(const EulerAnglesDeg& attitude);
ONEQ_API RotationMatrix3d BuildRotationMatrix(const EulerAnglesDeg& attitude_deg);
ONEQ_API EulerAnglesDeg ToEulerAnglesDeg(const RotationMatrix3d& rotation);
ONEQ_API RotationMatrix3d Inverse(const RotationMatrix3d& rotation);
ONEQ_API RotationMatrix3d Compose(const RotationMatrix3d& parent_to_child,
                                  const RotationMatrix3d& child_to_grandchild);
ONEQ_API EulerAnglesDeg ComposeAttitudeDeg(const EulerAnglesDeg& parent_to_child,
                                           const EulerAnglesDeg& child_to_grandchild);
```

设计约束：

- 公共头不暴露 Eigen，保持坐标工具公共 API 的轻依赖边界。
- `.cpp` 内部可以使用 Eigen 实现矩阵运算，但 public header 只能暴露普通结构体。
- 姿态约定必须写在头文件注释中：采用当前仓库已有的 Z-Y-X yaw/pitch/roll 语义，并保留“正 pitch 表示正仰角”的现有约定。
- `RotateVectorToLocalFrame` 本质是“姿态旋转向量”，不应继续挂在通用 `GeometryTransform`；应迁移到 attitude 或 frame transform 能力中。如果为了不混淆“速度转换”和“姿态转换”，建议提供明确命名：

```cpp
ONEQ_API EnuVelocityMps RotateEnuVelocityToFrame(const EnuVelocityMps& velocity,
                                                 const EulerAnglesDeg& frame_attitude_deg);
```

但这类函数只应在确定目标 frame 类型后增加，避免产生“LocalVector”这种语义不明的公共类型。

## 与现有模块的收敛关系

### foundation

删除 `include/1q/foundation/coordinate_transform.h` 作为坐标工具入口。原有 `foundation` 下的坐标类型和转换函数迁移到 `include/1q/coordinate/`。

如果某些公共 API 仍需要基础姿态或三维向量，应分两类处理：

- 纯坐标/速度/姿态输入：改用 `oneq::coordinate` 类型。
- 与模块模型强绑定的 pose 输出：保留模块自己的高层语义结构，但内部字段类型应逐步改成 `oneq::coordinate`。

### common/geometry

`GeometryTransform` 保留视线角、扫描限位、角度窗口等几何工具，不再承载坐标 frame 转换。

迁移后：

- `src/common/geometry/CoordinateTransform.cpp` 删除。
- `src/common/geometry/CoordinateConversion.{h,cpp}` 删除或缩小为模块内部适配 helper，不再拥有公共坐标转换公式。
- `src/common/coordinate/*.cpp` 成为唯一坐标转换实现。

### AR/EOS/ESR

三模块输入适配层应从：

```cpp
#include "1q/foundation/coordinate_transform.h"
#include "common/geometry/CoordinateConversion.h"
```

收敛到：

```cpp
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/coordinate/attitude_transform.h"
```

模块自己的 `RadarExternalInputAdapter`、`EosExternalInputAdapter`、`EsrExternalInputAdapter` 可以保留高层语义输入枚举，但底层转换必须调用 `oneq::coordinate` 的分类 API。

## 实施步骤

### 阶段 1：建立目标文件结构

1. 新增 `include/1q/coordinate/types.h`。
2. 新增 `include/1q/coordinate/position_transform.h` 与 `src/common/coordinate/PositionTransform.cpp`。
3. 新增 `include/1q/coordinate/velocity_transform.h` 与 `src/common/coordinate/VelocityTransform.cpp`。
4. 新增 `include/1q/coordinate/attitude_transform.h` 与 `src/common/coordinate/AttitudeTransform.cpp`。
5. 更新 `src/common/CMakeLists.txt` 的源文件与 install header 清单。

验收：

- `rg "include/1q/coordinate/coordinate.h|src/common/coordinate/coordinate.cpp"` 不再作为目标结构的一部分出现。
- `tests/contract/check_public_api_boundary.cmake` 包含新坐标头文件清单。

### 阶段 2：迁移位置转换

1. 把 `coordinate.cpp` 和 `CoordinateTransform.cpp` 中重复的 WGS84 公式合并到 `PositionTransform.cpp`。
2. 删除 `foundation/coordinate_transform.h` 中坐标结构和位置函数的使用面。
3. 更新所有 `foundation::LlaCoordinateDegM`、`foundation::EcefCoordinateM`、`foundation::EnuCoordinateM`、`foundation::NueCoordinateM` 调用点为 `coordinate::*Position*`。
4. 更新位置转换单测，覆盖 LLA/ECEF 往返、ECEF/LLA 到 ENU、ENU/NED/NUE 轴重排。

验收：

- `rg "foundation::.*Coordinate|coordinate_transform.h|CoordinateTransform.cpp"` 无残留业务调用。
- 位置转换测试通过。

### 阶段 3：迁移速度转换

1. 把 `TryConvertEcefVelocityToEnu`、`ToEnuFromNed` 等速度能力迁移到 `VelocityTransform.cpp`。
2. 三模块外部输入适配器改为调用 `coordinate::TryEcefToEnuVelocity` 和局部速度轴重排 API。
3. 删除 `VelocityFrame` 这类内部通用枚举，模块输入枚举到坐标 API 的映射放在各模块 adapter 内。

验收：

- `rg "TryConvertEcefVelocityToEnu|TryConvertVelocityToLocal|VelocityFrame"` 无残留。
- AR/EOS/ESR cycle input builder 与 coordinate utils 测试覆盖 ECEF/ENU/NED 速度路径。

### 阶段 4：迁移姿态转换

1. 把 `BuildRotationMatrix`、`RotateVectorToLocalFrame`、姿态复合能力迁移到 `AttitudeTransform.cpp`。
2. AR `RadarOrientationUtils` 中重复的旋转矩阵实现收敛到 `coordinate::BuildRotationMatrix` / `coordinate::ComposeAttitudeDeg`。
3. EOS/ESR 中把外部 frame attitude 到局部 frame 的旋转改为调用 coordinate 姿态转换。
4. `GeometryTransform` 只保留视线角、扫描窗口等非坐标 frame 职责。

验收：

- `rg "BuildRotationMatrix|RotateVectorToLocalFrame"` 只剩 `coordinate` 实现和明确调用点。
- AR orientation、geometry transform、public convenience 测试通过。

### 阶段 5：删除旧入口和双路径

1. 删除 `include/1q/coordinate/coordinate.h`。
2. 删除 `src/common/coordinate/coordinate.cpp`。
3. 删除 `include/1q/foundation/coordinate_transform.h`。
4. 删除 `src/common/geometry/CoordinateTransform.cpp`。
5. 删除或改名 `tests/unit/coordinate_test.cpp`，按位置/速度/姿态拆分为：

```text
tests/unit/coordinate_position_transform_test.cpp
tests/unit/coordinate_velocity_transform_test.cpp
tests/unit/coordinate_attitude_transform_test.cpp
```

验收：

- `rg "coordinate/coordinate.h|foundation/coordinate_transform.h|CoordinateTransform.cpp|common/geometry/CoordinateConversion"` 无残留。
- public header smoke test、public API boundary test、三模块输入适配相关测试通过。

## 测试计划

优先运行：

```bash
cmake --build --preset llvm-ninja-debug-local >/tmp/1q-build.log 2>&1 || { tail -n 80 /tmp/1q-build.log; false; }
ctest --preset llvm-ninja-debug-local -Q --output-on-failure -R "coordinate|cycle_input_builder|coordinate_utils|orientation|public_headers|public_api_boundary"
```

最终验收运行：

```bash
ctest --preset llvm-ninja-debug-local -Q --output-on-failure
```

如本轮修改涉及 CMake 源文件清单或安装清单，但 preset 配置未变化，不需要每次先跑 configure；若构建系统未拾取新增源文件，再单独执行一次 configure。

## 风险与处理

1. 公共 API 破坏范围较大。
   三模块的 external input adapter 公共头目前直接暴露 `foundation::LlaCoordinateDegM` 和 `foundation::EulerAnglesDeg`。迁移后下游编译会失败，这是预期破坏性结果，需要同步改测试和示例。

2. 姿态角 float/double 精度差异。
   当前 internal geometry 使用 `float`，新增公共 coordinate 姿态建议用 `double`。迁移时模块内部若仍使用 `Vector3f`，边界转换必须集中在 adapter 或 `.cpp` 内，不能把 Eigen 或 internal geometry 类型暴露到 public header。

3. 速度和位置不能共用同一结构体。
   位置转换包含平移，速度转换只旋转自由向量；如果继续复用 `EnuCoordinateM` 表示速度，会把物理语义重新混在一起。

4. 不要保留兼容壳。
   本次重构目标是分类清晰和唯一实现。旧 `coordinate.h`、旧 `foundation/coordinate_transform.h`、旧 `CoordinateConversion` 不应留下转发函数或 typedef 兼容层。

## 完成标准

- 坐标结构体只在 `include/1q/coordinate/types.h` 定义。
- 位置、速度、姿态转换分别位于独立头文件和 `.cpp`。
- WGS84 公式只有一处实现。
- 公共头不包含 Eigen、不包含 `src/` 内部头。
- AR/EOS/ESR 的外部输入适配调用统一的 `oneq::coordinate` API。
- 相关测试和公共 API 边界清单随代码同步更新。
