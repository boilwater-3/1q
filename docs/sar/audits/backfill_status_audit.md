# SAR 补齐阶段(批次1-6)现状审计报告

Date: 2026-06-23

## 1. 审计目的

`module_design.md` 把"补齐"(窗函数 / 斜距 / 多普勒 / 杂波 / 天线 / 输出格式,批次1-6,
2026-06-22)标为 `✅ 已实现`。本审计逐条比对该描述与 `src/sar/{output,signal,geometry,echo}/`
及对应测试的实际实现,重点核查输出层(Binary/GeoTIFF sidecar/HDF5 条件编译)。
本审计不修改代码。

注:窗函数/斜距/多普勒/杂波/天线的实现细节在 Phase 1 审计中已覆盖(均一致),本审计
聚焦**输出层**及 6 模块完成度证据。

## 2. 审计结论

补齐阶段 **6 模块全部交付,但输出层有 2 处文档/注释与实际不符**:

| 审计项 | 实际状态 | 结论 |
|---|---|---|
| 窗函数(WindowType/GenerateWindow/加窗重载) | 存在,7 个测试 | ✅ 一致 |
| 斜距/多普勒/几何模型 | 存在,18 个测试 | ✅ 一致 |
| 杂波(Gamma/Sea/Scene) | 存在,15 个测试 | ✅ 一致 |
| 天线(AntennaParams/Pattern/Gain) | 存在,10 个测试 | ✅ 一致 |
| 输出 ImageFormatter.{h,cpp} | 存在,6 个测试 | ✅ 一致 |
| HDF5 条件编译(默认 OFF) | `cmake/ProjectOptions.cmake:63-66` | ✅ 一致 |
| **Binary magic 字节数(L403, L49 注释)** | 实际 7 字节,注释写 6 字节 | ❌ **不一致(代码注释自相矛盾)** |
| **Binary 数据布局措辞(L403 "交错实虚")** | 实际 **planar**(real 块 + imag 块) | ❌ **不一致** |

## 3. 输出层实证(`src/sar/output/ImageFormatter.{h,cpp}`)

### 3.1 函数签名(全部一致)

- `WriteBinaryImage(image, meta, filepath)`(`ImageFormatter.h:45-46`)
- `WriteGeoTiffSidecar(image, meta, base_filepath)`(`:56-57`)
- `WriteHdf5Image(image, meta, filepath)`(`:68-70`,内 `#if defined(ONEQ_ENABLE_HDF5_OUTPUT)`)

入参为 `SarFocusedImage`(`SarCycleResult.h:78`),非设计初稿 `ImagingResult`。✅ 一致。

### 3.2 ❌ 不一致 1:Binary magic 字节数(代码注释自相矛盾)

**代码**(`ImageFormatter.cpp:71-73`,已直接读取确认):
```cpp
// Magic: "1QSAR\x01\x00" (6 bytes)        ← 注释写 6 字节
const char magic[] = {'1','Q','S','A','R','\x01','\x00'};   ← 实际 7 元素
file.write(magic, 7);                       ← 实际写 7 字节
```
- `ImageFormatter.h:35` 注释同样写 `(6 bytes)`。
- 测试 `sar_image_output_test.cpp:85-93` 读 `char magic[7]; in.read(magic, 7);` 并校验 0-6 位,**证实磁盘上 7 字节**。
- **结论**:magic 实际 7 字节;`.cpp:71` 与 `.h:35` 注释的"(6 bytes)"是**代码自身注释自相矛盾**,应为"(7 bytes)"。文档 L403 写 `1QSAR\x01\x00`(8 字节)也偏离实际(应为 7,非 8)。

### 3.3 ❌ 不一致 2:Binary 数据布局("交错" vs 实际 planar)

**代码**(`ImageFormatter.cpp:79-86`,已直接读取确认):
```cpp
// Data: real[] + imag[] as float32 LE      ← planar
for (i in total) WriteFloat32Le(real_values[i]);   // 先写全部 real
for (i in total) WriteFloat32Le(imaginary_values[i]); // 再写全部 imag
```
- **Binary 是 planar 布局**(全部 real 块 + 全部 imag 块)。
- 文档 L403 称"float32 **交错实虚**"——"交错"会误导读者以为逐元素 real,imag,real,imag。
- 真正交错的是 **GeoTIFF sidecar 的 `.raw`**(`ImageFormatter.cpp:107-110`,逐元素 real,imag 交替)。
- **结论**:文档 L403 的"交错实虚"描述适用于 sidecar `.raw`,不适用于 Binary。Binary 应描述为"planar(实部全块 + 虚部全块)"。

### 3.4 GeoTIFF sidecar(基本一致,1 处措辞偏差)

- 写 `base.raw`(float32 逐元素交错)+ `base.json` manifest。✅ 与"降级为 sidecar manifest"一致。
- manifest 字段:`row_count`/`column_count`/`center_slant_range_m`/`estimated_snr_db`/`range_pixel_spacing_m`/`azimuth_pixel_spacing_m`/`origin_lat_deg`/`origin_lon_deg`/`source`/`format="float32_interleaved_real_imag"`/`raw_file`(`:125-137`)。
- **措辞偏差**:文档 L404 称 manifest 含"投影说明";实际只有静态 `format` 字符串(`"float32_interleaved_real_imag"`),**无真实投影字段(EPSG/WKT)**。

### 3.5 HDF5(一致,dtype 值得标注)

- `#if defined(ONEQ_ENABLE_HDF5_OUTPUT)` 包裹(`.cpp:150-201`),`#include <highfive/H5File.hpp>` 在守卫内(`:151`)。
- dataset:`/image/real`、`/image/imag`(`:179,183`),存为 **double**(非 float32,Binary/sidecar 用 float32)。
- attrs:`center_slant_range_m`、`estimated_snr_db`、`source`(`:189-194`);**未写** origin_lat/lon/pixel_spacing。
- try/catch 返回 false(`:160,197`)。
- ✅ 与"ONEQ_ENABLE_HDF5_OUTPUT 默认 OFF"一致。

## 4. CMake HDF5 门控实证

- `cmake/ProjectOptions.cmake:63-66`:`option(ONEQ_ENABLE_HDF5_OUTPUT "Enable SAR HDF5 image output (requires HighFive)" OFF)`,默认 OFF。
- `cmake/ProjectDependencies.cmake:152-162`:`if(ONEQ_ENABLE_HDF5_OUTPUT) find_package(HighFive CONFIG REQUIRED) ... target_compile_definitions(sar_engine PRIVATE ONEQ_ENABLE_HDF5_OUTPUT)`。
- compile 定义只附在 `sar_engine` 目标,且门控于 option AND `if(TARGET sar_engine)`。

## 5. 依赖决策实证

- `conanfile.py:38-47`:option `enable_hdf5` 默认 `False`。
- `conanfile.py:91-93`:`if self.options.enable_hdf5: self.requires("highfive/2.10.0")`(非 Windows)。
- `third_party/` 无 highfive(vendor/none 模式下开启 option 会在 `find_package` 失败)。
- 无 TIFF/GeoTIFF 依赖。
- ✅ 与"项目当前无 HDF5/TIFF/GeoTIFF 依赖,受 C++11 + Eigen 3.3.9 + VS2015 vendor 约束"一致。

**轻微不一致(依赖侧)**:Conan option 名 `enable_hdf5`,CMake option 名 `ONEQ_ENABLE_HDF5_OUTPUT`,两者同名-ish 但 `conanfile.py:generate()` 未见显式映射(`CMakeToolchain(self)` 通用调用)。两个开关未在此文件可证明地联动。

## 6. 6 模块完成度证据(测试 TEST() 计数)

| 模块 | 测试文件 | TEST 数 | 源文件 |
|---|---|---|---|
| 窗函数 | `sar_window_function_test.cpp` | 7 | SarWaveform.cpp |
| 斜距/多普勒/几何 | `sar_geometry_model_test.cpp` | 18 | SarGeometry.cpp |
| 杂波 | `sar_echo_clutter_test.cpp` | 15 | SarEcho.cpp |
| 天线 | `sar_antenna_pattern_test.cpp` | 10 | SarAntenna.cpp |
| 输出格式 | `sar_image_output_test.cpp` | 6 | ImageFormatter.cpp |

`ImageFormatter.cpp` 在 `SarSources.cmake:16` 的 `SAR_ENGINE_SOURCES`,构建入 `sar_engine`。
✅ 6 模块全部有源 + 测试,与"批次1-6"一致。

## 7. 处置建议

- **不一致 1、2(输出层)**:建议修正:
  - `module_design.md` L403:把"magic `1QSAR\x01\x00`(8 字节)"改为"(7 字节)",把"float32 交错实虚"改为"float32 planar(实部全块 + 虚部全块)"(交错用于 sidecar .raw)。
  - `src/sar/output/ImageFormatter.{h:35,cpp:71}` 代码注释的"(6 bytes)"改为"(7 bytes)"—— 这是**代码自身注释自相矛盾**,修正属于注释清理,不改逻辑。
- **措辞偏差(GeoTIFF 投影说明)**:可选,把 L404"投影说明"改为"格式说明(format 字符串,无真实投影)"。
- **Conan/CMake option 名**:可选,补 `conanfile.py:generate()` 的显式映射或文档说明。

## 8. 不一致性优先级

| 优先级 | 不一致 | 影响 |
|---|---|---|
| 中 | Binary magic 字节数(文档 8 / 注释 6 / 实际 7) | 文档 + 代码注释双重失真 |
| 中 | Binary 布局措辞("交错" vs planar) | 误导读者对磁盘格式理解 |
| 低 | GeoTIFF "投影说明" 措辞 | manifest 无真实投影字段 |
| 低 | Conan/CMake option 名不联动 | 非阻塞,需手动传 CMake 变量 |

## 9. 本审计的非目标

- 不修改任何 C++ 源代码逻辑(输出格式字节布局、HDF5 dtype 等保持不变)。
- 不变更冻结清单或公共 API。
- 不补 HDF5 测试(默认 OFF,属设计决策)。
- 不构成生产级输出格式或真实 GeoTIFF 授权。
