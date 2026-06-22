# SAR 图像边界与采样/硬件参数适用性矩阵审批报告

## 1. 审批结论

B1-B4 图像边界与 P1-P4 参数适用性矩阵已完成当前平台审批。

- 距离/方位内部目标保持当前 L1 RDA/GBP 质量门。
- 图像网格边缘目标即使跨算法相关性较高，也必须归类为 `boundary_degraded`。
- raw echo 已裁剪时，跨算法比较仍可能显示极高相关，必须由独立裁剪诊断门禁。
- `80-120 MHz` 采样率和 `0.8-1.2 GHz` 载频首批档位通过。
- PRF/平台速度扫描暴露了共同的方位采样间距适用边界；不能冻结为单独 PRF 或速度阈值。
- Auto、尺寸扩展和参数范围外推继续后置。

## 2. 边界矩阵

| 场景 | 分类 | NRMS | 相干相关系数 | 裁剪证据 |
|---|---|---:|---:|---|
| B1 中心 | `interior_pass` | `0.042218` | `0.999109` | 无 |
| B2 下侧内部 | `interior_pass` | `0.046455` | `0.998921` | 无 |
| B2 下侧图像边缘 | `boundary_degraded` | `0.050023` | `0.998749` | 峰值位于网格边缘 |
| B3 上侧内部 | `interior_pass` | `0.038521` | `0.999258` | 无 |
| B3 上侧图像边缘 | `boundary_degraded` | `0.036074` | `0.999349` | 峰值位于网格边缘 |
| B3 上侧 raw 边界 | `echo_clipped` | `0.014853` | `0.999890` | 9 个脉冲、72 个样本裁剪 |
| B4 方位内部偏置 | `interior_pass` | `0.075643` | `0.997139` | 无 |
| B4 方位网格边缘 | `boundary_degraded` | `0.132409` | `0.991234` | 峰值位于网格边缘 |

关键结论：NRMS/相关性只比较两种算法输出的一致性，不能证明输入回波完整或输出主瓣未被窗口截断。

## 3. 参数矩阵

### P1 采样率

| 采样率 | 分类 | NRMS | 相干相关系数 |
|---:|---|---:|---:|
| `80 MHz` | `interior_pass` | `0.033714` | `0.999432` |
| `100 MHz` | `interior_pass` | `0.042218` | `0.999109` |
| `120 MHz` | `interior_pass` | `0.050886` | `0.998705` |

### P2 载频

| 载频 | 分类 | NRMS | 相干相关系数 |
|---:|---|---:|---:|
| `0.8 GHz` | `interior_pass` | `0.033870` | `0.999426` |
| `1.0 GHz` | `interior_pass` | `0.042218` | `0.999109` |
| `1.2 GHz` | `interior_pass` | `0.050602` | `0.998720` |

### P3/P4 方位采样耦合

| PRF | 平台速度 | 方位采样间距 `v/PRF` | 分类 | NRMS | 相干相关系数 |
|---:|---:|---:|---|---:|---:|
| `10 Hz` | `2 m/s` | `0.2 m/pulse` | `boundary_degraded` | `0.177589` | `0.984231` |
| `20 Hz` | `2 m/s` | `0.1 m/pulse` | `interior_pass` | `0.042218` | `0.999109` |
| `40 Hz` | `2 m/s` | `0.05 m/pulse` | `interior_pass` | `0.010554` | `0.999944` |
| `20 Hz` | `1 m/s` | `0.05 m/pulse` | `interior_pass` | `0.010554` | `0.999944` |
| `20 Hz` | `4 m/s` | `0.2 m/pulse` | `boundary_degraded` | `0.177589` | `0.984231` |

相同 `v/PRF` 得到相同质量结果，证明当前失效证据应归因于方位采样间距耦合，而不是孤立的 PRF 或平台速度值。

## 4. 实现边界

- 参考 raw-history helper 可选汇总裁剪脉冲、目标和样本数，现有调用行为不变。
- `SarBoundaryParameterMatrixTest` 按 `interior_pass / boundary_degraded / echo_clipped / invalid` 分类。
- 所有可执行场景继续验证 BP 与 GBP 复图逐样本一致。
- 未修改生产算法、public API、replay 或尺寸门。

## 5. 验证结果

- 默认与 Conan Eigen 3.3.9 M1-M7 + B1-B4 + P1-P4：各 `13/13 passed`。
- 默认与 Conan Eigen 3.3.9 全部 `Sar*` 单测：各 `86/86 passed`。
- 默认与 Conan Eigen 3.3.9 `ctest -L sar_ci`：各 `4/4 passed`。
- 默认 `ctest -L sar_performance`：`1/1 passed`。
- Conan Eigen 3.3.9 `ctest -L sar_cxx11_compat`：`1/1 passed`。
- `git diff --check`：passed。

## 6. 冻结边界

- 不把首批参数档位外推为通用硬件支持范围。
- 不把 PRF 或平台速度单独作为质量选择阈值。
- 不忽略 raw echo 裁剪或图像边缘主瓣截断。
- 不启用 Auto、时变 PRF、尺寸扩展、高阶补偿或全图 replay。

## 7. 下一阶段建议

进入方位采样充分性决策门，研究 `platform_velocity / PRF` 与载频、斜距、孔径和波束/多普勒带宽之间的工程关系，冻结可解释的结构化诊断前置条件；不得直接将 `0.2 m/pulse` 写成通用硬阈值。
