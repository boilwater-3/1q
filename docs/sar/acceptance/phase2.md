# SAR Phase 2 参考级成像与算法对比审批报告

## 1. 审批结论

Phase 2“参考级成像与算法对比闭环”在当前平台完成审批。

- Phase 2A 统一质量指标、全局常数相位重参考和显式 Sinc RCMC：通过。
- Phase 2B 严格 `128x128` 门禁下的小场景 GBP 与 RDA/GBP 对比：通过。
- Auto algorithm selector：不批准，继续后置。
- Sinc RCMC 作为内部显式实验路径：通过。
- Sinc RCMC 成为 public Session 默认路径：不批准，继续使用 linear。

## 2. 冻结边界

- public Session 继续固定 RDA + linear RCMC。
- public Session `1024x1024` 当前平台上限不变。
- GBP 仅为内部参考算法，任一图像维度不得超过 `128`。
- 不开放 GBP/Auto public 选择接口。
- 不增加全图复数矩阵 replay。
- L2/L3、运动补偿、自聚焦、辐射定标和 HDF5/GeoTIFF 继续后置。

## 3. 质量证据

### RDA 与 GBP

同一确定性单点 raw echo、匹配滤波和平台轨迹：

| 指标 | 结果 | 审批阈值 |
|---|---:|---:|
| 全局相位偏移 | `-0.047659 rad` | 记录项 |
| 单位能量形状 NRMS | `0.042218` | `< 0.1` |
| 相干相关系数 | `0.999109` | `> 0.99` |

GBP 单点峰值定位正确，三点目标保持预期幅度顺序。

### Linear 与 Sinc RCMC 相对 GBP

| 路径 | NRMS | 相干相关系数 |
|---|---:|---:|
| Linear RCMC | `0.042218` | `0.999109` |
| Sinc RCMC | `0.042107` | `0.999114` |

Sinc 在当前单点参考场景中存在可测但极小的改善，不足以支持默认切换。

## 4. 性能证据

当前平台 Debug 观测：

| 项目 | 结果 |
|---|---:|
| 1024x1024 linear RCMC 独立处理 | 约 `0.024230 s` |
| 1024x1024 Sinc RCMC，半宽 4 | 约 `0.178153 s` |
| 128x128 GBP 参考场景 | 约 `0.149 s` |

Sinc 独立 RCMC 代价约为 linear 的 7.4 倍。GBP 通过当前小场景性能门，但不得据此扩大上限。

## 5. Auto 审批审计

Auto 不满足正式运行条件：

1. 用户已批准 Auto 继续后置。
2. 当前只有 RDA 与受限 GBP；BP、CSA、Omega-K 均未实现和审批。
3. 当前参考场景仅覆盖 L1 broadside 点目标，不能支撑通用自动策略。
4. 尚未冻结 Auto 的选择阈值、降级规则和结构化选择诊断。
5. GBP 未进入 public Session，且不得作为自动降级目标。

因此 Phase 2C 的正式结论为：**Auto 继续门禁，不增加 public 入口。**

## 6. 验证入口

- `ctest --test-dir build/llvm-ninja-debug-local -L sar_ci --output-on-failure`
- `ctest --test-dir build/llvm-ninja-debug-local -L sar_performance --output-on-failure`
- `ctest --test-dir build/llvm-ninja-debug-eigen339-local -L sar_cxx11_compat --output-on-failure`
- Eigen 3.3.9 下的 SAR GBP/RDA/Session 聚焦过滤测试
- `git diff --check`

## 7. 下一阶段建议

下一扩展方向建议进入 L2 轨迹误差与一阶运动补偿。该方向必须另行批准并新增工程契约，不属于本次 Phase 2 审批范围。
