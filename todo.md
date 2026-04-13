1、把 AR 示例/测试里仍引用 TargetFeatureUtils.h 的地方改为按职责拆分：
只用外部输入适配就 RadarExternalInputAdapter.h
只用几何构造就保留 TargetFeatureUtils.h
给 TargetFeatureUtils.h 增注释：已不再承载外部坐标适配职责。
2、建议做一轮“全量回归”再封板
跑完整 ctest --preset llvm-ninja-debug-local -Q --output-on-failure
现在我跑的是关键子集，核心链路没问题，但全量还能再兜一遍。
3、真正的后续大项（如果继续这个重构方向）
三模块外部输入类型再抽一层 shared/foundation（现在是“目录统一”，语义仍各模块一套类型）。
API 文档/README 更新到新入口（避免后续接入方走旧路径）。