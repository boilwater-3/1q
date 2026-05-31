# 进度：JSBSim 飞行机动模块

## 当前状态

分支 `refactor/jsbsim-integration`，fd 6/6 绿（release ~8s，debug ~62s）。

## 本次会话（2026-05-31）

1. **f22 trim recovery 修复** — `GetFCS()->InitModel()` 重置 FCS 内部状态
2. **c310 known-limit** — 原生 AP 速度衰减，`FlyToWaypoint` 归入 known-limit
3. **`kTakeoff` 机动** — 引擎→滑跑→抬轮→爬升；Vr 从翼载实时计算
4. **`kLand` 机动** — pitch-for-speed/throttle-for-altitude PD；进近→下滑→拉平→触地→滑跑
5. **`EngineManager`** — 引擎类型检测、启动、油门、刹车、襟翼、起落架
6. **gtest→CSV 迁移** — 移除 12 个耗时测试，fd 全量 117→62s debug / 8s release
7. **CSV 工具** — `takeoff_land_csv`、`maneuver_sweep_csv`、`analyze_takeoff.py`
8. **Release 预设** — 建议测试用 `llvm-ninja-release-local`（~6× 快）
9. **规划文件清理** — 删除已完成项

## 起降实测（最新代码）

```
c172x  ✅✅✅  全任务通过 (461s)
c310   ✅✅❌  起飞+巡航完成，着陆坠毁 (256s)
f16    ✅❌—   起飞完成，巡航超音速坠毁 (97s)
f22    ⚠️——   能离地爬升到 357m，50s 后失控
737    ✅❌—   起飞完成，560 节无法捕获航路点
B17    ❌——   82 节<Vr=91 节，唯一不能起飞
```

## 待处理

- 起降稳定性：f22 高空失控、c310 着陆 crash、f16/737 超速、B17 推力
- PD 增益从 CSV 迭代调优
- Public 示例（遗留项 8.3/12.4）
