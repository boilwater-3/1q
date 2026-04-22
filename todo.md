FlatBuffers 已经引入了，但现在没有被真正用于 AR replay payload 的 typed 序列化/反序列化。AR replay payload 层：没有真正 FlatBuffers schema。

目前进度：
1、文档明确：FlatBuffers 是 replay payload 主路径，JSON 只是过渡/调试/兼容。
2、ReplayTraceEvent 增加 payload_bytes，非 JSON payload 落盘为 payload_base64。
3、payload hash 对原始 bytes 计算，而不是对 base64 文本或 debug JSON 计算。
4、新增 AR replay schema：airborne_radar_replay.fbs
5、新增 AR FlatBuffers codec：RadarReplayFlatbufferCodec.cpp
6、Windows AR cycle_input 写侧已切到 payload_encoding="flatbuffers"。
7、B 电脑 replay 读侧对 cycle_input 优先走 FlatBuffers decode，旧 JSON trace 仍 fallback。
8、新增 schema：airborne_radar_scene_replay.fbs
9、生成头：airborne_radar_scene_replay_generated.h
10、扩展 codec（新增 scene_state encode/decode）：
RadarReplayFlatbufferCodec.h
RadarReplayFlatbufferCodec.cpp
11、写侧切换 scene_state 为 payload_encoding="flatbuffers"：
RadarTraceSession.windows.cpp
12、读侧回放优先解码 scene_state bytes（保留 JSON fallback）：
RadarReplaySession.cpp
13、测试同步更新：
ar_trace_session_adapter_test.cpp
14、把 session_config 切到 FlatBuffers（写侧+回放解码+测试），初始化配置摆脱 JSON 字段对齐风险。
重新生成了 airborne_radar_session_replay 的 FlatBuffers 头文件

airborne_radar_session_replay_generated.h
基于你已扩展好的 airborne_radar_session_replay.fbs（包含 RadarRuntimeConfigPatch 与环境补丁表）
扩展 AR codec，新增 runtime patch 编解码

RadarReplayFlatbufferCodec.h
RadarReplayFlatbufferCodec.cpp
新增：
EncodeRuntimeConfigPatchFlatbuffer(...)
DecodeRuntimeConfigPatchFlatbuffer(...)
同时补了 environment patch 相关的内部 encode/decode 映射。
接入写侧（Windows trace）

RadarTraceSession.windows.cpp
runtime_config_patch 事件改为：
payload_encoding = "flatbuffers"
payload_bytes = ...
payload_json = "{}"（仅占位/兼容）
接入读侧（replay）

RadarReplaySession.cpp
OnRuntimeConfigPatch 现在优先解码 FlatBuffers bytes，保留 JSON fallback。
更新单测断言

ar_trace_session_adapter_test.cpp
不再依赖 runtime patch 的 JSON 字段文本查找，改为验证：
payload_type/payload_encoding/payload_bytes/payload_hash_matches
并对 bytes 做 runtime patch decode 后检查关键字段值。

已把 AR cycle_output 从“仅 JSON”切到“payload_encoding = flatbuffers + payload_bytes”。
保留了 JSON 阴影字段用于现有 replay 比较框架兼容（否则会直接把 payload 读成 null 导致误判）。
回放侧已在 cycle_output 处理里优先走 FlatBuffers decode + typed compare。
新增/更新了对应测试断言，覆盖 cycle_output bytes 的 decode 校验。

已把 failure_marker 切到 typed payload：
15、新增 schema：FailureMarker 表（airborne_radar_replay.fbs）
16、生成头：airborne_radar_replay_generated.h（flatc 重新生成）
17、扩展 codec：
EncodeFailureMarkerFlatbuffer(...)
DecodeFailureMarkerFlatbuffer(...)
18、WriteFailureMarker 新增 payload_bytes 重载（ReplayTrace.h / ReplayTrace.cpp）
写侧传入 flatbuffers bytes，payload_encoding = “flatbuffers”
19、读侧 OnFailureMarker 优先走 FlatBuffers decode，旧 JSON fallback 保留
20、RadarReplaySessionResult 新增 failure_marker_data（ReplayTraceFailure）
21、测试改为验证 typed decode 后的字段值（error_code、message、has_cycle_index）
