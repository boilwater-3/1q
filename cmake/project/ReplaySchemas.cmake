# Replay FlatBuffers schema manifest.

set(ONEQ_FLATBUFFERS_SCHEMAS
    "sar_replay.fbs:sar:sar_core"
    "sar_session_replay.fbs:sar:sar_core"
    "eos_replay.fbs:electro_optical_sensor:eos_core"
    "eos_session_replay.fbs:electro_optical_sensor:eos_core"
    "esr_replay.fbs:electronic_surveillance_radar:esr_core"
    "esr_session_replay.fbs:electronic_surveillance_radar:esr_core"
    "airborne_radar_replay.fbs:airborne_radar:airborne_core"
    "airborne_radar_session_replay.fbs:airborne_radar:airborne_core"
    "sbirs_replay.fbs:sbirs_sensor:sbirs_core"
    "sbirs_session_replay.fbs:sbirs_sensor:sbirs_core")
