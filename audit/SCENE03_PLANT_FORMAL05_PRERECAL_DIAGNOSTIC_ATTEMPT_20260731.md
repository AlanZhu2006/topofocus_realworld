# Scene 03 · Formal 05 重标定前诊断尝试

本轮保留为工程诊断证据，暂不占用 Formal 05 正式指标身份。

Robot 0 在第 7 轮形成了仅 7 cells 的 `plant` 语义区域，但没有产生
`ARRIVED`，随后出现路径过期、无进展和局部目标不可达。现场操作者认为
该语义区域疑似误判；机器证据本身不能确认真假，因此归档仅记录
“疑似误判、待 RGB/语义地图复核”。

本轮运行 600 秒、14 个 source rounds 后按超时安全结束。Robot 0 路径
`11.839039 m`，Robot 1 路径 `22.780065 m`。两台机器人最终均为 HOLD、
零速度，Hub GOAL 输出均已关闭。

本轮不计入 SR/SPL。Robot 0 下电再上电后将重新标定，并以新 session
重跑 Scene 03 Formal 05。

机器可读记录：
[`scene03_plant_formal05_prerecal_diagnostic_attempt_20260731.json`](../manifests/scene03_plant_formal05_prerecal_diagnostic_attempt_20260731.json)。
