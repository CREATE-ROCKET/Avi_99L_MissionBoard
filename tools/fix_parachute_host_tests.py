from pathlib import Path

path = Path("host_test/mission_host_tests.cpp")
s = path.read_text()

s = s.replace(
    "  context.persistence_load_complete = true;\n"
    "  context.persistence_runtime_available = true;\n",
    "")

old_ready = (
    "  const PreflightReadinessSnapshot ready{1, 1234, true, true, true, true,\n"
    "                                         true, true, true, true};"
)
new_ready = (
    "  const PreflightReadinessSnapshot ready{1, 1234, true, true, true, true,\n"
    "                                         true, true};"
)
if old_ready not in s:
    raise RuntimeError("preflight readiness aggregate marker not found")
s = s.replace(old_ready, new_ready)

anchor = "  context.calibration_supported = true;\n"
if anchor not in s:
    raise RuntimeError("CommandContext test anchor not found")
insert = anchor + r'''

  // 旧パラシュートcommandはstate/argsより先にNotSupportedとし、
  // 同一transaction replayでもservo操作へ到達させない。
  for (uint8_t raw = static_cast<uint8_t>(CommandCode::para_free);
       raw <= static_cast<uint8_t>(CommandCode::set_para_close); ++raw) {
    CommandContext retired_context = context;
    retired_context.state = MissionState::engine_burn;
    auto retired = command(static_cast<uint8_t>(0xD0U + raw - 0x20U), raw);
    retired.arguments[0] = 0xA5;
    auto retired_decision = executor.begin(retired, retired_context);
    assert(!retired_decision.execute && !retired_decision.replay);
    assert(retired_decision.result.phase == CommandPhase::rejected);
    assert(retired_decision.result.reason == CommandReason::not_supported);
    retired_decision = executor.begin(retired, retired_context);
    assert(!retired_decision.execute && retired_decision.replay);
    assert(retired_decision.result.reason == CommandReason::not_supported);
  }
'''
s = s.replace(anchor, insert, 1)

path.write_text(s)
