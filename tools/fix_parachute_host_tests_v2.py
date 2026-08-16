from pathlib import Path
import re

path = Path("host_test/mission_host_tests.cpp")
s = path.read_text()

old_fin = '''  auto fin = command(7, static_cast<uint8_t>(CommandCode::fin_move_relative));
  fin.arguments[0] = 10;
  decision = executor.begin(fin, context);
  assert(decision.execute && decision.result.phase == CommandPhase::accepted);
'''
new_fin = '''  auto fin = command(7, static_cast<uint8_t>(CommandCode::fin_move_relative));
  decision = executor.begin(fin, context);
  assert(decision.execute && decision.result.phase == CommandPhase::accepted);
'''
if old_fin not in s:
    raise RuntimeError("simplified fin command test anchor not found")
s = s.replace(old_fin, new_fin, 1)

old_limit = '''  auto para_limit =
      command(13, static_cast<uint8_t>(CommandCode::para_move_relative));
  para_limit.arguments[0] = 0x08;
  para_limit.arguments[1] = 0x07;
  assert(executor.begin(para_limit, context).result.reason ==
         CommandReason::invalid_argument);
'''
new_limit = '''  auto para_limit =
      command(13, static_cast<uint8_t>(CommandCode::para_move_relative));
  para_limit.arguments[0] = 0x08;
  para_limit.arguments[1] = 0x07;
  assert(executor.begin(para_limit, context).result.reason ==
         CommandReason::not_supported);
'''
if old_limit not in s:
    raise RuntimeError("retired relative command test anchor not found")
s = s.replace(old_limit, new_limit, 1)

# set endpoint/relative-moveの旧validity testは廃止command一括testで置換済み。
s, n = re.subn(
    r"\n  CommandExecutor parachute_arguments;\n.*?(?=\n  CommandExecutor start_busy;)",
    "\n", s, count=1, flags=re.S)
if n != 1:
    raise RuntimeError(f"old parachute arguments block removal count={n}")

# Start busyは有効なParaOpen pendingで作る。ParaHoldはretiredなのでbusy sourceに使わない。
s = s.replace(
    "static_cast<uint8_t>(CommandCode::para_hold)",
    "static_cast<uint8_t>(CommandCode::para_open)",
    1,
)

# sequence pending中でもretired commandはBusyよりNotSupportedを優先する。
old_busy = '''  assert(parachute_busy
             .begin(command(20,
                            static_cast<uint8_t>(CommandCode::para_free)),
                    context)
             .result.reason == CommandReason::busy);
'''
new_busy = '''  assert(parachute_busy
             .begin(command(20,
                            static_cast<uint8_t>(CommandCode::para_free)),
                    context)
             .result.reason == CommandReason::not_supported);
'''
if old_busy not in s:
    raise RuntimeError("retired command busy precedence anchor not found")
s = s.replace(old_busy, new_busy, 1)

path.write_text(s)
