from pathlib import Path

p = Path("tools/apply_runtime_v2.py")
s = p.read_text()
marker = "for forbidden in ["
pos = s.rfind(marker)
if pos < 0:
    raise RuntimeError("forbidden marker not found")

cleanup = r'''
# 旧queue bufferは型名を含むため、空白・改行差を許容して除去する。
s = re.sub(
    r"std::array<uint8_t,\s*sizeof\(ParachutePersistenceRequest\)\s*\*\s*4>\s*"
    r"parachute_persistence_request_queue_buffer\{\};\s*",
    "", s, count=1)
s = re.sub(
    r"std::array<uint8_t,\s*sizeof\(ParachutePersistenceResponse\)\s*\*\s*4>\s*"
    r"parachute_persistence_response_queue_buffer\{\};\s*",
    "", s, count=1)
'''

p.write_text(s[:pos] + cleanup + s[pos:])
