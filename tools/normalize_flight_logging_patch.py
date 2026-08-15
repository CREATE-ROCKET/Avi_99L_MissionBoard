from pathlib import Path

path = Path("tools/apply_flight_logging_runtime.py")
text = path.read_text(encoding="utf-8")

first = 'replace_once(\n    """      status.flight_elapsed_us = mission_snapshot.elapsed_us;'
second = 'replace_once(\n    """      (void)xQueueOverwrite(status_queue, &status);'
third = 'replace_once(\n    """      context.resources_preallocated ='

first_index = text.find(first)
second_index = text.find(second)
third_index = text.find(third)
if min(first_index, second_index, third_index) < 0 or not (
    first_index < second_index < third_index
):
    raise SystemExit("flight logging patch anchors could not be normalized safely")

# Runtimeの実際のstatus更新位置とevent生成位置へpost patchで挿入するため、
# 古い2つの置換だけを一時patch scriptから除外する。
text = text[:first_index] + text[third_index:]
path.write_text(text, encoding="utf-8")
