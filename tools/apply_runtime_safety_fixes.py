from pathlib import Path

path = Path("src/runtime/production_runtime.cpp")
text = path.read_text()

old = '''    } else if (!motor_ready.load(std::memory_order_acquire) ||
               !motor_driver.initialized()) {
      motor_output_result = motor_driver.coast();
      motor_output_coasting = true;
      torque_error = protocol::quantization::TorqueError::internal_error;
    } else if (mission_snapshot.fin == mission::FinDirective::zero_hold) {
'''
new = '''    } else if (!motor_ready.load(std::memory_order_acquire) ||
               !motor_driver.initialized()) {
      motor_output_result = motor_driver.coast();
      motor_output_coasting = true;
      torque_error = protocol::quantization::TorqueError::internal_error;
    } else if (!flight_config::motorProfileValid()) {
      // CommandReceiveの明示試験は許可するが、飛行sequenceでは未認定profileを
      // ZeroHold/Roll出力へ接続しない。ForceStartでもvalidへ偽装しない。
      motor_output_result = motor_driver.brake();
      motor_output_braking = true;
      torque_error = protocol::quantization::TorqueError::limit_config_invalid;
    } else if (mission_snapshot.fin == mission::FinDirective::zero_hold) {
'''

if new in text:
    print("forced-start motor profile guard already present")
    raise SystemExit(0)

count = text.count(old)
if count != 1:
    raise SystemExit(f"guard insertion point count={count}, expected=1")

path.write_text(text.replace(old, new, 1))
print("inserted forced-start motor profile guard")
