#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "protocol/can_protocol.hpp"

namespace mission {

enum class CommandCode : uint8_t {
  start_sequence = 0x01,
  cancel_sequence = 0x02,
  disable_fin_control = 0x03,
  force_start_sequence = 0x04,
  fin_free = 0x10,
  set_fin_zero = 0x11,
  start_fin_zero_hold = 0x12,
  fin_move_relative = 0x13,
  para_free = 0x20,
  para_hold = 0x21,
  para_move_relative = 0x22,
  set_para_open = 0x23,
  set_para_close = 0x24,
  para_open = 0x25,
  para_close = 0x26,
  run_preflight_calibration = 0x30,
  export_flash_log = 0x31,
  enter_recovery = 0x33,
  actuator_emergency_result = 0xF0,
  liftoff_emergency_result = 0xF1,
};

enum class CommandDomain : uint8_t {
  sequence,
  fin,
  parachute,
  calibration,
  storage,
  recovery,
  count,
};

struct CommandContext {
  protocol::MissionState state{protocol::MissionState::command_receive};
  bool resources_preallocated{};
  bool persistence_load_complete{};
  bool persistence_runtime_available{};
  bool fin_available{true};
  bool parachute_available{true};
  bool motor_test_busy{};
  bool calibration_supported{};
  bool storage_export_supported{};
  bool deployment_power_cutoff_done{};
  bool fin_safe_commands_supported{};
};

struct CommandDecision {
  protocol::CommandResult result{};
  bool execute{};
  bool replay{};
  CommandDomain domain{CommandDomain::sequence};
};

struct EmergencyDecision {
  protocol::CommandResult result{};
  std::array<protocol::CommandResult, 2> interrupted{};
  std::size_t interrupted_count{};
  bool execute{};
};

class CommandExecutor {
public:
  static constexpr std::size_t kResultCacheSize = 16;
  [[nodiscard]] CommandDecision
  begin(const protocol::GenericCommandRequest &request,
        const CommandContext &context);
  [[nodiscard]] protocol::CommandResult
  finish(uint8_t transaction_id, protocol::CommandPhase phase,
         protocol::CommandReason reason = protocol::CommandReason::none,
         uint32_t detail = 0);
  [[nodiscard]] EmergencyDecision
  actuatorEmergency(uint8_t transaction_id, protocol::MissionState state);
  [[nodiscard]] protocol::CommandResult
  liftoffEmergencyResult(uint8_t transaction_id, bool accepted);
  [[nodiscard]] bool busy(CommandDomain domain) const;
  [[nodiscard]] std::size_t cachedCount() const;

private:
  struct Entry {
    bool valid{};
    bool pending{};
    uint32_t age{};
    protocol::GenericCommandRequest request{};
    protocol::CommandResult result{};
    CommandDomain domain{CommandDomain::sequence};
  };
  [[nodiscard]] Entry *find(uint8_t transaction_id);
  [[nodiscard]] const Entry *find(uint8_t transaction_id) const;
  [[nodiscard]] Entry *allocate();
  void remember(const protocol::GenericCommandRequest &request,
                const protocol::CommandResult &result, bool pending,
                CommandDomain domain);
  std::array<Entry, kResultCacheSize> entries_{};
  uint32_t age_{};
};

} // 名前空間 mission
