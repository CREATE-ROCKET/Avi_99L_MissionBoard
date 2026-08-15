#include "mission/command_executor.hpp"

#include <algorithm>
#include <cstdlib>

namespace mission {
namespace {
using protocol::CommandPhase;
using protocol::CommandReason;
using protocol::CommandResult;
using protocol::GenericCommandRequest;
using protocol::MissionState;

bool sameRequest(const GenericCommandRequest &left,
                 const GenericCommandRequest &right) {
  return left.transaction_id == right.transaction_id &&
         left.command == right.command && left.arguments == right.arguments;
}

bool allZero(const GenericCommandRequest &request, std::size_t first) {
  return std::all_of(request.arguments.begin() + first, request.arguments.end(),
                     [](uint8_t value) { return value == 0; });
}

bool commandKnown(CommandCode code) {
  const uint8_t raw = static_cast<uint8_t>(code);
  return (raw >= 0x01 && raw <= 0x04) ||
         (raw >= 0x10 && raw <= 0x13) ||
         (raw >= 0x20 && raw <= 0x26) ||
         (raw >= 0x30 && raw <= 0x31);
}

CommandDomain domainFor(CommandCode code) {
  switch (code) {
  case CommandCode::start_sequence:
  case CommandCode::cancel_sequence:
  case CommandCode::disable_fin_control:
  case CommandCode::force_start_sequence:
    return CommandDomain::sequence;
  case CommandCode::fin_free:
  case CommandCode::set_fin_zero:
  case CommandCode::start_fin_zero_hold:
  case CommandCode::fin_move_relative:
    return CommandDomain::fin;
  case CommandCode::para_free:
  case CommandCode::para_hold:
  case CommandCode::para_move_relative:
  case CommandCode::set_para_open:
  case CommandCode::set_para_close:
  case CommandCode::para_open:
  case CommandCode::para_close:
    return CommandDomain::parachute;
  case CommandCode::run_preflight_calibration:
    return CommandDomain::calibration;
  case CommandCode::export_flash_log:
    return CommandDomain::storage;
  case CommandCode::select_motor_profile:
    return CommandDomain::motor_profile;
  default:
    return CommandDomain::sequence;
  }
}

bool argumentsValid(const GenericCommandRequest &request, CommandCode code) {
  switch (code) {
  case CommandCode::fin_move_relative:
    return allZero(request, 2);
  case CommandCode::para_move_relative: {
    if (!allZero(request, 2))
      return false;
    const auto raw = static_cast<uint16_t>(request.arguments[0]) |
                     static_cast<uint16_t>(request.arguments[1]) << 8U;
    return std::abs(static_cast<int32_t>(static_cast<int16_t>(raw))) < 1800;
  }
  case CommandCode::select_motor_profile:
    return request.arguments[0] != 0 && allZero(request, 1);
  default:
    return allZero(request, 0);
  }
}

bool stateValid(CommandCode code, MissionState state) {
  switch (code) {
  case CommandCode::start_sequence:
  case CommandCode::force_start_sequence:
  case CommandCode::fin_free:
  case CommandCode::set_fin_zero:
  case CommandCode::start_fin_zero_hold:
  case CommandCode::fin_move_relative:
  case CommandCode::para_free:
  case CommandCode::para_hold:
  case CommandCode::para_move_relative:
  case CommandCode::set_para_open:
  case CommandCode::set_para_close:
  case CommandCode::para_open:
  case CommandCode::para_close:
  case CommandCode::run_preflight_calibration:
  case CommandCode::export_flash_log:
  case CommandCode::select_motor_profile:
    return state == MissionState::command_receive;
  case CommandCode::cancel_sequence:
    return state == MissionState::liftoff_detection;
  case CommandCode::disable_fin_control:
    return state == MissionState::liftoff_detection ||
           state == MissionState::engine_burn || state == MissionState::control;
  default:
    return false;
  }
}

bool isStart(CommandCode code) {
  return code == CommandCode::start_sequence ||
         code == CommandCode::force_start_sequence;
}

CommandResult resultFor(const GenericCommandRequest &request,
                        CommandPhase phase, CommandReason reason,
                        uint32_t detail = 0) {
  return {request.transaction_id, request.command, phase, reason, detail};
}
} // 無名名前空間

CommandDecision CommandExecutor::begin(const GenericCommandRequest &request,
                                       const CommandContext &context) {
  if (request.transaction_id == 0)
    return {resultFor(request, CommandPhase::rejected,
                      CommandReason::invalid_argument),
            false, false, CommandDomain::sequence};

  if (const Entry *const existing = find(request.transaction_id)) {
    if (sameRequest(existing->request, request))
      return {existing->result, false, true, existing->domain};
    return {resultFor(request, CommandPhase::rejected,
                      CommandReason::protocol_error),
            false, false, existing->domain};
  }

  const auto code = static_cast<CommandCode>(request.command);
  const CommandDomain domain = domainFor(code);
  CommandReason rejection = CommandReason::none;
  if (!commandKnown(code))
    rejection = CommandReason::not_supported;
  else if (!argumentsValid(request, code))
    rejection = CommandReason::invalid_argument;
  else if (!stateValid(code, context.state))
    rejection = CommandReason::invalid_state;
  else if (isStart(code) &&
           (busy(CommandDomain::parachute) || busy(CommandDomain::fin) ||
            busy(CommandDomain::calibration) || context.motor_test_busy))
    rejection = CommandReason::busy;
  else if (isStart(code) && !context.resources_preallocated)
    rejection = CommandReason::internal_error;
  else if (isStart(code) && !context.persistence_load_complete)
    rejection = CommandReason::busy;
  else if (isStart(code) && !context.persistence_runtime_available)
    rejection = CommandReason::persistence_error;
  else if (domain == CommandDomain::parachute && busy(CommandDomain::sequence))
    rejection = CommandReason::busy;
  else if ((domain == CommandDomain::fin && !context.fin_available) ||
           (domain == CommandDomain::parachute && !context.parachute_available))
    rejection = CommandReason::device_unavailable;
  else if (domain == CommandDomain::fin &&
           !context.fin_safe_commands_supported)
    rejection = CommandReason::not_supported;
  else if ((domain == CommandDomain::calibration &&
            !context.calibration_supported) ||
           (domain == CommandDomain::storage &&
            !context.storage_export_supported) ||
           (domain == CommandDomain::motor_profile &&
            !context.motor_profile_selection_supported))
    rejection = CommandReason::not_supported;
  else if (busy(domain))
    rejection = CommandReason::busy;
  else if (isStart(code) && cachedCount() != 0 &&
           std::any_of(entries_.begin(), entries_.end(),
                       [](const Entry &entry) { return entry.pending; }))
    rejection = CommandReason::busy;
  else if (domain == CommandDomain::calibration &&
           (busy(CommandDomain::fin) || busy(CommandDomain::parachute) ||
            context.motor_test_busy))
    rejection = CommandReason::busy;
  else if ((domain == CommandDomain::fin || domain == CommandDomain::parachute) &&
           busy(CommandDomain::calibration))
    rejection = CommandReason::busy;

  if (rejection != CommandReason::none) {
    const auto result = resultFor(request, CommandPhase::rejected, rejection);
    remember(request, result, false, domain);
    return {result, false, false, domain};
  }
  const auto result = resultFor(request, CommandPhase::accepted, CommandReason::none);
  remember(request, result, true, domain);
  return {result, true, false, domain};
}

CommandResult CommandExecutor::finish(uint8_t transaction_id,
                                      CommandPhase phase,
                                      CommandReason reason, uint32_t detail) {
  Entry *const entry = find(transaction_id);
  if (entry == nullptr || !entry->pending ||
      (phase != CommandPhase::completed && phase != CommandPhase::failed))
    return {transaction_id, 0, CommandPhase::failed,
            CommandReason::internal_error, detail};
  entry->pending = false;
  entry->age = ++age_;
  entry->result.phase = phase;
  entry->result.reason = reason;
  entry->result.detail = detail;
  return entry->result;
}

EmergencyDecision CommandExecutor::actuatorEmergency(uint8_t transaction_id,
                                                      MissionState state) {
  EmergencyDecision decision{};
  const bool accepted = transaction_id != 0 && state == MissionState::command_receive;
  decision.result = {transaction_id,
                     static_cast<uint8_t>(CommandCode::actuator_emergency_result),
                     accepted ? CommandPhase::completed : CommandPhase::rejected,
                     accepted ? CommandReason::none
                              : (transaction_id == 0 ? CommandReason::invalid_argument
                                                     : CommandReason::invalid_state),
                     0};
  decision.execute = accepted;
  if (!decision.execute)
    return decision;
  for (auto &entry : entries_) {
    if (!entry.pending ||
        (entry.domain != CommandDomain::fin &&
         entry.domain != CommandDomain::parachute))
      continue;
    entry.pending = false;
    entry.age = ++age_;
    entry.result.phase = CommandPhase::failed;
    entry.result.reason = CommandReason::interrupted_by_emergency;
    if (decision.interrupted_count < decision.interrupted.size())
      decision.interrupted[decision.interrupted_count++] = entry.result;
  }
  return decision;
}

CommandResult CommandExecutor::liftoffEmergencyResult(uint8_t transaction_id,
                                                      bool accepted) {
  const bool valid = transaction_id != 0 && accepted;
  return {transaction_id,
          static_cast<uint8_t>(CommandCode::liftoff_emergency_result),
          valid ? CommandPhase::completed : CommandPhase::rejected,
          valid ? CommandReason::none
                : (transaction_id == 0 ? CommandReason::invalid_argument
                                       : CommandReason::invalid_state),
          0};
}

bool CommandExecutor::busy(CommandDomain domain) const {
  return std::any_of(entries_.begin(), entries_.end(), [domain](const Entry &entry) {
    return entry.valid && entry.pending && entry.domain == domain;
  });
}

std::size_t CommandExecutor::cachedCount() const {
  return static_cast<std::size_t>(std::count_if(
      entries_.begin(), entries_.end(), [](const Entry &entry) { return entry.valid; }));
}

CommandExecutor::Entry *CommandExecutor::find(uint8_t transaction_id) {
  const auto iterator = std::find_if(entries_.begin(), entries_.end(),
                                     [transaction_id](const Entry &entry) {
    return entry.valid && entry.request.transaction_id == transaction_id;
  });
  return iterator == entries_.end() ? nullptr : &*iterator;
}
const CommandExecutor::Entry *CommandExecutor::find(uint8_t transaction_id) const {
  const auto iterator = std::find_if(entries_.begin(), entries_.end(),
                                     [transaction_id](const Entry &entry) {
    return entry.valid && entry.request.transaction_id == transaction_id;
  });
  return iterator == entries_.end() ? nullptr : &*iterator;
}
CommandExecutor::Entry *CommandExecutor::allocate() {
  const auto unused = std::find_if(entries_.begin(), entries_.end(),
                                   [](const Entry &entry) { return !entry.valid; });
  if (unused != entries_.end())
    return &*unused;
  auto oldest = entries_.end();
  for (auto iterator = entries_.begin(); iterator != entries_.end(); ++iterator) {
    if (!iterator->pending &&
        (oldest == entries_.end() || iterator->age < oldest->age))
      oldest = iterator;
  }
  return oldest == entries_.end() ? nullptr : &*oldest;
}
void CommandExecutor::remember(const GenericCommandRequest &request,
                               const CommandResult &result, bool pending,
                               CommandDomain domain) {
  Entry *const entry = allocate();
  if (entry == nullptr)
    return;
  *entry = {true, pending, ++age_, request, result, domain};
}

} // 名前空間 mission
