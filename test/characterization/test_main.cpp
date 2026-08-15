#include "characterization/campaign_state_machine.hpp"
#include "characterization/best_effort_file_close.hpp"
#include "characterization/characterization_config.hpp"
#include "characterization/command_journal.hpp"
#include "characterization/fin_angle.hpp"
#include "characterization/fixed_epoch_assembler.hpp"
#include "characterization/log_format_v5.hpp"
#include "characterization/position_guard.hpp"
#include "characterization/profile_plan.hpp"
#include "characterization/record_validation.hpp"
#include "characterization/shutdown_sequence.hpp"
#include "characterization/spsc_ring.hpp"
#include "characterization/zero_approach.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <thread>
#include <vector>

using namespace avi::characterization;

namespace {

int failures = 0;

#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      std::cerr << __FILE__ << ':' << __LINE__ << " CHECK failed: "           \
                << #condition << '\n';                                         \
      ++failures;                                                              \
    }                                                                          \
  } while (false)

SessionId sessionId(const char *text = "session-A") {
  SessionId id{};
  std::strncpy(id.data(), text, id.size() - 1U);
  return id;
}

RawEncoderSample sample(std::uint64_t generation,
                        std::uint64_t scheduled_us,
                        std::uint64_t capture_us,
                        std::uint16_t angle = 1000U,
                        bool valid = true) {
  RawEncoderSample value{};
  value.generation = generation;
  value.scheduled_timestamp_us = scheduled_us;
  value.capture_timestamp_us = capture_us;
  value.angle_raw = angle;
  value.diagnostic_flags =
      valid ? static_cast<std::uint16_t>(
                  DiagnosticOffsetCompensationFinished)
            : 0U;
  value.read_result_code = valid ? 0 : -1;
  value.valid = valid;
  return value;
}

void addCompleteEpoch(FixedEpochAssembler &assembler, EncoderRate rate,
                      std::uint64_t epoch_zero, std::uint64_t epoch_index,
                      std::uint64_t generation_base) {
  const std::uint8_t count = expectedSamplesPerEpoch(rate);
  for (std::uint8_t slot = 0U; slot < count; ++slot) {
    const std::uint64_t offset =
        ((2U * static_cast<std::uint64_t>(slot) + 1U) *
         kEpochDurationUs) /
        (2U * static_cast<std::uint64_t>(count));
    CHECK(assembler.add(
              sample(generation_base + slot,
                     epoch_zero + epoch_index * kEpochDurationUs + offset,
                     epoch_zero + epoch_index * kEpochDurationUs + offset,
                     static_cast<std::uint16_t>(1000U + slot))) ==
          AddResult::Accepted);
  }
}

void testCompleteRates() {
  constexpr std::uint64_t epoch_zero = 1'000'000U;
  for (const EncoderRate rate :
       {EncoderRate::Hz1000, EncoderRate::Hz2000, EncoderRate::Hz5000}) {
    FixedEpochAssembler assembler;
    assembler.reset(rate, epoch_zero);
    addCompleteEpoch(assembler, rate, epoch_zero, 0U, 1U);
    const EncoderEpochBlock block =
        assembler.release(0U, epoch_zero + 1'050U);
    CHECK(block.expected_sample_count == expectedSamplesPerEpoch(rate));
    CHECK(block.actual_sample_count == expectedSamplesPerEpoch(rate));
    CHECK(block.valid_sample_count == expectedSamplesPerEpoch(rate));
    CHECK((block.flags & EpochAggregateValid) != 0U);
    CHECK((block.flags & EpochIncomplete) == 0U);
    CHECK((block.flags & EpochStartup) == 0U);
  }
}

void testMissingDoesNotShift() {
  constexpr std::uint64_t epoch_zero = 2'000'000U;
  FixedEpochAssembler assembler(EncoderRate::Hz5000);
  assembler.reset(epoch_zero);
  const std::array<std::uint64_t, 4> offsets{100U, 300U, 700U, 900U};
  for (std::size_t index = 0; index < offsets.size(); ++index)
    CHECK(assembler.add(sample(index + 1U, epoch_zero + offsets[index],
                               epoch_zero + offsets[index])) ==
          AddResult::Accepted);
  CHECK(assembler.add(sample(5U, epoch_zero + 1'100U,
                             epoch_zero + 1'100U)) ==
        AddResult::Accepted);

  const EncoderEpochBlock first =
      assembler.release(0U, epoch_zero + 1'010U);
  CHECK(first.actual_sample_count == 4U);
  CHECK(first.skipped_sample_count == 1U);
  CHECK((first.flags & EpochIncomplete) != 0U);
  CHECK((first.flags & EpochStartup) != 0U);

  for (std::uint8_t slot = 1U; slot < 5U; ++slot) {
    const std::uint64_t capture =
        epoch_zero + 1'000U + 100U + 200U * slot;
    CHECK(assembler.add(sample(5U + slot, capture, capture)) ==
          AddResult::Accepted);
  }
  const EncoderEpochBlock second =
      assembler.release(1U, epoch_zero + 2'010U);
  CHECK(second.actual_sample_count == 5U);
  CHECK(second.skipped_sample_count == 0U);
  CHECK((second.flags & EpochAggregateValid) != 0U);
  CHECK(assembler.statistics().startup_incomplete_epochs == 1U);
  CHECK(assembler.statistics().steady_state_incomplete_epochs == 0U);
}

void testSeparateCountersAndTimestamps() {
  constexpr std::uint64_t epoch_zero = 3'000'000U;
  FixedEpochAssembler assembler(EncoderRate::Hz2000, 50U);
  assembler.reset(epoch_zero);
  CHECK(assembler.add(sample(0U, epoch_zero - 1U,
                             epoch_zero - 1U)) ==
        AddResult::PreEpoch);
  CHECK(assembler.add(sample(1U, epoch_zero + 250U,
                             epoch_zero + 250U)) ==
        AddResult::Accepted);
  CHECK(assembler.add(sample(2U, epoch_zero + 750U,
                             epoch_zero + 750U, 1001U, false)) ==
        AddResult::Accepted);
  CHECK(assembler.add(sample(3U, epoch_zero + 760U,
                             epoch_zero + 760U)) ==
        AddResult::Accepted);
  const EncoderEpochBlock block =
      assembler.release(0U, epoch_zero + 1'100U);
  CHECK(block.actual_sample_count == 3U);
  CHECK(block.repeated_sample_count == 1U);
  CHECK(block.invalid_sample_count == 1U);
  CHECK(block.consumer_lateness_us == 100);
  CHECK((block.flags & EpochRepeated) != 0U);
  CHECK((block.flags & EpochInvalid) != 0U);
  CHECK((block.flags & EpochDeadline) != 0U);
  CHECK(block.samples[0].capture_timestamp_us !=
        block.release_timestamp_us);
  CHECK(assembler.add(sample(4U, epoch_zero + 500U,
                             epoch_zero + 500U)) ==
        AddResult::AlreadyReleased);
  CHECK(assembler.statistics().pre_epoch_samples == 1U);
  CHECK(assembler.statistics().late_after_release == 1U);
  CHECK(assembler.statistics().consumer_deadline_misses == 1U);
  CHECK(assembler.add(sample(5U, epoch_zero + 1'250U,
                             epoch_zero + 1'250U)) ==
        AddResult::Accepted);
  const EncoderEpochBlock next =
      assembler.release(1U, epoch_zero + 2'010U);
  CHECK((next.flags & EpochLate) != 0U);
}

void testNegativeZeroWrap() {
  constexpr std::uint64_t epoch_zero = 4'000'000U;
  FixedEpochAssembler assembler(EncoderRate::Hz1000);
  assembler.reset(epoch_zero);
  CHECK(assembler.add(sample(1U, epoch_zero + 500U,
                             epoch_zero + 500U, 0U)) ==
        AddResult::Accepted);
  CHECK((assembler.release(0U, epoch_zero + 1'010U).flags &
         EpochAggregateValid) != 0U);
  CHECK(assembler.add(sample(2U, epoch_zero + 1'500U,
                             epoch_zero + 1'500U, 16'383U)) ==
        AddResult::Accepted);
  const EncoderEpochBlock wrapped =
      assembler.release(1U, epoch_zero + 2'010U);
  CHECK(wrapped.mean_unwrapped_counts_q16 == -65'536);
}

void testSeededZeroWrap() {
  constexpr std::uint64_t epoch_zero = 4'100'000U;
  FixedEpochAssembler positive(EncoderRate::Hz1000);
  positive.reset(epoch_zero, 16'383U);
  CHECK(positive.add(sample(1U, epoch_zero + 500U,
                            epoch_zero + 500U, 0U)) ==
        AddResult::Accepted);
  CHECK(positive.release(0U, epoch_zero + 1'010U)
            .mean_unwrapped_counts_q16 == 16'384LL * 65'536LL);
  CHECK(positive.add(sample(2U, epoch_zero + 1'500U,
                            epoch_zero + 1'500U, 1U)) ==
        AddResult::Accepted);
  CHECK(positive.release(1U, epoch_zero + 2'010U)
            .mean_unwrapped_counts_q16 == 16'385LL * 65'536LL);

  FixedEpochAssembler negative(EncoderRate::Hz1000);
  negative.reset(epoch_zero, 0U);
  CHECK(negative.add(sample(1U, epoch_zero + 500U,
                            epoch_zero + 500U, 16'383U)) ==
        AddResult::Accepted);
  CHECK(negative.release(0U, epoch_zero + 1'010U)
            .mean_unwrapped_counts_q16 == -65'536LL);
  CHECK(negative.add(sample(2U, epoch_zero + 1'500U,
                            epoch_zero + 1'500U, 16'382U)) ==
        AddResult::Accepted);
  CHECK(negative.release(1U, epoch_zero + 2'010U)
            .mean_unwrapped_counts_q16 == -2LL * 65'536LL);
}

struct FakeMotor {
  MotorCommandRequest last{};
  std::uint32_t calls{0};
  esp_err_t next_result{ESP_OK};
  esp_err_t coast_result{ESP_OK};
};

esp_err_t fakeApply(void *context,
                    const MotorCommandRequest &request,
                    std::uint64_t &completed_at_us) {
  auto &motor = *static_cast<FakeMotor *>(context);
  motor.last = request;
  ++motor.calls;
  ++completed_at_us;
  return request.mode == MotorMode::Coast ? motor.coast_result
                                          : motor.next_result;
}

void testCommandJournal() {
  FakeMotor motor{};
  CommandJournal journal(fakeApply, &motor);
  CHECK(journal.apply({20, MotorMode::DriveIn2}, 9'000U) ==
        ESP_ERR_INVALID_STATE);
  CHECK(journal.currentApplied().mode == MotorMode::Coast);
  CHECK(journal.arm() == ESP_OK);
  CHECK(journal.apply({30, MotorMode::DriveIn2}, 10'000U) == ESP_OK);
  const ImmutableCommandEvidence first = journal.snapshot(10'900U);
  CHECK(first.command_generation == 2U);
  CHECK(first.requested_command_permille == 30);
  CHECK(first.applied_command_permille == 30);
  CHECK(first.command_apply_timestamp_us == 10'001U);
  CHECK(first.logger_snapshot_timestamp_us == 10'900U);
  CHECK(motor.last.mode == MotorMode::DriveIn2);

  motor.next_result = ESP_FAIL;
  CHECK(journal.apply({-30, MotorMode::DriveIn1}, 11'000U) == ESP_FAIL);
  CHECK(!journal.armed());
  CHECK(journal.currentApplied().command_permille == 0);
  CHECK(journal.currentApplied().mode == MotorMode::Coast);

  CHECK(journal.arm() == ESP_OK);
  motor.next_result = ESP_ERR_TIMEOUT;
  motor.coast_result = ESP_FAIL;
  CHECK(journal.apply({30, MotorMode::DriveIn2}, 12'000U) ==
        ESP_ERR_TIMEOUT);
  const ImmutableCommandEvidence failed = journal.snapshot(12'100U);
  CHECK(failed.apply_result_code == ESP_ERR_TIMEOUT);
  CHECK(failed.applied_motor_mode == MotorMode::Coast);
  CHECK(journal.currentApplied().result_code == ESP_FAIL);
  CHECK(journal.stopForError(ESP_ERR_NO_MEM, 12'200U) == ESP_ERR_NO_MEM);
}

void testImmutableQueueStress() {
  constexpr std::uint64_t count = 50'000U;
  SpscRing<ImmutableLogRecord, 128U> queue;
  std::atomic<bool> done{false};
  std::atomic<bool> failed{false};
  std::thread producer([&] {
    for (std::uint64_t sequence = 1U; sequence <= count; ++sequence) {
      ImmutableLogRecord record{};
      record.sequence = sequence;
      record.command.command_generation = sequence;
      const bool positive = (sequence & 1U) != 0U;
      record.command.requested_command_permille = positive ? 30 : -30;
      record.command.requested_motor_mode =
          positive ? MotorMode::DriveIn2 : MotorMode::DriveIn1;
      record.command.applied_command_permille =
          record.command.requested_command_permille;
      record.command.applied_motor_mode =
          record.command.requested_motor_mode;
      while (!queue.push(record))
        std::this_thread::yield();
    }
    done.store(true, std::memory_order_release);
  });
  std::thread consumer([&] {
    ImmutableLogRecord record{};
    std::uint64_t expected = 1U;
    while (!done.load(std::memory_order_acquire) || !queue.empty()) {
      if (!queue.pop(record)) {
        std::this_thread::yield();
        continue;
      }
      if (record.sequence != expected ||
          record.command.command_generation != expected ||
          !motorModeMatchesCommand(
              record.command.requested_command_permille,
              record.command.requested_motor_mode))
        failed.store(true, std::memory_order_release);
      ++expected;
    }
  });
  producer.join();
  consumer.join();
  CHECK(!failed.load());
}

void resolveStage(CampaignStateMachine &campaign) {
  CHECK(campaign.beginRun(EncoderRate::Hz1000, RunKind::RateCheck) ==
        CampaignStatus::Ok);
  CHECK(campaign.finishRun(RunOutcome::Accepted) == CampaignStatus::Ok);
  CHECK(campaign.beginRun(EncoderRate::Hz1000, RunKind::Full) ==
        CampaignStatus::Ok);
  CHECK(campaign.finishRun(RunOutcome::Accepted) == CampaignStatus::Ok);
  CHECK(campaign.beginRun(EncoderRate::Hz2000, RunKind::RateCheck) ==
        CampaignStatus::Ok);
  CHECK(campaign.finishRun(RunOutcome::Unsupported,
                           UnsupportedReason::IncompleteEpoch) ==
        CampaignStatus::Ok);
  CHECK(campaign.beginRun(EncoderRate::Hz5000, RunKind::RateCheck) ==
        CampaignStatus::Ok);
  CHECK(campaign.finishRun(RunOutcome::Unsupported,
                           UnsupportedReason::TriggerCoalesced) ==
        CampaignStatus::Ok);
}

void testStageOrderAndM0Gate() {
  CampaignStateMachine campaign;
  CHECK(campaign.startNewSession(sessionId(), 0x12345678U) ==
        CampaignStatus::Ok);
  CHECK(campaign.stage() == AssemblyStage::FV);
  CHECK(campaign.confirmStage(AssemblyStage::FHPositive) ==
        CampaignStatus::WrongStage);
  CHECK(campaign.confirmStage(AssemblyStage::FV) == CampaignStatus::Ok);
  CHECK(campaign.beginRun(EncoderRate::Hz2000, RunKind::Full) ==
        CampaignStatus::RateNotQualified);
  resolveStage(campaign);
  CHECK(campaign.completeStage() == CampaignStatus::Ok);
  CHECK(campaign.stage() == AssemblyStage::FHPositive);
  CHECK(campaign.confirmStage(AssemblyStage::FHPositive) ==
        CampaignStatus::Ok);
  resolveStage(campaign);
  CHECK(campaign.completeStage() == CampaignStatus::Ok);
  CHECK(campaign.stage() == AssemblyStage::FHNegative);
  CHECK(campaign.confirmStage(AssemblyStage::FHNegative) ==
        CampaignStatus::Ok);
  resolveStage(campaign);
  CHECK(campaign.completeStage() == CampaignStatus::Ok);

  M0Handoff handoff{};
  CHECK(campaign.prepareM0(handoff) == CampaignStatus::Ok);
  CHECK(campaign.state() == CampaignState::PowerCycleRequired);
  CHECK(!campaign.canArmMotor());
  CHECK(CampaignStateMachine::validateHandoff(handoff));

  for (const ResetKind kind :
       {ResetKind::Software, ResetKind::Watchdog, ResetKind::Panic,
        ResetKind::Brownout, ResetKind::DeepSleep}) {
    CampaignStateMachine rejected;
    CHECK(rejected.resumeM0(handoff, kind, false) ==
          CampaignStatus::PowerCycleRequired);
  }
  CampaignStateMachine cookie_survived;
  CHECK(cookie_survived.resumeM0(handoff, ResetKind::PowerOn, true) ==
        CampaignStatus::ColdPowerCycleNotProven);

  CampaignStateMachine m0;
  CHECK(m0.resumeM0(handoff, ResetKind::PowerOn, false) ==
        CampaignStatus::Ok);
  CHECK(!m0.canArmMotor());
  CHECK(m0.resumeM0(sessionId("wrong"), "FIN_REMOVED") ==
        CampaignStatus::SessionIdMismatch);
  CHECK(m0.resumeM0(sessionId(), "fin_removed") ==
        CampaignStatus::FinRemovalConfirmationRequired);
  CHECK(m0.resumeM0(sessionId(), "FIN_REMOVED") ==
        CampaignStatus::Ok);
  CHECK(m0.stage() == AssemblyStage::M0);
  CHECK(m0.canArmMotor());
}

void testSessionValidationAndMandatoryRate() {
  CampaignStateMachine empty;
  CHECK(empty.startNewSession({}, 1U) ==
        CampaignStatus::InvalidArgument);
  SessionId unterminated{};
  unterminated.fill('x');
  CHECK(empty.startNewSession(unterminated, 1U) ==
        CampaignStatus::InvalidArgument);

  CampaignStateMachine mandatory;
  CHECK(mandatory.startNewSession(sessionId(), 1U) ==
        CampaignStatus::Ok);
  CHECK(mandatory.confirmStage(AssemblyStage::FV) ==
        CampaignStatus::Ok);
  CHECK(mandatory.beginRun(EncoderRate::Hz1000, RunKind::RateCheck) ==
        CampaignStatus::Ok);
  CHECK(mandatory.finishRun(RunOutcome::Unsupported,
                            UnsupportedReason::IncompleteEpoch) ==
        CampaignStatus::MandatoryRateUnsupported);
}

void testZeroApproach() {
  ZeroApproachPlan positive(ApproachBranch::FromPositive, 20, 100, 1);
  CHECK(positive.targets().front() == 1000);
  CHECK(positive.targets().back() == 0);
  for (const std::int32_t target : positive.targets()) {
    const ApproachUpdate update = positive.update(target, 0);
    CHECK(update == (target == 0 ? ApproachUpdate::complete
                                : ApproachUpdate::target_advanced));
  }
  CHECK(positive.complete());

  ZeroApproachPlan negative(ApproachBranch::FromNegative, 20, 100, 1);
  CHECK(negative.targets().front() == -1000);
  CHECK(negative.targets().back() == 0);
  CHECK(negative.update(100, 0) == ApproachUpdate::overshoot_abort);

  ZeroApproachController timeout(ApproachBranch::FromPositive, 1'000U,
                                 20, 100, 1);
  timeout.reset(10'000U);
  CHECK(timeout.update(2'000, 0, 10'500U) ==
        ApproachUpdate::holding);
  CHECK(timeout.update(2'000, 0, 11'001U) ==
        ApproachUpdate::timeout_abort);
}

void testPositionGuard() {
  PositionGuard guard({8'000, 10'000, 1});
  PositionGuardInput input{};
  input.fin_angle_millideg = 8'100;
  input.requested = {30, MotorMode::DriveIn2};
  input.encoder_valid = true;
  input.consumer_deadline_met = true;
  CHECK(guard.evaluate(input).action == GuardState::Coast);

  input.requested = {-30, MotorMode::DriveIn1};
  CHECK(guard.evaluate(input).action == GuardState::Allow);
  input.fin_angle_millideg = -8'100;
  CHECK(guard.evaluate(input).action == GuardState::Coast);
  input.requested = {30, MotorMode::DriveIn2};
  CHECK(guard.evaluate(input).action == GuardState::Allow);

  input.fin_angle_millideg = 10'000;
  CHECK(guard.evaluate(input).action == GuardState::Abort);
  input.fin_angle_millideg = 7'900;
  input.predicted_stopping_delta_millideg = 2'200;
  input.requested = {0, MotorMode::Coast};
  CHECK(guard.evaluate(input).reason ==
        PositionGuardReason::PredictedStopExceedsLimit);
  input.predicted_stopping_delta_millideg = 0;
  input.encoder_valid = false;
  CHECK(guard.evaluate(input).reason ==
        PositionGuardReason::EncoderInvalid);
}

void testProfilePlan() {
  constexpr std::uint32_t seed = 0x4D595DF4U;
  ProfilePlan fv(AssemblyStage::FV, EncoderRate::Hz1000, seed);
  ProfilePlan fh(AssemblyStage::FHPositive, EncoderRate::Hz2000, seed);
  ProfilePlan m0(AssemblyStage::M0, EncoderRate::Hz5000, seed);
  CHECK(fv.valid());
  CHECK(fv.commonComparableTo(fh));
  CHECK(fv.commonComparableTo(m0));
  CHECK(fv.zeroReferenceKind() == ZeroReferenceKind::Common);
  CHECK(m0.zeroReferenceKind() == ZeroReferenceKind::M0);
  std::array<bool, 15> seen{};
  for (const ProfileEpisode &episode : fv.commonEpisodes()) {
    const std::size_t phase =
        static_cast<std::size_t>(episode.phase);
    CHECK(phase < seen.size());
    if (phase < seen.size())
      seen[phase] = true;
  }
  for (std::size_t phase = 1U; phase < seen.size(); ++phase)
    CHECK(seen[phase]);
  ProfilePlan different_seed(AssemblyStage::FV, EncoderRate::Hz1000,
                             seed + 1U);
  CHECK(!fv.commonComparableTo(different_seed));
}

void testQueueFullAndShutdownOrder() {
  SpscRing<ImmutableLogRecord, 3U> queue;
  ImmutableLogRecord record{};
  CHECK(queue.push(record));
  CHECK(queue.push(record));
  CHECK(!queue.push(record));

  FakeMotor motor{};
  CommandJournal journal(fakeApply, &motor);
  CHECK(journal.arm() == ESP_OK);
  CHECK(journal.apply({30, MotorMode::DriveIn2}, 1'000U) == ESP_OK);
  const ImmutableCommandEvidence epoch_command = journal.snapshot(1'050U);
  CHECK(journal.stopForError(ESP_ERR_NO_MEM, 1'100U) == ESP_ERR_NO_MEM);
  CHECK(!journal.armed());
  CHECK(motor.last.mode == MotorMode::Coast);
  CHECK(motor.last.command_permille == 0);
  CHECK(journal.currentApplied().mode == MotorMode::Coast);
  const ImmutableCommandEvidence stopped = journal.snapshot(1'200U);
  CHECK(stopped.command_generation == epoch_command.command_generation);
  CHECK(stopped.requested_command_permille == 30);
  CHECK(stopped.applied_motor_mode == MotorMode::DriveIn2);

  ShutdownSequence shutdown;
  CHECK(!shutdown.mark(ShutdownStep::Disarm));
  for (const ShutdownStep step :
       {ShutdownStep::Coast, ShutdownStep::Disarm,
        ShutdownStep::SamplingStop, ShutdownStep::RealtimeQueueDrain,
        ShutdownStep::WriterQueueDrain, ShutdownStep::Sync,
        ShutdownStep::Footer, ShutdownStep::Close,
        ShutdownStep::PersistM0,
        ShutdownStep::PowerCycleRequired})
    CHECK(shutdown.mark(step));
  CHECK(shutdown.completedSteps() == 10U);
  CHECK(shutdown.mask() == 0x3FFU);
}

struct FakeFileClose {
  std::array<unsigned, 4> calls{};
  esp_err_t write_result{ESP_OK};
  esp_err_t flush_result{ESP_OK};
  esp_err_t sync_result{ESP_OK};
  esp_err_t close_result{ESP_OK};
};

esp_err_t fakeFileWrite(void *context, const std::uint8_t *,
                        std::size_t) {
  auto &file = *static_cast<FakeFileClose *>(context);
  ++file.calls[0];
  return file.write_result;
}

esp_err_t fakeFileFlush(void *context) {
  auto &file = *static_cast<FakeFileClose *>(context);
  ++file.calls[1];
  return file.flush_result;
}

esp_err_t fakeFileSync(void *context) {
  auto &file = *static_cast<FakeFileClose *>(context);
  ++file.calls[2];
  return file.sync_result;
}

esp_err_t fakeFileClose(void *context) {
  auto &file = *static_cast<FakeFileClose *>(context);
  ++file.calls[3];
  return file.close_result;
}

void testBestEffortCloseAndAngleConversion() {
  std::array<std::uint8_t, 4> footer{1U, 2U, 3U, 4U};
  FakeFileClose file{};
  file.write_result = ESP_FAIL;
  file.flush_result = ESP_ERR_TIMEOUT;
  file.sync_result = ESP_ERR_INVALID_STATE;
  const FileCloseOperations operations{&file, fakeFileWrite, fakeFileFlush,
                                       fakeFileSync, fakeFileClose};
  CHECK(bestEffortFinalizeFile(operations, footer.data(), footer.size(),
                               ESP_OK) == ESP_FAIL);
  const std::array<unsigned, 4> all_called{1U, 1U, 1U, 1U};
  CHECK(file.calls == all_called);

  file = {};
  file.close_result = ESP_FAIL;
  CHECK(bestEffortFinalizeFile(operations, footer.data(), footer.size(),
                               ESP_ERR_TIMEOUT) == ESP_ERR_TIMEOUT);
  CHECK(file.calls == all_called);

  std::int32_t fin_angle = 0;
  constexpr std::uint16_t zero_raw = 1'000U;
  constexpr std::int64_t two_motor_turns_q16 =
      static_cast<std::int64_t>(zero_raw + 2U * 16'384U) << 16U;
  CHECK(finAngleMilliDegreesFromUnwrappedQ16(
      two_motor_turns_q16, zero_raw, fin_angle));
  CHECK(fin_angle >= 4'086 && fin_angle <= 4'088);
  CHECK(!finAngleMilliDegreesFromUnwrappedQ16(
      std::numeric_limits<std::int64_t>::max(), zero_raw, fin_angle));
  CHECK(!kHardwareDriveApproved);
  CHECK(kCommandToFinSign == 0);
  CHECK(commandForFinError(100, 20, 1) == 20);
  CHECK(commandForFinError(-100, 20, 1) == -20);
  CHECK(commandForFinError(100, 20, -1) == -20);
  CHECK(commandForFinError(-100, 20, -1) == 20);
  CHECK(commandForFinError(0, 20, 1) == 0);
}

ImmutableLogRecord validRecord() {
  ImmutableLogRecord record{};
  record.sequence = 52'764U;
  record.stage = AssemblyStage::FV;
  record.encoder_rate = EncoderRate::Hz1000;
  record.profile_phase = ProfilePhase::StationaryBaseline;
  record.run_kind = RunKind::RateCheck;
  record.episode_index = 1U;
  record.command.command_generation = 8U;
  record.command.requested_command_permille = 0;
  record.command.requested_motor_mode = MotorMode::Coast;
  record.command.applied_command_permille = 0;
  record.command.applied_motor_mode = MotorMode::Coast;
  record.command.command_apply_timestamp_us = 10'000U;
  record.command.logger_snapshot_timestamp_us = 11'100U;
  record.power.capture_timestamp_us = 10'500U;
  record.power.motor_millivolts = 9'000U;
  record.power.read_result = 0;
  record.power.valid = true;
  record.encoder.epoch_index = 1U;
  record.encoder.epoch_start_timestamp_us = 10'000U;
  record.encoder.epoch_end_timestamp_us = 11'000U;
  record.encoder.release_timestamp_us = 11'050U;
  record.encoder.consumer_lateness_us = 50;
  record.encoder.expected_sample_count = 1U;
  record.encoder.actual_sample_count = 1U;
  record.encoder.observed_sample_count = 1U;
  record.encoder.selected_sample_count = 1U;
  record.encoder.valid_sample_count = 1U;
  record.encoder.flags = EpochAggregateValid;
  record.encoder.sample_present[0] = true;
  record.encoder.samples[0] =
      sample(9U, 10'500U, 10'500U, 0x1234U, true);
  record.encoder.samples[0].slot = 0U;
  return record;
}

LogHeaderV5 validHeader() {
  LogHeaderV5 header{};
  header.stage = AssemblyStage::FV;
  header.encoder_rate = EncoderRate::Hz1000;
  header.run_kind = RunKind::RateCheck;
  header.profile_seed = 0x12345678U;
  header.pwm_frequency_hz = 30'000U;
  header.reset_reason = 1U;
  header.epoch_zero_timestamp_us = 9'000U;
  header.session_id = sessionId();
  std::memcpy(header.firmware_sha.data(),
              "0123456789abcdef0123456789abcdef01234567",
              header.firmware_sha.size());
  std::memcpy(header.avi_esp_libs_sha.data(),
              "89abcdef0123456789abcdef0123456789abcdef",
              header.avi_esp_libs_sha.size());
  std::strncpy(header.board_build_id.data(), "avi_99l_missionboard",
               header.board_build_id.size() - 1U);
  return header;
}

void testRecordValidation() {
  ImmutableLogRecord record = validRecord();
  CHECK(!hasError(validateRecord(record)));

  record.command.command_apply_timestamp_us = 10'100U;
  CHECK(!hasError(validateRecord(record)));
  record.command.command_apply_timestamp_us = 10'101U;
  CHECK(hasError(validateRecord(record)));
  record.encoder.flags =
      static_cast<std::uint16_t>(record.encoder.flags | EpochDeadline);
  CHECK(!hasError(validateRecord(record)));
  record.run_kind = RunKind::Full;
  record.qualification = 1U;
  CHECK(hasError(validateRecord(record)));
  record.first_error = ESP_ERR_TIMEOUT;
  record.abort_reason = AbortReason::Deadline;
  CHECK(!hasError(validateRecord(record)));

  record.command.applied_motor_mode = MotorMode::DriveIn2;
  CHECK(hasError(validateRecord(record)));
  record = validRecord();
  record.encoder.actual_sample_count = 2U;
  CHECK(hasError(validateRecord(record)));
  record = validRecord();
  record.encoder.flags =
      static_cast<std::uint16_t>(EpochAggregateValid | EpochStartup);
  CHECK(hasError(validateRecord(record)));
  record = validRecord();
  record.encoder.samples[0].diagnostic_flags |= DiagnosticParityError;
  CHECK(hasError(validateRecord(record)));
  record = validRecord();
  record.stage = AssemblyStage::M0;
  CHECK(hasError(validateRecord(record)));
}

void testV5WireContract() {
  using namespace avi::characterization::wire_v5;
  constexpr std::array<std::uint8_t, 9> check{
      '1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(crc32(check.data(), check.size()) == 0xCBF43926U);
  CHECK(static_cast<std::uint8_t>(
            UnsupportedReason::TriggerCoalesced) == 1U);
  CHECK(static_cast<std::uint8_t>(
            UnsupportedReason::OperatorMarkedUnsupported) == 8U);

  HeaderBytes header_bytes{};
  RecordBytes record_bytes{};
  FooterBytes footer_bytes{};
  const LogHeaderV5 header = validHeader();
  LogHeaderV5 invalid_header = header;
  invalid_header.routine_guard_deg = invalid_header.hard_abort_deg;
  CHECK(!encodeHeader(invalid_header, header_bytes));
  const ImmutableLogRecord record = validRecord();
  ImmutableLogRecord error_record = record;
  error_record.first_error = -77;
  error_record.abort_reason = AbortReason::ValidationError;
  CHECK(encodeHeader(header, header_bytes));
  CHECK(encodeRecord(error_record, record_bytes));
  CHECK(record_bytes[132] == 0xB3U);
  CHECK(record_bytes[133] == 0xFFU);
  CHECK(record_bytes[136] == 9U);
  CHECK(record_bytes[160] == 0x34U);
  CHECK(record_bytes[161] == 0x12U);
  CHECK(std::all_of(record_bytes.begin() + 172,
                    record_bytes.begin() + 316,
                    [](std::uint8_t value) { return value == 0U; }));

  LogHeaderV5 decoded_header{};
  ImmutableLogRecord decoded_record{};
  CHECK(decodeHeader(header_bytes.data(), header_bytes.size(),
                     decoded_header) == DecodeError::None);
  CHECK(decodeRecord(record_bytes.data(), record_bytes.size(),
                     decoded_record) == DecodeError::None);
  CHECK(decoded_record.first_error == -77);
  CHECK(decoded_record.encoder.samples[0].angle_raw == 0x1234U);

  std::vector<std::uint8_t> capture;
  capture.insert(capture.end(), header_bytes.begin(), header_bytes.end());
  CHECK(encodeRecord(record, record_bytes));
  capture.insert(capture.end(), record_bytes.begin(), record_bytes.end());
  LogFooterV5 footer{};
  footer.completion = CompletionCode::Normal;
  footer.rate_supported = true;
  footer.total_records = 1U;
  footer.first_sequence = record.sequence;
  footer.last_sequence = record.sequence;
  footer.qualification_valid_epochs = 1U;
  footer.qualification_total_epochs = 1U;
  footer.shutdown_step_mask = 0x3FU;
  footer.file_crc32 = crc32(capture.data(), capture.size());
  CHECK(encodeFooter(footer, footer_bytes));
  capture.insert(capture.end(), footer_bytes.begin(), footer_bytes.end());
  const CaptureSummary summary =
      validateCapture(capture.data(), capture.size());
  CHECK(summary.error == DecodeError::None);
  CHECK(summary.records == 1U);

  const auto validateFooterOnly = [&](LogFooterV5 diagnostic_footer) {
    std::vector<std::uint8_t> diagnostic_capture{
        header_bytes.begin(), header_bytes.end()};
    diagnostic_footer.file_crc32 =
        crc32(diagnostic_capture.data(), diagnostic_capture.size());
    FooterBytes diagnostic_bytes{};
    CHECK(encodeFooter(diagnostic_footer, diagnostic_bytes));
    diagnostic_capture.insert(diagnostic_capture.end(),
                              diagnostic_bytes.begin(),
                              diagnostic_bytes.end());
    return validateCapture(diagnostic_capture.data(),
                           diagnostic_capture.size());
  };
  LogFooterV5 deadline_footer{};
  deadline_footer.completion = CompletionCode::Unsupported;
  deadline_footer.rate_supported = false;
  deadline_footer.unsupported_reason = UnsupportedReason::DeadlineMiss;
  deadline_footer.statistics.consumer_deadline_misses = 1U;
  deadline_footer.shutdown_step_mask = 0x3FU;
  CHECK(validateFooterOnly(deadline_footer).error == DecodeError::None);

  LogFooterV5 vbus_footer{};
  vbus_footer.completion = CompletionCode::Aborted;
  vbus_footer.statistics.first_error = ESP_FAIL;
  vbus_footer.statistics.vbus_invalid_samples = 1U;
  vbus_footer.shutdown_step_mask = 0x3FU;
  CHECK(validateFooterOnly(vbus_footer).error == DecodeError::None);
  vbus_footer.completion = CompletionCode::Unsupported;
  vbus_footer.unsupported_reason = UnsupportedReason::DeadlineMiss;
  vbus_footer.statistics.consumer_deadline_misses = 1U;
  CHECK(validateFooterOnly(vbus_footer).error == DecodeError::Invariant);

  CHECK(validateCapture(capture.data(), capture.size() - 1U).error !=
        DecodeError::None);
  capture.push_back(0U);
  CHECK(validateCapture(capture.data(), capture.size()).error ==
        DecodeError::TrailingBytes);
  capture.pop_back();

  RecordBytes unknown = record_bytes;
  unknown[113] = 99U;
  const std::uint32_t unknown_crc = crc32(unknown.data(), 316U);
  for (std::size_t index = 0U; index < 4U; ++index)
    unknown[316U + index] =
        static_cast<std::uint8_t>(unknown_crc >> (index * 8U));
  CHECK(decodeRecord(unknown.data(), unknown.size(), decoded_record) ==
        DecodeError::Enum);
}

void testPythonGoldenFixture() {
  using namespace avi::characterization::wire_v5;
  std::ifstream stream(
      "test/characterization/fixtures/99lmcv5_golden.bin",
      std::ios::binary);
  CHECK(stream.is_open());
  const std::vector<std::uint8_t> capture{
      std::istreambuf_iterator<char>(stream),
      std::istreambuf_iterator<char>()};
  CHECK(capture.size() == 1'088U);
  CHECK(crc32(capture.data(), capture.size()) == 0x34A11FA9U);
  const CaptureSummary summary =
      validateCapture(capture.data(), capture.size());
  CHECK(summary.error == DecodeError::None);
  CHECK(summary.records == 2U);

  LogHeaderV5 header{};
  HeaderBytes header_bytes{};
  CHECK(decodeHeader(capture.data(), kHeaderBytes, header) ==
        DecodeError::None);
  CHECK(encodeHeader(header, header_bytes));
  CHECK(std::equal(header_bytes.begin(), header_bytes.end(),
                   capture.begin()));

  for (std::size_t index = 0U; index < 2U; ++index) {
    const std::size_t offset = kHeaderBytes + index * kRecordBytes;
    ImmutableLogRecord record{};
    RecordBytes record_bytes{};
    CHECK(decodeRecord(capture.data() + offset, kRecordBytes, record) ==
          DecodeError::None);
    CHECK(encodeRecord(record, record_bytes));
    CHECK(std::equal(record_bytes.begin(), record_bytes.end(),
                     capture.begin() + offset));
  }

  const std::size_t footer_offset = capture.size() - kFooterBytes;
  LogFooterV5 footer{};
  FooterBytes footer_bytes{};
  CHECK(decodeFooter(capture.data() + footer_offset, kFooterBytes,
                     footer) == DecodeError::None);
  CHECK(encodeFooter(footer, footer_bytes));
  CHECK(std::equal(footer_bytes.begin(), footer_bytes.end(),
                   capture.begin() + footer_offset));
}

} // 無名名前空間

int main() {
  testCompleteRates();
  testMissingDoesNotShift();
  testSeparateCountersAndTimestamps();
  testNegativeZeroWrap();
  testSeededZeroWrap();
  testCommandJournal();
  testImmutableQueueStress();
  testStageOrderAndM0Gate();
  testSessionValidationAndMandatoryRate();
  testZeroApproach();
  testPositionGuard();
  testProfilePlan();
  testQueueFullAndShutdownOrder();
  testBestEffortCloseAndAngleConversion();
  testRecordValidation();
  testV5WireContract();
  testPythonGoldenFixture();

  if (failures != 0) {
    std::cerr << failures << " characterization assertion(s) failed\n";
    return EXIT_FAILURE;
  }
  std::cout << "all characterization native tests passed\n";
  return EXIT_SUCCESS;
}
