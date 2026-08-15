from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / "src/runtime/production_runtime.cpp"


def replace_once(old: str, new: str) -> None:
    text = PATH.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise RuntimeError(f"expected one match, got {count}: {old[:140]!r}")
    PATH.write_text(text.replace(old, new, 1), encoding="utf-8")


# 通常Startの7項目missingはSTS availabilityより先にterminal NotConfiguredへする。
replace_once(
    "        } else if (command_request.kind ==\n"
    "                       ParachuteCommandRequest::Kind::generic &&\n"
    "                   code == mission::CommandCode::para_close &&\n"
    "                   !configuration.active().closeConfigured()) {\n"
    "          requestFinish(protocol::CommandReason::not_configured);\n"
    "        } else if (!requestPower(pending.started_at_us)) {",
    "        } else if (command_request.kind ==\n"
    "                       ParachuteCommandRequest::Kind::generic &&\n"
    "                   code == mission::CommandCode::para_close &&\n"
    "                   !configuration.active().closeConfigured()) {\n"
    "          requestFinish(protocol::CommandReason::not_configured);\n"
    "        } else if (command_request.kind ==\n"
    "                       ParachuteCommandRequest::Kind::start_preparation &&\n"
    "                   !parachute_config_load_complete.load(\n"
    "                       std::memory_order_acquire)) {\n"
    "          requestFinish(protocol::CommandReason::busy,\n"
    "                        kDetailConfigurationLoad);\n"
    "        } else if (command_request.kind ==\n"
    "                       ParachuteCommandRequest::Kind::start_preparation &&\n"
    "                   !parachute_persistence_ready.load(\n"
    "                       std::memory_order_acquire)) {\n"
    "          requestFinish(protocol::CommandReason::persistence_error,\n"
    "                        kDetailConfigurationLoad);\n"
    "        } else if (command_request.kind ==\n"
    "                       ParachuteCommandRequest::Kind::start_preparation &&\n"
    "                   code == mission::CommandCode::start_sequence &&\n"
    "                   command_request.readiness.missingMask() != 0) {\n"
    "          requestFinish(protocol::CommandReason::not_configured,\n"
    "                        command_request.readiness.missingMask());\n"
    "        } else if (!requestPower(pending.started_at_us)) {",
)

# SetParaOpen/CloseのNVS完了後は通常CommandReceive policyどおりpower/Holdを維持する。
replace_once(
    "      if (persistence_response.success) {\n"
    "        // Emergency後でもcommit済みならRAMをNVSへ合わせ、Completedは送らない。\n"
    "        configuration.activatePersistedCandidate(pending.candidate);\n"
    "        updateConfigurationMirrors();\n"
    "        if (!pending.interrupted)\n"
    "          powerOff(false, true, protocol::ParaMode::powered_off);\n"
    "        requestFinish(protocol::CommandReason::none);\n"
    "      } else {\n"
    "        if (!pending.interrupted)\n"
    "          powerOff(false, true, protocol::ParaMode::powered_off);\n"
    "        requestFinish(protocol::CommandReason::persistence_error);\n"
    "      }",
    "      if (persistence_response.success) {\n"
    "        // Emergency後でもcommit済みならRAMをNVSへ合わせ、Completedは送らない。\n"
    "        configuration.activatePersistedCandidate(pending.candidate);\n"
    "        updateConfigurationMirrors();\n"
    "        if (!pending.interrupted) {\n"
    "          desired = DesiredState::holding;\n"
    "          para_mode_actual.store(protocol::ParaMode::hold,\n"
    "                                 std::memory_order_release);\n"
    "        }\n"
    "        requestFinish(protocol::CommandReason::none);\n"
    "      } else {\n"
    "        if (!pending.interrupted) {\n"
    "          desired = DesiredState::holding;\n"
    "          hold_established = false;\n"
    "        }\n"
    "        requestFinish(protocol::CommandReason::persistence_error);\n"
    "      }",
)

# CommandReceive/Force preparation中のHold失敗はdeployment failure latchを消費しない。
replace_once(
    "          if (hold_established)\n"
    "            para_mode_actual.store(protocol::ParaMode::hold,\n"
    "                                   std::memory_order_release);\n"
    "          else\n"
    "            recordParachuteFailure(ParachuteDeploymentFailure::hold_failed);",
    "          if (hold_established) {\n"
    "            para_mode_actual.store(protocol::ParaMode::hold,\n"
    "                                   std::memory_order_release);\n"
    "          } else if (active_epoch != 0) {\n"
    "            recordParachuteFailure(\n"
    "                ParachuteDeploymentFailure::hold_failed);\n"
    "          } else {\n"
    "            std::printf(\"parachute hold failed outside deployment\\n\");\n"
    "          }",
)

print("ForceStart Stage 2 runtime ordering/power policy fixed")
