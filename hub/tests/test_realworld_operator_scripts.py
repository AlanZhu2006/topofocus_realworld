from __future__ import annotations

from pathlib import Path
import subprocess


HUB = Path(__file__).resolve().parents[1]
SCRIPTS = HUB / "scripts"
OVERLAY = HUB / "robot_overlay"


def test_oneclick_is_session_bound_and_has_no_forensic_bypass():
    source = (SCRIPTS / "realworld_oneclick.sh").read_text()

    assert "--session-file" in source
    assert "manage_realworld_session.py" in source
    assert "freeze_realworld_inputs.py" in source
    assert "--allow-map-rebuild" in source
    assert "mark-debug" in source
    assert "verify_remote_release" in source
    assert "DEBUG_STACK_NO_MOTION_VERIFIED" in source
    assert "FOCUS_SESSION_CONTRACT_SHA256" in source
    assert "FOCUS_WSJ_REMOTE_CALIBRATION" in source
    assert "FOCUS_YUNJI_REMOTE_CALIBRATION" in source
    assert "--allow-stale-shadow-input" not in source
    assert "--allow-blocked-shadow-input" not in source
    assert "map_out_wsj_20260724" not in source
    assert "shared-board-odin1-20260723-v3" not in source
    assert '"${map_resume_args[@]}"' in source
    assert "both maps must exist or both be absent" in source


def test_live_arming_precedes_continuous_runner_and_has_exit_disarm():
    source = (SCRIPTS / "realworld_oneclick.sh").read_text()
    runner = (HUB / "tools/run_v2_source_episode.py").read_text()

    assert "trap cleanup_on_exit EXIT INT TERM" in source
    assert 'restart_hub "$FOCUS_DEBUG_ROBOT_CONFIG" false' in source
    assert "OPERATOR_PRESENT_AND_ROBOTS_CLEAR" in source
    arm = source.index("\n  arm_live_robots\n")
    ready = source.index("\n  wait_for_live_readiness\n")
    episode = source.index('"$HUB_DIR/tools/run_v2_source_episode.py"')
    assert arm < ready < episode
    assert "freeze_next_round(" in runner
    assert "run_shadow_round(" in runner
    assert "wait_and_seal_terminal_evidence(" in runner
    assert "semantic_arrival_episode_complete_hold" in runner
    assert "LIVE_RECEIVERS_READY_NO_GOAL" in source
    assert 'payload.get("ready_for_goal") is not True' in source
    assert 'payload.get("health_source") != "heartbeat"' in source


def test_oneclick_stop_publishes_are_bounded_and_glm_can_be_adopted():
    source = (SCRIPTS / "realworld_oneclick.sh").read_text()

    assert source.count("timeout 5 ros2 topic pub --once") == 8
    assert "tmux rename-session" in source
    assert "run_glm_offline.sh" in source
    assert "GLM endpoint is live but not owned by a verified GLM tmux." in source
    assert "deadline=$((SECONDS + 90))" in source


def test_remote_completion_marker_always_starts_on_a_new_line():
    for name in ("realworld_oneclick.sh", "calibrate_realworld_session.sh"):
        source = (SCRIPTS / name).read_text()
        assert "'bash -lc %q; rc=$?; echo; echo __%s_RC=$rc'" in source


def test_calibration_wrapper_is_board_only_and_runs_strict_debug():
    source = (SCRIPTS / "calibrate_realworld_session.sh").read_text()

    assert "OPERATOR_PRESENT_AND_BOARD_ONLY" in source
    assert "select_live_board_pair.py" in source
    assert "--holdout-reference-sequence" in source
    assert "verify_remote_release" in source
    assert "realworld_oneclick.sh" in source
    assert "--mode debug" in source
    assert "OPERATOR_PRESENT_AND_ROBOTS_CLEAR" not in source
    assert "CALIBRATION_PREVIEW_READY" in source
    assert "INITIAL_BOARD_FIT_READY" in source
    assert "CALIBRATION_HOLDOUT_PASSED" in source
    assert "while true; do" in source
    assert "SECONDS >= deadline" in source
    assert source.index("CALIBRATION_PREVIEW_READY") < source.index(
        "INITIAL_BOARD_FIT_READY"
    )
    assert source.index("INITIAL_BOARD_FIT_READY") < source.index(
        "CALIBRATION_HOLDOUT_PASSED"
    )
    assert source.count("read -r -p") == 2
    assert 'row.get("camera_ready") is not True' in source
    assert "--min-board-spacing-px" in source
    assert "BOARD_TOO_SMALL" in source


def test_wsj_calibration_uses_one_native_infrared_geometry_frame():
    launcher = (OVERLAY / "start_wsj_calibration_observation.sh").read_text()

    assert "--rgb-topic /camera/camera/infra1/image_rect_raw" in launcher
    assert "--rgb-topic /camera/camera/color/image_raw" not in launcher
    assert "--register-rgb-to-depth" not in launcher
    assert "no RGB-to-depth mosaic can create a second board" in launcher


def test_wsj_formal_observation_replaces_non_color_calibration_preview():
    launcher = (
        OVERLAY / "start_wsj_command_observation.sh"
    ).read_text()

    assert (
        'COLOR_PREVIEW_TOPIC="/camera/camera/color/image_raw"'
        in launcher
    )
    assert 'grep -Fv -- "--rgb-topic $COLOR_PREVIEW_TOPIC"' in launcher
    assert "An untracked non-color WSJ preview is still running" in launcher
    assert "--register-rgb-to-depth" in launcher
    assert (
        'FOCUS_WSJ_REGISTRATION_MIN_COVERAGE:-0.38'
        in launcher
    )
    assert '--registration-min-coverage "$REGISTRATION_MIN_COVERAGE"' in launcher


def test_calibration_robot_entries_contain_no_live_motion_flag():
    sources = "\n".join(
        (OVERLAY / name).read_text()
        for name in (
            "start_wsj_calibration_observation.sh",
            "start_yunji_calibration_observation.sh",
        )
    )

    assert "--enable-live-go2-motion" not in sources
    assert "--enable-live-water-motion" not in sources
    assert "OPERATOR_PRESENT_AND_BOARD_ONLY" in sources


def test_yunji_calibration_recovers_only_the_readonly_odin_driver():
    launcher = (OVERLAY / "start_yunji_calibration_observation.sh").read_text()
    recovery = (OVERLAY / "prepare_yunji_odin1_calibration_driver.sh").read_text()

    assert "prepare_yunji_odin1_calibration_driver.sh" in launcher
    assert "focus-yunji-odin1-driver.service" in recovery
    assert "verify_odin1.sh" in recovery
    assert "systemctl enable" in recovery
    assert "systemctl start" in recovery
    assert "/api/move" not in recovery
    assert "/api/joy_control" not in recovery
    assert "water_cmd_vel_bridge" not in recovery


def test_incomplete_calibration_attempt_is_archived_for_retry():
    source = (SCRIPTS / "calibrate_realworld_session.sh").read_text()

    assert "Archived incomplete calibration attempt" in source
    assert "calibration_sessions/failed" in source
    assert "Refusing to replace a completed calibration directory" in source


def test_robot_launchers_require_explicit_session_identity():
    wsj = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text()
    yunji = (OVERLAY / "start_yunji_v2.sh").read_text()

    assert 'CALIBRATION_FILE="${FOCUS_SHARED_CALIBRATION_FILE:-}"' in wsj
    assert 'TRANSFORM_VERSION="${FOCUS_WSJ_TRANSFORM_VERSION:-}"' in wsj
    assert 'CALIBRATION_FILE="${FOCUS_YUNJI_SHARED_CALIBRATION_FILE:-}"' in yunji
    assert 'TRANSFORM_VERSION="${FOCUS_YUNJI_TRANSFORM_VERSION:-}"' in yunji
    assert "shared-board-odin1-20260723-v3" not in wsj + yunji


def test_operator_entry_help_is_noninteractive():
    for name in ("realworld_oneclick.sh", "calibrate_realworld_session.sh"):
        result = subprocess.run(
            ["bash", str(SCRIPTS / name), "--help"],
            capture_output=True,
            text=True,
            check=False,
        )
        assert result.returncode == 0, result.stderr
        assert "Usage" in result.stdout

    result = subprocess.run(
        [
            str(HUB / ".venv/bin/python"),
            str(HUB / "tools/record_realworld_trial.py"),
            "--help",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert "--robot-0-shortest-evidence" in result.stdout


def test_oneclick_reuses_verified_yunji_core_for_live_mode():
    source = (SCRIPTS / "realworld_oneclick.sh").read_text()

    assert "--reuse-verified-debug-core" in source
    assert "remote_queue" in source
    assert "remote_pair" in source
    assert "completion barrier" in source
    assert source.index("start_read_only_robots") < source.index("arm_live_robots")
    assert source.rindex("start_read_only_robots") < source.rindex("ensure_foxglove")


def test_live_fast_path_proves_tracking_epoch_and_preserves_warm_core():
    source = (SCRIPTS / "realworld_oneclick.sh").read_text()

    assert "probe_tracking_epoch.py" in source
    assert "FOCUS_DEBUG_PASSED_AT_NS" in source
    assert "TRACKING_EPOCH_CONTINUITY_PASSED" in source
    assert "FAST_LIVE_REUSE_READY" in source
    assert "FULL_DEBUG_RUNTIME_RECOVERY" in source
    assert "--full-preflight" in source
    assert "WARM_READONLY_CORE_PRESERVED" in source
    assert source.index("arm_live_robots") < source.index(
        "wait_for_live_readiness"
    )


def test_hub_launcher_does_not_embed_admin_token_value():
    source = (SCRIPTS / "focus_hub_up.sh").read_text()

    assert 'FOCUS_HUB_ADMIN_TOKEN=\\"\\$(cat ' in source
    assert 'admin_token="$(cat ' not in source
    assert "--print-generated-tokens" in source
    assert 'chmod 600 "$compact_tokens_file"' in source


def test_map_restart_binds_sequence_and_code_contract():
    source = (SCRIPTS / "start_fresh_dual_maps.sh").read_text()

    assert "focus-realworld-map-session-contract-v1" in source
    assert '"start_after_sequence": boundary' in source
    assert '"code_git_commit": code_commit' in source
    assert "existing map session contract mismatch" in source
