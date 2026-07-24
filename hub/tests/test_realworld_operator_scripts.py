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


def test_live_arming_happens_after_shadow_and_has_exit_disarm():
    source = (SCRIPTS / "realworld_oneclick.sh").read_text()

    assert source.index('"${shadow_args[@]}"') < source.rindex(
        "\narm_live_robots\n"
    )
    assert "trap cleanup_on_exit EXIT INT TERM" in source
    assert "restart_hub \"$FOCUS_DEBUG_ROBOT_CONFIG\" false" in source
    assert "OPERATOR_PRESENT_AND_ROBOTS_CLEAR" in source


def test_oneclick_stop_publishes_are_bounded_and_glm_can_be_adopted():
    source = (SCRIPTS / "realworld_oneclick.sh").read_text()

    assert source.count("timeout 5 ros2 topic pub --once") == 6
    assert "tmux rename-session" in source
    assert "run_glm_offline.sh" in source
    assert "GLM endpoint is live but not owned by a verified GLM tmux." in source


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
