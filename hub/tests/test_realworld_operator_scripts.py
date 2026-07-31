from __future__ import annotations

from pathlib import Path
import subprocess


HUB = Path(__file__).resolve().parents[1]
WORKSPACE = HUB.parent
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
    assert "retire_other_managed_map_sessions" in source
    assert 'session" == shared_maps_*' in source
    assert 'session" != "$MAP_SESSION"' in source
    assert "Retiring stale read-only map workers" in source
    assert "A retired map worker survived its managed tmux session" in source
    assert source.index("retire_other_managed_map_sessions\n") < source.index(
        "if map_window_matches"
    )
    assert "rm -r" not in source
    assert "map_out_wsj_20260724" not in source
    assert "shared-board-odin1-20260723-v3" not in source
    assert '"${map_resume_args[@]}"' in source
    assert "both maps must exist or both be absent" in source
    fresh_maps = (SCRIPTS / "start_fresh_dual_maps.sh").read_text()
    assert "--allow-ground-height-translation-for-2d" in fresh_maps
    assert "--ground-drift-min-duration-s 5.0" in fresh_maps
    wsj = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text()
    yunji = (OVERLAY / "start_yunji_v2.sh").read_text()
    assert "FOCUS_WSJ_SEMANTIC_ARRIVAL_RADIUS_M:-0.15" in wsj
    assert (
        "FOCUS_WSJ_SEMANTIC_TERMINAL_PLANNING_MARGIN_M:-0.15"
        in wsj
    )
    assert "FOCUS_YUNJI_SEMANTIC_ARRIVAL_RADIUS_M:-0.15" in yunji
    assert "FOCUS_YUNJI_LINEAR_COMMAND_FLOOR_MPS:-0.18" in yunji
    assert '--linear-command-floor-mps "$LINEAR_COMMAND_FLOOR_MPS"' in yunji
    assert "--max-linear-mps 0.20" in yunji


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
    continuity = runner.index("apply_frontier_goal_continuity(")
    source_replan = runner.index("evaluate_source_replan(")
    clearance = runner.index("apply_frontier_clearance_guard(")
    assert continuity < source_replan < clearance
    assert "NavigationFailureMemory(" in runner
    assert "CROSS_ROUND_SOURCE_STALL" in runner
    assert "wait_and_seal_terminal_evidence(" in runner
    assert "semantic_arrival_episode_complete_hold" in runner
    assert "LIVE_RECEIVERS_READY_NO_GOAL" in source
    assert "--round-input-timeout-s 45" in source
    assert 'expected_ready = robot_id in active' in source
    assert 'payload.get("ready_for_goal") is not True' in source
    assert '"GOAL_POLICY_DISABLED" not in blockers' in source
    assert "--force-hold-robot-id" in source
    assert "episode_robot_config" in source
    assert 'payload.get("health_source") != "heartbeat"' in source


def test_oneclick_stop_publishes_are_bounded_and_glm_can_be_adopted():
    source = (SCRIPTS / "realworld_oneclick.sh").read_text()
    stopper = (
        OVERLAY / "stop_wsj_live_command_path.sh"
    ).read_text()

    assert source.count("bash $WSJ_STOPPER") == 4
    assert stopper.count("timeout 5 ros2 topic pub --once") == 2
    assert "timeout 5 ros2 topic list" in stopper
    assert "/focus_guarded_cmd_vel geometry_msgs/msg/Twist '{}'" in stopper
    assert "tmux rename-session" in source
    assert "run_glm_offline.sh" in source
    assert "GLM endpoint is live but not owned by a verified GLM tmux." in source
    assert (
        "Replacing verified GLM owner with an incompatible model contract"
        in source
    )
    assert "FOCUS_GLM_MODELS_PAYLOAD" in source
    assert "deadline=$((SECONDS + 90))" in source
    assert "cogvlm2-19b-focus-score-contract-v1" in source
    assert source.count("glm_contract_ready") >= 3


def test_remote_completion_marker_always_starts_on_a_new_line():
    for name in ("realworld_oneclick.sh", "calibrate_realworld_session.sh"):
        source = (SCRIPTS / name).read_text()
        assert (
            "'set +e; bash -lc %q; rc=$?; echo; "
            "echo __%s_RC=$rc; true'"
        ) in source
        assert "'set +e; bash -lc %q; :'" in source


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
    assert "Inspect BOTH Foxglove camera previews." in source
    assert source.index("CALIBRATION_PREVIEW_READY") < source.index(
        "Inspect BOTH Foxglove camera previews."
    )
    assert source.index("Inspect BOTH Foxglove camera previews.") < source.index(
        "Capturing the initial fit from this fresh read-only sensor epoch."
    )
    assert "Capturing the initial fit from this fresh read-only sensor epoch." in source
    assert 'row.get("camera_ready") is not True' in source
    assert "--min-board-spacing-px" in source
    assert "--stationary-camera-holdout" in source
    assert "BOARD_TOO_SMALL" in source
    assert "ensure_ssh_tmux_shell" in source
    assert "tmux respawn-pane -k" in source
    assert "SSH_TMUX_PANE_RESPAWNED" in source
    assert "SSH_TMUX_SHELL_READY" in source
    assert source.index("ensure_ssh_tmux_shell") < source.index(
        "Verifying byte-identical robot release roots before calibration."
    )


def test_wsj_calibration_uses_one_native_infrared_geometry_frame():
    launcher = (OVERLAY / "start_wsj_calibration_observation.sh").read_text()

    assert "--rgb-topic /camera/camera/infra1/image_rect_raw" in launcher
    assert "--rgb-topic /camera/camera/color/image_raw" not in launcher
    assert "--register-rgb-to-depth" not in launcher
    assert "no RGB-to-depth mosaic can create a second board" in launcher


def test_wsj_calibration_retries_publisher_discovery_without_replacing_sender():
    launcher = (OVERLAY / "start_wsj_calibration_observation.sh").read_text()

    sender_pid_probe = launcher.index("calibration_sender_pid()")
    python_executable_gate = launcher.index(
        '[[ "${executable##*/}" == python* ]]',
        sender_pid_probe,
    )
    prewarm = launcher.index(
        'until [[ -n "$(calibration_sender_pid)" ]]'
    )
    sender_pid = launcher.index(
        'expected_calibration_sender_pid="$(calibration_sender_pid)"'
    )
    first_baseline = launcher.index(
        'calibration_epoch_baseline="$(latest_hub_sequence)"'
    )
    first_recovery = launcher.index("restart_calibration_publishers initial")
    retry = launcher.index(
        "restart_calibration_publishers rediscovery_retry"
    )

    assert (
        sender_pid_probe
        < python_executable_gate
        < prewarm
        < sender_pid
        < first_baseline
        < first_recovery
        < retry
    )
    assert "until pgrep -af" not in launcher
    assert (
        'current_calibration_sender_pid="$(calibration_sender_pid)"'
        in launcher
    )
    assert "WSJ calibration sender PID changed during" in launcher
    assert (
        'wait_for_calibration_sequence_advance '
        '"$calibration_epoch_baseline" 45'
        in launcher
    )
    assert "sender_pid_preserved=true" in launcher
    assert (
        "No fresh WSJ calibration observation arrived after bounded "
        "publisher rediscovery."
        in launcher
    )


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


def test_wsj_formal_observation_recovers_only_a_proven_stale_sender_reader():
    launcher = (
        OVERLAY / "start_wsj_command_observation.sh"
    ).read_text()
    calibration = (
        OVERLAY / "start_wsj_calibration_observation.sh"
    ).read_text()

    assert "wait_for_hub_sequence_advance" in launcher
    assert "FOCUS_WSJ_SENDER_ADVANCE_TIMEOUT_S:-12" in launcher
    assert "FOCUS_WSJ_SYNC_PROBE_TIMEOUT_S:-12" in launcher
    assert "--depth-topic /slam/depth" in launcher
    assert "--pose-topic /slam/odometry_visual" in launcher
    assert "--latest-rgb-for-depth" in launcher
    assert (
        'FOCUS_WSJ_LATEST_RGB_MAX_SKEW_S:-0.05'
        in launcher
    )
    assert 'FOCUS_WSJ_RGB_CACHE_SIZE:-90' in launcher
    assert '--rgb-cache-size "$RGB_CACHE_SIZE"' in launcher
    assert '--latest-rgb-max-skew-s "$LATEST_RGB_MAX_SKEW_S"' in launcher
    assert "@focus_sender_process_contract_sha256" in launcher
    assert "--runtime-command-contract-file" in launcher
    assert "--runtime-command-receipt-file" in launcher
    assert "--park-only" in launcher
    assert "sender_process_rows()" in launcher
    assert 'readlink -f "/proc/$pid/exe"' in launcher
    assert '"${executable##*/}" == python*' in launcher
    assert 'processes="$(sender_process_rows)"' in launcher
    assert "runtime_sender_pid()" in calibration
    assert 'persistent_sender_pid="$(runtime_sender_pid)"' in calibration
    assert 'current_pid="$(runtime_sender_pid)"' in calibration
    sender = (OVERLAY / "focus_ros_sender.py").read_text()
    probe = (OVERLAY / "probe_wsj_observation_sync.py").read_text()
    assert "preserve this ROS/DDS participant" in sender
    assert "self._reset_session()" in sender
    assert "Preserving the DDS participant; do not restart it" in launcher
    assert "fresh_reader_can_assemble_observation" in launcher
    assert "recover_stale_sender_reader" in launcher
    assert "WSJ_STALE_SENDER_READER_RECOVERED" in launcher
    assert "active_receipt_frames_seen" in launcher
    assert "sender_frame_error_since" in launcher
    assert (
        "Activate a Hub session with the same transform/calibration contract"
        in launcher
    )
    assert "probe_wsj_observation_sync.py" in launcher
    assert "observed_read_only_fresh_dds_reader" in probe
    assert '"robot_commands_issued": False' in probe
    probe_gate = launcher.index("if fresh_reader_can_assemble_observation")
    sender_recovery = launcher.index("recover_stale_sender_reader", probe_gate)
    publisher_instruction = launcher.index(
        "If publisher recovery is authorized", probe_gate
    )
    assert probe_gate < sender_recovery < publisher_instruction
    assert 'write_parked_contract\n  wait_for_receipt parked ""' in launcher
    assert "tracking publishers restarted: false" in launcher
    assert (
        "Refusing sender reader recovery while a command path exists"
        in launcher
    )
    assert launcher.index("go2_cmd_bridge") < launcher.index(
        'wait_for_hub_sequence_advance "$initial_sequence"'
    )
    assert "start_go2" not in launcher
    assert "water_cmd_vel_bridge" not in launcher


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
    network = (OVERLAY / "ensure_yunji_water_link.sh").read_text()

    assert "prepare_yunji_odin1_calibration_driver.sh" in launcher
    assert "ensure_yunji_water_link.sh" in launcher
    assert "initial_sequence" in launcher
    assert "latest_sequence > initial_sequence" in launcher
    assert "FOCUS_YUNJI_CALIBRATION_READY_TIMEOUT_S:-30" in launcher
    assert "focus-yunji-odin1-driver.service" in recovery
    assert "verify_odin1.sh" in recovery
    assert "systemctl enable" in recovery
    assert "systemctl start" in recovery
    assert "/api/move" not in recovery
    assert "/api/joy_control" not in recovery
    assert "water_cmd_vel_bridge" not in recovery
    assert "YUNJI_WATER_LINK_NO_CARRIER" in network
    assert "YUNJI_WATER_LINK_READY" in network
    assert "Yunji-Robot" in network
    assert "/api/move" not in network
    assert "/api/joy_control" not in network


def test_odin_verifier_retries_transient_ros_graph_discovery():
    verifier = (OVERLAY / "verify_odin1.sh").read_text()

    assert "ODIN_TOPIC_READY_TIMEOUT_S:-12" in verifier
    assert "wait_for_live_topic()" in verifier
    assert "timeout 4 ros2 topic list -t" in verifier
    assert 'timeout 5 ros2 topic echo --once "${topic}"' in verifier
    assert "while (( SECONDS < deadline ))" in verifier
    assert "ros2 daemon stop" not in verifier


def test_every_yunji_observation_entry_verifies_the_water_link():
    for name in (
        "run_yunji_mapping_observation.sh",
        "start_yunji_calibration_observation.sh",
        "start_yunji_v2.sh",
    ):
        assert "ensure_yunji_water_link.sh" in (OVERLAY / name).read_text()


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


def test_wsj_planner_and_controller_reload_wait_out_stale_dds_identity():
    source = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text()

    assert "remain-on-exit on" in source
    assert 'tmux send-keys -t "$SESSION:planning" C-c' in source
    assert 'tmux send-keys -t "$SESSION:control" C-c' in source
    assert "old WSJ planner publisher to leave DDS" in source
    assert "old WSJ controller publisher to leave DDS" in source
    assert "Node name: planning_node" in source
    assert "Node name: cmd_vel_control_node" in source
    assert "_NODE_.*_UNKNOWN_" in source
    assert "run_yunji_tinynav_planner.py" in source
    assert "--robot-profile source-default" in source
    assert "--rotate-first-on-reverse" in source
    assert "--stabilize-large-turn" in source
    assert source.index("remain-on-exit on") < source.index(
        "tmux respawn-pane -t"
    )
    assert source.index("tmux respawn-pane -t") < source.index(
        "Node name: planning_node"
    )


def test_yunji_cleanup_requires_explicit_chassis_zero_acknowledgement():
    oneclick = (SCRIPTS / "realworld_oneclick.sh").read_text()
    stop = (
        OVERLAY / "stop_yunji_live_command_path.sh"
    ).read_text(encoding="utf-8")
    bridge = (OVERLAY / "water_cmd_vel_bridge.py").read_text(
        encoding="utf-8"
    )

    assert "stop_yunji_live_command_path.sh" in oneclick
    assert "--send-explicit-zero" in stop
    assert stop.count("--send-explicit-zero") == 2
    assert "YUNJI_EXPLICIT_ZERO_CONFIRMED" in stop
    assert "yunji_[w]asd_teleop" in stop
    assert 'kill -TERM "${manual_pids[@]}"' in stop
    assert stop.index('kill -TERM "${manual_pids[@]}"') < stop.index(
        "--send-explicit-zero"
    )
    assert "focus-water-explicit-zero-v1" in bridge


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


def test_scene02_plant_operator_commands_are_explicit_and_bounded():
    source = (WORKSPACE / "command.txt").read_text()

    assert 'SESSION_ID="scene02-plant-$(date +%Y%m%d-%H%M%S)"' in source
    assert "--scene-id debug-plant" in source
    assert "--goal-category chair" not in source
    assert source.count("--episode-id scene02-plant-run") == 5
    for trial_index in range(1, 6):
        assert (
            f"--episode-id scene02-plant-run{trial_index:02d}"
            in source
        )
    assert 'ROBOT_0_SHORTEST_M=""' in source
    assert 'ROBOT_1_SHORTEST_M=""' in source
    assert "scene02-plant-wsj.json" in source
    assert "scene02-plant-yunji.json" in source


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
    assert source.count('chmod 600 "$compact_tokens_file"') == 1
    assert "curl -fsS --max-time 2" in source


def test_map_restart_binds_sequence_and_code_contract():
    source = (SCRIPTS / "start_fresh_dual_maps.sh").read_text()

    assert "focus-realworld-map-session-contract-v1" in source
    assert '"start_after_sequence": boundary' in source
    assert '"code_git_commit": code_commit' in source
    assert "existing map session contract mismatch" in source
    semantic_preflight = source.index(
        "verify_source_semantic_stack.py"
    )
    first_map_output = source.index('wsj_out="$hub_dir/runtime/map_out_wsj_')
    first_tmux = source.index("tmux new-session")
    assert semantic_preflight < first_map_output < first_tmux
    assert "semantic_preflight_timestamp" in source
    assert (
        "source_semantic_preflight_${session_tag}_"
        "${semantic_preflight_timestamp}.json"
    ) in source
    assert (
        "git -C \"$workspace\" status --porcelain "
        "--untracked-files=normal"
    ) in source


def test_oneclick_recovers_and_probes_the_existing_ssh_panes():
    source = (SCRIPTS / "realworld_oneclick.sh").read_text()

    assert "ensure_ssh_tmux_shell" in source
    assert "probe_ssh_tmux_shell" in source
    assert "FOCUS_SSH_PROBE_TIMEOUT_S:-15" in source
    assert "tmux respawn-pane -k" in source
    assert "SSH_TMUX_PANE_RESPAWNED" in source
    assert "SSH_TMUX_SHELL_READY" in source
    assert "disconnected during its probe" in source
    assert source.index("ensure_ssh_tmux_shell") < source.index(
        "Verifying that both robot release roots match this Git checkout."
    )


def test_oneclick_parallelizes_only_independent_startup_gates():
    source = (SCRIPTS / "realworld_oneclick.sh").read_text()

    assert "verify_remote_release_pair" in source
    assert "ensure_local_services_parallel" in source
    assert "recover_full_readonly_runtime_parallel" in source
    assert "parallel_full_readonly_runtime" in source
    assert "ONECLICK_TIMING" in source
    full = source.index("recover_full_readonly_runtime_parallel()")
    tracking = source.index("verify_tracking_epoch_continuity()")
    parallel_body = source[full:tracking]
    assert "ensure_glm" in parallel_body
    assert "start_read_only_robots" in parallel_body
    assert "ensure_foxglove" in parallel_body
    assert parallel_body.count("wait ") == 3
    live = source[source.index('if [[ "$mode" == live ]]; then', full) :]
    assert live.index("arm_live_robots") < live.index("wait_for_live_readiness")
    assert "FOCUS_EPOCH_NS" in source
    assert "last_observation_received_at_ns" in source
    assert "SECONDS + 25" in source


def test_calibration_parallelizes_dual_robot_transport_and_startup():
    source = (SCRIPTS / "calibrate_realworld_session.sh").read_text()

    assert "remote_begin" in source
    assert "remote_finish" in source
    assert "remote_pair" in source
    assert "verify_remote_release_pair" in source
    assert "deploy_calibration_pair" in source
    assert "dual_raw_observation_start" in source
    assert "dual_calibrated_debug_start" in source
    assert "FOCUS_DEPLOYMENT_COMMIT='$code_commit'" in source


def test_remote_timeouts_interrupt_before_fail_closed_cleanup():
    for script in ("calibrate_realworld_session.sh", "realworld_oneclick.sh"):
        source = (SCRIPTS / script).read_text()
        timeout = source.index("Remote command timed out on")
        interrupt = source.index('tmux send-keys -t "$target" C-c', timeout)
        marker = source.index("REMOTE_TIMEOUT_INTERRUPTED", interrupt)
        reconnect = source.index("tmux respawn-pane -k", marker)
        assert timeout < interrupt < marker < reconnect


def test_runtime_processes_are_bound_to_the_checked_deployment_commit():
    oneclick = (SCRIPTS / "realworld_oneclick.sh").read_text()
    calibration = (SCRIPTS / "calibrate_realworld_session.sh").read_text()
    wsj = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text()
    wsj_sender = (OVERLAY / "start_wsj_command_observation.sh").read_text()
    yunji = (OVERLAY / "start_yunji_v2.sh").read_text()

    assert "One-click requires committed runtime code" in oneclick
    assert oneclick.count("FOCUS_DEPLOYMENT_COMMIT=%q") == 2
    assert "FOCUS_DEPLOYMENT_COMMIT='$code_commit'" in calibration
    assert "FOCUS_DEPLOYMENT_COMMIT must be the explicit" in wsj
    assert "@focus_deployment_commit" in wsj_sender
    assert "@focus_sender_process_contract_sha256" in wsj_sender
    assert "legacy_process_contract_sha256" in wsj_sender
    assert "SENDER_PROCESS_DEPLOYMENT_COMMIT" in wsj_sender
    assert '"session_deployment_commit"' in wsj_sender
    assert "--runtime-command-contract-file" in wsj_sender
    assert "write_active_contract" in wsj_sender
    assert "validated_contract_applied_without_dds_restart" in (
        OVERLAY / "focus_ros_sender.py"
    ).read_text()
    assert '--setenv="FOCUS_DEPLOYMENT_COMMIT=$DEPLOYMENT_COMMIT"' in yunji
    assert "FOCUS_YUNJI_CORE_CONTRACT_SHA256" in yunji
    assert "unit_matches_core_contract" in yunji
    assert "Verified Yunji debug core has a different process contract" in yunji


def test_warm_live_reuse_avoids_restarting_verified_navigation_core():
    oneclick = (SCRIPTS / "realworld_oneclick.sh").read_text()
    wsj = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text()
    yunji = (OVERLAY / "start_yunji_v2.sh").read_text()

    assert oneclick.count("--reuse-verified-debug-core") >= 2
    assert "@focus_component_contract_sha256" in wsj
    assert "component_contract_matches" in wsj
    assert (
        "Reusing verified WSJ planner/controller without DDS participant churn"
        in wsj
    )
    assert "unit_matches_core_contract" in yunji
    assert (
        "Reusing the verified Yunji "
        "perception/planning/router/controller core without process restarts"
        in yunji
    )


def test_wsj_sender_is_parked_before_any_publisher_recovery():
    launcher = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text()

    park = launcher.index("--park-only")
    recovery = launcher.index("recover_online_map_publisher()")
    activation = launcher.index(
        'bash "$SCRIPT_DIR/start_wsj_command_observation.sh" \\\n'
        '  --session "$SESSION"',
        park,
    )
    assert park < recovery < activation


def test_wsj_observation_paths_pin_verified_udp_transport():
    sender = (OVERLAY / "start_wsj_command_observation.sh").read_text()
    launcher = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text()
    recovery = (
        OVERLAY / "recover_wsj_publishers_after_sender.sh"
    ).read_text()
    calibration = (
        OVERLAY / "start_wsj_calibration_observation.sh"
    ).read_text()

    for script in (sender, launcher, recovery, calibration):
        assert "FOCUS_WSJ_FASTDDS_BUILTIN_TRANSPORTS:-UDPv4" in script
        assert "== UDPv4" in script
    assert (
        'env "FASTDDS_BUILTIN_TRANSPORTS=$FASTDDS_BUILTIN_TRANSPORTS_VALUE"'
        in sender
    )
    assert "@focus_fastrtps_builtin_transports" in sender
    assert "publisher recovery must follow before use" not in sender
    assert "WSJ_DDS_UDP_PARTICIPANT_PARKED" in sender
    assert (
        'export FASTDDS_BUILTIN_TRANSPORTS="$FASTDDS_BUILTIN_TRANSPORTS_VALUE"'
        in launcher
    )


def test_dual_robot_occupancy_liveness_timeout_is_consistent():
    wsj = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text()
    yunji = (OVERLAY / "start_yunji_v2.sh").read_text()

    assert 'FOCUS_WSJ_RECEIVER_OCCUPANCY_TIMEOUT_S:-5.0' in wsj
    assert 'FOCUS_YUNJI_RECEIVER_OCCUPANCY_TIMEOUT_S:-5.0' in yunji
    assert (
        'FOCUS_WSJ_RECEIVER_OCCUPANCY_RECOVERY_GRACE_S:-7.0'
        in wsj
    )
    assert (
        'FOCUS_YUNJI_RECEIVER_OCCUPANCY_RECOVERY_GRACE_S:-7.0'
        in yunji
    )
    assert (
        '--occupancy-recovery-grace-s '
        '"$RECEIVER_OCCUPANCY_RECOVERY_GRACE_S"'
        in wsj
    )
    assert (
        '--occupancy-recovery-grace-s '
        '"$RECEIVER_OCCUPANCY_RECOVERY_GRACE_S"'
        in yunji
    )
    for launcher in (wsj, yunji):
        assert 'FOCUS_MAX_CACHED_MAP_MOTION_M:-0.25' in launcher
        assert (
            '--max-cached-occupancy-motion-m '
            '"$MAX_CACHED_MAP_MOTION_M"'
            in launcher
        )


def test_goal_category_reaches_both_persistent_observation_senders():
    oneclick = (SCRIPTS / "realworld_oneclick.sh").read_text()
    calibration = (SCRIPTS / "calibrate_realworld_session.sh").read_text()
    wsj = (OVERLAY / "start_wsj_command_observation.sh").read_text()
    yunji = (OVERLAY / "start_yunji_v2.sh").read_text()
    yunji_sender = (
        OVERLAY / "run_yunji_mapping_observation.sh"
    ).read_text()

    assert "FOCUS_WSJ_GOAL_CATEGORY=%q" in oneclick
    assert "FOCUS_YUNJI_GOAL_CATEGORY=%q" in oneclick
    assert "FOCUS_WSJ_GOAL_CATEGORY='$goal_category'" in calibration
    assert "FOCUS_YUNJI_GOAL_CATEGORY='$goal_category'" in calibration
    assert 'GOAL_CATEGORY="${FOCUS_WSJ_GOAL_CATEGORY:-chair}"' in wsj
    assert 'GOAL_CATEGORY="${FOCUS_YUNJI_GOAL_CATEGORY:-chair}"' in yunji
    assert '--goal-category "$GOAL_CATEGORY"' in yunji
    assert "unit_matches_sender_contract" in yunji
    assert "FOCUS_YUNJI_SENDER_CONTRACT_SHA256" in yunji
    assert '--goal-category "$goal_category"' in yunji_sender


def test_robot_sender_liveness_is_bounded_without_duplicate_wsj_probes():
    oneclick = (SCRIPTS / "realworld_oneclick.sh").read_text()
    wsj = (OVERLAY / "start_wsj_buildmap_v2.sh").read_text()
    yunji = (OVERLAY / "start_yunji_v2.sh").read_text()

    read_only_start = oneclick[
        oneclick.index("start_read_only_robots()") :
        oneclick.index("ensure_local_services_parallel()")
    ]
    assert "tinynav_semantic_nav_auto:hub-sender" not in read_only_start
    assert "bash $WSJ_STOPPER" in read_only_start
    assert "calibration-sender" in (
        OVERLAY / "stop_wsj_live_command_path.sh"
    ).read_text()
    assert "fresh_topic_once" not in wsj
    assert "timeout -k 2 15 ros2 topic echo" not in wsj
    assert "--fresh-camera-info-topic /camera/camera/color/camera_info" in wsj
    assert "--fresh-camera-info-topic /slam/camera_info" in wsj
    assert "--odom-topic /slam/odometry_visual" in wsj
    assert "FOCUS_YUNJI_SENDER_ADVANCE_TIMEOUT_S:-10" in yunji
    assert "ensure_yunji_sender_advance" in yunji
    assert "restarting only the read-only sender once" in yunji
    assert "failed to advance after one bounded read-only restart" in yunji
    assert yunji.index('ensure_yunji_sender_advance "$sender_baseline"') < (
        yunji.index('wait "$sender_watchdog_pid"')
    )


def test_wsj_publisher_recovery_preserves_sender_and_requires_reanchor():
    recovery = (
        OVERLAY / "recover_wsj_publishers_after_sender.sh"
    ).read_text()

    assert "OPERATOR_PRESENT_AND_WSJ_STATIONARY" in recovery
    assert "--perception-only" in recovery
    assert 'if [[ "$PERCEPTION_ONLY" != true ]]; then' in recovery
    assert '"camera_preserved": perception_only' in recovery
    assert "--runtime-command-contract-file" in recovery
    sender_gate = recovery.index(
        "Persistent runtime-configurable WSJ sender is not running."
    )
    stop_perception = recovery.index(
        'tmux set-option -w -t "$SESSION:perception" remain-on-exit on'
    )
    restart_camera = recovery.index(
        'tmux respawn-pane -k -t "$SESSION:camera"'
    )
    restart_perception = recovery.index(
        'tmux respawn-pane -k -t "$SESSION:perception"'
    )
    marker_started = recovery.index(
        "write_reanchor_marker recovery_started"
    )
    marker_completed = recovery.index(
        "write_reanchor_marker publishers_recovered"
    )
    assert sender_gate < stop_perception < restart_camera < restart_perception
    assert marker_started < stop_perception
    assert restart_perception < marker_completed
    assert "tmux kill-window -t \"$SESSION:hub-sender\"" not in recovery
    assert "sender_pid_preserved" in recovery
    assert "robot_commands_issued" in recovery
    assert "publisher_order_complete" in recovery
    assert "sender_process_deployment_commit" in recovery
    assert "runtime_sender_pids()" in recovery
    assert 'executable##*/}" == python*' in recovery
    assert 'tr \'\\0\' \' \' <"/proc/$pid/cmdline"' in recovery
    assert "Expected exactly one persistent runtime-configurable WSJ sender." in (
        recovery
    )
    assert recovery.index(
        'parked_tuple_baseline="$(parked_tuple_count)"'
    ) < restart_camera
    assert restart_perception < recovery.index(
        'wait_for_sender_tuple_advance "$parked_tuple_baseline"'
    )
    assert "WSJ DDS sender PID changed during publisher recovery" in recovery
    assert "--capture-stationary-reanchor" in recovery
    assert (
        "WSJ_STATIONARY_REANCHOR_SUBSCRIBER_PREWARMED" in recovery
    )
    capture_start = recovery.index(
        "start_stationary_reanchor_capture"
    )
    marker_started = recovery.index(
        "write_reanchor_marker recovery_started"
    )
    assert capture_start < marker_started < stop_perception
    assert "focus-wsj-stationary-reanchor-capture-v1" in recovery
    assert "STATIONARY_REANCHOR_PRE_RANGE" in recovery
    assert "STATIONARY_REANCHOR_POST_RANGE" in recovery
    assert '"camera_preserved": True' in recovery


def test_wsj_mapping_only_launcher_has_stationary_reanchor_mode():
    launcher = (
        OVERLAY / "start_wsj_calibration_observation.sh"
    ).read_text()

    assert "--stationary-reanchor" in launcher
    assert "OPERATOR_PRESENT_AND_WSJ_STATIONARY" in launcher
    assert 'OBSERVATION_PURPOSE="stationary_reanchor"' in launcher
    assert "validated_stationary_reanchor_or_new_board_calibration_required" in (
        launcher
    )
    assert "WSJ stationary re-anchor evidence ready" in launcher
    assert "no planner, receiver or Go2 bridge is running" in launcher
