# Yunji WATER link preflight — 2026-07-27

## Observed failure

Three calibration attempts started
`focus-yunji-calibration-observation-v1.service`, reported it active after a
two-second check, and then lost it about four seconds later. The sender failed
at the first read-only WATER `/api/robot_info` request. The outer calibration
therefore waited the full 90-second preview deadline before exposing a failure
that was already present.

Read-only network evidence from `nyush-nuc`:

- expected NIC: `enp114s0`, MAC `48:21:0b:6e:1f:bd`;
- observed carrier: `0` (`NO-CARRIER`);
- observed IPv4 addresses: none on the Ethernet NIC;
- observed route to `192.168.10.10`: incorrect fallback through campus Wi-Fi;
- observed WATER ports `31001`, `9090`, `9001` and `8809`: all timed out;
- observed NetworkManager profile `Yunji-Robot`: manual
  `192.168.10.112/24`, MAC-bound, autoconnect priority `10`.

This is an observed physical-link failure. Software cannot create Ethernet
carrier; the cable and WATER-side port power must be restored on site.

## Implemented recovery

`ensure_yunji_water_link.sh` is now shared by calibration, standalone mapping
observation and the debug/live Yunji entry point. It:

1. resolves the expected NIC by permanent MAC;
2. rejects missing carrier immediately with a precise operator action;
3. when carrier exists, restores the existing MAC-bound `Yunji-Robot`
   NetworkManager profile only if the dedicated route is absent;
4. requires the WATER TCP API port to accept a connection;
5. issues no WATER request, move target or velocity command.

The calibration launcher no longer accepts a service merely because it
survives two seconds. It requires an actual robot-1 Hub observation sequence
advance within 30 seconds, reports service logs immediately on failure, and
stops a non-advancing observation unit.

## Verification

- shell syntax passed for all four affected launchers;
- all 500 Hub tests passed;
- `source/` and `dependencies/` were not changed;
- the Hub remained `goal_output_enabled=false` for both robots.
