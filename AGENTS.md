# Working rules

- Keep `source/` and `dependencies/` immutable; add deployment code in a separate `hub/` package only after the transport contract is approved.
- Do not download HM3D datasets, simulator scenes, the overlay, or SIF unless a simulator-validation gate explicitly needs them.
- Preserve provenance: record source path, size/checksum, and whether a result is observed, source-derived, or unverified.
- Treat physical robot commands as safety-critical.  The hub emits versioned, expiring high-level targets; the robot keeps final authority to stop or reject.
- Use one existing Torch SSH/tmux session for remote checks.  Keep temporary transfer plumbing loopback-only and remove it after use.
