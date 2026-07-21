# Security and robot safety

Do not commit Hub tokens, robot tokens, VNC passwords, SSH keys, camera URLs with credentials, or private model credentials. Runtime secrets belong in ignored chmod-600 files or an external secret manager.

If a credential reaches Git history, rotate it first, then remove it from every published ref. A later deletion commit is not sufficient.

Physical commands are safety-critical. The repository defaults to mapping-only, `allow_goal=false`, versioned expiring decisions, and robot-local rejection. Source reconstruction, USB setup, repository verification, camera/perception launch and BuildMap persistence do not authorize autonomous motion.

Before enabling any command receiver, require an operator-controlled HIL gate covering emergency stop, stale command expiry, network loss, localization degradation, obstacle braking, transform mismatch and Hub restart. The robot must keep final stop authority.

Report security or unsafe-motion findings privately to the repository owner before opening a public issue containing exploit details or credentials.
