# Runtime runbook

This file lists only the current entry points. Dated commands and superseded
procedures are retained in [`docs/archive/`](docs/archive/) for provenance.

## 1. Verify a checkout

```bash
python3 hub/tools/verify_public_baseline.py --workspace .
bash hub/scripts/verify_repository.sh --tests
```

The public-baseline check is read-only and does not install software, connect
to a robot or issue a physical command.

## 2. Reconstruct the deployment

Use the [documentation index](docs/README.md) and
[reproduction guide](docs/REPRODUCE.md). The locked Hub and Robot 0 bootstrap
commands are also shown in the main [README](README.md).

## 3. Prepare a supervised session

Follow the
[persistent one-click workflow](hub/docs/ONECLICK_SESSION_WORKFLOW.md) for
sensor recovery, preview-first calibration, fresh-map validation, supervised
execution and shutdown. Physical execution is permitted only with an onsite
operator, standing robots, a clear workspace and explicit authorization for
that run. Each robot retains final authority to reject or stop.

## 4. Archive a run

Record the episode identity, Git revision, calibration, exact trajectories,
SR/SPL inputs, result classification and media hashes immediately after the
run. Store machine-readable results under `manifests/`, dated evidence under
`audit/`, and public media provenance in `media/README.md`.
