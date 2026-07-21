# Contributing

Keep `source/` and `dependencies/` immutable. New deployment code belongs under `hub/`, and any physical-command path must preserve the versioned, expiring, fail-closed contract.

Work on a branch, keep commits scoped, add tests for behavior changes, and run:

```bash
bash hub/scripts/verify_repository.sh --tests
```

Pull requests must state what was observed, what was derived from source, what remains unverified, and whether robot motion is possible. Never attach raw bags, maps, model weights, tokens or machine virtual environments.
