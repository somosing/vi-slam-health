# Contributing

Open an issue before large changes. Keep the online detector independent of ground truth, add tests for state-machine changes, and document every parameter that changes benchmark behavior.

Before a pull request:

```bash
pre-commit run --all-files
colcon build --symlink-install
colcon test --event-handlers console_direct+
colcon test-result --verbose
pytest evaluation/tests
python3 -m unittest discover -s tools/benchmark/tests -v
```

Use Conventional Commit prefixes (`feat:`, `fix:`, `docs:`, `test:`, `chore:`). Benchmark changes must state the dataset split, configuration commit, and whether thresholds were tuned before or after seeing test results.
