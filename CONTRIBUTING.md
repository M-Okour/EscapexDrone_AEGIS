# Contributing

## Branches

- `main` is always buildable and runs in sim. Nobody pushes to it directly.
- Create a branch per piece of work, named `<subsystem>/<short-description>`:

  ```
  camera/usb-driver        lidar/ground-filter      flight/mavros-backend
  mission/return-home      system/systemd-service   interfaces/add-battery-topic
  ```
- Keep branches short-lived (days, not weeks). Rebase on `main` often:
  `git fetch origin && git rebase origin/main`.

## Pull requests

1. Build and test locally:
   ```bash
   colcon build --symlink-install && colcon test && colcon test-result
   source install/setup.bash && ./scripts/smoke_test.sh
   ```
2. Open a PR into `main` and fill in the template.
3. CI (build + unit tests + sim smoke test) must be green.
4. At least **one review** from another student. Changes to `exd_interfaces`,
   `exd_common` or `docs/ARCHITECTURE.md` need approval from every affected owner.

## Stay in your lane

- Edit files in **your** package freely.
- Don't edit another student's package — open an issue or ask them.
- Need a new shared topic or message? See "Changing the contract" in
  `docs/ARCHITECTURE.md`.

## Code style

- Python: PEP 8, 100-char lines. C++ is fine too (`ament_cmake` package) if you need speed.
- New nodes inherit `ExdNode` and use names from `exd_common.topics`.
- Algorithms go in plain-Python modules with unit tests; the ROS node stays thin.
- Mark unfinished work with `TODO(<subsystem>): ...` so it's searchable.
- Set yourself as `<maintainer>` in your package's `package.xml` and `setup.py`.

## Issues

Use GitHub Issues with a label per subsystem (`camera`, `lidar`, `flight`,
`mission`, `system`, `interfaces`). Link PRs to issues (`Closes #12`).
