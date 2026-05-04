# Troubleshooting

This guide lists common PX4 SITL pipeline failures and the action to take.
Most runtime details are also written to `logs/sitl.log` and copied into each
run artifact directory under `results/runs/<run_id>/sitl.log`.

## Quick Checks

Run these first when `bash run.sh` fails:

```bash
tail -n 120 logs/sitl.log
python3 -m unittest discover -s tests
```

If PX4 is installed somewhere other than `~/PX4-Autopilot`, pass `PX4_DIR`:

```bash
PX4_DIR=/path/to/PX4-Autopilot bash run.sh
```

## `Gazebo simulation dependencies not found`

Typical log output:

```text
ERROR: Gazebo simulation dependencies not found!
Could NOT find gz-transport
Could NOT find gz-sim
Could NOT find gz-sensors
Could NOT find gz-plugin
Could NOT find Protobuf
```

Cause:

PX4 cannot find the Gazebo/Gz development packages required by the `gz_x500`
SITL target. This can happen because the packages were not installed, or
because PX4 is reusing an old CMake build cache created before the packages
were installed.

Actions:

```bash
sudo bash setup.sh
rm -rf ~/PX4-Autopilot/build/px4_sitl_default
bash run.sh
```

If the first run after deleting the build directory times out while compiling,
run it once more:

```bash
bash run.sh
```

Useful checks:

```bash
gz sim --versions
find /usr/lib /usr/share -name '*gz-sim*config.cmake' -o -name '*gz-sim*Config.cmake'
```

## `SITL readiness timed out after 60.0s`

Typical log output:

```text
TimeoutError: SITL readiness timed out after 60.0s.
Expected one of: Startup script returned successfully
```

Cause:

PX4 did not print the configured readiness pattern before the timeout. The most
common reason is the first clean PX4 build taking longer than the configured
startup timeout.

Actions:

```bash
bash run.sh
```

If the timeout repeats, inspect the SITL log:

```bash
tail -n 160 logs/sitl.log
```

For slow machines, increase `sitl.startup_timeout` in the scenario YAML file.
For example:

```yaml
sitl:
  startup_timeout: 120
```

## `No module named 'menuconfig'` or `kconfiglib is not installed`

Typical log output:

```text
ModuleNotFoundError: No module named 'menuconfig'
CMake Error at cmake/kconfig.cmake:6 (message):
kconfiglib is not installed or not in PATH
```

Cause:

PX4's Python dependency `kconfiglib` is missing from the active Python
environment.

Actions:

```bash
sudo bash setup.sh
```

Or install the Python package directly:

```bash
python3 -m pip install --user kconfiglib
rm -rf ~/PX4-Autopilot/build/px4_sitl_default
bash run.sh
```

## `sudo: a terminal is required to read the password`

Typical output:

```text
sudo: a terminal is required to read the password
sudo: a password is required
```

Cause:

`setup.sh` delegates to PX4's Ubuntu setup script, which installs system
packages with `sudo`. Non-interactive environments may not be able to provide
the password.

Action:

Run setup manually from a normal terminal:

```bash
sudo bash setup.sh
```

Then run the pipeline again:

```bash
bash run.sh
```

## `Read-only file system` under `~/PX4-Autopilot/build`

Typical log output:

```text
mkdir: cannot create directory '/home/ubuntu/PX4-Autopilot/build/px4_sitl_default': Read-only file system
```

Cause:

The command cannot write to the PX4 build directory. This can happen in a
sandboxed or restricted execution environment.

Actions:

Run the pipeline from a terminal with normal filesystem permissions:

```bash
bash run.sh
```

Also verify the PX4 directory is writable:

```bash
test -w ~/PX4-Autopilot && echo writable
```

## `Arm reddedildi, tekrar denenecek`

Typical output:

```text
Arm reddedildi, tekrar denenecek: COMMAND_DENIED
```

Cause:

PX4 was not ready to arm at that exact moment. The mission script retries arming
until `connection.arm_timeout` expires.

Action:

No action is needed if the next line is:

```text
[mission] armed
```

If arming eventually times out, inspect `logs/sitl.log` for PX4 preflight
checks and consider increasing `connection.arm_timeout` in the scenario YAML.

## Sim vs Real Comparison Is Skipped

Typical output:

```text
Sim vs Real karşılaştırması atlandı: data/real/real_log.ulg ve data/sim/sim_log.ulg gerekiyor.
```

Cause:

The default comparison input files do not exist.

Actions:

This is not a pipeline failure. Single-scenario simulation, log extraction, and
metric analysis can still pass.

To compare explicit logs:

```bash
python3 scripts/compare_logs.py \
  --config scenarios/takeoff_land_20m.yaml \
  --alignment takeoff \
  --sim results/runs/<run_id>/px4.ulg \
  --real path/to/real.ulg
```

## Recovery Sequence

When the failure is unclear, use this conservative recovery sequence:

```bash
python3 -m unittest discover -s tests
sudo bash setup.sh
rm -rf ~/PX4-Autopilot/build/px4_sitl_default
bash run.sh
bash run.sh
```

The second `bash run.sh` is only needed if the first run spends most of the
startup timeout compiling PX4.
