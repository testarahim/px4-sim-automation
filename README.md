# PX4 Simulation Automation Pipeline

This project provides an automated PX4 SITL simulation pipeline for:

- Running simulations
- Executing missions
- Extracting logs
- Analyzing flight performance

## 🚀 Features

- One-command simulation execution
- MAVSDK-based mission control
- Automatic PX4 log extraction
- Post-flight data analysis with Python
- Repeatable test workflow

## 📈 Simulation vs Real Comparison

This project includes a comparison tool that evaluates differences between simulation and real flight logs.

### Metrics

- Altitude RMSE
- Velocity RMSE
- Attitude RMSE (Roll / Pitch / Yaw)

### Output

- Comparison plots (`results/plots/`)
- Numerical error metrics (`results/metrics.json`)

## 🧪 Regression Testing

This project supports automated regression testing:

- Multi-metric comparison (altitude, velocity, attitude)
- Threshold-based pass/fail evaluation
- Batch simulation comparison

Run batch tests:

```bash
python3 scripts/batch_compare.py
```

## 🛠 Requirements

- Ubuntu 20.04+
- PX4 Autopilot
- Python 3.8+

## ⚙️ Setup

```bash
git clone https://github.com/testarahim/px4-sim-automation.git
cd px4-sim-automation
bash setup.sh
```