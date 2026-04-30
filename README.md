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

### Metrics:

- Altitude RMSE

### Output:

- Comparison plots
- Numerical error metrics (JSON)

## 🛠 Requirements

- Ubuntu 20.04+
- PX4 Autopilot
- Python 3.8+

## ⚙️ Setup

```bash
git clone https://github.com/testarahim/px4-sim-automation.git
cd px4-sim-automation
bash setup.sh
