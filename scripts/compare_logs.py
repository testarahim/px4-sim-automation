from pyulog import ULog
import numpy as np
import os
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
import json

def load_log(log_path):
    return ULog(log_path)

def get_altitude(ulog):
    data = ulog.get_dataset('vehicle_local_position')
    
    t = data.data['timestamp'] * 1e-6  # microsecond → second
    z = -data.data['z']  # down → up
    
    return t, z
    
def get_velocity(ulog):
    data = ulog.get_dataset('vehicle_local_position')
    
    t = data.data['timestamp'] * 1e-6
    vx = data.data['vx']
    vy = data.data['vy']
    vz = data.data['vz']
    
    speed = np.sqrt(vx**2 + vy**2 + vz**2)
    
    return t, speed

def get_attitude(ulog):
    data = ulog.get_dataset('vehicle_attitude')
    
    t = data.data['timestamp'] * 1e-6

    # FIX: pyulog quaternion order is [w, x, y, z]
    # scipy's from_quat expects [x, y, z, w] — so we reorder here
    q = np.vstack((
        data.data['q[1]'],  # x
        data.data['q[2]'],  # y
        data.data['q[3]'],  # z
        data.data['q[0]']   # w
    )).T

    r = R.from_quat(q)
    euler = r.as_euler('xyz', degrees=True)

    roll  = euler[:, 0]
    pitch = euler[:, 1]
    yaw   = euler[:, 2]

    return t, roll, pitch, yaw

def compute_rmse(a, b):
    return np.sqrt(np.mean((a - b) ** 2))

def align_and_compare(sim_t, sim_z, real_t, real_z):
    # Time alignment via interpolation
    common_t_start = max(sim_t[0], real_t[0])
    common_t_end   = min(sim_t[-1], real_t[-1])

    mask = (sim_t >= common_t_start) & (sim_t <= common_t_end)

    sim_t = sim_t[mask]
    sim_z = sim_z[mask]

    f = interp1d(real_t, real_z)
    real_z_interp = f(sim_t)

    rmse = compute_rmse(sim_z, real_z_interp)
    return sim_t, sim_z, real_z_interp, rmse
    # FIX: also return (possibly trimmed) sim_t and sim_z so callers
    # always plot with the aligned time axis

def evaluate(metrics, thresholds):
    status = {}

    status["altitude"] = metrics["altitude_rmse"] < thresholds["altitude_rmse"]
    status["velocity"] = metrics["velocity_rmse"] < thresholds["velocity_rmse"]

    attitude_ok = (
        metrics["roll_rmse"]  < thresholds["attitude_rmse"] and
        metrics["pitch_rmse"] < thresholds["attitude_rmse"] and
        metrics["yaw_rmse"]   < thresholds["attitude_rmse"]
    )

    status["attitude"] = attitude_ok

    return status

def plot(sim_t, sim_data, real_data, name):
    plt.figure()
    plt.plot(sim_t, sim_data, label="Simulation")
    plt.plot(sim_t, real_data, label="Real")
    plt.legend()
    plt.title(f"{name.capitalize()} Comparison")
    plt.xlabel("Time (s)")
    plt.ylabel(name)
    plt.savefig(f"results/plots/{name}.png")
    plt.close()

def main():
    sim_ulog  = load_log("data/sim/sim_log.ulg")
    real_ulog = load_log("data/real/real_log.ulg")

    os.makedirs("results/plots", exist_ok=True)

    # ======================
    # ALTITUDE
    # ======================
    sim_t,   sim_z   = get_altitude(sim_ulog)
    real_t,  real_z  = get_altitude(real_ulog)

    # FIX: unpack 4 values now returned by align_and_compare
    sim_t_aligned, sim_z_aligned, real_z_interp, altitude_rmse = align_and_compare(
        sim_t, sim_z, real_t, real_z
    )
    plot(sim_t_aligned, sim_z_aligned, real_z_interp, "altitude")

    # ======================
    # VELOCITY
    # ======================
    sim_t_v,  sim_speed  = get_velocity(sim_ulog)
    real_t_v, real_speed = get_velocity(real_ulog)

    sim_t_v_al, sim_speed_al, real_speed_i, velocity_rmse = align_and_compare(
        sim_t_v, sim_speed, real_t_v, real_speed
    )
    plot(sim_t_v_al, sim_speed_al, real_speed_i, "velocity")

    # ======================
    # ATTITUDE
    # ======================
    sim_t_a,  sim_roll,  sim_pitch,  sim_yaw  = get_attitude(sim_ulog)
    real_t_a, real_roll, real_pitch, real_yaw = get_attitude(real_ulog)

    sim_t_r, sim_roll_al, real_roll_i,   roll_rmse  = align_and_compare(sim_t_a, sim_roll,  real_t_a, real_roll)
    sim_t_p, sim_pitch_al, real_pitch_i, pitch_rmse = align_and_compare(sim_t_a, sim_pitch, real_t_a, real_pitch)
    sim_t_y, sim_yaw_al,   real_yaw_i,   yaw_rmse   = align_and_compare(sim_t_a, sim_yaw,   real_t_a, real_yaw)

    plot(sim_t_r, sim_roll_al,  real_roll_i,  "roll")
    plot(sim_t_p, sim_pitch_al, real_pitch_i, "pitch")
    plot(sim_t_y, sim_yaw_al,   real_yaw_i,   "yaw")

    # ======================
    # METRICS
    # ======================
    metrics = {
        "altitude_rmse": float(altitude_rmse),
        "velocity_rmse": float(velocity_rmse),
        "roll_rmse":     float(roll_rmse),
        "pitch_rmse":    float(pitch_rmse),
        "yaw_rmse":      float(yaw_rmse)
    }

    thresholds = {
        "altitude_rmse": 1.5,
        "velocity_rmse": 1.0,
        "attitude_rmse": 5.0
    }

    evaluation = evaluate(metrics, thresholds)

    # FIX: metrics.json uses _pass suffix keys; align evaluate() output accordingly
    metrics["evaluation"] = {
        "altitude_pass": evaluation["altitude"],
        "velocity_pass": evaluation["velocity"],
        "attitude_pass": evaluation["attitude"],
        "overall_pass":  all(evaluation.values())
    }

    os.makedirs("results", exist_ok=True)
    with open("results/metrics.json", "w") as f:
        json.dump(metrics, f, indent=4)

    print("Altitude RMSE:", altitude_rmse)
    print("Velocity RMSE:", velocity_rmse)
    print("Roll RMSE:",     roll_rmse)
    print("Pitch RMSE:",    pitch_rmse)
    print("Yaw RMSE:",      yaw_rmse)
    print("Overall pass:",  metrics["evaluation"]["overall_pass"])

if __name__ == "__main__":
    main()