from pyulog import ULog
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.spatial.transform import Rotation as R
import json

def load_altitude(log_path):
    ulog = ULog(log_path)
    data = ulog.get_dataset('vehicle_local_position')
    
    t = data.data['timestamp'] * 1e-6  # microsecond → second
    z = -data.data['z']  # down → up
    
    return t, z
    
def load_velocity(log_path):
    ulog = ULog(log_path)
    data = ulog.get_dataset('vehicle_local_position')
    
    t = data.data['timestamp'] * 1e-6
    vx = data.data['vx']
    vy = data.data['vy']
    vz = data.data['vz']
    
    speed = np.sqrt(vx**2 + vy**2 + vz**2)
    
    return t, speed

def load_attitude(log_path):
    ulog = ULog(log_path)
    data = ulog.get_dataset('vehicle_attitude')
    
    t = data.data['timestamp'] * 1e-6
    q = np.vstack((
        data.data['q[0]'],
        data.data['q[1]'],
        data.data['q[2]'],
        data.data['q[3]']
    )).T

    r = R.from_quat(q)
    euler = r.as_euler('xyz', degrees=True)

    roll = euler[:, 0]
    pitch = euler[:, 1]
    yaw = euler[:, 2]

    return t, roll, pitch, yaw

def compute_rmse(a, b):
    return np.sqrt(np.mean((a - b) ** 2))

def align_and_compare(sim_t, sim_z, real_t, real_z):
    # zaman hizalama (interpolation)
    f = interp1d(real_t, real_z, fill_value="extrapolate")
    real_z_interp = f(sim_t)

    rmse = compute_rmse(sim_z, real_z_interp)
    return real_z_interp, rmse

def plot(sim_t, sim_z, real_z_interp):
    plt.figure()
    plt.plot(sim_t, sim_z, label="Simulation")
    plt.plot(sim_t, real_z_interp, label="Real")
    plt.legend()
    plt.title("Altitude Comparison")
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.savefig("results/plots/altitude.png")
    plt.close()

def main():
    sim_t, sim_z = load_altitude("data/sim/sim_log.ulg")
    real_t, real_z = load_altitude("data/real/real_log.ulg")
    
    sim_t_v, sim_speed = load_velocity("data/sim/sim_log.ulg")
    real_t_v, real_speed = load_velocity("data/real/real_log.ulg")
    
    sim_t_a, sim_roll, sim_pitch, sim_yaw = load_attitude("data/sim/sim_log.ulg")
    real_t_a, real_roll, real_pitch, real_yaw = load_attitude("data/real/real_log.ulg")

    real_interp, rmse = align_and_compare(sim_t, sim_z, real_t, real_z)
    
    real_speed_interp, vel_rmse = align_and_compare(
    	sim_t_v, sim_speed,
    	real_t_v, real_speed
    )
    
    real_roll_i, roll_rmse = align_and_compare(sim_t_a, sim_roll, real_t_a, real_roll)
    real_pitch_i, pitch_rmse = align_and_compare(sim_t_a, sim_pitch, real_t_a, real_pitch)
    real_yaw_i, yaw_rmse = align_and_compare(sim_t_a, sim_yaw, real_t_a, real_yaw)
    plot(sim_t, sim_z, real_interp)
    plot(sim_t_v, sim_speed, real_speed_interp)
    plot(sim_t, sim_z, real_interp)    

    metrics = {
        "altitude_rmse": float(rmse)
        "velocity_rmse": float(vel_rmse)
        "roll_rmse": float(roll_rmse)
        "pitch_rmse": float(pitch_rmse)
        "yaw_rmse": float(yaw_rmse)
    }

    with open("results/metrics.json", "w") as f:
        json.dump(metrics, f, indent=4)

    print(f"RMSE: {rmse:.3f} m")

if __name__ == "__main__":
    main()
