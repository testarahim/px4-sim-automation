from pyulog import ULog
import matplotlib.pyplot as plt

ulog = ULog("logs/latest_log.ulg")

# Örnek: altitude
data = ulog.get_dataset('vehicle_local_position')

z = data.data['z']
t = data.data['timestamp']

plt.plot(t, z)
plt.title("Altitude over time")
plt.xlabel("Time")
plt.ylabel("Z (Down)")
plt.show()
