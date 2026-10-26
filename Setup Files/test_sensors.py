# import time
# from litewing import LiteWing

# drone = LiteWing("192.168.0.12")
# drone.connect()
# time.sleep(2)
# print(f"Battery: {drone.battery:.2f}V")
# print(f"Height:  {drone.height:.3f}m")
# drone.disconnect()

import time
from litewing import LiteWing

drone = LiteWing("192.168.0.12")
drone.connect()
time.sleep(2)
print("Lift the drone by hand — 15 seconds")
for i in range(30):
    s = drone.read_sensors()
    print(f"  Filtered: {s.height:.3f}m   Raw ToF: {s.range_height:.3f}m")
    time.sleep(0.5)
drone.disconnect()
