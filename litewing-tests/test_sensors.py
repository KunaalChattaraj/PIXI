import time
from litewing import LiteWing

drone = LiteWing("192.168.0.12")
drone.connect()
time.sleep(2)
print(f"Battery: {drone.battery:.2f}V")
print(f"Height:  {drone.height:.3f}m")
drone.disconnect()
