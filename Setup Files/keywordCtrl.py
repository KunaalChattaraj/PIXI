import time
import keyboard
import cflib.crtp
from cflib.crazyflie import Crazyflie

DRONE_URI = "udp://192.168.43.42:14550"

cflib.crtp.init_drivers()
cf = Crazyflie()

cf.open_link(DRONE_URI)
time.sleep(2)

cf.commander.send_setpoint(0, 0, 0, 0)

roll, pitch, yaw = 0, 0, 0
thrust = 10000

print("Press 's' to start, 'x' to stop, 'q' to quit")

while True:
    if keyboard.is_pressed('s'):
        print("Starting motors")
        cf.commander.send_setpoint(roll, pitch, yaw, thrust)
        time.sleep(20)

    elif keyboard.is_pressed('x'):
        print("Stopping motors")
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.5)

    elif keyboard.is_pressed('q'):
        print("Exiting")
        cf.commander.send_setpoint(0, 0, 0, 0)
        break

cf.close_link()