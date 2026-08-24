from openrdk import CommsRuntime
from openrdk import Motors
from odometry import Odometry
from pid import PID
from CommandDriver import LatestCommandDriver
from line_functions import (
    set_robot_context,
    both_drivers_stop,
    is_clear_intersection,
    is_left_90_candidate,
    is_right_90_candidate,
    is_gap,
    is_green,
    is_red,
    is_obstacle,
    detect_color_marking,
    handle_left_candidate,
    handle_right_candidate,
    handle_color_marking,
    handle_intersection,
    handle_lost_line,
    handle_obstacle,
    try_cross_gap,
    update_odometry_motors,
    follow_line,
)
import time

BASE_SPEED = 20.0
last_position = 0.0

runtime = CommsRuntime(
    auto_start=True,
    enable_webview=True,
    enable_webview_updates=True,
)

motor_r = runtime.traction("98:3D:AE:43:50:50")
motor_l = runtime.traction("10:20:BA:AA:E7:28")

line_sensor = runtime.line_sensor("10:20:BA:AC:F4:B0")

distance_sensor = runtime.distance_sensor("7C:4F:AD:79:B0:44")

color_sensor_r = runtime.color_sensor("7C:4F:AD:79:94:B0")
color_sensor_l = runtime.color_sensor("24:EC:4A:CB:05:90")

motors = Motors(right=motor_r, left=motor_l)

driver_r = LatestCommandDriver(motor_r)
driver_l = LatestCommandDriver(motor_l)

odometry = Odometry()

pid = PID()

set_robot_context(driver_r, driver_l, motors, odometry)

try:
    while True:
        reading = line_sensor.get_data()
        digital = reading["digital"]
        update_odometry_motors()
        color_r = color_sensor_r.get_color()
        color_l = color_sensor_l.get_color()

        if is_green(color_r) or is_green(color_l):
            print("Green detected", flush=True)
            color_marking = detect_color_marking(color_sensor_r, color_sensor_l)
        else:
            color_marking = None

        if reading["line_detected"]:
            last_position = reading["position"]

        if is_obstacle(distance_sensor):
            handle_obstacle(line_sensor)
            print("-------------------------------------------------\n", flush=True)
            continue

        if handle_color_marking(color_marking):
            print("-------------------------------------------------\n", flush=True)
            continue

        if is_clear_intersection(digital):
            print("Intersection detected", flush=True)

            color_marking = detect_color_marking(color_sensor_r, color_sensor_l)

            if handle_color_marking(color_marking):
                print("Color marking handled after intersection", flush=True)
                print("-------------------------------------------------\n", flush=True)
                continue

            handle_intersection()
            print("Handling intersection", flush=True)
            print("-------------------------------------------------\n", flush=True)
            continue

        if is_left_90_candidate(digital):
            print("Left 90 candidate detected", flush=True)

            color_marking = detect_color_marking(color_sensor_r, color_sensor_l)

            if handle_color_marking(color_marking):
                print("Color marking handled after intersection", flush=True)
                print("-------------------------------------------------\n", flush=True)
                continue

            handle_left_candidate(line_sensor)
            print("-------------------------------------------------\n", flush=True)
            continue

        if is_right_90_candidate(digital):
            print("Right 90 candidate detected", flush=True)

            color_marking = detect_color_marking(color_sensor_r, color_sensor_l)

            if handle_color_marking(color_marking):
                print("Color marking handled after intersection", flush=True)
                print("-------------------------------------------------\n", flush=True)
                continue

            handle_right_candidate(line_sensor)
            print("-------------------------------------------------\n", flush=True)
            continue

        if is_gap(reading):
            gap_found = try_cross_gap(line_sensor)

            if not gap_found:
                handle_lost_line(line_sensor, last_position)

            continue

        if is_red(color_r) and is_red(color_l):
            print("Red detected, stopping...", flush=True)
            both_drivers_stop()
            break

        follow_line(reading, pid, BASE_SPEED)
        print("-------------------------------------------------\n", flush=True)

except KeyboardInterrupt:
    print("Stopping...", flush=True)
finally:
    both_drivers_stop()
