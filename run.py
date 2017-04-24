import numpy as np
import flight_cam as flight
import cv2, fcntl, multiprocessing, os, select, socket, struct, sys
import tempfile, threading, time, thread, signal, subprocess, termios

def simple_flight(drone):
    drone.takeoff()
    time.sleep(5)
    drone.setSpeed(0.2)
    time.sleep(1)
    drone.moveForward()
    time.sleep(2)
    drone.stop()
    time.sleep(1)
    drone.moveBackward()
    time.sleep(2)
    drone.stop()
    drone.land()

def drone_act(drone, in_list, com):
    if com == "z":
        drone.land()
        in_list.remove(file)
    elif com == "s":
        simple_flight(drone)
    return in_list


def print_bat(drone):
    bat = drone.getBattery()
    drone.printBlue("Battery: {}% {}\n\n".format(bat[0], bat[1]))


def drone_init(drone):
    drone.startup()
    drone.reset()
    while drone.getBattery()[0] == -1: time.sleep(0.01)
    time.sleep(0.1)
    print_bat(drone)
    drone.useDemoMode(False)
    drone.setConfigAllID()
    drone.groundCam()
    drone.midVideo()
    drone.sdVideo()
    CDC = drone.ConfigDataCount
    while CDC == drone.ConfigDataCount: time.sleep(0.0001)


# init
drone = flight.Drone()
drone_init(drone)

# main loop
stop, read_list, timer = False, [sys.stdin], time.time()
try:
    while read_list:
        # get keypress and act
        ready = select.select(read_list, [], [], 0.1)[0]
        if ready:
            for file in ready:
                read_list = drone_act(drone, read_list, file.readline()[0])
        if (time.time() - timer) >= 15:
            print_bat(drone)
            timer = time.time()

except KeyboardInterrupt as e:
    print e
    
# drone has shut down, print battery and status
print_bat(drone)

