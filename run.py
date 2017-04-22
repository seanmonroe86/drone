import numpy as np
import flight_cam as flight
import cv2, fcntl, multiprocessing, os, select, socket, struct, sys
import tempfile, threading, time, thread, signal, subprocess, termios


def getGPS(drone):
    big = drone.ConfigData
    lat, lon, alt = float(big[129][1]), float(big[130][1]), float(big[131][1])
    return lat, lon, alt


def drone_act(drone, in_list, com):
    if com == "z": in_list.remove(file)
    return in_list


def print_bat(drone):
    bat = drone.getBattery()
    drone.printBlue("Battery: {}% {}".format(bat[0], bat[1]))


def drone_init(drone):
    drone.startup()
    drone.reset()
    while drone.getBattery()[0] == -1: time.sleep(0.1)
    time.sleep(0.5)
    print_bat(drone)
    drone.useDemoMode(False)
    drone.setConfigAllID()
    drone.hdVideo(False)
    drone.frontCam(False)
    CDC = drone.ConfigDataCount
    while CDC == drone.ConfigDataCount: time.sleep(0.0001)
    drone.startVideo()


# init
drone = flight.Drone()
drone_init(drone)
init_lat, init_lon, init_alt = getGPS(drone)

# main loop
stop, IMC, read_list, bat_time = False, drone.VideoImageCount, [sys.stdin], time.time()
while read_list:
    while drone.VideoImageCount == IMC: time.sleep(0.1)
    IMC = drone.VideoImageCount
    lat, lon, alt = getGPS(drone)

    # video image
    img = drone.VideoImage # 640x360
    img = img[:, 185:455]
    img_out = cv2.resize(img, (120, 160))

    # show video
    cv2.imshow("video", img_out)
    cv2.waitKey(1)

    # get keypress and act
    ready = select.select(read_list, [], [], 0.1)[0]
    if ready:
        for file in ready:
            read_list = drone_act(drone, read_list, file.readline()[0])
    if (time.time() - bat_time) >= 15:
        print_bat(drone)
        bat_time = time.time()


# drone has shut down, print battery and status
print_bat(drone)

