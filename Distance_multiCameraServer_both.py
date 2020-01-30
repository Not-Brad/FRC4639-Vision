#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------

import json
import time
import sys

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink
from networktables import NetworkTablesInstance
import ntcore
import cv2
import numpy


from reflective_tape_new import GripPipelineGreen
from yellow_ball_test import GripPipelineYellow


#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#       "switched cameras": [
#           {
#               "name": <virtual camera name>
#               "key": <network table key used for selection>
#               // if NT value is a string, it's treated as a name
#               // if NT value is a double, it's treated as an integer index
#           }
#       ]
#   }

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.getInstance().addSwitchedCamera(config.name)

    def listener(fromobj, key, value, isNew):
        if isinstance(value, float):
            i = int(value)
            if i >= 0 and i < len(cameras):
              server.setSource(cameras[i])
        elif isinstance(value, str):
            for i in range(len(cameraConfigs)):
                if value == cameraConfigs[i].name:
                    server.setSource(cameras[i])
                    break

    NetworkTablesInstance.getDefault().getEntry(config.key).addListener(
        listener,
        ntcore.constants.NT_NOTIFY_IMMEDIATE |
        ntcore.constants.NT_NOTIFY_NEW |
        ntcore.constants.NT_NOTIFY_UPDATE)

    return server

#Distance function
def distance_to_camera(Width, focalLength, perceivedWidth):
	return (Width * focalLength) / perceivedWidth

def getValuesGreen(image):

    contourPoints = contours_output_green[0][:,0]

    x_points_green = contourPoints[:,0]
    y_points_green = contourPoints[:,1]

    x_min_green = numpy.amin(x_points_green)
    x_max_green = numpy.amax(x_points_green)
    #for distance
    Green_Width = x_max_green - x_min_green
    #initialized distance and width (for focal)
    Control_Distance = 70
    Control_Width = 400
    focalLength = (Green_Width * Control_Distance) / Control_Width
        #call distance function to return widths
    inchesG = distance_to_camera(Control_Width, focalLength, Green_Width)
    sd.putNumber('Distance to Green', inchesG)


    y_min_green = numpy.amin(y_points_green)
    y_max_green = numpy.amax(y_points_green)

    sd.putNumber('Min X Green', x_min_green)
    sd.putNumber('Max X Green', x_max_green)
    sd.putNumber('Min Y Green', y_min_green)
    sd.putNumber('Max Y Green', y_max_green)

    area_green = (x_max_green - x_min_green) * (y_max_green - y_min_green)

    sd.putNumber('Green Area', area_green)

    x_center_green = ((x_max_green - x_min_green)/2) + x_min_green
    y_center_green = ((y_max_green - y_min_green)/2) + y_min_green

    sd.putNumber('Center X Green', x_center_green)
    sd.putNumber('Center Y Green', y_center_green)


    image = cv2.line(image, ((x_center_green).astype(numpy.int64),((y_center_green) - 15).astype(numpy.int64)),((x_center_green).astype(numpy.int64),((y_center_green) + 15).astype(numpy.int64)),(255,0,255),5)
    image = cv2.line(image, (((x_center_green) - 15).astype(numpy.int64),(y_center_green).astype(numpy.int64)),(((x_center_green) + 15).astype(numpy.int64),(y_center_green).astype(numpy.int64)),(255,0,255),5)

        #Should display green distance?
    box = cv2.cv.BoxPoints(Green_Width) if imutils.is_cv2() else cv2.boxPoints(Green_Width)
	box = np.int0(box)
	cv2.drawContours(image, [box], -1, (0, 255, 0), 2)
	cv2.putText(image, "%.2fft" % (inches / 12),(image.shape[1] - 200, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 3)
	cv2.imshow("image", image)

    #OUTLINE CONTOUR
    image = cv2.line(image, ((x_max_green).astype(numpy.int64),((y_max_green)).astype(numpy.int64)),((x_max_green).astype(numpy.int64),((y_min_green)).astype(numpy.int64)),(255,0,255),5)
    image = cv2.line(image, (((x_min_green)).astype(numpy.int64),(y_max_green).astype(numpy.int64)),(((x_min_green)).astype(numpy.int64),(y_min_green).astype(numpy.int64)),(255,0,255),5)
    image = cv2.line(image, ((x_max_green).astype(numpy.int64),((y_max_green)).astype(numpy.int64)),((x_min_green).astype(numpy.int64),((y_max_green)).astype(numpy.int64)),(255,0,255),5)
    image = cv2.line(image, (((x_max_green)).astype(numpy.int64),(y_min_green).astype(numpy.int64)),(((x_min_green)).astype(numpy.int64),(y_min_green).astype(numpy.int64)),(255,0,255),5)

    return image

def getValuesYellow(image):

    contourPoints = contours_output_yellow[0][:,0]

    x_points_yellow = contourPoints[:,0]
    y_points_yellow = contourPoints[:,1]

    x_min_yellow = numpy.amin(x_points_yellow)
    x_max_yellow = numpy.amax(x_points_yellow)

    #for distance
    Yellow_Width = x_max_yellow - x_min_yellow
    #initialized distance and width (for focal)
    Control_Distance = 70
    Control_Width = 400
    focalLength = (Yellow_Width * Control_Distance) / Control_Width
    #call distance function to return widths
    inchesY = distance_to_camera(Control_Width, focalLength, Yellow_Width)

    sd.putNumber('Distance to Yellow', inchesY)

    y_min_yellow = numpy.amin(y_points_yellow)
    y_max_yellow = numpy.amax(y_points_yellow)

    sd.putNumber('Min X Yellow', x_min_yellow)
    sd.putNumber('Max X Yellow', x_max_yellow)
    sd.putNumber('Min Y Yellow', y_min_yellow)
    sd.putNumber('Max Y Yellow', y_max_yellow)

    area_yellow = (x_max_yellow - x_min_yellow) * (y_max_yellow - y_min_yellow)

    sd.putNumber('Yellow Area', area_yellow)

    x_center_yellow = ((x_max_yellow - x_min_yellow)/2) + x_min_yellow
    y_center_yellow = ((y_max_yellow - y_min_yellow)/2) + y_min_yellow

    sd.putNumber('Center X Yellow', x_center_yellow)
    sd.putNumber('Center Y Yellow', y_center_yellow)

    image = cv2.line(image, ((x_center_yellow).astype(numpy.int64),((y_center_yellow) - 15).astype(numpy.int64)),((x_center_yellow).astype(numpy.int64),((y_center_yellow) + 15).astype(numpy.int64)),(0,0,0),5)
    image = cv2.line(image, (((x_center_yellow) - 15).astype(numpy.int64),(y_center_yellow).astype(numpy.int64)),(((x_center_yellow) + 15).astype(numpy.int64),(y_center_yellow).astype(numpy.int64)),(0,0,0),5)

    image = cv2.line(image, ((x_max_yellow).astype(numpy.int64),((y_max_yellow)).astype(numpy.int64)),((x_max_yellow).astype(numpy.int64),((y_min_yellow)).astype(numpy.int64)),(0,0,0),5)
    image = cv2.line(image, (((x_min_yellow)).astype(numpy.int64),(y_max_yellow).astype(numpy.int64)),(((x_min_yellow)).astype(numpy.int64),(y_min_yellow).astype(numpy.int64)),(0,0,0),5)
    image = cv2.line(image, ((x_max_yellow).astype(numpy.int64),((y_max_yellow)).astype(numpy.int64)),((x_min_yellow).astype(numpy.int64),((y_max_yellow)).astype(numpy.int64)),(0,0,0),5)
    image = cv2.line(image, (((x_max_yellow)).astype(numpy.int64),(y_min_yellow).astype(numpy.int64)),(((x_min_yellow)).astype(numpy.int64),(y_min_yellow).astype(numpy.int64)),(0,0,0),5)
        #Should display yellow distance?
    box = cv2.cv.BoxPoints(Yellow_Width) if imutils.is_cv2() else cv2.boxPoints(Yellow_Width)
    box = np.int0(box)    	cv2.drawContours(image, [box], -1, (0, 255, 0), 2)
	cv2.putText(image, "%.2fft" % (inches / 12),(image.shape[1] - 200, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 3)
    cv2.imshow("image", image)


    return image

def getValuesBoth(image):

    contourPoints_green = contours_output_green[0][:,0]

    x_points_green = contourPoints_green[:,0]
    y_points_green = contourPoints_green[:,1]

    x_min_green = numpy.amin(x_points_green)
    x_max_green = numpy.amax(x_points_green)

    y_min_green = numpy.amin(y_points_green)
    y_max_green = numpy.amax(y_points_green)
    #for distance
    Green_Width = x_max_green - x_min_green
    #initialized distance and width (for focal)
    Control_Distance = 70
    Control_Width = 400
    focalLength = (Green_Width * Control_Distance) / Control_Width
    #call distance function to return widths
    inchesG = distance_to_camera(Control_Width, focalLength, Green_Width)

    sd.putNumber('Min X Green', x_min_green)
    sd.putNumber('Max X Green', x_max_green)
    sd.putNumber('Min Y Green', y_min_green)
    sd.putNumber('Max Y Green', y_max_green)

    sd.putNumber('Distance to Green', inchesG)


    area_green = (x_max_green - x_min_green) * (y_max_green - y_min_green)

    sd.putNumber('Green Area', area_green)

    x_center_green = ((x_max_green - x_min_green)/2) + x_min_green
    y_center_green = ((y_max_green - y_min_green)/2) + y_min_green

    sd.putNumber('Center X Green', x_center_green)
    sd.putNumber('Center Y Green', y_center_green)

    image = cv2.line(image, ((x_center_green).astype(numpy.int64),((y_center_green) - 15).astype(numpy.int64)),((x_center_green).astype(numpy.int64),((y_center_green) + 15).astype(numpy.int64)),(255,0,255),5)
    image = cv2.line(image, (((x_center_green) - 15).astype(numpy.int64),(y_center_green).astype(numpy.int64)),(((x_center_green) + 15).astype(numpy.int64),(y_center_green).astype(numpy.int64)),(255,0,255),5)

    image = cv2.line(image, ((x_max_green).astype(numpy.int64),((y_max_green)).astype(numpy.int64)),((x_max_green).astype(numpy.int64),((y_min_green)).astype(numpy.int64)),(255,0,255),5)
    image = cv2.line(image, (((x_min_green)).astype(numpy.int64),(y_max_green).astype(numpy.int64)),(((x_min_green)).astype(numpy.int64),(y_min_green).astype(numpy.int64)),(255,0,255),5)
    image = cv2.line(image, ((x_max_green).astype(numpy.int64),((y_max_green)).astype(numpy.int64)),((x_min_green).astype(numpy.int64),((y_max_green)).astype(numpy.int64)),(255,0,255),5)
    image = cv2.line(image, (((x_max_green)).astype(numpy.int64),(y_min_green).astype(numpy.int64)),(((x_min_green)).astype(numpy.int64),(y_min_green).astype(numpy.int64)),(255,0,255),5)
    #Should display green distance?
    box = cv2.cv.BoxPoints(Green_Width) if imutils.is_cv2() else cv2.boxPoints(Green_Width)
	box = np.int0(box)
	cv2.drawContours(image, [box], -1, (0, 255, 0), 2)
	cv2.putText(image, "%.2fft" % (inches / 12),(image.shape[1] - 200, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 3)
	cv2.imshow("image", image)


    contourPoints_yellow = contours_output_yellow[0][:,0]

    x_points_yellow = contourPoints_yellow[:,0]
    y_points_yellow = contourPoints_yellow[:,1]

    x_min_yellow = numpy.amin(x_points_yellow)
    x_max_yellow = numpy.amax(x_points_yellow)

    #for distance
    Yellow_Width = x_max_yellow - x_min_yellow
    #call distance function to return widths
    inchesY = distance_to_camera(Control_Width, focalLength, Yellow_Width)

    y_min_yellow = numpy.amin(y_points_yellow)
    y_max_yellow = numpy.amax(y_points_yellow)

    sd.putNumber('Min X Yellow', x_min_yellow)
    sd.putNumber('Max X Yellow', x_max_yellow)
    sd.putNumber('Min Y Yellow', y_min_yellow)
    sd.putNumber('Max Y Yellow', y_max_yellow)

    sd.putNumber('Distance to Yellow', inchesY)


    area_yellow = (x_max_yellow - x_min_yellow) * (y_max_yellow - y_min_yellow)

    sd.putNumber('Yellow Area', area_yellow)

    x_center_yellow = ((x_max_yellow - x_min_yellow)/2) + x_min_yellow
    y_center_yellow = ((y_max_yellow - y_min_yellow)/2) + y_min_yellow

    sd.putNumber('Center X Yellow', x_center_yellow)
    sd.putNumber('Center Y Yellow', y_center_yellow)



    image = cv2.line(image, ((x_center_yellow).astype(numpy.int64),((y_center_yellow) - 15).astype(numpy.int64)),((x_center_yellow).astype(numpy.int64),((y_center_yellow) + 15).astype(numpy.int64)),(0,0,0),5)
    image = cv2.line(image, (((x_center_yellow) - 15).astype(numpy.int64),(y_center_yellow).astype(numpy.int64)),(((x_center_yellow) + 15).astype(numpy.int64),(y_center_yellow).astype(numpy.int64)),(0,0,0),5)

    image = cv2.line(image, ((x_max_yellow).astype(numpy.int64),((y_max_yellow)).astype(numpy.int64)),((x_max_yellow).astype(numpy.int64),((y_min_yellow)).astype(numpy.int64)),(0,0,0),5)
    image = cv2.line(image, (((x_min_yellow)).astype(numpy.int64),(y_max_yellow).astype(numpy.int64)),(((x_min_yellow)).astype(numpy.int64),(y_min_yellow).astype(numpy.int64)),(0,0,0),5)
    image = cv2.line(image, ((x_max_yellow).astype(numpy.int64),((y_max_yellow)).astype(numpy.int64)),((x_min_yellow).astype(numpy.int64),((y_max_yellow)).astype(numpy.int64)),(0,0,0),5)
    image = cv2.line(image, (((x_max_yellow)).astype(numpy.int64),(y_min_yellow).astype(numpy.int64)),(((x_min_yellow)).astype(numpy.int64),(y_min_yellow).astype(numpy.int64)),(0,0,0),5)
    #Should display yellow distance?
    box = cv2.cv.BoxPoints(Yellow_Width) if imutils.is_cv2() else cv2.boxPoints(Yellow_Width)
	box = np.int0(box)
	cv2.drawContours(image, [box], -1, (0, 255, 0), 2)
	cv2.putText(image, "%.2fft" % (inches / 12),(image.shape[1] - 200, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 3)
	cv2.imshow("image", image)

    return image



if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

    # read configuration
    if not readConfig():
        sys.exit(1)

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClientTeam(team)

    # start cameras
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)

    #Init Grip
    grip_green = GripPipelineGreen()
    grip_yellow = GripPipelineYellow()
    sink = CvSink("vision")
    sink.setSource(cameras[0]) #Was 1, trying 0
    image = numpy.ndarray((640,480,3), dtype = numpy.uint8) #Mid val was 360

    camservInst = CameraServer.getInstance()
    dashSource = camservInst.putVideo("UI Cam", 640, 480)

    # loop forever

    sd = ntinst.getTable('SmartDashboard')

    while True:

        #Change the time


        timestamp,image = sink.grabFrame(image)
        grip_green.process(image)
        grip_yellow.process(image)

        contours_output_green = grip_green.filter_contours_output
        contours_output_yellow = grip_yellow.filter_contours_output

        #print(contours_output_green)

        if (contours_output_green and contours_output_yellow):

            image = getValuesBoth(image)


        elif(contours_output_green):

            image = getValuesGreen(image)

            x_min_yellow = -1
            x_max_yellow = -1

            y_min_yellow = -1
            y_max_yellow = -1

            sd.putNumber('Min X Yellow', x_min_yellow)
            sd.putNumber('Max X Yellow', x_max_yellow)
            sd.putNumber('Min Y Yellow', y_min_yellow)
            sd.putNumber('Max Y Yellow', y_max_yellow)

            print("Bruh, Yellow objects not detected")

            x_center_yellow = -1
            y_center_yellow = -1

            sd.putNumber('Center X Yellow', x_center_yellow)
            sd.putNumber('Center Y Yellow', y_center_yellow)

            area_yellow = -1

            sd.putNumber('Yellow Area', area_yellow)

            #only get green distance
            inchesY = -1
            sd.putNumber('Distance to Yellow', inchesY)

        elif(contours_output_yellow):

            image = getValuesYellow(image)

            x_min_green = -1
            x_max_green = -1

            y_min_green = -1
            y_max_green = -1

            sd.putNumber('Min X Green', x_min_green)
            sd.putNumber('Max X Green', x_max_green)
            sd.putNumber('Min Y Green', y_min_green)
            sd.putNumber('Max Y Green', y_max_green)

            print("Bruh, Green objects not detected")

            x_center_green = -1
            y_center_green = -1

            sd.putNumber('Center X Green', x_center_green)
            sd.putNumber('Center Y Green', y_center_green)

            area_green = -1

            sd.putNumber('Green Area', area_green)

            #only get yellow distance
            inchesG = -1
            sd.putNumber('Distance to Green', inchesG)
        else:

            x_min_green = -1
            x_max_green = -1

            y_min_green = -1
            y_max_green = -1

            sd.putNumber('Min X Green', x_min_green)
            sd.putNumber('Max X Green', x_max_green)
            sd.putNumber('Min Y Green', y_min_green)
            sd.putNumber('Max Y Green', y_max_green)

            print("Bruh, objects not detected")

            x_center_green = -1
            y_center_green = -1

            sd.putNumber('Center X Green', x_center_green)
            sd.putNumber('Center Y Green', y_center_green)

            area_green = -1

            sd.putNumber('Green Area', area_green)

            #only get yellow distance
            inchesG = -1
            sd.putNumber('Distance to Green', inchesG)

            x_min_yellow = -1
            x_max_yellow = -1

            y_min_yellow = -1
            y_max_yellow = -1

            sd.putNumber('Min X Yellow', x_min_yellow)
            sd.putNumber('Max X Yellow', x_max_yellow)
            sd.putNumber('Min Y Yellow', y_min_yellow)
            sd.putNumber('Max Y Yellow', y_max_yellow)


            x_center_yellow = -1
            y_center_yellow = -1

            sd.putNumber('Center X Yellow', x_center_yellow)
            sd.putNumber('Center Y Yellow', y_center_yellow)

            area_yellow = -1

            sd.putNumber('Yellow Area', area_yellow)

            #only get green distance
            inchesY = -1
            sd.putNumber('Distance to Yellow', inchesY)

        #END OF IF STATEMENTS/SEND DATA
        dashSource.putFrame(image)
