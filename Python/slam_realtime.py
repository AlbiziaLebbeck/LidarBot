import paho.mqtt.client as mqtt
import math
import matplotlib.pyplot as plt
import numpy as np
import time

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from PIL import Image

MAP_SIZE_PIXELS         = 300
MAP_SIZE_METERS         = 50
MIN_SAMPLES   = 200

slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS,random_seed=1)
mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

Dist = {}

def mm2pix(mm):
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))

def on_connect(self, client, userdata, rc):
    print("MQTT Connected.")
    self.subscribe("/ldb01/mapdata")

def on_message(client, userdata,msg):
    # print("Messsage")

    data = [x for x in msg.payload]

    for i in range(45):
        angle = data[4*i]*256 + data[4*i+1]
        angle = (angle + 180) % 360

        dist = data[4*i+2]*256 + data[4*i+3]

        if dist == 250:
            dist = 0

        Dist[angle] = dist


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.loop_start()
client.connect("103.20.207.171", 1883, 120)

startTime = time.time()

nextTime = startTime + 1
showTime = startTime + 1

trajectory = []
plt.figure()

while time.time() - startTime <= 120000006:
    if (time.time() - nextTime) >= 0.1:
        nextTime = time.time()

        angles = list(Dist.keys())
        distances = list(Dist.values())

        slam.update(distances, scan_angles_degrees=angles)

        x_mm, y_mm, theta_degrees = slam.getpos()  
        trajectory.append((x_mm, y_mm))

        slam.getmap(mapbytes)

    if (time.time() - showTime) >= 1:
        showTime = time.time()

        for coords in trajectory:
            x_mm, y_mm = coords
            x_pix = mm2pix(x_mm)
            y_pix = mm2pix(y_mm)

            mapbytes[y_pix * MAP_SIZE_PIXELS + x_pix] = 0

        image = Image.frombuffer('L', (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), mapbytes, 'raw', 'L', 0, 1)
        plt.imshow(image)
        plt.pause(0.01)


client.loop_stop()