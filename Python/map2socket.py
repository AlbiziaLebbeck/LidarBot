import paho.mqtt.client as mqtt
import math
import matplotlib.pyplot as plt
import numpy as np
import socket
import time

from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from PIL import Image

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 35
MIN_SAMPLES   = 200

slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS,random_seed=1)
mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

Dist = {}

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('127.0.0.1',4013))

def mm2pix(mm):
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))

def on_connect(self, client, userdata, rc):
    print("MQTT Connected.")
    self.subscribe("/ldb01/mapdata")

def on_message(client, userdata,msg):
    print("Messsage")

    sock.sendall(msg.payload)

    # data = [x for x in msg.payload]

    # for i in range(45):
    #     angle = data[4*i]*256 + data[4*i+1]
    #     angle = (angle + 180) % 360

    #     dist = data[4*i+2]*256 + data[4*i+3]

    #     if dist == 250:
    #         dist = 0

    #     Dist[angle] = dist


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect("103.20.207.171", 1883, 120)


client.loop_forever()