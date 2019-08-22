import paho.mqtt.client as mqtt
import math
import matplotlib.pyplot as plt
import scipy.io as sio
import numpy as np
import time

Dist = {}
scans = []


def on_connect(self, client, userdata, rc):
    print("MQTT Connected.")
    self.subscribe("/ldb01/mapdata")

def on_message(client, userdata,msg):
    # print("Messsage")

    data = [x for x in msg.payload]

    for i in range(45):
        angle = data[4*i]*256 + data[4*i+1]
        angle_rad = (-3.14159*angle/180 - 0.13)%6.28318


        dist = data[4*i+2]*256 + data[4*i+3]

        if dist == 250:
            dist = 0

        # print("angle:",angle_rad,"distance",dist)
        Dist[angle_rad] = dist

        x = dist*math.cos(angle_rad)
        y = dist*math.sin(angle_rad)


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.loop_start()
client.connect("103.20.207.171", 1883, 120)

startTime = time.time()

nextTime = startTime + 3



while time.time() - startTime <= 60:
    if (time.time() - nextTime) >= 0.5:
        nextTime = time.time()
        print("record" + str(int(time.time() - startTime)))
        cell = {}

        cell["Ranges"] = []
        cell["Angles"] = []
        cell["Catesian"] = []
        cell["Count"] = 45

        for d in Dist:
            x = Dist[d]*math.cos(d)
            y = Dist[d]*math.sin(d)
            
            cell["Ranges"].append([float(Dist[d])/10000])
            cell["Angles"].append([d])
            cell["Catesian"].append([x,y])

        scans.append(cell)
client.loop_stop()

sio.savemat('mapdata.mat', {'data':scans})

print(len(Dist))

# plt.figure()
# for d in Dist:
#     print(d,Dist[d])

#     if Dist[d] != 250:
#         x = Dist[d]*math.cos(d)
#         y = Dist[d]*math.sin(d)
#         plt.plot(x,y,'r.')

# plt.show()