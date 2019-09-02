import scipy.io as sio
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from PIL import Image
import matplotlib.pyplot as plt
import numpy as np

MAP_SIZE_PIXELS         = 500
MAP_SIZE_METERS         = 15
MIN_SAMPLES   = 200

slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)

mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

mapdata = sio.loadmat('mapdata.mat')
scans = []
trajectory = []

for d in mapdata['data'][0]:

    ranges = [r*1000 for r in d[0,0]['Ranges'].reshape(360)]
    angles = [180*rad/3.14159 for rad in d[0,0]['Angles'].reshape(360)]

    scans.append([ranges,angles])

for i in np.arange(0,len(scans),1):

    # Extract distances and angles from triples
    distances = scans[i][0]
    angles    = scans[i][1]

    # Update SLAM with current Lidar scan and scan angles if adequate
    slam.update(distances, scan_angles_degrees=angles)

    # Get current robot position
    x_mm, y_mm, theta_degrees = slam.getpos()  
    trajectory.append((x_mm, y_mm))

def mm2pix(mm):
    return int(mm / (MAP_SIZE_METERS * 1000. / MAP_SIZE_PIXELS))

slam.getmap(mapbytes)
for coords in trajectory:
    x_mm, y_mm = coords
    x_pix = mm2pix(x_mm)
    y_pix = mm2pix(y_mm)

    mapbytes[y_pix * MAP_SIZE_PIXELS + x_pix] = 0

image = Image.frombuffer('L', (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), mapbytes, 'raw', 'L', 0, 1)
# image.save('img_map.png')
plt.figure()
plt.imshow(image)
plt.show()