import scipy.io as sio
import matplotlib.pyplot as plt
import numpy as np
import ekf_slam


mapdata = sio.loadmat('mapdata.mat')
scans = []
trajectory = []

for d in mapdata['data'][0]:

    j = 0
    z = np.zeros((0, 3))
    for a in range(180):
        i = a*2
        if d[0,0]['Ranges'][i,0] != 0 and d[0,0]['Ranges'][i,0] != 0.05:
            zi = np.array([100*d[0,0]['Ranges'][i,0],d[0,0]['Angles'][i,0],j])
            z = np.vstack((z, zi))
            j = j + 1

    scans.append(z)


STATE_SIZE = 3  # State size [x,y,yaw]
show_animation = True

xEst = np.zeros((STATE_SIZE, 1)) 
PEst = np.eye(STATE_SIZE)

# # history
hxEst = xEst

for i in np.arange(0,100):
    print("Scan",i)

    # Extract distances and angles from triples
    z = scans[i]
 
    ud = np.array([[0,0]]).T

    # Update SLAM with current Lidar scan and scan angles if adequate
    xEst, PEst = ekf_slam.ekf_slam(xEst, PEst, ud, z)

    x_state = xEst[0:STATE_SIZE]

    # store data history
    hxEst = np.hstack((hxEst, x_state))

    if show_animation:  # pragma: no cover
        plt.cla()

        plt.plot(xEst[0], xEst[1], ".r")

        # plot landmark
        for i in range(ekf_slam.calc_n_LM(xEst)):
            plt.plot(xEst[STATE_SIZE + i * 2],
                        xEst[STATE_SIZE + i * 2 + 1], "xg")

        plt.plot(hxEst[0, :],
                    hxEst[1, :], "-r")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.001)
