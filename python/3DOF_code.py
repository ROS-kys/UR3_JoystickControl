from mpl_toolkits.mplot3d import Axes3D
from serial import Serial
from math import cos
from math import sin
from math import atan2
from math import sqrt
from math import pi
import time
import matplotlib.pyplot as plt

def transform(location):
    location = (location - 512.0) / 800.0
    if -0.1 <= location <= 0.1:
        location = 0
    return location        

def get_theta(x,y,z):
    D = (x**2.0 + y**2.0 + z**2.0 - l1**2 - l2**2)/(2.0*l1*l2)
    theta_3 = atan2(sqrt(1.0 - D**2.0), D)
    theta_2 = atan2(z, sqrt(x**2.0 + y**2.0)) - atan2(l2*sin(theta_3), l1+l2*cos(theta_3))
    theta_1 = atan2(y,x)

    result = [theta_1, theta_2, theta_3]
    return result    

arduino = Serial('/dev/ttyACM0', 9600)
l1 = 24.365
l2 = 21.325 + 8.535 
x = 0.0
y = 0.0
z = 0.0
x_axis = 0.0
y_axis = 0.0
z_axis = l1 + l2 - 1
synchro = time.time()
cnt = 0
fig=plt.figure(figsize=(10,10))

while True:
    if arduino.readable():
        while cnt < 10:
            arduino.readline()
            cnt += 1

        read_val = arduino.readline()
        val = read_val[:-2].decode()
        if val != '':
            x = int(val[val.find('x') + 1 : val.find('y')])
            y = int(val[val.find('y') + 1 : val.find('z')])
            z = int(val[val.find('z') + 1:])

            x = transform(x)
            y = transform(y)
            z = transform(z)

    if time.time() - synchro >= 0.1: #100ms
        x_axis += x
        y_axis += y
        z_axis += z
        synchro = time.time()
    
    if x_axis**2.0 + y_axis**2.0 + z_axis**2.0 >= (l1 + l2 - 0.1)**2.0:
        grad_1 = atan2(z_axis, sqrt(x_axis**2.0 + y_axis**2.0))
        grad_2 = atan2(y_axis, x_axis)
        x_axis = (l1 + l2 - 0.1)*cos(grad_1)*cos(grad_2)
        y_axis = (l1 + l2 - 0.1)*cos(grad_1)*sin(grad_2)
        z_axis = (l1 + l2 - 0.1)*sin(grad_1)   
        theta = get_theta(x_axis, y_axis, z_axis)
        theta_1 = theta[0]
        theta_2 = theta[1] 
        theta_3 = theta[2]

    elif x_axis**2.0 + y_axis**2.0 + z_axis**2.0 <= (l2-l1 +0.1)**2.0:
        grad_1 = atan2(z_axis, sqrt(x_axis**2.0 + y_axis**2.0))
        grad_2 = atan2(y_axis, x_axis)
        x_axis = (l2-l1 +0.1)*cos(grad_1)*cos(grad_2)
        y_axis = (l2-l1 +0.1)*cos(grad_1)*sin(grad_2)
        z_axis = (l2-l1 +0.1)*sin(grad_1)    
        theta = get_theta(x_axis, y_axis, z_axis)
        theta_1 = theta[0]
        theta_2 = theta[1] 
        theta_3 = theta[2]

    else:
        theta = get_theta(x_axis, y_axis, z_axis)
        theta_1 = theta[0]
        theta_2 = theta[1]
        theta_3 = theta[2]

    x_1 = l1*cos(theta_2)*cos(theta_1)
    x_2 = (l1*cos(theta_2) + l2*cos(theta_2+theta_3))*cos(theta_1)
    y_1 = l1*cos(theta_2)*sin(theta_1)
    y_2 = (l1*cos(theta_2) + l2*cos(theta_2+theta_3))*sin(theta_1)
    z_1 = l1*sin(theta_2)
    z_2 = l2*sin(theta_2+theta_3) + l1*sin(theta_2)

    x_grap = [0.0, 0., x_1, x_2]
    y_grap = [0.0, 0., y_1, y_2]
    z_grap = [0.0, 15.19, 15.19+z_1, 15.19+z_2]

    plt.clf()
    plt.rc('font', size=15)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_xlim(-55, 55)
    ax.set_ylim(-55, 55)
    ax.set_zlim(-70, 70)
    ax.view_init(elev=20, azim=-50)
    ax.dist=6
    ax.plot(x_grap, y_grap, z_grap, 'r', marker='o', linewidth=3.0)
    plt.draw() 
    plt.pause(0.01)


   




