import serial
from hokuyo.driver import hokuyo
from hokuyo.tools import serial_port
import numpy as np
import cv2

uart_port = '/dev/ttyACM0'
uart_speed = 19200
Factor=23.33

if __name__ == '__main__':
    laser_serial = serial.Serial(port=uart_port, baudrate=uart_speed, timeout=0.5)
    port = serial_port.SerialPort(laser_serial)

    laser = hokuyo.Hokuyo(port)
    laser.laser_on()
    #print(laser.get_sensor_specs())
    while(True):
        image=np.zeros((480,640))
        image2=np.zeros((480,640))
        image[240,320]=200
        angles, distances, timestamp=laser.get_scan()
        angles=np.array(angles)
        distances=np.array(distances)
        for i in range(len(distances)):
            #print distances[i]
            x=distances[i]*np.sin(angles[i]*np.pi/180)/Factor
            y=distances[i]*np.cos(angles[i]*np.pi/180)/Factor
            x1=distances[i]*np.sin(angles[i])/Factor
            y1=distances[i]*np.cos(angles[i])/Factor
            r=np.sqrt(x**2+y**2)
            r1=np.sqrt(x1**2+y1**2)
            if((abs(r)<=239 and abs(r)>=10) and (abs(r)<=239 and abs(r)>=10)):
                #print [x,y]
                image[240-y,320+x]=255
            if((abs(r1)<=239) and (abs(r1)<=239)):
                #print [x1,y1]
                image2[240-y1,320+x1]=255
            
        k = cv2.waitKey(1)
        print(k)
        if k==115:
            Factor=Factor/1.5
        if k==119:
            Factor=Factor*1.5
        print(Factor)
        cv2.imshow('Hokuyo',image)
        cv2.imshow('Arte Abstracto',image2)
        if k==27:
            break

    laser.laser_off()
