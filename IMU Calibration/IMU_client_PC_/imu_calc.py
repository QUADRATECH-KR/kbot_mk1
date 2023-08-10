from cmath import acos, asin, atan, cos, pi, sqrt
from ctypes import sizeof
from math import atan2
from operator import countOf, indexOf
from pickletools import uint8
from sre_constants import NEGATE
from xml.etree.ElementTree import PI
import client as rx_
import imufusion
from threading import Thread
from time import sleep
import re
import numpy

import gl

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import serial

import time

##REF  https://github.com/kriswiner/MPU9250/issues/164, https://github.com/xioTechnologies/Fusion



port_ = 'COM12'
baud_ = 250000

rx_.IP = IP   = '192.168.1.36'  #Raspberry Pi IP

#ser = serial.Serial(port_, baud_, timeout=0.01, write_timeout = 0.01)#, parity=serial.PARITY_EVEN, rtscts=1)


#pattern to find all numbers in string
#pattern = r"\d+"      #pattern for int
#pattern = r"\d*\.\d+"  #pattern for float

rx_.exit_sig = 0
gl.exit_sig  = 0

thread_ = Thread(target = rx_.receive_loop, args = ())
thread_.start()

thread_1 = Thread(target = gl.gl_draw_, args = ())
thread_1.start()


#Variables For Magnetometer / Gyroscope Calibration
mag_history_X = []
mag_history_Y = []
mag_history_Z = []

acc_count_calib = 0
acc_range       = 8000

accel_history_X = []
accel_history_Y = []
accel_history_Z = []
gyro_history_X  = []
gyro_history_Y  = []
gyro_history_Z  = []
time_           = []

accel_ned = [0.0, 0.0, 0.0]
gyro_ned  = [0.0, 0.0, 0.0]

gyro_acc  = [0.0, 0.0, 0.0]
accel_acc = [0.0, 0.0, 0.0]
acc_count = 0
dw        = [1, 1, 1]

sample_time_ = 0.0083
sample_rate_ = int(1 / sample_time_)


mat_ = numpy.zeros((3,3))


def kill_():
    global thread_, thread_1, rx_

    rx_.exit_sig = 1
    gl.exit_sig  = 1
    thread_.join()
    #thread_1.join()
    rx_.close()
    exit()



def mag_plot_():
    global mag_history_X, mag_history_Y, mag_history_Z
    fig, ax = plt.subplots(1, 1)
    ax.set_aspect(1)

    #for _ in range(30):
    #    ret = get_imu_data()
    #    if not ret:
    #        continue
    #    x, y, z = ret[6:9]
    #    mag_x.append(x)
    #    mag_y.append(y)
    #    mag_z.append(z)

    # Clear all axis
    ax.cla()

    # Display the sub-plots
    ax.scatter(mag_history_X, mag_history_Y, color='r', alpha=0.7)
    ax.scatter(mag_history_Y, mag_history_Z, color='g', alpha=0.7)
    ax.scatter(mag_history_Z, mag_history_X, color='b', alpha=0.7)

    ax.title.set_text("MAGNETOMETER")
    ax.set(ylabel="Tesla")

    plt.grid(color = 'gray', linestyle = '--', linewidth = 0.5)

    plt.show()

def gyro_plot_():
    global gyro_history_X, gyro_history_Y, gyro_history_Z, time_
    fig, ax = plt.subplots(1, 1)

    #ax.set_aspect(2)

    # Clear all axis
    ax.cla()

    # Display the sub-plots
    ax.plot(time_, gyro_history_X, color='r', alpha=0.7)
    ax.plot(time_, gyro_history_Y, color='g', alpha=0.7)
    ax.plot(time_, gyro_history_Z, color='b', alpha=0.7)

    ax.title.set_text("GYROSCOPE")
    ax.set(ylabel="rad/s")

    plt.grid(color = 'gray', linestyle = '--', linewidth = 0.5)

    plt.xlim([min(time_), max(time_)])

    plt.show()

"""
def serial_write(data):
    global ser
    index_ = ["x", "y", "z", "w"]
    data_ = ""

    try:
        #dummy_read = ser.read(ser.in_waiting)
        #ser.reset_input_buffer()
        #ser.reset_output_buffer()

        for val in data:
            data_ += index_[indexOf(data, val)] + " : " + str(val) + " "
        
        data_ += "\n"
        
        #data_ = "x : " + str(data[0]) + " , y : " + str(data[1]) + " , z : " + str(data[2]) 
        #if(data[3]):
        #    data_ += " , w : " + str(data[3]) + "\n"
        #else:
        #    data_ += "\n"

        data_ = bytes(data_, 'utf-8')
        ser.write(data_)
        


    except Exception as e:
        #print(str(e))
        pass
"""


def calibration_(accel_, gyro_, mag_):
    global acc_count_calib, time_, gyro_acc, acc_range, sample_time_
    mag_bias  = [0.0, 0.0, 0.0] 
    gyro_bias = [0.0, 0.0, 0.0]

    ### FOR MAGNETOMETER CALIBRATION
    ##Rough MAG BIAS
    #-0.30 -1.09 
    #mag_[0] +=  0.07051 #0.1051
    #mag_[1] += -0.5217
    #mag_[2] +=  0.0256

    mag_history_X.append(mag_[0])
    mag_history_Y.append(mag_[1])
    mag_history_Z.append(mag_[2])

    acc_count_calib += 1
    #print("COUNT : ", acc_count)
    if(acc_count_calib % 100 == 0):
        print("COUNT : ", acc_count_calib)

    if(acc_count_calib == acc_range):
        mag_plot_()
        print((max(mag_history_X) + min(mag_history_X)) * -1 / 2 , (max(mag_history_Y) + min(mag_history_Y)) * -1 / 2, (max(mag_history_Z) + min(mag_history_Z)) * -1 / 2)
        #print(mag_history_X, mag_history_Y, mag_history_Z)
        #exit()
    ### CALIBRATION END

    ### FOR GYROSCOPE CALIBRATION
    #Rough GYRO BIAS
    #gyro_bias[0] =  0.81
    #gyro_bias[1] =  0.60
    #gyro_bias[2] = -0.32

    gyro_history_X.append(gyro_[0])
    gyro_history_Y.append(gyro_[1])
    gyro_history_Z.append(gyro_[2])

    #acc_count += 1
    #print("COUNT : ", acc_count)

    time_.append(sample_time_ * acc_count_calib)
    #print(time_)

    if(acc_count_calib == acc_range):
        gyro_plot_()
        print((max(gyro_history_X) + min(gyro_history_X)) * -1 / 2 , (max(gyro_history_Y) + min(gyro_history_Y)) * -1 / 2, (max(gyro_history_Z) + min(gyro_history_Z)) * -1 / 2)
        kill_()
        #exit()
    ### CALIBRATION END

    return 0


offset = imufusion.Offset(sample_rate_)
ahrs = imufusion.Ahrs()


ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,  # convention
                                   5.5,  # gain
                                   0.5,  # acceleration rejection
                                   1,  # magnetic rejection
                                   int(0.0 * sample_rate_))  # rejection timeout = 5 seconds

x_dps = 0
yaw_  = 0

def imu_fusion(accel_raw, gyro_raw, mag_raw):
    global acc_count, time_, gyro_acc, gyro_acc_count, dw, ahrs, accel_ned, gyro_ned, sample_time_, x_dps, yaw_, mat_

    mag_bias  = [-0.1297760009765625, -0.343597412109375, 0.4165191650390625]  #[x, y, z] Magnetometer calibration bias
    #gyro_bias = [0.81, 0.60, -0.32]  #[x, y, z] Gyroscope calibration bias
    gyro_bias = [0.0718248, -0.013848437668412037, 0.024833977164988898]  #[x, y, z] Gyroscope calibration bias

    accel_range = 2.0   # Measurement Scale in +/- 2 g
    gyro_range  = 250   # Measurement Scale in +/- 250 deg/s
    mag_range   = 8.1   #1.3   # Measurement Scale in +/- Gauss

    accel_ = [((x * accel_range * 9.80665) / (32768)) for x in accel_raw]                 # Measurement in m/s^2
    #accel_ = [((x * accel_range) / (32768)) for x in accel_raw]                          # Measurement in g

    #gyro_  = [((x * gyro_range) / (32768))  for x in gyro_raw]                           # Measurement in deg/s
    gyro_  = [((x * gyro_range * pi) / (32768 * 180))  for x in gyro_raw]   # Measurement in rad/s

    #mag_   = [((x  * mag_range) /  32768) for x in mag_raw]                              # Measurement in Gauss
    mag_   = [((x * mag_range * 10)  / (32768)) for x in mag_raw]                         # Measurement in uT (micro Tesla)




    ## CALIBRATION OFFSETS
    mag_[0]  += mag_bias[0]
    mag_[1]  += mag_bias[1]
    mag_[2]  += mag_bias[2]

    gyro_[0] += gyro_bias[0]
    gyro_[1] += gyro_bias[1]
    gyro_[2] += gyro_bias[2]
    ##


    ## CALIBRATION FUNCTION Uncomment to calculate Calibration
    calibration_(accel_, gyro_, mag_)
    ##


    #test_madgwick(accel_, gyro_, mag_)

    #accel_norm = sqrt((accel_[0] * accel_[0]) + (accel_[1] * accel_[1]) + (accel_[2]) * accel_[2])
    #accel_[0] /= accel_norm
    #accel_[1] /= accel_norm
    #accel_[2] /= accel_norm

    #print("RAW (A G M) : ", accel_raw, gyro_raw, mag_raw)
    #print("ACCEL : ", [round(x, 4) for x in accel_])
    #print("GYRO  : ", [round(x, 4) for x in gyro_])
    #print("MAG   : ", [round(x, 4) for x in mag_])

    #Data In NED Format If X-Axis Points North
    #Ay, Ax, -Az    Gy, Gx, -Gz    Mx, My, Mz
    accel_ned[0] = accel_[1] #accel_[1]
    accel_ned[1] = accel_[0] #accel_[0]
    accel_ned[2] = -1 * accel_[2] #-1 * accel_[2]

    gyro_ned[0] = gyro_[1]   #gyro_[1]
    gyro_ned[1] = gyro_[0]   #-1 * gyro_[0]
    gyro_ned[2] = -1 * gyro_[2]   #-1 * gyro_[2]


    #accel_ned[0] = accel_[1]
    #accel_ned[1] = accel_[0]
    #accel_ned[2] = -1 * accel_[2]

    #gyro_ned[0] = gyro_[1]
    #gyro_ned[1] = -1 * gyro_[0]
    #gyro_ned[2] = -1 * gyro_[2]


    #ahrs.update_no_magnetometer(numpy.array(gyro_), numpy.array(accel_), 0.006)             #(gyro_, accel_, sample_rate)
    ahrs.update(numpy.array(gyro_ned), numpy.array(accel_ned), numpy.array(mag_), sample_time_)#0.006)#104)          #(gyro, accel, mag, dt)

    quat  = ahrs.quaternion
    euler = quat.to_euler()
    #mat_  = quat.to_matrix() 

    #print(mat_)

    #print("QUAT  : ",)
    #print("EULER : ", euler)
    
    ax_z = numpy.real(sqrt(1 - (quat.w * quat.w)))

    #gl.X_AXIS = euler[1]
    #gl.Y_AXIS = euler[2]
    #gl.Z_AXIS = euler[0]

    #gl.X_AXIS = euler[0]
    #gl.Y_AXIS = euler[1]
    #gl.Z_AXIS = euler[2]

    #sc_ = numpy.real(2 * cos(quat.w))
    sc_ = numpy.real(sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z ))

    #gl.X_AXIS = (quat.x * 180 )#/ sc_) * 180 #numpy.real(quat.x / (asin(sc_ / 2))) * 180
    #gl.Y_AXIS = (quat.y * 180 )#/ sc_) * 180 #numpy.real(quat.z / (asin(sc_ / 2))) * 180
    #gl.Z_AXIS = (quat.z * 180 )#/ sc_) * 180 #numpy.real(quat.y / (asin(sc_ / 2))) * 180
    #gl.W_     = numpy.real(acos(quat.w)) * 2 #numpy.real(quat.w / (asin(sc_ / 2))) * 180

    #mahrs_ = mahrs.MadgwickAHRS()
    #mahrs_.update(numpy.array(gyro_), numpy.array(accel_), numpy.array(mag_))
    
    #Quaternion to euler
    #angle = 2 * acos(mahrs_.quaternion[1])
    #angle = 2 * acos(0.7071)
    #s = sqrt(1 - mahrs_.quaternion[0])
    #euler_x = round(numpy.real(mahrs_.quaternion[1] / s), 8)
    #euler_y = round(numpy.real(mahrs_.quaternion[2] / s), 8)
    #euler_z = round(numpy.real(mahrs_.quaternion[3] / s), 8)

    #gl.X_AXIS = euler_x
    #gl.Y_AXIS = euler_y
    #gl.Z_AXIS = euler_z

    #gyro_acc_count += 1

    #if(gyro_acc_count > 10):
    #   dw = [ ( (((round(x, 0) / 0.02 ) != 0) * 1) ) for x in gyro_]

    #gyro_acc = [((((round(x, 0)) * 0.02 * dw[indexOf(gyro_, x)]) + gyro_acc[indexOf(gyro_, x)])) for x in gyro_]
    #gyro_acc = [(x * accel_[indexOf(gyro_acc, x)]) for x in gyro_acc]

    #gl.X_AXIS = #gyro_acc[1] #- euler[1] #* numpy.real(accel_[0]) 
    #gl.Y_AXIS = #gyro_acc[2] #- euler[2] #* numpy.real(accel_[1]) 
    #gl.Z_AXIS = #gyro_acc[0] #- euler[0] #* numpy.real(accel_[2]) 
   
    #print("[x y z w]", mahrs_.quaternion[1], mahrs_.quaternion[2], mahrs_.quaternion[3], mahrs_.quaternion[0])
    #print("EULER : ", (euler_x * angle), (euler_y * angle), (euler_z * angle))
    #serial_write(euler)

    sinr_cosp = numpy.real(2 * (quat.w * quat.x + quat.y * quat.z))
    #cosr_cosp = numpy.real(1 - 2 * (quat.x * quat.x + quat.y * quat.y))
    cosr_cosp = numpy.real(quat.w * quat.w - quat.x * quat.x - quat.y * quat.y + quat.z * quat.z)
    roll      = atan2(sinr_cosp, cosr_cosp)

    
    #sinp  = numpy.real(sqrt(1 + 2 * (quat.w * quat.y - quat.x * quat.z)))
    #cosp  = numpy.real(sqrt(1 - 2 * (quat.w * quat.y - quat.x * quat.z)))
    p_  = 2 * (quat.x * quat.z - quat.w * quat.y)
    #pitch = 2 * atan2(sinp, cosp) - (pi / 2)
    pitch = numpy.real(-asin(p_))

    siny_cosp = numpy.real(2 * (quat.w * quat.z + quat.x * quat.y))
    #cosy_cosp = numpy.real(1 - 2 * (quat.y * quat.y + quat.z * quat.z))
    cosy_cosp = numpy.real(quat.w * quat.w + quat.x * quat.x - quat.y * quat.y + quat.z * quat.z)
    yaw       = atan2(siny_cosp, cosy_cosp)

    roll  = roll  * 180 / pi
    pitch = pitch * 180 / pi
    yaw   = yaw   * 180 / pi

    #gl.X_AXIS = numpy.real(pitch)
    #gl.Y_AXIS = numpy.real(gyro_acc[2])
    #gl.Z_AXIS = numpy.real(roll)

    #gyro_acc = [((((round(x, 0)) * 0.02 * dw[indexOf(gyro_, x)]) + gyro_acc[indexOf(gyro_, x)])) for x in gyro_]
    gyro_acc[2] += (gyro_[2] * sample_time_ * 180 / pi)
    ref_ = numpy.real(sqrt(numpy.real(yaw) * numpy.real(yaw)))
    #acc_count += 1
    
    #if ((ref_ > 177 or ref_ < 11) and acc_count > 800):
    #    #gyro_acc[2] = ref_
    #    acc_count   = 0
    gyro_acc[0] += (gyro_[0] * sample_time_ * 180 / pi)

    x_dps += numpy.real((atan(accel_[0]) * 180 * sample_time_ * sample_time_ / pi))

    accel_acc[0] = numpy.real((acos(accel_[0] / 9.80665) * 180 )/ pi)
    accel_acc[1] = numpy.real((acos(accel_[1] / 9.80665) * 180 )/ pi)
    accel_acc[2] = numpy.real((acos(accel_[2] / 9.80665) * 180 )/ pi)

    #if((numpy.real(sqrt(yaw * yaw)) > 178.5) and (numpy.real(sqrt(yaw * yaw)) < 1.5)):
        #yaw_ = numpy.real(sqrt(gyro_acc[2] * gyro_acc[2])) #* (1 and (yaw < 0) + 2 and (yaw > 0))
    #else:
        #gyro_acc[2] = yaw
        #yaw_        = yaw
        #gyro_acc[2] = numpy.real(sqrt(yaw * yaw)) * (1 and (gyro_acc[2] > 0) + 2 and (gyro_acc[2] < 0))
        #yaw_ = numpy.real(sqrt(gyro_acc[2] * gyro_acc[2])) * (1 and (yaw < 0) + 2 and (yaw > 0))

    #gl.X_AXIS = roll
    gl.Y_AXIS = -yaw
    #gl.Z_AXIS = pitch

    #gl.m00, gl.m01, gl.m02 = mat_[0][0], mat_[0][1], mat_[0][2]
    #gl.m10, gl.m11, gl.m12 = mat_[1][0], mat_[1][1], mat_[1][2]
    #gl.m20, gl.m21, gl.m22 = mat_[2][0], mat_[2][1], mat_[2][2]


    #gyro_acc[1] = ((gyro_acc[1] - gyro_[2]) * 100) #- round(numpy.real(yaw))

    #scale = numpy.real(sqrt(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z ))
    #serial_write([round(numpy.real(roll), 1), round(numpy.real(pitch), 1), round(numpy.real(yaw), 1), round(gyro_acc[2], 3)])

    #quat_array = [round(quat.x * 180 / scale, 2), round(quat.y * 180 / scale, 2), round(quat.z * 180 / scale, 2), round(quat.w, 2)]
    #serial_write(quat_array)


#VISUALIZATION / CALIBRATION FROM RAW DATA    (Befor running call function )
def raw_viz_calib():
    pattern = r"\d+"       #pattern for int
    while True:
        #print("VAL : ", rx_.val_)
    
        data = rx_.val_
        #numeric_values = [int(num) for num in re.findall(pattern, data)]
        #data = [numeric_values[i:i+3] for i in range(0, len(numeric_values), 3)]
        #print(numeric_values)
        if('[' in data):
            data_ = [numpy.short(num) for num in re.findall(pattern, data)]  # cast as short as values ar in 2's complement form
            #data_ = [numpy.float(num) for num in re.findall(pattern, data)]  # cast as short as values ar in 2's complement form
            
            #print(data)
            #print(data[0])
    
            accel = [data_[0], data_[1], data_[2]]
            gyro  = [data_[3], data_[4], data_[5]]
            mag   = [data_[6], data_[7], data_[8]]
    
            #print("RAW   : ", data)
            #print("ACCEL : ", accel)
            #print("GYRO  : ", gyro)
            #print("MAG   : ", mag)
    
            imu_fusion(accel, gyro, mag)
            #mag_calib.mag_calibration(mag)
    
        sleep(sample_time_)


def viz_calib_processed(roll, pitch, yaw):
    #calibration_(accel_, gyro_, mag_)
    
    #gl.X_AXIS = roll
    gl.Y_AXIS = -yaw
    #gl.Z_AXIS = pitch



#VISUALIZATION / CALIBRATION FROM PROCESSED DATA
def processed_viz_calib():
    pattern = r"\d*\.\d+"  #pattern for float
    pattern = r"[-+]?(?:\d*\.*\d+)"  #pattern for float
    rx_.update_rate = (1 / (120 * 2))  # 2 times operating freq [(120 hz) * 2]
    while True:
        data = rx_.val_
        #print("VAL : ", data)
        if('[' in data):
            data_ = [numpy.float(num) for num in re.findall(pattern, data)]  # cast as short as values ar in 2's complement form
            
            #print(data_)
            print(data_[2])
    
            #print("RAW    : ", data_)
            #print("ROLL   : ", data_[0])
            #print("PITCH  : ", data_[1])
            #print("YAW    : ", data_[2])
    
            viz_calib_processed(data_[0], data_[1], data_[2])

    
        sleep(sample_time_)




#processed_viz_calib()
#raw_viz_calib()




#thread_.join()
#thread_1.join()
#rx_.close()
