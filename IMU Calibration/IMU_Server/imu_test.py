#!/usr/bin/python3
import imu_ix  as imu_
import send_ix as tx_
from threading import Thread
from time import sleep


imu_.sample_time_ = 0.0083
imu_.sample_rate_ = 1 / imu_.sample_time_

imu_.mag_bias  = [0.061798095703125, 0.10134887695312494, 0.1717987060546875]         #[x, y, z] Magnetometer calibration bias
imu_.gyro_bias = [0.014114753777375467, 0.010852381442038685, -0.006657902725177109]  #[x, y, z] Gyroscope calibration bias

imu_.i2c_bus_init()

imu_.imu_init()

thread_ = Thread(target = tx_.send_ix, args = ())
thread_.start()

print("Publishing IMU DATA")


while True:
    #Read processed Roll, Pitch, Yaw 
    #data_ = imu_.imu_fusion_()

    #Read Raw IMU Data For Calibration
    data_ = imu_.imu_read_raw()
    
    print(data_)
    
    data_ = str(data_)
    tx_.data_ = bytes(data_, 'utf-8')
    sleep(imu_.sample_time_)

thread_.join()

imu_.i2c_close_()
