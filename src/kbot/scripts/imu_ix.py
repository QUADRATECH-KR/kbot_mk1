#!/usr/bin/python3
import smbus
from time import sleep
import imufusion
import numpy
from cmath import acos, asin, atan, cos, pi, sqrt
from math import atan2


#I2C HW OBJECT
i2c_hw = 0

MPU6050_DEV_ADDR  = 0x68
HMC5883L_DEV_ADDR = 0x1E

SAMPL_RATE_DIV = 0x19

CFG      = 0x1A
GYRO_CFG = 0x1B
ACCL_CFG = 0x1C

INT_PIN_CFG = 0x37

ACC_XOUT_H = 0x3B

MPU6050_USER_CTRL = 0x6A
MPU6050_PWR_MGMT1 = 0x6B
MPU6050_PWR_MGMT2 = 0x6C

MPU6050_DEV_ID = 0x75


HMC5883L_CFG_A  = 0x00
HMC5883L_CFG_B  = 0x01
HMC5883L_MODE_R = 0x02

MAG_XOUT_H = 0x03


accel_range = 2.0   # Measurement Scale in +/- (Sensitivity 2 g)
gyro_range  = 250   # Measurement Scale in +/- (Sensitivity 250 deg/s)
mag_range   = 8.1   # Measurement Scale in +/- (Sensitivity 8.1)

buffer = []

sample_time_ = 0.000005
sample_rate_ = 1 / sample_time_

offset = imufusion.Offset(int(sample_rate_))
ahrs = imufusion.Ahrs()

ahrs.settings = imufusion.Settings( imufusion.CONVENTION_NWU,  # convention (NED(North East Down), ENU(East North Up), NWU (North West Up))
                                    5.5,                       # gain
                                    0.5,                       # acceleration rejection in degree
                                    1,                         # magnetic rejection in degree
                                    int(0.0 * sample_rate_) )  # rejection timeout

accel_ned = [0.0, 0.0, 0.0]
gyro_ned  = [0.0, 0.0, 0.0]

mag_bias  = [0.061798095703125, 0.10134887695312494, 0.1717987060546875]         #[x, y, z] Magnetometer calibration bias
gyro_bias = [0.014114753777375467, 0.010852381442038685, -0.006657902725177109]  #[x, y, z] Gyroscope calibration bias



def i2c_bus_init():
    global i2c_hw

    i2c_hw = smbus.SMBus(1) #0 for I2C0 and 1 for I2C1


def imu_init():
    global i2c_hw

    ##INIT MPU6050
    pwr_mgmt_reg  = i2c_hw.read_byte_data(MPU6050_DEV_ADDR, MPU6050_PWR_MGMT1)
    #Disable Sleep, Cycle And Set CLock Source To PLL With X Gyro As Ref
    pwr_mgmt_reg = (pwr_mgmt_reg & 0x9F) #| (0x01)
    i2c_hw.write_byte_data(MPU6050_DEV_ADDR, MPU6050_PWR_MGMT1, pwr_mgmt_reg)

    i2c_hw.write_byte_data(MPU6050_DEV_ADDR, GYRO_CFG, 0)           #Set Gyro to max scale (250 deg/s)
    i2c_hw.write_byte_data(MPU6050_DEV_ADDR, ACCL_CFG, 0)           #Set Accelerometer to max scale (2 g)

    #Disable I2C Master Mode (I2C_MST_EN BIT)
    user_ctrl_reg = i2c_hw.read_byte_data(MPU6050_DEV_ADDR, MPU6050_USER_CTRL)
    user_ctrl_reg = user_ctrl_reg & 0xDF
    i2c_hw.write_byte_data(MPU6050_DEV_ADDR, MPU6050_USER_CTRL, user_ctrl_reg)

    #Enable I2C Bypass to Access Magnetometer (Set I2C_BYPASS_EN BIT To 0)
    int_pin_cfg_reg = i2c_hw.read_byte_data(MPU6050_DEV_ADDR, INT_PIN_CFG)
    int_pin_cfg_reg = (int_pin_cfg_reg & 0xFD) | 0x02
    i2c_hw.write_byte_data(MPU6050_DEV_ADDR, INT_PIN_CFG, int_pin_cfg_reg)

    #print("AAA")
    #print(pwr_mgmt_reg)
    #print(user_ctrl_reg)
    #print(int_pin_cfg_reg)
    #print("AAA")

    ##INIT HMC5883L
    #Set CFG_REG_A :: Set Data Rate To 75Hz, No Of Samples Averaged To 8 and Set Measurement Mode To Normal 
    cfg_reg_a = 0x78
    i2c_hw.write_byte_data(HMC5883L_DEV_ADDR, HMC5883L_CFG_A, cfg_reg_a)

    #Set Gain To 230 In CFG_REG_B
    cfg_reg_b = 0x70
    i2c_hw.write_byte_data(HMC5883L_DEV_ADDR, HMC5883L_CFG_B, cfg_reg_b)

    #Set Mode Operating Mode To Continuous Measurement Mode In MODE_REG
    mode_reg = 0x00
    i2c_hw.write_byte_data(HMC5883L_DEV_ADDR, HMC5883L_MODE_R, mode_reg)


def imu_read_raw():
    i = 0
    while(i < 14):
      id_ = ACC_XOUT_H + i
      #print(id_)
      #exit()
      val_ = i2c_hw.read_byte_data(MPU6050_DEV_ADDR, id_)
      buffer.append(val_)
      i += 1

    x_a = (buffer[0] << 8)  | buffer[1]
    y_a = (buffer[2] << 8)  | buffer[3]
    z_a = (buffer[4] << 8)  | buffer[5]
 
    x_g = (buffer[8] << 8)  | buffer[9]
    y_g = (buffer[10] << 8) | buffer[11]
    z_g = (buffer[12] << 8) | buffer[13]

    buffer.clear()

    i = 0
    while(i < 6):
      id_ = MAG_XOUT_H + i
      #print(id_)
      #exit()
      val_ = i2c_hw.read_byte_data(HMC5883L_DEV_ADDR, id_)
      buffer.append(val_)
      i += 1

    x_m = (buffer[0] << 8)  | buffer[1]
    y_m = (buffer[2] << 8)  | buffer[3]
    z_m = (buffer[4] << 8)  | buffer[5]

    buffer.clear()

    accel = numpy.array([x_a, y_a, z_a])
    gyro  = numpy.array([x_g, y_g, z_g])
    mag   = numpy.array([x_m, y_m, z_m])

    #print(accel)
    #print(gyro)
    #print(mag)

    #sleep(0.005)

    return [accel, gyro, mag] #[0,0,0,0]


def imu_read_processed():
    global ACC_XOUT_H, MAG_XOUT_H, acc_count, time_, accel_range, gyro_range, mag_range
    global ahrs, accel_ned, gyro_ned, sample_time_, mag_bias, gyro_bias

    i = 0
    while(i < 14):
      id_ = ACC_XOUT_H + i
      #print(id_)
      #exit()
      val_ = i2c_hw.read_byte_data(MPU6050_DEV_ADDR, id_)
      buffer.append(val_)
      i += 1


    x_a = (buffer[0] << 8)  | buffer[1]
    y_a = (buffer[2] << 8)  | buffer[3]
    z_a = (buffer[4] << 8)  | buffer[5]

    x_g = (buffer[8]  << 8) | buffer[9]
    y_g = (buffer[10] << 8) | buffer[11]
    z_g = (buffer[12] << 8) | buffer[13]

    buffer.clear()

    i = 0
    while(i < 6):
      id_ = MAG_XOUT_H + i
      #print(id_)
      #exit()
      val_ = i2c_hw.read_byte_data(HMC5883L_DEV_ADDR, id_)
      buffer.append(val_)
      i += 1

    x_m = (buffer[0] << 8)  | buffer[1]
    y_m = (buffer[2] << 8)  | buffer[3]
    z_m = (buffer[4] << 8)  | buffer[5]

    buffer.clear()

    accel_raw = numpy.array([x_a, y_a, z_a])
    gyro_raw  = numpy.array([x_g, y_g, z_g])
    mag_raw   = numpy.array([x_m, y_m, z_m])

    #accel_ = [((x * accel_range) / (32768)) for x in accel_raw]                          # Measurement in g
    accel_ = [((numpy.short(x) * accel_range * 9.80665) / (32768)) for x in accel_raw]                 # Measurement in m/s^2

    #gyro_  = [((x * gyro_range) / (32768))  for x in gyro_raw]                           # Measurement in deg/s
    gyro_  = [((numpy.short(x) * gyro_range * pi) / (32768 * 180))  for x in gyro_raw]                 # Measurement in rad/s

    #mag_   = [((x  * mag_range) /  32768) for x in mag_raw]                              # Measurement in Gauss
    mag_   = [((numpy.short(x) * mag_range * 10)  / (32768)) for x in mag_raw]                         # Measurement in uT (micro Tesla)

    mag_[0]  += mag_bias[0]
    mag_[1]  += mag_bias[1]
    mag_[2]  += mag_bias[2]

    #mag_[0]  += 0.061798095703125
    #mag_[1]  += 0.10134887695312494
    #mag_[2]  += 0.1717987060546875

    #mag_ = [100 * x for x in mag_]

    gyro_[0] += gyro_bias[0]
    gyro_[1] += gyro_bias[1]
    gyro_[2] += gyro_bias[2]

    #print(accel)
    #print(gyro)
    #print(mag)

    #sleep(0.005)

    return [accel_, gyro_, mag_] #[0,0,0,0]


def imu_fusion_():
    global sample_time_, accel_ned, gyro_ned

    imu_data = imu_read_processed()

    accel_ = imu_data[0]
    gyro_  = imu_data[1]
    mag_   = imu_data[2]

    #Data In NED Format If X-Axis Points North
    #Ay, Ax, -Az    Gy, Gx, -Gz    Mx, My, Mz
    accel_ned[0] = accel_[1]
    accel_ned[1] = accel_[0]
    accel_ned[2] = -1 * accel_[2]

    gyro_ned[0] = gyro_[1]
    gyro_ned[1] = gyro_[0]
    gyro_ned[2] = -1 * gyro_[2]

    ahrs.update(numpy.array(gyro_ned), numpy.array(accel_ned), numpy.array(mag_), 0.006)

    quat  = ahrs.quaternion
    euler = quat.to_euler()

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

    roll  = round(roll  * 180 / pi, 2)
    pitch = round(pitch * 180 / pi, 2)
    yaw   = round(yaw   * 180 / pi, 2)

    IMU_OUT_RPY = numpy.array([roll, pitch, yaw])

    #print(imu_data)
    #print(yaw)

    return IMU_OUT_RPY
    #return imu_data


def i2c_close_():
    i2c_hw.close()



#print("TEST")
#imu_init()
#
#deg = 0
#dat = bytes("0", 'utf-8')
#
#thread_ = Thread(target = tx_.send_ix, args = ())
#thread_.start()