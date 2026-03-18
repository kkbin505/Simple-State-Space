from mpu6050 import MPU6050
import time

device_address = 0x68
mpu = MPU6050(bus=0, sda=4, scl=5, address=device_address)

mpu.dmp_initialize()
mpu.set_DMP_enabled(True)
mpu_int_status = mpu.get_int_status()
print(hex(mpu_int_status))

packet_size = mpu.DMP_get_FIFO_packet_size()
print(packet_size)
FIFO_count = mpu.get_FIFO_count()
print(FIFO_count)

count = 0
FIFO_buffer = [0]*64
FIFO_count_list = list()

while True:
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()
    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
        print('overflow!')
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        
        quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        roll, pitch, yaw = mpu.DMP_get_roll_pitch_yaw(quat, grav)
        gx, gy, gz = mpu.get_rotation()

        if count % 10 == 0:  # Printing every 10th measurement
            print(f' roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}, gx={gx:.2f}, gy={gy:.2f}, gz={gz:.2f}')
        count += 1
