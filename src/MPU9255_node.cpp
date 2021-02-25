#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <err.h>
#include <errno.h>

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <string>
#include "std_msgs/String.h"
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <sstream>
#include "sensor_msgs/MagneticField.h"

using namespace std;

//string i2cDeviceName_0 = "/dev/i2c-0";
string i2cDeviceName_1 = "/dev/i2c-1";
//int file_i0;
int file_i1;

#define    MPU9250_ADDRESS            0x68
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00  
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command, int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;
	return ioctl(file, I2C_SMBUS, &args);
}

static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	if (i2c_smbus_access(file, I2C_SMBUS_READ, command,	I2C_SMBUS_BYTE_DATA, &data))
		return -1;
	else
		return 0x0FF & data.byte;
}

static inline __s32 i2c_smbus_write_byte_data(int file, __u8 command, uint8_t data_in)
{
	union i2c_smbus_data data;
	data.byte = data_in;
	
	if (i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data))
		return -1;
	else
		return 0x0FF & data.byte;
}

uint8_t i2c_read(uint8_t dev_addr, uint8_t reg)
{
	int rc;
	uint8_t read_back;

	rc = ioctl(file_i1, I2C_SLAVE, dev_addr); // Sets the device address
	if (rc < 0)
	{
		ROS_INFO("ERROR - couldn't set device address");
		return false;
	}

	// Actually perform the write and then read back immediately
	read_back = i2c_smbus_read_byte_data(file_i1, reg);
	//ROS_INFO("read_back = 0x%x", read_back);
	
	return read_back;
}

uint8_t i2c_write(uint8_t dev_addr, uint8_t reg, uint8_t data_in)
{
	int rc;
	uint8_t read_back;

	rc = ioctl(file_i1, I2C_SLAVE, dev_addr); // Sets the device address
	if (rc < 0)
	{
		ROS_INFO("ERROR - couldn't set device address");
		return false;
	}

	// Actually perform the write and then read back immediately
	read_back = i2c_smbus_write_byte_data(file_i1, reg, data_in);
	//ROS_INFO("read_back = 0x%x", read_back);
	
	return read_back;
}


int main(int argc, char **argv){

  ros::init(argc, argv, "IMU_pub");
  
  ros::NodeHandle n;
  ros::Publisher pub_imu = n.advertise<sensor_msgs::Imu>("imu/data_raw", 2);
  ros::Publisher pub_mag = n.advertise<sensor_msgs::MagneticField>("imu/mag", 2);

	// I2C
	file_i1 = open(i2cDeviceName_1.c_str(), O_RDWR);
	if (file_i1 < 0)
		ROS_INFO("ERROR - i2c 0 file not open!");
		
	// Configure gyroscope range
	//i2c_write(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_2000_DPS);

	// Configure accelerometers range
	//i2c_write(MPU9250_ADDRESS,28,ACC_FULL_SCALE_16_G);

	// Set by pass mode for the magnetometers
	i2c_write(MPU9250_ADDRESS,0x37,0x02);

	// Request first magnetometer single measurement
	i2c_write(MAG_ADDRESS,0x0A,0x01);

    float conversion_gyro = 3.1415/(180.0*32.8f);
    float conversion_acce = 9.8/16384.0f;
	float conversion_magno = 0.15;
 
  int16_t InBuffer[9] = {0}; 
  static int32_t OutBuffer[3] = {0};
  
  ros::Rate loop_rate(25); 

  while (ros::ok()){
    //http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html 
    //http://docs.ros.org/api/sensor_msgs/html/msg/MagneticField.html
    sensor_msgs::Imu data_imu;    
    sensor_msgs::MagneticField data_mag;

    data_mag.header.stamp = ros::Time::now();
    data_imu.header.stamp = data_mag.header.stamp;
    data_imu.header.frame_id = "imu_link";

    //datos acelerómetro
    InBuffer[0]=  (i2c_read(MPU9250_ADDRESS, 0x3B)<<8)|i2c_read(MPU9250_ADDRESS, 0x3C);
    InBuffer[1]=  (i2c_read(MPU9250_ADDRESS, 0x3D)<<8)|i2c_read(MPU9250_ADDRESS, 0x3E);
    InBuffer[2]=  (i2c_read(MPU9250_ADDRESS, 0x3F)<<8)|i2c_read(MPU9250_ADDRESS, 0x40);   
    
    data_imu.linear_acceleration.x = InBuffer[0]*conversion_acce;
    data_imu.linear_acceleration.y = InBuffer[1]*conversion_acce;
    data_imu.linear_acceleration.z = InBuffer[2]*conversion_acce;

     //datos giroscopio
    InBuffer[3]=  (i2c_read(MPU9250_ADDRESS, 0x43)<<8)|i2c_read(MPU9250_ADDRESS, 0x44);
    InBuffer[4]=  (i2c_read(MPU9250_ADDRESS, 0x45)<<8)|i2c_read(MPU9250_ADDRESS, 0x46);
    InBuffer[5]=  (i2c_read(MPU9250_ADDRESS, 0x47)<<8)|i2c_read(MPU9250_ADDRESS, 0x48); 

    data_imu.angular_velocity.x = InBuffer[3]*conversion_gyro;
    data_imu.angular_velocity.y = InBuffer[4]*conversion_gyro;
    data_imu.angular_velocity.z = InBuffer[5]*conversion_gyro; 

    //datos magnetómetro
    InBuffer[6]=  (i2c_read(MAG_ADDRESS, 0x04)<<8)|i2c_read(MAG_ADDRESS, 0x03);
    InBuffer[7]=  (i2c_read(MAG_ADDRESS, 0x06)<<8)|i2c_read(MAG_ADDRESS, 0x05);
    InBuffer[8]=  (i2c_read(MAG_ADDRESS, 0x08)<<8)|i2c_read(MAG_ADDRESS, 0x07);
	
	// Request magnetometer single measurement
	i2c_write(MAG_ADDRESS, 0x0A, 0x01);
    
    data_mag.magnetic_field.x = InBuffer[6]*conversion_magno;
    data_mag.magnetic_field.y = InBuffer[7]*conversion_magno;
    data_mag.magnetic_field.z = InBuffer[8]*conversion_magno;
    
    pub_imu.publish(data_imu);

    pub_mag.publish(data_mag);

    ros::spinOnce();
	loop_rate.sleep();
    }
  return 0;
 }
 

