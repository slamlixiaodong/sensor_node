
#ifndef __sensor_node_H_
#define __sensor_node_H_
// 头文件
#include "ros/ros.h"
#include <iostream>
#include <string.h>
#include <string> 
#include <iostream>
#include <math.h>
#include <stdlib.h>    
#include <unistd.h>      
#include <sys/types.h>
#include <sys/stat.h>
#include <serial/serial.h>
#include <fcntl.h>          
#include <stdbool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
using namespace std;
#define FRAME_HEADER      0X55  //帧头，和下位机一致
#define FRAME_TAIL  0X0A //帧尾
#define RECEIVE_DATA_SIZE		21//下位机发过来的数据的长度
#define SEND_DATA_SIZE_0			12//向下位机发送
#define SEND_DATA_SIZE_1			6//向下位机发送
#define SET_LED_TYPE   0x01     //自定义模式
#define UNSET_LED_TYPE 0x02     //指定灯光模式

typedef uint8_t u8;
typedef uint16_t u16;


typedef struct _SEND_DATA_  
{
	    uint8_t tx_0[SEND_DATA_SIZE_0]; 
		uint8_t tx_1[SEND_DATA_SIZE_1];       
		unsigned char Frame_Tail;    //1个字节  帧尾 校验位 

}SEND_DATA;



typedef struct _RECEIVE_DATA_     
{
	    uint8_t rx[RECEIVE_DATA_SIZE];
	    uint8_t Flag_Stop;
		u8 led_back_flag[20];
		unsigned char Frame_Header; //1个字节 帧头
		unsigned char Frame_Tail;//1个字节  帧尾 校验位
}RECEIVE_DATA;

typedef struct _SENSOR_DATA_     
{
		float Wave[5];
		float Switch;
		float Temp;
		float Humidity;
		float PM_25;
}SENSOR_DATA;

//DATE：2020-5-31
//类，巧妙使用构造函数初始化数据和发布话题等
class sensor_serial
{
	public:
		sensor_serial(); //构造函数
		~sensor_serial(); //析构函数
		serial::Serial sensor_Serial; //声明串口对象 
	private:
		/* /cmd_val topic call function 回调函数声明*/

		/* Read/Write data from ttyUSB 串口和控制函数声明 */
		bool Get_Sensor_Data();
		void Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux);
		void Robot_Led_Callback(const std_msgs::UInt8::ConstPtr& msg);
		float shorttofloat(uint8_t Data_High,uint8_t Data_Low);
		u16 MODBUS_CRC(u8 *buff,u8 len);
		u8 Check_CRC(u8 *data, int size);
		int serial_baud_rate;//波特率
		string usart_port_name;
		ros::NodeHandle n;//创建句柄
		ros::Subscriber Cmd_Vel_Sub,Robot_Led_Sub;
		ros::Publisher cmd_vel_publisher;
		ros::Time _Now, _Last_Time;//时间相关
		float Sampling_Time; //采样时间
		RECEIVE_DATA receive_data;  //接收结构体   Receive_Data  
		SENSOR_DATA  sensor_data;
		SEND_DATA Send_Data;  //发送结构体  Send_Data 
		int sensor_error_count=0;
};
#endif
