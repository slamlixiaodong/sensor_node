
#ifndef __battery_node_H_
#define __battery_node_H_
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
#include <std_msgs/Float32.h>
using namespace std;
#define SEND_DATA_CHECK   1     //标志位，发送端做校验位
#define READ_DATA_CHECK   0     //标志位，接收端做校验位
#define FRAME_HEADER      0X55  //帧头，和下位机一致
#define FRAME_TAIL  0X0A //帧尾
#define RECEIVE_DATA_SIZE		18//下位机发过来的数据的长度
#define PI 				3.1415926f//圆周率

typedef uint8_t u8;
typedef uint16_t u16;




typedef struct _RECEIVE_DATA_     
{
	    uint8_t rx[RECEIVE_DATA_SIZE];
	    uint8_t Flag_Stop;
		unsigned char Frame_Header; //1个字节 帧头
		int Voltage;
		float Current;
		int Quantity ;
		int Capacity ;
		int RSOC ;
		int Ctrl ;
		int Protect ;
		int Cycle ;
		float CRC;
		unsigned char Frame_Tail;//1个字节  帧尾 校验位
}RECEIVE_DATA;

//DATE：2020-5-31
//类，巧妙使用构造函数初始化数据和发布话题等
class battery_serial
{
	public:
		battery_serial(); //构造函数
		~battery_serial(); //析构函数
		void serial();//循环控制代码
		serial::Serial battery_Serial; //声明串口对象 
	private:
		/* /cmd_val topic call function 回调函数声明*/

		/* Read/Write data from ttyUSB 串口和控制函数声明 */
		bool Get_Battery_Data();
		float shorttofloat(uint8_t Data_High,uint8_t Data_Low);
		u16 MODBUS_CRC(u8 *buff,u8 len);
		u8 Check_CRC(u8 *data, int size);
		int serial_baud_rate;//波特率
		string usart_port_name;
		ros::NodeHandle n;//创建句柄
		ros::Time _Now, _Last_Time;//时间相关
		float Sampling_Time; //采样时间
		RECEIVE_DATA battery_data;  //接收结构体   Receive_Data    
        //SEND_DATA Send_Data;  //发送结构体  Send_Data
};
#endif
