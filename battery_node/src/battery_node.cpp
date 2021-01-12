#include "battery_node/battery_node.h"
/**************************************
Date: May 31, 2020
Function: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "battery_serial");//ROS初始化 并设置节点名称，可修改
  battery_serial battery_node; //实例化一个对象
  battery_node.serial();  //循环执行数据采集和发布topic等操作
  return 0;
} 


/**************************************
Date: June 29, 2020
Function: 串口通讯校验函数，数据包除最后一个字节，其他的全部数据按位异或的结果作为帧尾
***************************************/
u8 battery_serial::Check_CRC(u8 *data, int size) //CRC 校验
{
    if(MODBUS_CRC(data,size-2)==(((u16)data[size-2]<<8)|data[size-1]))
    {
        return 1;
    }
    else
        return 0;
}
u16 battery_serial::MODBUS_CRC(u8 *buff,u8 len) //modbus 协议的 crc 校验
{
    unsigned short tmp = 0xffff;
    unsigned short ret1 = 0;
    u8 n = 0;
    u8 i = 0;
    for (n = 0; n < len; n++) 
    { //要校验的位数为 len 个
        tmp = buff[n] ^ tmp;
        for (i = 0; i < 8; i++) 
        { //此处的 8 -- 指每一个 char 类型又 8bit，每 bit 都要处理
            if (tmp & 0x01) 
            {
              tmp = tmp >> 1;
              tmp = tmp ^ 0xa001;
            }
            else 
            {
              tmp = tmp >> 1;
              }
        }
      } /*CRC 校验后的值*/
      ret1 = tmp >> 8;
      ret1 = ret1 | (tmp << 8);
      return ret1;
}

/**************************************
Date: June 29, 2020
Function: 从串口读取数据 IMU是short类型的原始数据，单位需要结合MPU6050手册转化
***************************************/
bool battery_serial::Get_Battery_Data()
{ 
  short transition_16=0,j=0,Header_Pos=0,Tail_Pos=0;  //中间变量
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE]={0};
  battery_Serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr));//读串口数据
  for(j=0;j<18;j++)
    {
    if(Receive_Data_Pr[j]==FRAME_HEADER)
    Header_Pos=j;
    else if(Receive_Data_Pr[j]==FRAME_TAIL)
    Tail_Pos=j;    
    }
    //ROS_INFO("%x-%x",Header_Pos,Tail_Pos);
    if(Tail_Pos==(Header_Pos+17))
    {
         //ROS_INFO("1----");
      memcpy(battery_data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    }
    else if(Header_Pos==(1+Tail_Pos))
    {
        //ROS_INFO("2----");
        for(j=0;j<18;j++)
        battery_data.rx[j]=Receive_Data_Pr[(j+Header_Pos)%18];
    }
    else 
    {
     //ROS_INFO("3----");
     return false;
    }    
    //ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
    //Receive_Data.rx[0],Receive_Data.rx[1],Receive_Data.rx[2],Receive_Data.rx[3],Receive_Data.rx[4],Receive_Data.rx[5],Receive_Data.rx[6],Receive_Data.rx[7],
    //Receive_Data.rx[8],Receive_Data.rx[9],Receive_Data.rx[10],Receive_Data.rx[11],Receive_Data.rx[12],Receive_Data.rx[13],Receive_Data.rx[14],Receive_Data.rx[15],
    //Receive_Data.rx[16],Receive_Data.rx[17],Receive_Data.rx[18],Receive_Data.rx[19],Receive_Data.rx[20],Receive_Data.rx[21],Receive_Data.rx[22],Receive_Data.rx[23]); 
  
  battery_data.Frame_Header= battery_data.rx[0]; //数据的第一位是帧头（固定值）
  battery_data.Frame_Tail= battery_data.rx[17];  //数据的最后一位是帧尾（数据校验位）

 if (battery_data.Frame_Header == FRAME_HEADER )//判断帧头
  {
    if (battery_data.Frame_Tail == FRAME_TAIL) //判断帧尾
    { 
      //补充CRC校验
      if(Check_CRC(battery_data.rx, 17))
      { 
        battery_data.Voltage = battery_data.rx[1]*256+battery_data.rx[2];
		    battery_data.Current = shorttofloat(battery_data.rx[3],battery_data.rx[4]);
		    battery_data.Quantity = battery_data.rx[5]*256+battery_data.rx[6];
        battery_data.Capacity = battery_data.rx[7]*256+battery_data.rx[8];
        battery_data.RSOC = battery_data.rx[9];
        battery_data.Ctrl = battery_data.rx[10];
        battery_data.Protect = battery_data.rx[11]*256+battery_data.rx[12];
        battery_data.Cycle = battery_data.rx[13]*256+battery_data.rx[14];
        return true;
     }
    }
  } 
 return false;
}

float battery_serial::shorttofloat(uint8_t Data_High,uint8_t Data_Low)
{
  float data_return;
  short transition_16;
      transition_16 = 0;
      transition_16 |=  Data_High<<8;  //获取数据的高8位
      transition_16 |=  Data_Low;     //获取数据的低8位
      data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; //(发送端将数据放大1000倍发送，这里需要将数据单位还原)
  return data_return;
}
/**************************************
Date: May 31, 2020
Function: 这是相关控制代码，代码循环执行
***************************************/
void battery_serial::serial()
{
  _Last_Time = ros::Time::now();
  while(ros::ok())
  {
    _Now = ros::Time::now();
    Sampling_Time = (_Now - _Last_Time).toSec();
   //Sampling_time是采样时间，虽然下位机发送的数据频率是固定的，这里计算里程增量以ROS系统的时间更加可靠精确。
    if (true == Get_Battery_Data())  //从串口读取下位机法过来的全部数据
    {

    }
    _Last_Time = _Now;//记录时间
    ros::spinOnce();//循环等待回调函数
    }
}
/**************************************
Date: May 31, 2020
Function: 构造函数, 只执行一次，用于初始化
***************************************/
battery_serial::battery_serial():Sampling_Time(0)
{
  memset(&battery_data, 0, sizeof(battery_data)); //构造函数初始化
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("usart_port_name", usart_port_name, "/dev/ttyUSB1"); //固定串口
  private_nh.param<int>("serial_baud_rate", serial_baud_rate, 115200); //和下位机底层波特率115200 不建议更高的波特率了
  ROS_INFO_STREAM("Data ready");//ready显示状态
  //初始化串口
  try{
         battery_Serial.setPort(usart_port_name);//选择哪个口，如果选择的口没有接串口外设初始化会失败
         battery_Serial.setBaudrate(serial_baud_rate);//设置波特率
         serial::Timeout _time = serial::Timeout::simpleTimeout(2000);//超时等待
         battery_Serial.setTimeout(_time);
         battery_Serial.open();//串口开启
    }
  catch (serial::IOException& e){
     ROS_ERROR_STREAM("battery_node can not open serial port,Please check the serial port cable! ");//如果try失败，打印错误信息
  }
  if(battery_Serial.isOpen()){
    ROS_INFO_STREAM("battery_node serial port opened");//开启成功
  }else{
  }
}
/**************************************
Date: May 31, 2020
Function: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
battery_serial::~battery_serial()
{
//   Send_Data.tx[0]=FRAME_HEADER;//帧头 固定值
//   Send_Data.tx[1] = 0 ; //产品型号
//   Send_Data.tx[2] = 0;  //机器人使能控制标志位
//   //机器人x轴的目标线速度
//   Send_Data.tx[4] = 0;     //取数据的低8位
//   Send_Data.tx[3] = 0;  //取数据的高8位
//   //机器人y轴的目标线速度
//   Send_Data.tx[6] = 0;
//   Send_Data.tx[5] = 0;
//   //机器人z轴的目标角速度
//   Send_Data.tx[8] = 0;
//   Send_Data.tx[7] = 0;

//   Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK);//帧尾校验位，规则参见Check_Sum函数
//   Send_Data.tx[10]=FRAME_TAIL;  //数据的最后一位是帧尾（固定值）
  try
  {
  // if(Receive_Data.Flag_Stop==0) 
//   Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //向串口发数据
  ROS_INFO_STREAM("New control command");//显示受到了新的控制指令  
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //如果try失败,打印错误信息
  }
  battery_Serial.close();//关闭串口
  ROS_INFO_STREAM("Shutting down");//close
}