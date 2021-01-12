#include "sensor_node/sensor_node.h"
/**************************************
Date: May 31, 2020
Function: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_serial");//ROS初始化 并设置节点名称，可修改
  sensor_serial sensor_node; //实例化一个对象
  ros::spin();
  return 0;
} 
void sensor_serial::Robot_Led_Callback(const std_msgs::UInt8::ConstPtr& msg)
{
  short  transition;  //中间变量
  if(msg->data==SET_LED_TYPE)
  {
    Send_Data.tx_0[0]=FRAME_HEADER;//帧头 固定值
    Send_Data.tx_0[1]=msg->data;
    Send_Data.tx_0[2] = 0x01;
    Send_Data.tx_0[3] = 0x01;
    Send_Data.tx_0[4] = 0x01;
    Send_Data.tx_0[5] = 0x01;
    Send_Data.tx_0[6] = 0x01;
    Send_Data.tx_0[7] = 0x01;
    Send_Data.tx_0[8] = 0x01;
    Send_Data.tx_0[9] = 0x01;
    u16 tmp = MODBUS_CRC(Send_Data.tx_0,SEND_DATA_SIZE_0-2);
    Send_Data.tx_0[10] = tmp >> 8;
    Send_Data.tx_0[11] = tmp;
    try
    {
      sensor_Serial.write(Send_Data.tx_0,SEND_DATA_SIZE_0); //向串口发数据

      // sensor_Serial.read(receive_data.rx,sizeof(receive_data.led_back_flag));
    }
    catch (serial::IOException& e)
    {
      ROS_ERROR_STREAM("Unable to send data through serial port"); //如果try失败,打印错误信息
    }
  }
  else if(msg->data==UNSET_LED_TYPE)
  {
      Send_Data.tx_1[0] = FRAME_HEADER;//帧头 固定值
      Send_Data.tx_1[1] = msg->data;
      Send_Data.tx_1[2] = 0x01;
      Send_Data.tx_1[3] = 0x01;
      u16 tmp = MODBUS_CRC(Send_Data.tx_1,SEND_DATA_SIZE_1-2);
      Send_Data.tx_1[4] = tmp >> 8;
      Send_Data.tx_1[5] = tmp;
      try
      {
        // if(Receive_Data.Flag_Stop==0) 
        //while(1)
        {
          sensor_Serial.write(Send_Data.tx_1,SEND_DATA_SIZE_1); //向串口发数据 
          //sensor_Serial.read(receive_data.led_back_flag,sizeof(receive_data.led_back_flag));
          sensor_Serial.read(receive_data.led_back_flag,14);

          *(receive_data.led_back_flag+14) = 0;
          printf("%s\n",receive_data.led_back_flag);
          memset(receive_data.led_back_flag,0,20);
          // if(receive_data.led_back_flag=="Speci_success")
          //     break;
        }
        //ROS_INFO_STREAM("New control command");//显示受到了新的控制指令  
      }
      catch (serial::IOException& e)
      {
        ROS_ERROR_STREAM("Unable to send data through serial port"); //如果try失败,打印错误信息
      }
  }
  else
  {
      ROS_INFO("LED control error!!!");
  }
}
void sensor_serial::Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux)
{
  geometry_msgs::Twist xiaofu_controller;
  //如果得不到传感器数据,就让机器人停止
  if (true == Get_Sensor_Data())
  {
      //move_base传来线速度为正,根据传感器进行二次封装
      if(twist_aux.linear.x>0)
      {
        //判断前面是否有障碍物
        if(sensor_data.Wave[0]>10&&sensor_data.Wave[1]>10&&sensor_data.Wave[2]>10)
        {
          xiaofu_controller.linear.x = twist_aux.linear.x;
          xiaofu_controller.linear.y = twist_aux.linear.y;
          xiaofu_controller.angular.z = twist_aux.angular.z;
          cmd_vel_publisher.publish(xiaofu_controller);
          ROS_INFO("up,up");
        }
        else
        {
          //判断后面是否有障碍物
          if(sensor_data.Wave[3]>10&&sensor_data.Wave[4]>10&&sensor_data.Wave[5]>10)
          {
            xiaofu_controller.linear.x = -twist_aux.linear.x;
            xiaofu_controller.linear.y = twist_aux.linear.y;
            xiaofu_controller.angular.z = 0;
            cmd_vel_publisher.publish(xiaofu_controller);
            ROS_INFO("up,back");
          }
          else
          {
            xiaofu_controller.linear.x = 0;
            xiaofu_controller.linear.y = 0;
            xiaofu_controller.angular.z = 0;
            cmd_vel_publisher.publish(xiaofu_controller);
            ROS_INFO("up,stop");
          }
        }
      }
      //move_base传来线速度为负,根据传感器进行二次封装
      else
      {
        //后面没有障碍物
        if(sensor_data.Wave[3]>10&&sensor_data.Wave[4]>10&&sensor_data.Wave[5]>10)
        {
          xiaofu_controller.linear.x = twist_aux.linear.x;
          xiaofu_controller.linear.y = twist_aux.linear.y;
          xiaofu_controller.angular.z = 0;
          cmd_vel_publisher.publish(xiaofu_controller);
          ROS_INFO("back,back");
        }
        //后面有障碍物
        else
        {
          //检测前面有没有障碍物
          if(sensor_data.Wave[0]>10&&sensor_data.Wave[1]>10&&sensor_data.Wave[2]>10)
          {
            xiaofu_controller.linear.x = -twist_aux.linear.x;
            xiaofu_controller.linear.y = twist_aux.linear.y;
            xiaofu_controller.angular.z = 0;
            cmd_vel_publisher.publish(xiaofu_controller);
            ROS_INFO("back,up");
          }
          else
          {
            xiaofu_controller.linear.x = 0;
            xiaofu_controller.linear.y = 0;
            xiaofu_controller.angular.z = 0;
            cmd_vel_publisher.publish(xiaofu_controller);
            ROS_INFO("back,stop");
          }
        }
      }
  }
  else
  {
    xiaofu_controller.linear.x = 0;
    xiaofu_controller.linear.y = 0;
    xiaofu_controller.angular.z = 0;
    cmd_vel_publisher.publish(xiaofu_controller);
    sensor_error_count++;
    if(sensor_error_count == 10)
      {
        ROS_INFO("xiaofu speed:%f,%f,%f",xiaofu_controller.linear.x,xiaofu_controller.linear.y,xiaofu_controller.angular.z);
        ROS_INFO("sensor_data error,stop running!!!!");
        sensor_error_count = 0;
      }
    }
  }

/**************************************
Date: June 29, 2020
Function: 串口通讯校验函数，数据包除最后一个字节，其他的全部数据按位异或的结果作为帧尾
***************************************/
u8 sensor_serial::Check_CRC(u8 *data, int size) //CRC 校验
{
    if(MODBUS_CRC(data,size-2)==(((u16)data[size-2]<<8)|data[size-1]))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
u16 sensor_serial::MODBUS_CRC(u8 *buff,u8 len) //modbus 协议的 crc 校验
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
bool sensor_serial::Get_Sensor_Data()
{ 
  short transition_16=0,j=0,Header_Pos=0,Tail_Pos=0;  //中间变量
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE]={0};
  sensor_Serial.read(Receive_Data_Pr,sizeof (Receive_Data_Pr));//读串口数据
  for(j=0;j<21;j++)
    {
    if(Receive_Data_Pr[j]==FRAME_HEADER)
    Header_Pos=j;
    else if(Receive_Data_Pr[j]==FRAME_TAIL)
    Tail_Pos=j;    
    }
    //ROS_INFO("%x-%x",Header_Pos,Tail_Pos);
    if(Tail_Pos==(Header_Pos+20))
    {
         //ROS_INFO("1----");
      memcpy(receive_data.rx, Receive_Data_Pr, sizeof(Receive_Data_Pr));
    }
    else if(Header_Pos==(1+Tail_Pos))
    {
        //ROS_INFO("2----");
        for(j=0;j<21;j++)
        receive_data.rx[j]=Receive_Data_Pr[(j+Header_Pos)%21];
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
  receive_data.Frame_Header= receive_data.rx[0]; //数据的第一位是帧头（固定值）
  receive_data.Frame_Tail= receive_data.rx[20];  //数据的最后一位是帧尾（固定值）

 if (receive_data.Frame_Header == FRAME_HEADER )//判断帧头
  {
    if (receive_data.Frame_Tail == FRAME_TAIL) //判断帧尾
    {
      //补充CRC校验
      if(Check_CRC(receive_data.rx, 20))
      { 
        sensor_data.Wave[0] = shorttofloat(receive_data.rx[1],receive_data.rx[2]);
        sensor_data.Wave[1] = shorttofloat(receive_data.rx[3],receive_data.rx[4]);
        sensor_data.Wave[2] = shorttofloat(receive_data.rx[5],receive_data.rx[6]);
        sensor_data.Wave[3] = shorttofloat(receive_data.rx[7],receive_data.rx[8]);
        sensor_data.Wave[4] = shorttofloat(receive_data.rx[9],receive_data.rx[10]);
        sensor_data.Switch =  shorttofloat(0,receive_data.rx[11]);
        sensor_data.Temp =  shorttofloat(receive_data.rx[12],receive_data.rx[13]);
        sensor_data.Humidity = shorttofloat(receive_data.rx[14],receive_data.rx[15]);
        sensor_data.PM_25 =  shorttofloat(receive_data.rx[16],receive_data.rx[17]);
        return true;

     }
    }
  } 
 return false;
}

float sensor_serial::shorttofloat(uint8_t Data_High,uint8_t Data_Low)
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
Function: 构造函数, 只执行一次，用于初始化
***************************************/
sensor_serial::sensor_serial():Sampling_Time(0)
{
  memset(&sensor_data, 0, sizeof(sensor_data)); //构造函数初始化
  ros::NodeHandle private_nh("~");

  private_nh.param<std::string>("usart_port_name", usart_port_name, "/dev/ttyUSB0"); //固定串口
  private_nh.param<int>("serial_baud_rate", serial_baud_rate, 115200); //和下位机底层波特率115200 不建议更高的波特率了
  ROS_INFO_STREAM("Data ready");//ready显示状态
  Cmd_Vel_Sub = n.subscribe("/cmd_vel", 100, &sensor_serial::Cmd_Vel_Callback, this);
  Robot_Led_Sub = n.subscribe("/Robot_Led", 100, &sensor_serial::Robot_Led_Callback, this);
  cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/xiaofu_robot/cmd_vel", 20);
  //初始化串口
  try{
         sensor_Serial.setPort(usart_port_name);//选择哪个口，如果选择的口没有接串口外设初始化会失败
         sensor_Serial.setBaudrate(serial_baud_rate);//设置波特率
         serial::Timeout _time = serial::Timeout::simpleTimeout(2000);//超时等待
         sensor_Serial.setTimeout(_time);
         sensor_Serial.open();//串口开启
    }
  catch (serial::IOException& e){
     ROS_ERROR_STREAM("sensor_node can not open serial port,Please check the serial port cable! ");//如果try失败，打印错误信息
  }
  if(sensor_Serial.isOpen()){
    ROS_INFO_STREAM("sensor_node serial port opened");//开启成功
  }else{
  }
}
/**************************************
Date: May 31, 2020
Function: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
sensor_serial::~sensor_serial()
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
  sensor_Serial.close();//关闭串口
  ROS_INFO_STREAM("Shutting down");//close
}