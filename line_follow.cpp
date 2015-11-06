//2015.9.15 gongwenbo_tarsbot

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#define each_angle 0.1578     //The angle between the lamps 弧度制
#define CTR 2.00              //Centerline length
#define SPACING 1.07          //Vertical spacing
#define RADIUS 0.35           //Target point circle radius
#define RADIAN 57.29577951    //The angle of arc  360/（2×3.14）  弧度制转换 

geometry_msgs::Twist line_follower_vel;      //消息参数定义
char kb=0;
int high=0,k=0;
bool l=0,h=0,into=0,out=0;
double odom_direction=0.0,pre_odom_direction=0.0,error_direction=0.0,target_direction=0.0;
double odom_x=0.0,pre_odom_x=0.0,my_x=0.0,odom_y=0.0,pre_odom_y=0.0,my_y=0.0,translation=0.0;
double angle=0.0,drection_flag=0.0;
float angle_tem=0.0;
int f=0,flag_1=1,flag_2=0,flag_3=1,flag_4=0;


/*****PID control*****/
struct _pid                    //PID数据结构
{
  float SetSpeed;
  float ActualSpeed;
  float err;
  float err_last;
  float Kp,Ki,Kd;
  float voltage;
  float integral;
}pid;

void PID_init()                //PID初始化
{
  ROS_INFO("PID_init begin");
  pid.SetSpeed=0.0;
  pid.ActualSpeed=0.0;
  pid.err=0.0;
  pid.err_last=0.0;
  pid.voltage=0.0;
  pid.integral=0.0;
  pid.Kp=0.3;   //0.3,0.6,0.05
  pid.Ki=0.6;
  pid.Kd=0.05;
  ROS_INFO("PID_init end");
}

float PID_realize(float speed)  // PID实现
{

  pid.SetSpeed=speed;
  pid.err=pid.SetSpeed-pid.ActualSpeed;
  pid.integral+=pid.err;
  pid.voltage=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);   //PID公式
  pid.err_last=pid.err;
  pid.ActualSpeed=pid.voltage*1.0;

  return pid.ActualSpeed;
}
/*****This part of the end*****/



/*****line_follow control*****/

void angle_solve_YAW(const nav_msgs::Odometry::ConstPtr& odom_msg)

{
 
  odom_direction=tf::getYaw(odom_msg->pose.pose.orientation);       //陀螺仪方向

} 


double odom_solve(double angle_odom)                               //将陀螺仪（0～3.14）和（-3.14~0）转化为（0～6.28）
     {

     if (angle_odom<0)
      angle_odom=6.28+angle_odom;
     else
      angle_odom=angle_odom; 
     return angle_odom;

     }


/*****Aduino_data_solve*****/

void Aduino_data_solve(const std_msgs::String::ConstPtr& ir_line_scan)   //处理arduino数据

{
 char a[8];
 int i,j,n,switch_state=0,t=5000;
 double b[8]={0,0,0,0,0,0,0,0},speed_kobuki=0.0;
 float pid_angular_z=0;
 strcpy(a,ir_line_scan->data.c_str());                                 //led数据传达
 for (i=0;i<8;i++)
   {
    //b[i]=((a[i]-48)/20.0);                                          //数据翻译  D的ascII是68
    if (a[i]=='D') 
    {                                                                //信号判断，k为led触发数目个数
      k++;
      b[i]=1;
      ROS_INFO("k=%d",k); 
    }                                 
   

   }
  
 
 if (k<8&&k>3&&(flag_1==1))                                          //分差判断
     {
       flag_1=0;                                                     //分叉标识位
       flag_2=1;                                                     //分叉航向标志位
       flag_3=0;                                                     //分叉航向标志位比下一次k<4直线航向优先选择位
       drection_flag=odom_solve(odom_direction);
       f++;                                                          //当f为1或2正转否则反转
       if (f==5)  f=1;
       ROS_INFO("f=%d",f);  

       ROS_INFO("flag_11=%d",flag_1);                                //参数打印，方便调试；
       ROS_INFO("flag_12=%d",flag_2);
       ROS_INFO("flag_13=%d",flag_3);

      }


 if (flag_2==1)
      {
       if ((odom_solve(odom_direction)-drection_flag)*(odom_solve(odom_direction)-drection_flag)>0.75)     //分叉航向目标校准  0.75（经验值）转角的平方
          {
            flag_1=1;
            flag_2=0;
            flag_3=1;

            ROS_INFO("flag_21=%d",flag_1);
            ROS_INFO("flag_22=%d",flag_2);
            ROS_INFO("flag_23=%d",flag_3);
           }
        else
           {
             switch_state=1;                                            //机器状态标识位
             speed_kobuki=-0.15;

             if(f<3)
               angle=1.4;                                               //分叉航向角加速度
             else
               angle=-1.4;

            }
          }

  else                                                                 //直线循迹板块
    {
      switch (k)

      {
       case 0:
       case 8:
           switch_state=0;
           break;

       case 1:  
       case 2:
       case 3:
            if (flag_3==1)
            {
              //ROS_INFO("angle-----0:%f",angle);
              for (j=0;j<8;j++)
                 angle+=(b[j]*(3.5-j)*each_angle);
              // ROS_INFO("angle-1:%f",angle);
              angle=(angle/k)*2.0;                                     //角加速度算法
              speed_kobuki=-0.25;
              switch_state=1;
            }  
            break; 
     }
 }


  pid_angular_z=PID_realize(angle);                                   //pid校准
  //ROS_INFO("angle:%f",angle);
  angle=0;
  k=0;                                                                 //k清0


 if (switch_state>0)                                                
   {
    line_follower_vel.linear.x=speed_kobuki;  
    line_follower_vel.angular.z=pid_angular_z;
    //ROS_INFO("pid_angular_z=%f",pid_angular_z);
    //ROS_INFO("Go");
 
   }
 else
   {
    line_follower_vel.linear.x=0.0;
    line_follower_vel.angular.z=0.0;
    //ROS_INFO("Stop");
    
   }
}

/*****This part of the end*****/

int main(int argc, char** argv)                                           //主函数
{
  
  ros::init(argc,argv,"g_line_follow_1");   
  ros::NodeHandle nh;

  ros::Subscriber sub_1 = nh.subscribe("/odom",1,angle_solve_YAW);        //航向采集
  ros::Subscriber sub_2 = nh.subscribe("/ir_line_scanner",1,Aduino_data_solve);             //传感器数据订阅
  ros::Publisher velocity_command_publisher = nh.advertise< geometry_msgs::Twist >("/mobile_base/commands/velocity", 20);    //电机驱动 
   
  PID_init();
  ros::Rate loop_rate(2000);

  while (ros::ok())
   {
    ros::spinOnce();
    velocity_command_publisher.publish(line_follower_vel);
    loop_rate.sleep();
   }
  ros::spin();
  return 0;
}

 
