//2015.9.9 gong_tarsbot

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

geometry_msgs::Twist line_follower_vel;
char kb=0;
int high=0,k=0;
bool l=0,h=0,into=0,out=0,flag_1=1,flag_2=0,flag_3=0,flag_4=0;
double odom_direction=0.0,pre_odom_direction=0.0,error_direction=0.0,target_direction=0.0;
double odom_x=0.0,pre_odom_x=0.0,my_x=0.0,odom_y=0.0,pre_odom_y=0.0,my_y=0.0,translation=0.0;
double angle=0.0;
float angle_tem=0.0;
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
  
 // ROS_INFO("odom_direction=%f",odom_direction);
  //odom_direction*=RADIAN;
  
 /*
  if ((odom_direction*pre_odom_direction)>0)                               
     error_direction=odom_direction-pre_odom_direction;                     //角度偏量
  else if ((odom_direction>0)&&(pre_odom_direction<0))
     error_direction=3.1416-(odom_direction-pre_odom_direction);
  else if ((odom_direction<0)&&(pre_odom_direction>0))
     error_direction=3.1416-(pre_odom_direction-odom_direction); 
      
  pre_odom_direction=odom_direction;
  odom_direction+=error_direction;     //实际的转化航向
  odom_direction=0;
 
  if(odom_direction>=6.28)
       odom_direction-=6.28;
  ROS_INFO("odom_direction=%f",odom_direction);
 */

} 





/*****Aduino_data_solve*****/

void Aduino_data_solve(const std_msgs::String::ConstPtr& ir_line_scan)

{
 char a[8];
 int i,j,n,switch_state=0,t=5000;
 double b[8]={0,0,0,0,0,0,0,0};
 float pid_angular_z=0;
 strcpy(a,ir_line_scan->data.c_str());   //led数据传达
 for (i=0;i<8;i++)
   {
    //b[i]=((a[i]-48)/20.0);               //数据翻译  D的ascII是68
    if (a[i]=='D') 
    {                         //信号判断
      k++;
      b[i]=1;
    }                                 //led触发数目
   
    //ROS_INFO("b[%d]=%f",i,b[i]);

   }
  
 // ROS_INFO("k=%d",k);   
   

 
 
 switch (k)

    {
     case 0:
     case 8:
           angle=0.0;
           switch_state=0;
           break;
        
     case 1:  
     case 2:
     case 3: //ROS_INFO("angle-----0:%f",angle);
            for (j=0;j<8;j++)
              angle+=(b[j]*(3.5-j)*each_angle);
             // ROS_INFO("angle-1:%f",angle);
              

            angle=angle/k;
            switch_state=1;  
            break;                      
   
     case 4:
     case 5:
     case 6:
     case 7: 
     
             //if(k>3)            //里程计算法
             

               if(flag_4==1)
    
                   {
            
                     angle_tem=-odom_direction;           //    0 
                     ROS_INFO("angle_tem4:%f",angle_tem);
                     ROS_INFO("flag_4==1");
                     
                    if ((odom_direction>-0.047)&&(odom_direction<0.047))
                     {
                       flag_4=0;
                       flag_1=1;
                     }
                     
                     else
                     {
                     
                       flag_4=1;
                       flag_1=0;  
                     
                     }
                     
                     
                     

                   }

                else if(flag_3==1)
    
                   {
         
                     angle_tem=-1.067-odom_direction;     //   5.233=5*3.14/3 
                     ROS_INFO("angle_tem3:%f",angle_tem);
                     ROS_INFO("flag_3==1");
                     
                     if ((odom_direction>-1.087)&&(odom_direction<-1.047))
                         
                      {
                       flag_3=0;
                       flag_4=1;
            
                      }
                     
                     else
                     {
                     
                       flag_3=1;
                       flag_4=0;  
                     }
          
                     
                     
          

                   }

                 else if(flag_2==1)              
    
                  {
                     
                     angle_tem=-2.134-odom_direction;     //    6.28=2*3.14
                     ROS_INFO("odom_direction:%f",odom_direction);
                     ROS_INFO("flag_2==1");
                     
                       if ((odom_direction>-2.3)&&(odom_direction<-1.9))
                     {
                       flag_2=0;
                       flag_3=1;
                    
                     }
                     
                     else
                     {
                     
                       flag_2=1;
                       flag_3=0;  
                      
                     }
                     
                    
                        
                   }

                 else if(flag_1==1)              
    
                   {
              
                     angle_tem=1.067-odom_direction;     //   1.067=3.14\3
                     ROS_INFO("odom_direction:%f",odom_direction);
                     ROS_INFO("flag_1==1");
                     
                     if ((odom_direction>0.5)&&(odom_direction<1.5))
                     {
                       flag_1=0;
                       flag_2=1;
                      
                     }
                     
                     else
                      {
                       
                       flag_1=1;
                       flag_2=0;  
                      
                      }
                     
                     
                    }
      
   
   
            angle=angle_tem*1;
            switch_state=1;
            break; 

     
    }

    
     
  pid_angular_z=PID_realize(angle);
  
  //ROS_INFO("angle:%f",angle);
  angle=0;
  k=0;


    if (switch_state>0)
  {
    line_follower_vel.linear.x=-0.1;  
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

int main(int argc, char** argv)
{
  
  ros::init(argc,argv,"g_line_follow_1");   
  ros::NodeHandle nh;

  ros::Subscriber sub_1 = nh.subscribe("/odom",1,angle_solve_YAW);     //方向
  ros::Subscriber sub_2 = nh.subscribe("/ir_line_scanner",1,Aduino_data_solve);
  ros::Publisher velocity_command_publisher = nh.advertise< geometry_msgs::Twist >("/mobile_base/commands/velocity", 20);    //电机驱动 
   
  PID_init();
  ros::Rate loop_rate(20);
  while (ros::ok())
  {
    ros::spinOnce();
    velocity_command_publisher.publish(line_follower_vel);
    loop_rate.sleep();
  }
  ros::spin();
  return 0;

}

              

















              














