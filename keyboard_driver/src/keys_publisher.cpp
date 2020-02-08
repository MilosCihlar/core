#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>


#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "keyboard");
  ros::NodeHandle n;

  ros::Publisher pub_velocity_cmd = n.advertise<geometry_msgs::Twist>("pub_velocity_cmd", 1);

  struct termios oldSettings, newSettings;
  tcgetattr( fileno( stdin ), &oldSettings );
  newSettings = oldSettings;
  newSettings.c_lflag &= (~ICANON & ~ECHO);
  tcsetattr( fileno( stdin ), TCSANOW, &newSettings );

  char c;
  double x_vel = 0;
  n.getParam("key_driver/x_vel", x_vel);
  double z_ang = 0;
  n.getParam("key_driver/z_ang", z_ang);
  
  geometry_msgs::Twist velocity;

  double rate = 0;
  n.getParam("key_driver/freq", rate);
  ros::Rate loop_rate(rate);

  std::cout << "Teleop twist Keyboard" << std::endl;
  std::cout << "---------------------" << std::endl;
  std::cout << "          w          " << std::endl;
  std::cout << "       a  s  d       " << std::endl;
  std::cout << "---------------------" << std::endl;
  std::cout << "q/e Increase/Decrease linear - x velocity by 10% (default "<< x_vel << ")"<< std::endl;
  std::cout << "x/c Increase/Decrease linear - y velocity by 10% (default "<< z_ang << ")"<< std::endl;
  std::cout << " x  Exit" << std::endl;
  std::cout << "You can telep your robot" << std::endl;

  while(ros::ok())
  {
    fd_set set;
    struct timeval tv;
    tv.tv_sec = 10000000000;
    tv.tv_usec = 0;
    FD_ZERO(&set);
    FD_SET( fileno(stdin), &set );
    int res = select( fileno( stdin )+1, &set, NULL, NULL, &tv);


    if( res > 0 )
    {
        read( fileno(stdin), &c, 1);
        
        switch (c)
        {
        case 'w':
          if (velocity.linear.x == -x_vel)   
          {
            velocity.linear.x = 0;
          }
          else
          {
            velocity.linear.x = x_vel;
          }
          
          break;
    
        case 's':
          if ((velocity.linear.x == x_vel)  || (velocity.angular.z =! 0) )
          {
            velocity.linear.x = 0;
            velocity.angular.z = 0;
          }
          else
          {
            velocity.linear.x = -x_vel;
          }
          
          break;
        case 'a':
          if (velocity.angular.z < 0)
          {
            velocity.angular.z = 0;  
          }
          else
            velocity.angular.z = z_ang;
   
          break;
        case 'd':
          if (velocity.angular.z > 0)
          {
            velocity.angular.z = 0;  
          }
          else
            velocity.angular.z = -z_ang;
          break;
        case 'q':
            x_vel = x_vel + x_vel*0.1;
            std::cout<<"x-linear " << x_vel << std::endl;
          break;
        
        case 'e':
            x_vel = x_vel - x_vel*0.1;
            std::cout<<"x-linear " << x_vel << std::endl;
          break;
        case 'x':
            z_ang = z_ang + z_ang*0.1;
            std::cout<<"y-linear " << z_ang << std::endl;
          break;
        case 'c':
            z_ang = z_ang - z_ang*0.1;
            std::cout<<"y-linear " << z_ang << std::endl;
          break;
        
        default:
          break;
        }             

    }
    else
    {
        std::cout << "Unexpected error in keys_publisher" << '\n';
        break;
    }

    pub_velocity_cmd.publish(velocity);

    ros::spinOnce();
    loop_rate.sleep();

  }

  tcsetattr( fileno( stdin ), TCSANOW, &oldSettings );
  return 0;
}
