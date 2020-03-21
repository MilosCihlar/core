#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"

#include <sstream>

#include "m_odometry/odometry.h"

double r_pos_actual = 0;
double r_pos_last = 0;
double l_pos_actual = 0;
double l_pos_last = 0;

double t_time = 0;

enum l_r_value {LEFT_WHEEL, RIGHT_WHEEL};
enum position {X, Y, Theta};

struct Quaternion
{
    double  x, y, z, w;
};

Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}


void jointCallback(const sensor_msgs::JointState state)
{
    r_pos_last = r_pos_actual;
    r_pos_actual = state.position[RIGHT_WHEEL];

    l_pos_last = l_pos_actual;
    l_pos_actual = state.position[LEFT_WHEEL];
    t_time  = state.header.stamp.toSec();             // mozna predelat protoze bych mel brat cas z message ktera publikuje stav kola ne nejaky nahodny ktery zjistim az mi prijde ta zprava ...
}
 

int main(int argc, char **argv)
{
    // check input parameters
    ros::init(argc, argv, "m_odometry");
    ros::NodeHandle n;

    double rate = 0;
    double r = 0;
    double span = 0;
    bool simul = false;
    std::string world_frame;
    std::string robot_name;

    n.getParam("m_odometry/freq", rate);
    n.getParam("m_odometry/radius", r);
    n.getParam("m_odometry/span", span);
    n.getParam("m_odometry/simul", simul);
    n.getParam("m_odometry/world_frame", world_frame);
    n.getParam("m_odometry/robot_name", robot_name);

    double x;
    double y;
    double z;
    n.getParam("m_odometry/x", x);
    n.getParam("m_odometry/y", y);
    n.getParam("m_odometry/z", z);

    // publishers
    ros::Publisher pub_odom = n.advertise<nav_msgs::Odometry>("pub_odometry", 1);
    // subscribers 
    ros::Subscriber sub_joint = n.subscribe("sub_joint_states", 1, jointCallback);


    ros::Rate loop_rate(rate);

    double pos[3];
    pos[0] = x;
    pos[1] = y;
    pos[2] = z;
    double vel[3];

    nav_msgs::Odometry msg_odometry;
    Odometry odom(x, y, z, r, span);

    msg_odometry.header.frame_id = world_frame;
    Quaternion quaternion;

    tf::TransformBroadcaster broadcaster;
    tf::Transform transform;
    

    while (n.ok())
    {
        
        if(!(t_time == odom.getTime()))
        {
            odom.calculateOdometry(l_pos_last, l_pos_actual, r_pos_last, r_pos_actual, t_time);
            odom.getPosition(pos);
            odom.getVelocity(vel);
    
            msg_odometry.pose.pose.position.x = pos[X];
            msg_odometry.pose.pose.position.y = pos[Y];
            msg_odometry.pose.pose.position.z = 0;

            quaternion = ToQuaternion(pos[Theta], 0, 0);
            msg_odometry.pose.pose.orientation.x = quaternion.x;
            msg_odometry.pose.pose.orientation.y = quaternion.y;
            msg_odometry.pose.pose.orientation.z = quaternion.z;
            msg_odometry.pose.pose.orientation.w = quaternion.w;

            msg_odometry.twist.twist.linear.x = vel[X];
            msg_odometry.twist.twist.linear.y = vel[Y];
            msg_odometry.twist.twist.angular.z = vel[Theta];

            msg_odometry.header.seq = odom.getNumber();

            pub_odom.publish(msg_odometry);
        }

        if(!simul)
        {
            transform.setOrigin(tf::Vector3(pos[X], pos[Y], 0.0));
            transform.setRotation( tf::Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w));
            broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame, robot_name));
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
};


