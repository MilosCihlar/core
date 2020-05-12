    #include "ros/ros.h"
    #include <tf/transform_broadcaster.h>
    #include "sensor_msgs/JointState.h"
    #include "nav_msgs/Odometry.h"
    #include "nav_msgs/Path.h"
    #include "math.h"

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

    double GX, GY, GT, GTlast = 0;
    Quaternion GQuaternion;
    bool GFlag = false;



    double ToTheta(Quaternion q)
    {
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
    }

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

    void trajectoryCallback(const nav_msgs::Path& path)
    {
    GTlast = GT;
    geometry_msgs::PoseStamped p = path.poses.back();
    GX = p.pose.position.x;
    GY = p.pose.position.y;
    GQuaternion = {p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w};
    GT = ToTheta(GQuaternion);

    GFlag = true;

    }

    double Modulo(const double p, const double m) 
    {
    return std::fmod((std::fmod(p,m) + m) , m);
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
    ros::Subscriber s_traj = n.subscribe("s_trajectory", 1, trajectoryCallback);


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

            if(GFlag)
            {            
                double dir = Modulo(GT-pos[Theta] + M_PI, 2*M_PI)-M_PI;

                pos[Theta] = 0.99*pos[Theta] + 0.01*(pos[Theta]+dir);

                double xspeed = odom.getLinearSpeed()*cos(pos[Theta]);
                double yspeed = odom.getLinearSpeed()*sin(pos[Theta]);
                odom.setXSpeed(xspeed);
                odom.setYSpeed(yspeed);
                odom.setTheta(pos[Theta]);
                vel[X] = xspeed;
                vel[Y] = yspeed;
                
            }

            msg_odometry.header.frame_id = world_frame;
            msg_odometry.child_frame_id = robot_name;
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



        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
    };


