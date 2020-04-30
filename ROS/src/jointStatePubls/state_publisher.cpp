#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/UInt16.h>
#include <math.h>
#include <geometry_msgs/Twist.h>


/* TO DO ADD THE ORIENTATION OF TURRET TO MATCH FOR JOINT UPDATE OF TURRET */




/* ROBOT SPECIFICATION*/
//Dn = Nominal Wheel Diameter, Cn = Encoder Resolution
double Dn = 101.6, Ce = 64 ,n =131.25;
double cm = Dn/(n*Ce); // Conversion factor of Encoder Pulses to Linear Wheel Displacement
double b = 254; // Wheel Base
double dU_left=0.0, dU_right =0.0, dU =0.0, d_angle=0.0;
double encoder_left= 0.0, encoder_right = 0.0;




void publishcmd_vel( double v_rx , double omega_r, double v_ry = 0.0)
{
    geometry_msgs::Twist msg;
    msg.linear.x = v_rx;
    msg.linear.y = v_ry;
    msg.linear.z = 0.0;

    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = omega_r;

}



void callback_left (const std_msgs::UInt16& msg)
{
    encoder_left = msg.data;
}

void callback_right (const std_msgs::UInt16& msg)
{
    encoder_right = msg.data;
}

// int main(int argc, char** argv) {

int main(int argc, char** argv) {

    
    ros::init(argc,argv,"state_publisher");

    ros::NodeHandle nh;
 
    ros::Subscriber sub_left = nh.subscribe("encoder/left",1000,callback_left);
    ros::Subscriber sub_right = nh.subscribe("encoder/right",1000,callback_right);
    

    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states",1);
    // ros::Publisher pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);
    geometry_msgs::Twist msg;
    
    
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // Robot State
    double lw_joint = 0, rw_joint = 0, tinc = degree, turret_angle = 0, angle = 0 ;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = 0.0;
    odom_trans.transform.translation.y = 0.0;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);

    while (ros::ok()) {
        //Set Encoder Data
        
        ros::Subscriber sub_left = nh.subscribe("encoder/left",1000,callback_left);
        ros::Subscriber sub_right = nh.subscribe("encoder/right",1000,callback_right);
        

        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(4);
        joint_state.position.resize(4);
        joint_state.name[0] = 'left_wheel_joint';
        joint_state.position[0] = lw_joint;
        joint_state.name[1] = "right_wheel_joint";
        joint_state.position[1] = rw_joint;
        joint_state.name[2] = "turret_joint";
        joint_state.position[2] = turret_angle;
        joint_state.name[3] = "gun_joint";
        joint_state.position[3] = angle;

        // update transform
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x += ( dU * cos(angle + d_angle/2)*2 );
        odom_trans.transform.translation.y +=  ( dU * sin(angle + d_angle/2)*2 );
        odom_trans.transform.translation.z = 0.7;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle);

        // Send the joint state and transform
        joint_pub.publish(joint_state);
        // static tf::TransformBroadcaster br;
        tf2_ros::TransformBroadcaster broadcaster;
        broadcaster.sendTransform(odom_trans);

        // Do Incrementing Business Here
        dU_left = cm * encoder_left;
        dU_right = cm * encoder_right;
        dU = (dU_left + dU_right)/2.0;
        d_angle = (dU_right - dU_left)/b;
        //Imp step update
        angle += d_angle;

        // Publishing Twist message to topic cmd_vel
        
        msg.linear.x = dU;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = d_angle;
        // publishcmd_vel(dU, d_angle);
        // pub_cmd_vel.publish(msg);
        // TO DO IS TO CREATE THE JOINT UPDATE STEP FOR TURRET MOTORS


        loop_rate.sleep();
        
    }
    return 0;
}