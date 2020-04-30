#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_US_publisher");
    ros::NodeHandle nh;
    tf::TransformListener listener(ros::Duration(10));
    sensor_msgs::LaserScan laserscan;
    ros::Publisher pub_scan = nh.advertise<sensor_msgs::LaserScan>("scan",1000);
    ros::Rate loop_rate(10);

    tf::TransformListener listen_left;
    tf::TransformListener listen_right;
    tf::TransformListener listen_front;
    tf::StampedTransform A;
    tf::StampedTransform B;
    // tf::StampedTransform C;
    tf::StampedTransform D;
    tf::StampedTransform E;
    // tf::StampedTransform F;
    tf::StampedTransform G;
    tf::StampedTransform H;
    // tf::StampedTransform I;

    tf::Stamped<tf::Transform> C,F,I;

    float front =0.0,right=0.0,left=0.0;

    laserscan.header.stamp = ros::Time::now();
    laserscan.header.frame_id = "base_laser";
    laserscan.angle_min = M_PI/4 - 0.05;
    laserscan.angle_max = M_PI/4 + 0.05;
    laserscan.angle_increment = 0.1;
    laserscan.time_increment = 0.0;
    laserscan.range_min = 20; // in mm
    laserscan.range_max = 4000; // in mm

    while( ros::ok() ) 
    
    {   
        listen_front.lookupTransform("US_front","base_link",ros::Time::now(),B);
        listen_front.lookupTransform("US_front_view","US_front",ros::Time::now(),A);

        C.setData(B*A);

        front = C.getOrigin().x();

        listen_front.lookupTransform("US_right","base_link",ros::Time::now(),E);
        listen_front.lookupTransform("US_right_view","US_right",ros::Time::now(),D);

        F.setData(E*D);

        right = F.getOrigin().x();

        listen_front.lookupTransform("US_left","base_link",ros::Time::now(),H);
        listen_front.lookupTransform("US_left_view","US_left",ros::Time::now(),G);
        
        I.setData(H*G);

        left = I.getOrigin().x();

        // double ranges[3] = {front,right,left};
        // for (int i =0;i<3;i++)
        laserscan.ranges[0] = front;
        laserscan.ranges[1] = left;
        laserscan.ranges[2] = right;
        laserscan.header.stamp = ros::Time::now();
        pub_scan.publish(laserscan);
    }
return 0;

}