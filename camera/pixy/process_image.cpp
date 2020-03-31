#include "ros/ros.h"
#include "JACS/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include "JACS/cen_pos.h"
#include "JACS/trackerfeedback.h"


JACS::trackerfeedback feedbackmsg; // Global declaration of trackerFeedback

void process_image_callback(const JACS::cen_pos centroid)
{

    
    feedbackmsg.stamp = ros::Time::now();
    feedbackmsg.Y = false;
    feedbackmsg.P = false;
    

    
    int height = 400, width = 600; //To change 

    if( ( centroid.cenY1[0] >= 0.0 && centroid.cenY1[0] < 150.0 ) && ( centroid.cenY1[1] >= 0.0 && centroid.cenY1[1] <= 400.0 ) )
    {
        centroid.locY1 = true;
    }

    if( ( centroid.cenY2[0] >= 450.0 && centroid.cenY2[0] <= 600 ) && ( centroid.cenY2[1] >= 0.0 && centroid.cenY1[1] <= 400.0 ) )
    {
        centroid.locY2 = true;
    }

    if( ( centroid.cenP[0] >= 0.0 && centroid.cenP[0] <= 600 ) && ( centroid.cenP[1] >= 0.0 && centroid.cenP[1] <= 400.0 ) )
    {
        centroid.locP = true;
    }




    if( centroid.locY1 == true && centroid.locY2 == true && centroid.locP == false )
    {
        feedbackmsg.Y = true;
        feedbackmsg.P = false;
    }
    else if (centroid.locY1 == true && centroid.locY2 == true && centroid.locP == true)
    {
        feedbackmsg.Y = true;
        feedbackmsg.P = true;
    }
    else 
    {
        feedbackmsg.Y = true;
        feedbackmsg.P = true;
    }


}

int main(int argc, char** argv)
{

    feedbackmsg.Y = false;
    feedbackmsg.P = false;
    //Initialize the process_image node and creating anode handler to publish a BOOL yes 

    //Initialized the Camera Tracking Node

    ros::init(argc, argv, "camera_tracking");
    ros::Rate loop_rate(0.5); // 2 Messages per second
    ros::Publisher tracker_publisher = n.advertise<JACS::trackerfeedback>("Pixycam_feedback",1000);

    ros::initNodeHandle n;

    while (ros::ok) 
    { 
        ros::Subscriber sub1 = n.subscribe("pixy", 10, process_image_callback);      
        tracker_publisher.publish(feedbackmsg);
        loop_rate.sleep();
    }

    // Handles the ROS communication
    ros::spinOnce();
    

    return 0;
}