#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmvision_3d/Blobs3d.h>
#include <math.h>

ros::Publisher turtle_vel;


void trackCallback(const cmvision_3d::Blobs3d::ConstPtr& color){

    // no detected blob
    if(color->blob_count == 0) {
        geometry_msgs::Twist spin;

        //spin.angular.z = 1.57;
        turtle_vel.publish(spin);

        ROS_INFO("SPINING\n");
        return;
    }

    // there exists detected blobs
    ROS_INFO("Tracking\n");
    double threshold_distance = 0.5;
    geometry_msgs::Twist cmd_vel;
    
    for(int i=0; i<color->blob_count; i++){
        if(color->blobs[i].center.z > threshold_distance+0.1){
            if(color->blobs[i].area > 100){

                double scale_depth = 0.2;
                double scale_x = 0.005;
                double depth = color->blobs[i].center.z;
                double depth_diff = depth - threshold_distance;
                int x = color->blobs[i].center.x;
                int x_diff = x - 320;
                
                cmd_vel.angular.z = -scale_x * x_diff;
                cmd_vel.linear.x = scale_depth * depth_diff;
                turtle_vel.publish(cmd_vel);
            }
        }
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "color_tracker");

    ros::NodeHandle node;

    //ros::Rate loop_rate(10);

    turtle_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    ros::Subscriber sub = node.subscribe("blobs_3d", 3, &trackCallback);
    
    ros::spin();

    return 0;

};