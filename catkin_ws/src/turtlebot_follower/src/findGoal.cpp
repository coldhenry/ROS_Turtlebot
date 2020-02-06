#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmvision_3d/Blobs3d.h>
#include <sensor_msgs/Image.h>
#include <depth_image_proc/depth_traits.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

ros::Publisher turtle_vel;
ros::Subscriber blobs_sub;
ros::Subscriber depth_sub;

double min_y_ = -0.5;   // The minimum y position of the points in the box.
double max_y_ = 0.5;    // The maximum y position of the points in the box.
double min_x_ = -0.35;  // The minimum x position of the points in the box.
double max_x_ = 0.35;   // The maximum x position of the points in the box.
double max_z_ = 0.55;   // The maximum z position of the points in the box.
int noBlobCount = 0;    // Use to filter noise
bool avoid = false;     // True if obstacles are detected


void imageCallback(const sensor_msgs::ImageConstPtr& depth_msg){
    // Precompute the sin function for each row and column
    uint32_t image_width = depth_msg->width;
    float x_radians_per_pixel = 60.0/57.0/image_width;
    float sin_pixel_x[image_width];
    for (int x = 0; x < image_width; ++x) {
      sin_pixel_x[x] = sin((x - image_width/ 2.0)  * x_radians_per_pixel);
    }

    uint32_t image_height = depth_msg->height;
    float y_radians_per_pixel = 45.0/57.0/image_width;
    float sin_pixel_y[image_height];
    for (int y = 0; y < image_height; ++y) {
      // Sign opposite x for y up values
      sin_pixel_y[y] = sin((image_height/ 2.0 - y)  * y_radians_per_pixel);
    }

    //X,Y,Z of the centroid
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    //Number of points observed
    unsigned int n = 0;

    //Iterate through all the points in the region and find the average of the position
    const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(float);
    for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step)
    {
     for (int u = 0; u < (int)depth_msg->width; ++u)
     {
       float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);
       if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_) continue;
       float y_val = sin_pixel_y[v] * depth;
       float x_val = sin_pixel_x[u] * depth;

       if ( y_val > min_y_ && y_val < max_y_ &&
            x_val > min_x_ && x_val < max_x_)
       {
         x += x_val;
         y += y_val;
         z = std::min(z, depth); //approximate depth as forward.
         n++;
       }
     }
    }

    //If there are points, find the centroid and calculate the command goal.
    //If there are no points, simply publish a stop goal.
    if (n>3000)
    {
        x /= n;
        y /= n;
        if(z > max_z_) ROS_INFO_THROTTLE(1, "Centroid too far away %f, stopping the robot\n", z);
        else {
            // start avoiding obstacle
            avoid = true;
            ROS_INFO_THROTTLE(1, "Centroid at %f %f %f with %d points", x, y, z, n);

            ros::Rate rate(10);
            // turn
            ros::Time start = ros::Time::now();
            ros::Time end = start + ros::Duration(1.9);
            while(ros::Time::now() < end) {
                geometry_msgs::Twist cmd;
                if(x > 0) cmd.angular.z = 1.0;
                else cmd.angular.z = -1.0;
                turtle_vel.publish(cmd);
                rate.sleep();
            }

            // foward
            start = ros::Time::now();
            end = start + ros::Duration(1.4);
            while(ros::Time::now() < end) {
                geometry_msgs::Twist cmd;
                cmd.linear.x = 0.2;
                turtle_vel.publish(cmd);
                rate.sleep();
            }

            // turn back
            start = ros::Time::now();
            end = start + ros::Duration(1.9);
            while(ros::Time::now() < end) {
                geometry_msgs::Twist cmd;
                if(x > 0) cmd.angular.z = -1.0;
                else cmd.angular.z = 1.0;
                turtle_vel.publish(cmd);
                rate.sleep();
            }

            avoid = false;
        }
    }
    else ROS_INFO_THROTTLE(1, "Not enough points(%d) detected, stopping the robot", n);
}

void trackCallback(const cmvision_3d::Blobs3d::ConstPtr& color){
    // if not in avoid stage
    if(!avoid) {
        // no detected blob
        if(color->blob_count == 0) {
            if(noBlobCount >= 20) {
                ROS_INFO("No blobs\n");
                geometry_msgs::Twist spin;

                spin.angular.z = -1.57;
                turtle_vel.publish(spin);
                ros::Rate rate(30);
                rate.sleep();
                noBlobCount = 0;
            }
            else noBlobCount++;
            return;
        }

        // there exists detected blobs
        ROS_INFO("Tracking\n");

        int x = 0;
        double scale_depth = 0.2;
        double scale_x = 0.001;
        double threshold_distance = 0.6;
        double depth = 0;
        double x_diff = 0;
        double depth_diff = 0;
        bool hasFound = false;

        geometry_msgs::Twist cmd_vel;
        noBlobCount = 0;

        for(int i=0; i<color->blob_count; i++){
            // if detected target has area larger than 1000
            // or the target has been seen before
            if(color->blobs[i].area > 1000 || hasFound){
                hasFound = true;

                // if target has seen before
                // then store the parameters to filter noise
                if(color->blobs[i].area > 1000) {
                    x = color->blobs[i].center.x;
                    depth = color->blobs[i].center.z;
                    depth_diff = depth - threshold_distance;
                    x_diff = x - 320;
                }

                // if still not arrive
                if(depth > threshold_distance+0.1){
                    cmd_vel.angular.z = -scale_x * x_diff;
                    if(scale_depth * depth_diff > 0.2)
                        cmd_vel.linear.x = 0.2;
                    else if(scale_depth * depth_diff < 0.1)
                        cmd_vel.linear.x = 0.1;
                    else cmd_vel.linear.x = scale_depth * depth_diff;
                    turtle_vel.publish(cmd_vel);
                    ros::Rate rate(10);
                    rate.sleep();
                }
                else {
                    ROS_INFO("Arrived\n");
                    blobs_sub.shutdown();
                }
            }
        }

        // if no target is found, then spin
        if(!hasFound) {
            geometry_msgs::Twist cmd_vel;

            cmd_vel.angular.z = -0.5;
            turtle_vel.publish(cmd_vel);
            ros::Rate rate(30);
            rate.sleep();
        }
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "color_tracker");

    ros::NodeHandle node;

    turtle_vel = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);

    blobs_sub = node.subscribe("blobs_3d", 3, &trackCallback);
    depth_sub = node.subscribe<sensor_msgs::Image>("/camera/depth/image_rect", 1, &imageCallback);
    
    ros::spin();

    return 0;

};