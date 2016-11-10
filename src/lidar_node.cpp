#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

ros::Publisher *velocity_data_ptr;
float velocity;
int angle_threshold = 10;
float angle_min;
float angle_increment;
std::vector <float> ranges;
float minimum;
int zero_index;
void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    angle_min = msg->angle_min;
    angle_increment = msg->angle_increment;
    zero_index = -angle_min / angle_increment;
    ranges = msg->ranges;
    minimum = 20000;
    int start_index = zero_index - (angle_threshold/2)/angle_increment;
    int end_index = zero_index + (angle_threshold/2)/angle_increment;
    for(int i = start_index; i < end_index ; i++){
        if (ranges[i] < minimum){
                    minimum = ranges[i];

                    //std::cout << minimum<< std::endl;
            }
    }

    std::cout << "----- indices ----" << std::endl;
    std::cout << start_index << std::endl;
    std::cout << end_index << std::endl;


    velocity = (minimum - 0.5)*255;
    if (velocity < 0){
        velocity = 0;
    }
    if (velocity > 255){
        velocity = 255;
    }



    //ROS_INFO("I heard: [%s]", msg->data.c_str());


    //(*velocity_data_ptr).publish(velocity);

    std_msgs::Float32 msg_out;
    msg_out.data = velocity;

    std::cout << velocity << std::endl;
    velocity_data_ptr->publish(msg_out);

}

int main(int argc, char **argv)
{
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "listener");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The subscribe() call is how you tell ROS that you want to receive messages
     * on a given topic.  This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing.  Messages are passed to a callback function, here
     * called chatterCallback.  subscribe() returns a Subscriber object that you
     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
     * object go out of scope, this callback will automatically be unsubscribed from
     * this topic.
     *
     * The second parameter to the subscribe() function is the size of the message
     * queue.  If messages are arriving faster than they are being processed, this
     * is the number of messages that will be buffered up before beginning to throw
     * away the oldest ones.
     */
    ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);
    ros::Publisher velocity_data = n.advertise<std_msgs::Float32>("velocity_data", 1000);
    velocity_data_ptr = &velocity_data;

    /**
     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
     * callbacks will be called from within this thread (the main one).  ros::spin()
     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    ros::spin();

    return 0;
}