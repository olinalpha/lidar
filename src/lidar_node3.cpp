////
//// Created by daniel_english on 11/14/16.
////
//
//#include "ros/ros.h"
//#include "sensor_msgs/LaserScan.h"
//#include "std_msgs/Float32MultiArray.h"
//#include <math.h>
//
//#define PI 3.1415926535
//
///**
// * This tutorial demonstrates simple receipt of messages over the ROS system.
// */
//
//ros::Publisher *velocity_data_ptr;
//float robotLength = 0.5; // meters
//float robotLengthTolerance = 0.1; //meters
//int turnScaleFactor = 2;
//float angleThreshold = 180 * (PI / 180);
//
//struct ThresholdValues{
//    float x;
//    float r;
//    float angle;
//};
//
//
//bool whichisbigger(const ThresholdValues& a, const ThresholdValues& b){
//    return a.x < b.x;
//}
//void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
//    std::vector <ThresholdValues> xlengthArray;
//    float angleMin = msg->angle_min;
//    float angleIncrement = msg->angle_increment;
//    int zeroIndex = -angleMin / angleIncrement;
//    std::vector<float> ranges = msg->ranges;
//    int startIndex = zeroIndex - (angleThreshold / 2) / angleIncrement; //limits to angleThreshold
//    int endIndex = zeroIndex + (angleThreshold / 2) / angleIncrement;   //limits to angleThreshold
//
//    for (int i = endIndex; i > startIndex; i--) {
//        if (ranges[i] > .03) {
//            float currentAngle = (zeroIndex - i) * angleIncrement;
//            float x = ranges[i] * sin(currentAngle);
//            if (abs(x) < robotLength / 2 + robotLengthTolerance) {
//                ThresholdValues data;
//                data.x = x;
//                data.r = ranges[i];
//                data.angle = currentAngle;
//                xlengthArray.push_back(data);
//            }
//        }
//    }
//
//    std::vector<float> obsTurnArray(11);
//    bool withinStep = true;
//    float rMin = .5;
//    std::vector<float> rMinArray(11, rMin);
//    for (int i = 0; i < xlengthArray.size(); i++) {
//        int intIndex = int((xlengthArray[i].x + robotLengthTolerance + robotLength / 2) * 11 /
//                           (robotLength + (robotLengthTolerance * 2)));
//        if (intIndex >= 0 && intIndex < 11) {
//            if (xlengthArray[i].r < rMinArray[intIndex]) {
//                rMinArray[intIndex] = xlengthArray[i].r;
//            }
//        }
//    }
//
//    for (int i = 0; i < rMinArray.size(); i++) {
//        obsTurnArray[i] = (rMinArray[i]) * turnScaleFactor;
//    }
//
//    float closestDistance = rMinArray[0];
//    for (int i = 0; i < rMinArray.size(); i++) {
//        if (rMinArray[i] < closestDistance) {
//            closestDistance = rMinArray[i];
//        }
//    }
//    //std::cout << closestDistance << std::endl;
//
//    std::vector<float> obsVelArray(11);
//
//    int velocityIndex;
//
//    velocityIndex = int((closestDistance/rMin)*11) + 5;
//    if (velocityIndex > 10) {
//        velocityIndex = 10;
//    }
//    else if (velocityIndex < 0){
//        velocityIndex = 0;
//    }
//
//    obsVelArray[velocityIndex] = 1;
//
//    // reverse the order of the obsTurnArray and reverse sign of all values
//    //std::reverse(obsTurnArray.begin(), obsTurnArray.end());
////    for (int i = 0; i < obsTurnArray.size(); i++){
////        obsTurnArray[i] = -obsTurnArray[i];
////    }
//
//    std::vector<float> finalObsArray;
//    finalObsArray.insert(finalObsArray.end(), obsVelArray.begin(), obsVelArray.end() );
//    finalObsArray.insert(finalObsArray.end(), obsTurnArray.begin(), obsTurnArray.end() );
//
//    std_msgs::Float32MultiArray msg_out;
//    msg_out.data = finalObsArray;
//
//    for (int i = 11; i < 22; i++) {
//        std::cout << finalObsArray[i];
//        std::cout << ", ";
//        if (i == 21){
//            std::cout << "" << std::endl;
//        }
//    }
//
//    velocity_data_ptr->publish(msg_out);
//
//}
//
////std::vector<float> invertArray(std::vector<float> array)
////{
////    std::reverse(array.begin(), array.end());
////    std::reverse(array.begin(), array[4]);
////    std::reverse(array[6], array.end());
////
////    for (int i = 0; i < obsTurnArray.size(); i++){
////        obsTurnArray[i] = -obsTurnArray[i];
////    }
////
////    return array;
////}
//
//int main(int argc, char **argv)
//{
//    /**
//     * The ros::init() function needs to see argc and argv so that it can perform
//     * any ROS arguments and name remapping that were provided at the command line.
//     * For programmatic remappings you can use a different version of init() which takes
//     * remappings directly, but for most command-line programs, passing argc and argv is
//     * the easiest way to do it.  The third argument to init() is the name of the node.
//     *
//     * You must call one of the versions of ros::init() before using any other
//     * part of the ROS system.
//     */
//    ros::init(argc, argv, "listener");
//    std::cout << "Initializing" << std::endl;
//
//    /**
//     * NodeHandle is the main access point to communications with the ROS system.
//     * The first NodeHandle constructed will fully initialize this node, and the last
//     * NodeHandle destructed will close down the node.
//     */
//    ros::NodeHandle n;
//
//    /**
//     * The subscribe() call is how you tell ROS that you want to receive messages
//     * on a given topic.  This invokes a call to the ROS
//     * master node, which keeps a registry of who is publishing and who
//     * is subscribing.  Messages are passed to a callback function, here
//     * called chatterCallback.  subscribe() returns a Subscriber object that you
//     * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
//     * object go out of scope, this callback will automatically be unsubscribed from
//     * this topic.
//     *
//     * The second parameter to the subscribe() function is the size of the message
//     * queue.  If messages are arriving faster than they are being processed, this
//     * is the number of messages that will be buffered up before beginning to throw
//     * away the oldest ones.
//     */
//    ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);
//    ros::Publisher velocity_data = n.advertise<std_msgs::Float32MultiArray>("obst/cmd_vel", 1000);
//    std::cout << "Streaming Data" << std::endl;
//    velocity_data_ptr = &velocity_data;
//
//    /**
//     * ros::spin() will enter a loop, pumping callbacks.  With this version, all
//     * callbacks will be called from within this thread (the main one).  ros::spin()
//     * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
//     */
//    ros::spin();
//
//    return 0;
//}