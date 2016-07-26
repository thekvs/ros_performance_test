#include <ros/ros.h>

#include "ros_perfomance_test/TestMessage.h"

void data_cb(const ros_perfomance_test::TestMessage::ConstPtr& msg)
{
    ROS_INFO_STREAM(std::endl << *msg << "----");
}

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "consumer");

    ros::NodeHandle nh;

    uint32_t queue_size = 1000;
    ros::TransportHints transport_hints = ros::TransportHints().unreliable(); // UDP transport

    ros::Subscriber sub = nh.subscribe("producer", queue_size, data_cb, transport_hints);

    ros::spin();

    return 0;
}

