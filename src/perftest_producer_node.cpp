#include <ros/ros.h>
#include <unique_id/unique_id.h>

#include "ros_perfomance_test/TestMessage.h"

static const int kDefaultRateParameter = 10;

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "producer");

    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    uint32_t queue_size = 1000;
    int rate;

    if (!priv_nh.getParam("rate", rate)) {
        ROS_WARN_STREAM("couldn't find 'rate' configuration parameter, using the default=" << kDefaultRateParameter);
        rate = kDefaultRateParameter;
    }

    boost::uuids::uuid node_uid = unique_id::fromRandom();

    ROS_INFO_STREAM("Publishing messages at rate " << rate << " hz. Node's UUID: " << node_uid);

    ros::Publisher chatter_pub = nh.advertise<ros_perfomance_test::TestMessage>("producer", queue_size);
    ros::Rate loop_rate(rate);

    uint32_t seq = 0;

    while (ros::ok()) {
        ros_perfomance_test::TestMessage msg;

        msg.seq = ++seq;
        msg.ts = ros::Time::now();
        msg.uid = unique_id::toMsg(node_uid);

        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
