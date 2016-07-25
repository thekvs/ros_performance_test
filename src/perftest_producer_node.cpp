#include <ros/ros.h>
#include <unique_id/unique_id.h>

#include "ros_perfomance_test/TestMessage.h"

static const int kDefaultRateParameter = 10;
static const int kDefaultPayloadSize = 64;
static const int kDefaultQueueSize = 1000;

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "producer");

    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    int queue_size;
    int rate;
    int payload_size;

    if (!priv_nh.getParam("rate", rate)) {
        ROS_WARN_STREAM("couldn't find 'rate' configuration parameter, using the default=" << kDefaultRateParameter);
        rate = kDefaultRateParameter;
    }

    if (!priv_nh.getParam("payload_size", payload_size)) {
        ROS_WARN_STREAM("couldn't find 'payload_size' configuration parameter, using the default=" << kDefaultPayloadSize);
        payload_size = kDefaultPayloadSize;
    }

    if (!priv_nh.getParam("queue_size", queue_size)) {
        ROS_WARN_STREAM("couldn't find 'queue_size' configuration parameter, using the default=" << kDefaultQueueSize);
        queue_size = kDefaultQueueSize;
    }

    boost::uuids::uuid node_uid = unique_id::fromRandom();

    ROS_INFO_STREAM("Node's UUID: " << node_uid);
    ROS_INFO_STREAM("Payload size: " << payload_size);
    ROS_INFO_STREAM("Message queue size: " << queue_size);
    ROS_INFO_STREAM("Publish rate: " << rate);

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
