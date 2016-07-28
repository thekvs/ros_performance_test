#include <boost/functional/hash.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/random.hpp>
#include <boost/random/uniform_int_distribution.hpp>

#include <ros/ros.h>
#include <unique_id/unique_id.h>

#include "ros_performance_test/TestMessage.h"
#include "ros_performance_test/data_hash.hpp"

typedef boost::random::uniform_int_distribution<int> UniformDistribution;

static const int kDefaultRate = 10;
static const int kDefaultPayloadSize = 64;
static const int kDefaultQueueSize = 1000;
static const int kDefaultHashFunctionId = 2;

static boost::mt19937 gen;

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "producer");

    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    gen.seed(time(NULL));

    int queue_size;
    int rate;
    int payload_size;
    int hash_function_id;

    if (!priv_nh.getParam("rate", rate)) {
        ROS_WARN_STREAM("couldn't find 'rate' configuration parameter, using the default=" << kDefaultRate);
        rate = kDefaultRate;
    }

    if (!priv_nh.getParam("payload_size", payload_size)) {
        ROS_WARN_STREAM("couldn't find 'payload_size' configuration parameter, using the default=" << kDefaultPayloadSize);
        payload_size = kDefaultPayloadSize;
    }

    if (!priv_nh.getParam("queue_size", queue_size)) {
        ROS_WARN_STREAM("couldn't find 'queue_size' configuration parameter, using the default=" << kDefaultQueueSize);
        queue_size = kDefaultQueueSize;
    }

    if (!priv_nh.getParam("hash_function_id", hash_function_id)) {
        ROS_WARN_STREAM("couldn't find 'hash_function_id' configuration parameter, using the default=" << kDefaultHashFunctionId);
        hash_function_id = kDefaultHashFunctionId;
    }

    uint16_t uid = boost::hash_value<boost::uuids::uuid>(unique_id::fromRandom()) % std::numeric_limits<uint16_t>::max();

    ROS_INFO_STREAM("Node's ID: " << uid);
    ROS_INFO_STREAM("Payload size: " << payload_size);
    ROS_INFO_STREAM("Message queue size: " << queue_size);
    ROS_INFO_STREAM("Publish rate: " << rate);

    ros::Publisher chatter_pub = nh.advertise<ros_performance_test::TestMessage>("producer", queue_size);
    ros::Rate loop_rate(rate);

    uint32_t seq = 0;

    ros_performance_test::TestMessage msg;
    msg.data.reserve(payload_size);
    msg.header.uid = uid;

    boost::random::uniform_int_distribution<> small_ints(0, 255);

    while (ros::ok()) {
        msg.data.clear();

        msg.header.seq = ++seq;
        for (int i = 0; i < payload_size; i++) {
            msg.data.push_back(small_ints(gen));
        }

        if (hash_function_id == kDefaultHashFunctionId) {
            msg.header.data_hash = data_hash_2(msg.data.begin(), msg.data.end());
        } else if (hash_function_id == 1) {
            msg.header.data_hash = data_hash_1(msg.data.begin(), msg.data.end());
        } else {
            ROS_ERROR_STREAM("Unknown id of data hash function: " << hash_function_id);
        }

        msg.header.ts = ros::Time::now();

        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return EXIT_SUCCESS;
}
