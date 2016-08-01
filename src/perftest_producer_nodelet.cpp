#include <boost/functional/hash.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/random.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/thread/thread.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>

#include "ros_performance_test/TestMessage.h"
#include "ros_performance_test/data_hash.hpp"

namespace ros_performance_test
{

typedef boost::random::uniform_int_distribution<int> UniformDistribution;

static const int kDefaultRate = 10;
static const int kDefaultPayloadSize = 64;
static const int kDefaultQueueSize = 1000;
static const int kDefaultHashFunctionId = 2;

class PerftestProducer : public nodelet::Nodelet
{
public:
    PerftestProducer()
        : uid(0)
        , queue_size(0)
        , rate(0)
        , payload_size(0)
        , hash_function_id(0)
    {
    }

private:
    ros::Publisher pub;
    boost::mt19937 gen;
    int uid;

    int queue_size;
    int rate;
    int payload_size;
    int hash_function_id;

    void
    loop()
    {
        ros_performance_test::TestMessagePtr msg(new ros_performance_test::TestMessage());
        uint32_t seq = 0;

        boost::random::uniform_int_distribution<> small_ints(0, 255);
        ros::Rate loop_rate(rate);

        while (ros::ok()) {
            msg->data.clear();

            msg->header.seq = ++seq;
            for (int i = 0; i < payload_size; i++) {
                msg->data.push_back(small_ints(gen));
            }

            if (hash_function_id == kDefaultHashFunctionId) {
                msg->header.data_hash = data_hash_2(msg->data.begin(), msg->data.end());
            } else if (hash_function_id == 1) {
                msg->header.data_hash = data_hash_1(msg->data.begin(), msg->data.end());
            } else {
                NODELET_ERROR_STREAM("Unknown id of data hash function: " << hash_function_id);
            }

            msg->header.uid = uid;
            msg->header.hash_function_id = hash_function_id;
            msg->header.ts = ros::Time::now();

            pub.publish(msg);

            loop_rate.sleep();
        }

        NODELET_WARN_STREAM("Exit requested");
    }

    virtual void
    onInit()
    {
        gen.seed(time(NULL));

        ros::NodeHandle nh;
        ros::NodeHandle priv_nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        gen.seed(time(NULL));

        if (!priv_nh.getParam("uid", uid)) {
            ROS_ERROR_STREAM("couldn't find mandatory 'uid' configuration parameter");
            exit(EXIT_FAILURE);
        } else {
            if (uid < 0 || uid > std::numeric_limits<uint16_t>::max()) {
                ROS_ERROR_STREAM("parameter 'uid' must be within [0, " << std::numeric_limits<uint16_t>::max() << "] boundaries");
                exit(EXIT_FAILURE);
            }
        }

        if (!priv_nh.getParam("rate", rate)) {
            NODELET_WARN_STREAM("couldn't find 'rate' configuration parameter, using the default=" << kDefaultRate);
            rate = kDefaultRate;
        }

        if (!priv_nh.getParam("payload_size", payload_size)) {
            NODELET_WARN_STREAM("couldn't find 'payload_size' configuration parameter, using the default=" << kDefaultPayloadSize);
            payload_size = kDefaultPayloadSize;
        }

        if (!priv_nh.getParam("queue_size", queue_size)) {
            NODELET_WARN_STREAM("couldn't find 'queue_size' configuration parameter, using the default=" << kDefaultQueueSize);
            queue_size = kDefaultQueueSize;
        }

        if (!priv_nh.getParam("hash_function_id", hash_function_id)) {
            NODELET_WARN_STREAM("couldn't find 'hash_function_id' configuration parameter, using the default=" << kDefaultHashFunctionId);
            hash_function_id = kDefaultHashFunctionId;
        }

        NODELET_INFO_STREAM("Node's ID: " << uid);
        NODELET_INFO_STREAM("Payload size: " << payload_size);
        NODELET_INFO_STREAM("Message queue size: " << queue_size);
        NODELET_INFO_STREAM("Publish rate: " << rate);

        pub = nh.advertise<ros_performance_test::TestMessage>("producer", queue_size);

        boost::thread thr(&PerftestProducer::loop, this);
    }
};

PLUGINLIB_EXPORT_CLASS(ros_performance_test::PerftestProducer, nodelet::Nodelet);

} // namespace
