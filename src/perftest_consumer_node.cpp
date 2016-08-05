#include <fstream>
#include <numeric>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Time.h>

#include "ros_performance_test/TestMessage.h"
#include "ros_performance_test/data_hash.hpp"

struct LogEntry {
    ros_performance_test::TestMessageHeader incoming;
    uint32_t data_hash;
    ros::Time ts;
};

std::ostream& operator<<(std::ostream& stream, const LogEntry& e)
{
    stream << e.incoming.uid << "," << e.incoming.seq << "," << e.incoming.data_hash << "," << e.incoming.ts.sec << ","
           << e.incoming.ts.nsec << ",";
    stream << e.data_hash << "," << e.ts.sec << "," << e.ts.nsec;

    return stream;
}

typedef std::vector<LogEntry> LogEntries;

static const std::string kDefaultLogFile = "/tmp/ros_perftest.log";
static const int kDefaultQueueSize = 1000;
static const int kDefaultHashFunctionId = 2;

bool
write_log(const std::string& fname, const LogEntries& log_data)
{
    if (fname.empty()) {
        return false;
    }

    std::fstream result(fname.c_str(), std::fstream::out);

    // clang-format off
    const char* headers[] = {
        "node_id",
        "sequence_number",
        "data_hash_on_sender",
        "ts_sec_sent",
        "ts_nsec_sent",
        "data_hash_on_receiver",
        "ts_sec_received",
        "ts_nsec_received",
        NULL
    };
    // clang-format on

    int idx = 0;
    while (headers[idx] != NULL) {
        if (idx != 0) {
            result << ",";
        }
        result << headers[idx];
        idx++;
    }
    result << std::endl;

    for (size_t i = 0; i < log_data.size(); i++) {
        result << log_data[i] << std::endl;
    }

    return true;
}

void
data_cb(const ros_performance_test::TestMessage::ConstPtr& msg, LogEntries& log_data)
{
    LogEntry e;

    e.incoming = msg->header;
    int hash_function_id = msg->header.hash_function_id;

    if (hash_function_id == kDefaultHashFunctionId) {
        e.data_hash = data_hash_2(msg->data.begin(), msg->data.end());
    } else if (hash_function_id == 1) {
        e.data_hash = data_hash_1(msg->data.begin(), msg->data.end());
    } else {
        ROS_ERROR_STREAM("Unknown id of data hash function: " << hash_function_id);
    }

    e.ts = ros::Time::now();

    log_data.push_back(e);
}

int
main(int argc, char** argv)
{
    ros::init(argc, argv, "consumer");

    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    std::string log_file;
    int queue_size;
    bool unreliable_transport = false;

    if (!priv_nh.getParam("log_file", log_file)) {
        ROS_WARN_STREAM("couldn't find 'log_file' configuration parameter, using the default=" << kDefaultLogFile);
        log_file = kDefaultLogFile;
    }

    if (!priv_nh.getParam("queue_size", queue_size)) {
        ROS_WARN_STREAM("couldn't find 'queue_size' configuration parameter, using the default=" << kDefaultQueueSize);
        queue_size = kDefaultQueueSize;
    }

    if (!priv_nh.getParam("unreliable_transport", unreliable_transport)) {
        ROS_WARN_STREAM("couldn't find 'unreliable_transport' configuration parameter, using the default=false");
    }

    ros::TransportHints transport_hints;
    if (unreliable_transport) {
        transport_hints = ros::TransportHints().unreliable(); // UDP transport
    } else {
        transport_hints = ros::TransportHints();
    }

    LogEntries log_data;
    log_data.reserve(10000);

    ros::VoidConstPtr aux;
    ros::Subscriber sub = nh.subscribe<ros_performance_test::TestMessage>(
        "producer", queue_size, boost::bind(data_cb, _1, boost::ref(log_data)), aux, transport_hints);

    ros::spin();

    ROS_INFO_STREAM("Writing " << log_data.size() << " log entries");

    write_log(log_file, log_data);

    return 0;
}
