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

std::ostream&
operator<<(std::ostream& stream, const LogEntry& e)
{
    stream << e.incoming.uid << "," << e.incoming.seq << "," << e.incoming.data_hash << "," << e.incoming.ts.sec << ","
           << e.incoming.ts.nsec << ",";
    stream << e.data_hash << "," << e.ts.sec << "," << e.ts.nsec;

    return stream;
}

typedef std::vector<LogEntry> LogEntries;

static const std::string kDefaultLogFile = "/tmp/ros_perftest.log";
static const int kDefaultQueueSize = 1000;

bool
write_log(const std::string& fname, const LogEntries& log_data)
{
    if (fname.empty()) {
        return false;
    }

    std::fstream result(fname.c_str(), std::fstream::out);

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
    e.data_hash = data_hash_2(msg->data.begin(), msg->data.end());
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

    if (!priv_nh.getParam("log_file", log_file)) {
        ROS_WARN_STREAM("couldn't find 'log_file' configuration parameter, using the default=" << kDefaultLogFile);
        log_file = kDefaultLogFile;
    }

    if (!priv_nh.getParam("queue_size", queue_size)) {
        ROS_WARN_STREAM("couldn't find 'queue_size' configuration parameter, using the default=" << kDefaultQueueSize);
        queue_size = kDefaultQueueSize;
    }

    ros::TransportHints transport_hints = ros::TransportHints().unreliable(); // UDP transport

    LogEntries log_data;
    log_data.reserve(10000);

    ros::Subscriber sub
        = nh.subscribe<ros_performance_test::TestMessage>("producer", queue_size, boost::bind(data_cb, _1, boost::ref(log_data)));

    ros::spin();

    ROS_INFO_STREAM("Writing " << log_data.size() << " log entries");

    write_log(log_file, log_data);

    return 0;
}
