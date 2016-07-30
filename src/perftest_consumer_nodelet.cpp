#include <fstream>

#include <boost/functional/hash.hpp>
#include <boost/generator_iterator.hpp>
#include <boost/thread/thread.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <unique_id/unique_id.h>

#include "ros_performance_test/TestMessage.h"
#include "ros_performance_test/data_hash.hpp"

namespace ros_performance_test
{

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

static const std::string kDefaultLogFile = "/tmp/ros_perftest.csv";
static const int kDefaultQueueSize = 1000;
static const int kDefaultHashFunctionId = 2;

class PerftestConsumer : public nodelet::Nodelet
{
public:
    PerftestConsumer()
        : queue_size(0)
    {
        log_data.reserve(10000);
    }

    ~PerftestConsumer()
    {
        write_log();
    }

private:
    LogEntries log_data;
    ros::Subscriber sub;
    uint16_t uid;

    std::string log_file;
    int queue_size;

    bool
    write_log()
    {
        if (log_file.empty()) {
            return false;
        }

        std::fstream result(log_file.c_str(), std::fstream::out);

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
    data_cb(const ros_performance_test::TestMessage::ConstPtr& msg)
    {
        LogEntry e;

        e.incoming = msg->header;
        int hash_function_id = msg->header.hash_function_id;

        if (hash_function_id == kDefaultHashFunctionId) {
            e.data_hash = data_hash_2(msg->data.begin(), msg->data.end());
        } else if (hash_function_id == 1) {
            e.data_hash = data_hash_1(msg->data.begin(), msg->data.end());
        } else {
            NODELET_ERROR_STREAM("Unknown id of data hash function: " << hash_function_id);
        }

        e.ts = ros::Time::now();

        log_data.push_back(e);
    }

    virtual void
    onInit()
    {
        ros::NodeHandle nh = nodelet::Nodelet::getMTNodeHandle();
        ros::NodeHandle priv_nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        if (!priv_nh.getParam("log_file", log_file)) {
            NODELET_WARN_STREAM("couldn't find 'log_file' configuration parameter, using the default=" << kDefaultLogFile);
            log_file = kDefaultLogFile;
        }

        if (!priv_nh.getParam("queue_size", queue_size)) {
            NODELET_WARN_STREAM("couldn't find 'queue_size' configuration parameter, using the default=" << kDefaultQueueSize);
            queue_size = kDefaultQueueSize;
        }

        NODELET_INFO_STREAM("Message queue size: " << queue_size);

        sub = nh.subscribe<ros_performance_test::TestMessage>("producer", queue_size, &PerftestConsumer::data_cb, this);
    }
};

PLUGINLIB_EXPORT_CLASS(ros_performance_test::PerftestConsumer, nodelet::Nodelet);

} // namespace
