#ifndef MH5_HARDWARE_DYNAMIXEL_LOOP_HPP_
#define MH5_HARDWARE_DYNAMIXEL_LOOP_HPP_

#include <rclcpp/rclcpp.hpp>
// #include <hardware_interface/types/hardware_interface_return_values.hpp>
// #include "dynamixel_sdk/dynamixel_sdk.h"

// #include "mh5_hardware/dynamixel_bus.hpp"


namespace mh5_hardware
{


struct PacketCounter
{
    PacketCounter(int rate=0, int horizon=0, rclcpp::Time time_init=rclcpp::Time())
    : rate_{rate}, horizon_{horizon}, last_run_{time_init}, last_reset_{time_init},
      total_packets_{0}, total_errors_{0}, error_rate_{0.0},
      last_packets_{0}, last_errors_{0}, last_error_rate_{0.0},
      running_packets_{0}, running_errors_{0},
      slice_{0}
    {}

    void addRun(const rclcpp::Time & time)
    {
        total_packets_++;
        error_rate_ = total_errors_ * 100.0 / total_packets_;
        running_packets_++;
        last_run_ = time;
    };

    void addErr()
    {
        total_errors_++;
        error_rate_ = total_errors_ * 100.0 / total_packets_;
        running_errors_++;
    };

    bool shouldRun(const rclcpp::Time & time, const rclcpp::Duration & period)
    {
        // inspired from https://github.com/ros-controls/ros2_control/blob/ad47e2ae9036871b1528505fb2f4c65400604ad0/hardware_interface/src/resource_manager.cpp#L2323
        rclcpp::Duration actual_period = time - last_run_;
        const double error_now = std::abs(actual_period.seconds() * rate_ - 1.0);
        const double error_if_skipped = std::abs((actual_period + period).seconds() * rate_ - 1.0);
        return error_now <= error_if_skipped;
    };

    bool shouldReset(const rclcpp::Time & time)
    {
        return time - last_reset_ >= rclcpp::Duration::from_seconds(horizon_);
    };

    void reset(const rclcpp::Time & time)
    {
        last_packets_ = running_packets_;
        last_errors_ = running_errors_;
        last_error_rate_ = running_errors_ * 100.0 / running_packets_;
        running_packets_ = 0;
        running_errors_ = 0;
        last_reset_ = time;
    };

    int              rate_;
    int              horizon_;
    rclcpp::Time     last_run_;
    rclcpp::Time     last_reset_;
    int              total_packets_;
    int              total_errors_;
    double           error_rate_;
    int              last_packets_;
    int              last_errors_;
    double           last_error_rate_;
    int              running_packets_;
    int              running_errors_;
    int              slice_;              // can be used by loops for different reasons
};


} // namespace mh5_hardware

#endif