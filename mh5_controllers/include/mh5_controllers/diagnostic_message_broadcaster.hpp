// Copyright 2024 SYNTHesse
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MH5_HARDWARE_DIAGNOSTIC_MESSAGE_BROADCASTER_HPP_
#define MH5_HARDWARE_DIAGNOSTIC_MESSAGE_BROADCASTER_HPP_

#include "controller_interface/controller_interface.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "diagnostic_message_broadcaster_parameters.hpp"

namespace mh5_controllers
{

class DiagnosticSensor : public controller_interface::ControllerInterface
{
public:

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:

    std::shared_ptr<diagnostic_message_broadcaster::ParamListener> param_listener_;
    diagnostic_message_broadcaster::Params params_;

    using DiagnosticPublisher = realtime_tools::RealtimePublisher<diagnostic_msgs::msg::DiagnosticArray>;

};

}  // namespace mh5_controllers

#endif // MH5_HARDWARE_DIAGNOSTIC_MESSAGE_BROADCASTER_HPP_