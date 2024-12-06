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

#include "rclcpp/rclcpp.hpp"

#include "mh5_hardware/dynamixel_bus.hpp"

namespace mh5_hardware
{

#define LOG rclcpp::get_logger(info_.name)

hardware_interface::CallbackReturn 
MH5DynamixelBus::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(LOG, "initializing");
    if (!initPort()) return hardware_interface::CallbackReturn::ERROR;
    if (!initJoints()) return hardware_interface::CallbackReturn::ERROR;
    if (!initSensors()) return hardware_interface::CallbackReturn::ERROR;
    if (!setupDynamixelLoops()) return hardware_interface::CallbackReturn::ERROR;

    // //Register handles
    // for(int i=0; i<num_joints_; i++){
    //     //State
    //     joint_state_interface.registerHandle(joints_[i]->getJointStateHandle());
    //     joint_status_interface.registerHandle(joints_[i]->getJointStatusHandle());
    //     // control
    //     // position_joint_interface.registerHandle(joints_[i]->getJointPositionHandle());
    //     joint_control_interface.registerHandle(joints_[i]->getJointControlHandle());
    //     //Command Postion - Velocity
    //     // pos_vel_joint_interface.registerHandle(joints_[i]->getJointPosVelHandle());
    //     //Torque activation


    //     // joint_temp_volt_interface.registerHandle(joints_[i]->getTempVoltHandle());
    // }

    // for (int i=0; i<num_sensors_; i++) {
    //     sensor_volt_curr_interface.registerHandle(foot_sensors_[i]->getVoltCurrHandle());
    // }

    // //Register interfaces
    // // joint publishers
    // registerInterface(&joint_state_interface);
    // registerInterface(&joint_status_interface);
    // // joint controllers
    // // registerInterface(&position_joint_interface);
    // registerInterface(&joint_control_interface);

    // // registerInterface(&pos_vel_joint_interface);

    // // registerInterface(&joint_temp_volt_interface);
    // registerInterface(&sensor_volt_curr_interface);

    return hardware_interface::CallbackReturn::SUCCESS;
}


bool MH5DynamixelBus::initPort()
{
    // get the serial port configuration
    port_ = info_.hardware_parameters["port"];
    if (port_ == "") {
        RCLCPP_ERROR(LOG, "port name was not specified");
        return false;
    }
    RCLCPP_INFO(LOG, "using port %s", port_.c_str());

    auto baud_str = info_.hardware_parameters["baudrate"];
    if (baud_str == "") {
        RCLCPP_INFO(LOG, "no baudrate specified, will default to 1000000 bps");
        baudrate_ = 1000000;
    }
    else {
        baudrate_ = stoi(baud_str);
        RCLCPP_INFO(LOG, "baudrate %d", baudrate_);
    }

    rs485_ = (info_.hardware_parameters["rs485"] == "yes");
    if (rs485_) {
        RCLCPP_INFO(LOG, "using rs485 hardware control");
    }

    auto protocol_str = info_.hardware_parameters["protocol"];
    if(protocol_str == "") {
        RCLCPP_INFO(LOG, "no protocol specified, will default to 2.0");
        protocol_ = 2.0;
    } else if (protocol_str == "1.0") {
        protocol_ = 1.0;
    } else if (protocol_str == "2.0") {
        protocol_ = 2.0;
    } else {
        RCLCPP_ERROR(LOG, "protocol %f unsupported", protocol_);
        return false;
    }
    RCLCPP_INFO(LOG, "using protocol %3.1f", protocol_);

    // open the serial port
    portHandler_ = new mh5_port_handler::PortHandlerMH5(port_.c_str());
    if (! portHandler_->openPort()) {
        RCLCPP_ERROR(LOG, "failed to open port %s", port_.c_str());
        return false;
    }
    RCLCPP_INFO(LOG, "successfully opened port %s", port_.c_str());

    if (!portHandler_->setBaudRate(baudrate_)) {
        RCLCPP_ERROR(LOG, "failed to set baud rate %i bps on port %s", baudrate_, port_.c_str());
        return false;
    }
    RCLCPP_INFO(LOG, "successfully set baud rate %i bps on port %s", baudrate_, port_.c_str());
    
    if (rs485_) {
        if (!portHandler_->setRS485() ) {
            RCLCPP_ERROR(LOG, "failed to configure RS485 on port %s", port_.c_str());
        return false;
        }
        RCLCPP_INFO(LOG, "successfully configured RS485 on port %s", port_.c_str());
    }

    packetHandler_ = dynamixel::PacketHandler::getPacketHandler((float)protocol_);
    RCLCPP_INFO(LOG, "Dynamixel protocol %3.1f initialzed", protocol_);

    return true;
}

bool MH5DynamixelBus::initJoints()
{
    return true;
}


bool MH5DynamixelBus::initSensors()
{
    return true;
}

template <class Loop>
Loop* MH5DynamixelBus::setupLoop(std::string name, const double default_rate)
{
    double rate = stod(info_.hardware_parameters["loop_rates/"+name]);
    if (rate == 0.0)
    {
        RCLCPP_INFO(LOG, "loop %s: no 'loop_rates/%s' available, default to %.1f Hz", 
            name.c_str(), name.c_str(), default_rate);
        rate = default_rate;
    }
    else
    {
        RCLCPP_INFO(LOG, "loop %s initialized at %.1f Hz", name.c_str(), rate);
    }
    Loop* loop = new Loop(name, rate, portHandler_, packetHandler_);
    // loop->prepare(joints_);
    // communication_stats_interface.registerHandle(loop->getCommStatHandle());
    return loop;
}

bool MH5DynamixelBus::setupDynamixelLoops()
{
    return true;
}

std::vector<hardware_interface::StateInterface>
MH5DynamixelBus::export_state_interfaces()
{
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // for (auto resource : resources_) {
        //     resource->add_state_interfaces(&state_interfaces);
        // }

        // // sensors
        // state_interfaces.emplace_back(hardware_interface::StateInterface(ultrasonic_.name, trilobot_hardware::HW_IF_DISTANCE, &ultrasonic_.distance));

        return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MH5DynamixelBus::export_command_interfaces()
{
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // for (auto resource : resources_) {
        //     resource->add_command_interfaces(&command_interfaces);
        // }

        // command_interfaces.emplace_back(hardware_interface::CommandInterface(ultrasonic_.name, trilobot_hardware::HW_IF_OFFSET, &ultrasonic_.offset));
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(ultrasonic_.name, trilobot_hardware::HW_IF_SAMPLES, &ultrasonic_.samples));
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(ultrasonic_.name, trilobot_hardware::HW_IF_TIMEOUT, &ultrasonic_.timeout));

        return command_interfaces;
}

//   hardware_interface::return_type prepare_command_mode_switch(
//     const std::vector<std::string> & start_interfaces,
//     const std::vector<std::string> & stop_interfaces) override;

hardware_interface::CallbackReturn
MH5DynamixelBus::on_activate(const rclcpp_lifecycle::State & previous_state)
{

}

hardware_interface::CallbackReturn
MH5DynamixelBus::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{

}

hardware_interface::return_type
MH5DynamixelBus::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{

}

hardware_interface::return_type
MH5DynamixelBus::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{

}

} // namespace mh5_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    mh5_hardware::MH5DynamixelBus,
    hardware_interface::SystemInterface)