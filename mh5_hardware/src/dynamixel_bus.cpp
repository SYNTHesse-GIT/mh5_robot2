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
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "mh5_hardware/dynamixel_bus.hpp"

namespace mh5_hardware
{

#define LOG rclcpp::get_logger(info_.name)


MH5DynamixelBus::~MH5DynamixelBus() 
{
    // If the controller manager is shutdown via Ctrl + C the on_deactivate methods won't be called.
    // We therefore need to make sure to actually deactivate the communication
    //
    // see: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/e5ba31cd38c34076f95b25a047daaa16f7c3466c/ur_robot_driver/src/hardware_interface.cpp#L58C1-L63C2
    // and: https://github.com/ros-controls/ros2_control/issues/472
    on_deactivate(rclcpp_lifecycle::State());
}


hardware_interface::CallbackReturn 
MH5DynamixelBus::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.hardware_parameters["debug"] == "yes") {
        LOG.set_level(rclcpp::Logger::Level::Debug);
    }

    RCLCPP_INFO(LOG, "initializing");

    if (!initPort()) return hardware_interface::CallbackReturn::ERROR;
    if (!initJoints()) return hardware_interface::CallbackReturn::ERROR;
    if (!initSensors()) return hardware_interface::CallbackReturn::ERROR;
    if (!setupDynamixelLoops()) return hardware_interface::CallbackReturn::ERROR;

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

    // baud rate
    auto baud_str = info_.hardware_parameters["baudrate"];
    if (baud_str == "") {
        RCLCPP_INFO(LOG, "no baudrate specified, will default to 1000000 bps");
        baudrate_ = 1000000;
    }
    else {
        baudrate_ = stoi(baud_str);
        RCLCPP_INFO(LOG, "baudrate %d", baudrate_);
    }

    // rs485 config
    rs485_ = (info_.hardware_parameters["rs485"] == "yes");
    if (rs485_) {
        RCLCPP_INFO(LOG, "using rs485 hardware control");
    }

    // Dynamixel protocol
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

    // open and configure the serial port
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

    // Dynamixel packet handler
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler((float)protocol_);
    RCLCPP_INFO(LOG, "Dynamixel protocol %3.1f initialzed", protocol_);

    return true;
}


bool MH5DynamixelBus::initJoints()
{
    //get joint names and num of joint
    num_joints_ = info_.joints.size();
    if (num_joints_ == 0) {
        RCLCPP_ERROR(LOG, "no joints defined");
        return false;
    }
    RCLCPP_INFO(LOG, "configuring %d joints", num_joints_);

    for (auto joint : info_.joints) {
        if (joint.name == "") {
            RCLCPP_ERROR(LOG, "there is a joint without a name in the URDF");
            return false;
        }
        if (joint.parameters["id"] == "") {
            RCLCPP_ERROR(LOG, "missing ID for joint %s", joint.name.c_str());
            return false;
        }
        int id;
        try {
            id = stoi(joint.parameters["id"]);
        } 
        catch (std::invalid_argument const& ex) {
            RCLCPP_ERROR(LOG, "failed to parse ID of joint %s: %s", joint.name.c_str(), joint.parameters["id"].c_str());
            return false;
        }
        auto j = DynamixelJoint(joint.name, id);

        int dxl_comm_result;
        uint8_t dxl_error = 0;
        if (packetHandler_->ping(portHandler_, id, &dxl_error) != COMM_SUCCESS) {
            RCLCPP_WARN(LOG, "failed to communicate with joint %s[%i], will be marked as inactive", joint.name.c_str(), id);
            j.available_ = false;
        }
        else if (dxl_error != 0) {
            RCLCPP_WARN(LOG, "joint %s[%i] reported error %u, will be marked as inactive", joint.name.c_str(), id, dxl_error);
            j.available_ = false;
        }
        else {
            RCLCPP_INFO(LOG, "joint %s[%i] detected", joint.name.c_str(), id);
            j.available_ = true;
            // inits
            for (auto init : joint.parameters) {
                if (init.first.compare(0, 5, "init_") == 0) {
                    // RCLCPP_INFO(LOG, "init found %s: %s", init.first.c_str(), init.second.c_str());
                    std::string init_string = init.second;
                    std::vector<std::string> tokens;
                    size_t pos = 0;
                    std::string token;
                    while ((pos = init_string.find(",")) != std::string::npos) {
                        token = init_string.substr(0, pos);
                        tokens.push_back(token);
                        init_string.erase(0, pos + 1);
                    }
                    tokens.push_back(init_string);
                    if (tokens.size() < 3) {
                        RCLCPP_WARN(LOG, "init %s for joint %s incomplete: %s; will be ignored", 
                                    init.first.c_str(), joint.name.c_str(), init.second.c_str());
                    }
                    else {
                        uint16_t address;
                        int bytes;
                        int value;
                        try {
                            address = (uint16_t)stoi(tokens[0]);
                            bytes = stoi(tokens[1]);
                            value = stoi(tokens[2]);
                        }
                        catch(std::invalid_argument const& ex) {
                            RCLCPP_WARN(LOG, "init %s for joint %s cannot be parsed: %s; will be ignored", 
                                    init.first.c_str(), joint.name.c_str(), init.second.c_str());
                            continue;
                        }
                        switch (bytes) {
                            case 1:
                                dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, address, (uint8_t)value, &dxl_error);
                                break;
                            case 2:
                                dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, id, address, (uint16_t)value, &dxl_error);
                                break;
                            case 4:
                                dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, id, address, (uint32_t)value, &dxl_error);
                                break;
                            default:
                                RCLCPP_WARN(LOG, "init %s for joint %s has incorrect number of bytes: %s; will be ignored", 
                                        init.first.c_str(), joint.name.c_str(), init.second.c_str());
                                continue;
                                break;
                        }
                        if (dxl_comm_result != COMM_SUCCESS) {
                            RCLCPP_WARN(LOG, "failed to write init %s for joint %s[%i]", init.first.c_str(), joint.name.c_str(), id);
                        }
                        else if (dxl_error != 0) {
                            RCLCPP_WARN(LOG, "error writing init %s for joint %s[%i], error code: %i", init.first.c_str(), joint.name.c_str(), id, dxl_error);
                        }
                        else {
                            RCLCPP_DEBUG(LOG, "successfully wrote init %s for joint %s[%i]", init.first.c_str(), joint.name.c_str(), id);
                        }
                    }
                }
            }

            RCLCPP_INFO(LOG, "joint %s[%i] initialized", joint.name.c_str(), id);
        }

        joints_.emplace_back(DynamixelJoint(joint.name, id));
    }

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
    pve_read = std::make_unique<dynamixel::GroupSyncRead>(portHandler_, packetHandler_, 126, 10);
    for (auto & joint : joints_) {
        if (joint.available_) {
            pve_read->addParam(joint.id_);
            RCLCPP_DEBUG(LOG, "pve_read loop added joint %s[%i]", joint.name_.c_str(), joint.id_);
        }
    }
    RCLCPP_INFO(LOG, "pve_read loop configured");

    return true;
}

std::vector<hardware_interface::StateInterface>
MH5DynamixelBus::export_state_interfaces()
{
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto & joint : joints_) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, hardware_interface::HW_IF_POSITION, &joint.position_));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, hardware_interface::HW_IF_VELOCITY, &joint.velocity_));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, hardware_interface::HW_IF_EFFORT, &joint.effort_));
        }
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
MH5DynamixelBus::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // uint8_t dxl_error;

    // do not activate the servos; this will be the job of the controllers if they need to send commands
    // for (auto & joint : joints_) {
    //     if (packetHandler_->write1ByteTxRx(portHandler_, joint.id_, 64, 1, &dxl_error) != COMM_SUCCESS) {
    //         RCLCPP_WARN(LOG, "failed to activate joint %s; communication error; will be marked as inactive", joint.name_.c_str());
    //         joint.available_ =  false;
    //     }
    //     else if (dxl_error != 0) {
    //         RCLCPP_WARN(LOG, "failed to activate joint %s; device error %u; will be marked as inactive", joint.name_.c_str(), dxl_error);
    //         joint.available_ = false;
    //     }
    //     else {
    //         RCLCPP_INFO(LOG, "joint %s[%i] activated", joint.name_.c_str(), joint.id_);
    //     }
    // }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
MH5DynamixelBus::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    uint8_t dxl_error;

    for (auto & joint : joints_) {
        if (packetHandler_->write1ByteTxRx(portHandler_, joint.id_, 64, 0, &dxl_error) != COMM_SUCCESS) {
            RCLCPP_WARN(LOG, "failed to deactivate joint %s; communication error", joint.name_.c_str());
        }
        else if (dxl_error != 0) {
            RCLCPP_WARN(LOG, "failed to deactivate joint %s; device error %u", joint.name_.c_str(), dxl_error);
        }
        else {
            RCLCPP_INFO(LOG, "joint %s[%i] deactivated", joint.name_.c_str(), joint.id_);
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MH5DynamixelBus::read(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
    int dxl_comm_result = COMM_TX_FAIL;

    // position, velocity, effort
    dxl_comm_result = pve_read->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_DEBUG(LOG, "pve SyncRead communication failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
    }
    else {
        for (auto  & joint : joints_) {
            if (joint.available_) {
                if (! pve_read->isAvailable(joint.id_, 132, 4)) {
                    RCLCPP_DEBUG(LOG, "pve SyncRead getting position for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
                }
                else {
                    int32_t pos = pve_read->getData(joint.id_, 132, 4);
                    joint.position_ = (pos - 2047) * 0.001533980787886;
                }

                if (! pve_read->isAvailable(joint.id_, 128, 4)) {
                    RCLCPP_DEBUG(LOG, "pve SyncRead getting velocity for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
                }
                else {
                    int32_t vel = pve_read->getData(joint.id_, 128, 4);
                    joint.velocity_ = vel  * 0.023980823922402;
                }

                if (! pve_read->isAvailable(joint.id_, 126, 2)) {
                    RCLCPP_DEBUG(LOG, "pve SyncRead getting effort for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
                }
                else {
                    int16_t eff = pve_read->getData(joint.id_, 126, 24);
                    joint.effort_ = eff * 0.0014;
                }
            }
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
MH5DynamixelBus::write(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
    return hardware_interface::return_type::OK;
}


// bool MH5DynamixelBus::ping(const uint8_t id, const int num_tries)
// {
//     int dxl_comm_result = COMM_TX_FAIL;             // Communication result
//     uint8_t dxl_error = 0;                          // Dynamixel error
    
//     for (int n=0; n < num_tries; n++)
//     {
//         dxl_comm_result = packetHandler_->ping(portHandler_, id, &dxl_error);
//         if (dxl_comm_result == COMM_SUCCESS) {
//             return true;
//         }
//     }
//     return false;
// }



} // namespace mh5_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    mh5_hardware::MH5DynamixelBus,
    hardware_interface::SystemInterface)