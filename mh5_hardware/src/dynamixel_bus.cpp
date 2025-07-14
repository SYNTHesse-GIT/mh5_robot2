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


hardware_interface::CallbackReturn
MH5DynamixelBus::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.hardware_parameters["debug"] == "yes") {
        get_logger().set_level(rclcpp::Logger::Level::Debug);
    }

    RCLCPP_INFO(get_logger(), "initializing");

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
        RCLCPP_ERROR(get_logger(), "port name was not specified");
        return false;
    }
    RCLCPP_INFO(get_logger(), "using port %s", port_.c_str());

    // baud rate
    auto baud_str = info_.hardware_parameters["baudrate"];
    if (baud_str == "") {
        RCLCPP_INFO(get_logger(), "no baudrate specified, will default to 1000000 bps");
        baudrate_ = 1000000;
    }
    else {
        baudrate_ = stoi(baud_str);
        RCLCPP_INFO(get_logger(), "baudrate %d", baudrate_);
    }

    // rs485 config
    rs485_ = (info_.hardware_parameters["rs485"] == "yes");
    if (rs485_) {
        RCLCPP_INFO(get_logger(), "using rs485 hardware control");
    }

    // Dynamixel protocol
    auto protocol_str = info_.hardware_parameters["protocol"];
    if(protocol_str == "") {
        RCLCPP_INFO(get_logger(), "no protocol specified, will default to 2.0");
        protocol_ = 2.0;
    } else if (protocol_str == "1.0") {
        protocol_ = 1.0;
    } else if (protocol_str == "2.0") {
        protocol_ = 2.0;
    } else {
        RCLCPP_ERROR(get_logger(), "protocol %f unsupported", protocol_);
        return false;
    }
    RCLCPP_INFO(get_logger(), "using protocol %3.1f", protocol_);

    // Latency timer
    auto latency_str = info_.hardware_parameters["latency"];
    if(latency_str == "") {
        RCLCPP_INFO(get_logger(), "no latency specified, will default to 2 ms");
        latency_ = 2;
    } else {
        latency_ = stoi(latency_str);
        RCLCPP_INFO(get_logger(), "latency %d ms", latency_);
    }

    // open and configure the serial port
    portHandler_ = new mh5_port_handler::PortHandlerMH5(port_.c_str());
    if (! portHandler_->openPort()) {
        RCLCPP_ERROR(get_logger(), "failed to open port %s", port_.c_str());
        return false;
    }
    RCLCPP_INFO(get_logger(), "successfully opened port %s", port_.c_str());

    if (!portHandler_->setBaudRate(baudrate_)) {
        RCLCPP_ERROR(get_logger(), "failed to set baud rate %i bps on port %s", baudrate_, port_.c_str());
        return false;
    }
    RCLCPP_INFO(get_logger(), "successfully set baud rate %i bps on port %s", baudrate_, port_.c_str());

    if (rs485_) {
        if (!portHandler_->setRS485() ) {
            RCLCPP_ERROR(get_logger(), "failed to configure RS485 on port %s", port_.c_str());
        return false;
        }
        RCLCPP_INFO(get_logger(), "successfully configured RS485 on port %s", port_.c_str());
    }

    portHandler_->setLatencyTimer(latency_);

    // Dynamixel packet handler
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler((float)protocol_);
    RCLCPP_INFO(get_logger(), "Dynamixel protocol %3.1f initialzed", protocol_);

    return true;
}


bool MH5DynamixelBus::initJoints()
{
    //get joint names and num of joint
    num_joints_ = info_.joints.size();
    if (num_joints_ == 0) {
        RCLCPP_ERROR(get_logger(), "no joints defined");
        return false;
    }
    RCLCPP_INFO(get_logger(), "configuring %d joints", num_joints_);

    for (auto joint : info_.joints) {
        if (joint.name == "") {
            RCLCPP_ERROR(get_logger(), "there is a joint without a name in the URDF");
            return false;
        }
        if (joint.parameters["id"] == "") {
            RCLCPP_ERROR(get_logger(), "missing ID for joint %s", joint.name.c_str());
            return false;
        }
        int id;
        try {
            id = stoi(joint.parameters["id"]);
        }
        catch (std::invalid_argument const& ex) {
            RCLCPP_ERROR(get_logger(), "failed to parse ID of joint %s: %s", joint.name.c_str(), joint.parameters["id"].c_str());
            return false;
        }
        auto j = DynamixelJoint(joint.name, id);

        int dxl_comm_result;
        uint8_t dxl_error = 0;
        if (packetHandler_->ping(portHandler_, id, &dxl_error) != COMM_SUCCESS) {
            RCLCPP_WARN(get_logger(), "failed to communicate with joint %s[%i], will be marked as inactive", joint.name.c_str(), id);
            j.available_ = false;
        }
        else if (dxl_error != 0) {
            RCLCPP_WARN(get_logger(), "joint %s[%i] reported error %u, will be marked as inactive", joint.name.c_str(), id, dxl_error);
            j.available_ = false;
        }
        else {
            RCLCPP_INFO(get_logger(), "joint %s[%i] detected", joint.name.c_str(), id);
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
                        RCLCPP_WARN(get_logger(), "init %s for joint %s incomplete: %s; will be ignored",
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
                            RCLCPP_WARN(get_logger(), "init %s for joint %s cannot be parsed: %s; will be ignored",
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
                                RCLCPP_WARN(get_logger(), "init %s for joint %s has incorrect number of bytes: %s; will be ignored",
                                        init.first.c_str(), joint.name.c_str(), init.second.c_str());
                                continue;
                                break;
                        }
                        if (dxl_comm_result != COMM_SUCCESS) {
                            RCLCPP_WARN(get_logger(), "failed to write %s for %s[%i]", init.first.c_str(), joint.name.c_str(), id);
                        }
                        else if (dxl_error != 0) {
                            RCLCPP_WARN(get_logger(), "error writing %s for %s[%i], error code: %i", init.first.c_str(), joint.name.c_str(), id, dxl_error);
                        }
                        else {
                            RCLCPP_DEBUG(get_logger(), "successfully wrote %s for %s[%i]: %i", init.first.c_str(), joint.name.c_str(), id, value);
                        }
                    }
                }
            }

            RCLCPP_INFO(get_logger(), "joint %s[%i] initialized", joint.name.c_str(), id);
        }

        joints_.emplace_back(DynamixelJoint(joint.name, id));
    }

    return true;
}


bool MH5DynamixelBus::initSensors()
{
    return true;
}

// template <class Loop>
// Loop* MH5DynamixelBus::setupLoop(std::string name, const double default_rate)
// {
//     double rate = stod(info_.hardware_parameters["loop_rates/"+name]);
//     if (rate == 0.0)
//     {
//         RCLCPP_INFO(LOG, "loop %s: no 'loop_rates/%s' available, default to %.1f Hz",
//             name.c_str(), name.c_str(), default_rate);
//         rate = default_rate;
//     }
//     else
//     {
//         RCLCPP_INFO(LOG, "loop %s initialized at %.1f Hz", name.c_str(), rate);
//     }
//     Loop* loop = new Loop(name, rate, portHandler_, packetHandler_);
//     // loop->prepare(joints_);
//     // communication_stats_interface.registerHandle(loop->getCommStatHandle());
//     return loop;
// }

bool MH5DynamixelBus::setupDynamixelLoops()
{
    pve_read_ = std::make_unique<dynamixel::GroupSyncRead>(portHandler_, packetHandler_, 126, 10);
    for (auto & joint : joints_) {
        if (joint.available_) {
            pve_read_->addParam(joint.id_);
            RCLCPP_DEBUG(get_logger(), "pve_read loop added joint %s[%i]", joint.name_.c_str(), joint.id_);
        }
    }
    pve_read_stats_ = {0, 0, 0.0, 0, 0, 0.0, 0, 0};
    pve_read_stats_last_reset_ = get_clock()->now();
    RCLCPP_INFO(get_logger(), "pve_read loop configured");

    stat_read_ = std::make_unique<dynamixel::GroupSyncRead>(portHandler_, packetHandler_, 224, 6);
    for (auto & joint : joints_) {
        if (joint.available_) {
            stat_read_->addParam(joint.id_);
            RCLCPP_DEBUG(get_logger(), "stat_read loop added joint %s[%i]", joint.name_.c_str(), joint.id_);
        }
    }
    std::string rate_str = this->info_.hardware_parameters["status_read_rate"];
    if (rate_str == "") {
        RCLCPP_INFO(get_logger(), "stat_read_rate no specified, will default to 1Hz");
        status_read_rate_ = 1.0;
    }
    else {
        try {
            status_read_rate_ = stod(rate_str);
        }
        catch (std::invalid_argument const& ex) {
            RCLCPP_ERROR(get_logger(), "failed to parse stat_read rate %s", rate_str.c_str());
            return false;
        }
    }
    // stat_read_interval_ = rclcpp::Duration::from_seconds(1.0 / stat_read_rate);
    status_read_last_run_ = get_clock()->now();
    stat_read_stats_ = {0, 0, 0.0, 0, 0, 0.0, 0, 0};
    stat_read_stats_last_reset_ = get_clock()->now();

    RCLCPP_INFO(get_logger(), "stat_read loop configured");

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

            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, "torque_enable", &joint.torque_enable_));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, "temperature", &joint.temperature_));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, "voltage", &joint.voltage_));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, "error_overload", &joint.error_overload_));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, "error_electrical_shock", &joint.error_electrical_shock_));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, "error_motor_encoder", &joint.error_motor_encoder_));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, "error_overheating", &joint.error_overheating_));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, "error_input_voltage", &joint.error_input_voltage_));
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, "led", &joint.led_));
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
            RCLCPP_WARN(get_logger(), "failed to deactivate joint %s; communication error", joint.name_.c_str());
        }
        else if (dxl_error != 0) {
            RCLCPP_WARN(get_logger(), "failed to deactivate joint %s; device error %u", joint.name_.c_str(), dxl_error);
        }
        else {
            RCLCPP_INFO(get_logger(), "joint %s[%i] deactivated", joint.name_.c_str(), joint.id_);
        }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
MH5DynamixelBus::read(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
    int dxl_comm_result = COMM_TX_FAIL;

    // position, velocity, effort
    dxl_comm_result = pve_read_->txRxPacket();
    pve_read_stats_.total_packets++;
    pve_read_stats_.running_packets++;
    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_DEBUG(get_logger(), "pve SyncRead communication failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
        pve_read_stats_.total_errors++;
        pve_read_stats_.running_errors++;
    }
    else {
        for (auto  & joint : joints_) {
            if (joint.available_) {
                if (! pve_read_->isAvailable(joint.id_, 132, 4)) {
                    RCLCPP_DEBUG(get_logger(), "pve SyncRead getting position for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
                }
                else {
                    int32_t pos = pve_read_->getData(joint.id_, 132, 4);
                    joint.position_ = (pos - 2047) * 0.001533980787886;
                }

                if (! pve_read_->isAvailable(joint.id_, 128, 4)) {
                    RCLCPP_DEBUG(get_logger(), "pve SyncRead getting velocity for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
                }
                else {
                    int32_t vel = pve_read_->getData(joint.id_, 128, 4);
                    joint.velocity_ = vel  * 0.023980823922402;
                }

                if (! pve_read_->isAvailable(joint.id_, 126, 2)) {
                    RCLCPP_DEBUG(get_logger(), "pve SyncRead getting effort for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
                }
                else {
                    int16_t eff = pve_read_->getData(joint.id_, 126, 24);
                    joint.effort_ = eff * 0.0014;
                }
            }
        }
    }
    // update statistics
    pve_read_stats_.error_rate = pve_read_stats_.total_errors * 100.0 / pve_read_stats_.total_packets;
    if (time - pve_read_stats_last_reset_ > rclcpp::Duration::from_seconds(60)) {
        pve_read_stats_last_reset_ = time;
        pve_read_stats_.last_packets = pve_read_stats_.running_packets;
        pve_read_stats_.last_errors = pve_read_stats_.running_errors;
        pve_read_stats_.last_error_rate = pve_read_stats_.running_errors * 100.0 / pve_read_stats_.running_packets;
        pve_read_stats_.running_packets = 0;
        pve_read_stats_.running_errors = 0;
        RCLCPP_INFO(get_logger(), "pve_read stats: (%i, %i, %5.2f%%) (%i, %i, %5.2f%%, %5.2fHz)",
                    pve_read_stats_.total_packets, pve_read_stats_.total_errors, pve_read_stats_.error_rate,
                    pve_read_stats_.last_packets, pve_read_stats_.last_errors, pve_read_stats_.last_error_rate,
                    pve_read_stats_.last_packets / 60.0);
    }

    // status: torque, temperature, voltage, error, led
    rclcpp::Duration actual_passed = time - this->status_read_last_run_;
    if (actual_passed >= rclcpp::Duration::from_seconds(1.0 / status_read_rate_)) {
        dxl_comm_result = stat_read_->txRxPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_DEBUG(get_logger(), "stat SyncRead communication failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
        }
        else {
            this->status_read_last_run_ = time;
            for (auto  & joint : joints_) {
                if (joint.available_) {
                    if (! stat_read_->isAvailable(joint.id_, 224, 1)) {
                        RCLCPP_DEBUG(get_logger(), "stat SyncRead getting torque for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
                    }
                    else {
                        joint.torque_enable_ = stat_read_->getData(joint.id_, 224, 1);
                    }

                    if (! stat_read_->isAvailable(joint.id_, 225, 1)) {
                        RCLCPP_DEBUG(get_logger(), "stat SyncRead getting hwerr for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
                    }
                    else {
                        uint8_t data = stat_read_->getData(joint.id_, 225, 1);
                        joint.error_input_voltage_ = data & 1<<0;
                        joint.error_overheating_ = data & 1<<2;
                        joint.error_motor_encoder_ = data & 1<<3;
                        joint.error_electrical_shock_ = data & 1<<4;
                        joint.error_overload_ = data & 1<<5;
                    }

                    if (! stat_read_->isAvailable(joint.id_, 226, 2)) {
                        RCLCPP_DEBUG(get_logger(), "stat SyncRead getting voltage for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
                    }
                    else {
                        joint.voltage_ = stat_read_->getData(joint.id_, 226, 2);
                    }

                    if (! stat_read_->isAvailable(joint.id_, 228, 1)) {
                        RCLCPP_DEBUG(get_logger(), "stat SyncRead getting temp for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
                    }
                    else {
                        joint.temperature_ = stat_read_->getData(joint.id_, 228, 1);
                    }
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