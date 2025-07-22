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

        if (joint.parameters["model"] == "") {
            RCLCPP_ERROR(get_logger(), "missing model for joint %s", joint.name.c_str());
            return false;
        }
        if (joint.parameters["model"] != "XL430" && joint.parameters["model"] != "XL330") {
            RCLCPP_ERROR(get_logger(), "unknown model for joint %s: %s", joint.name.c_str(), joint.parameters["model"].c_str());
            return false;
        }
        j.model_ = joint.parameters["model"];
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


int MH5DynamixelBus::parseIntParam(const std::string & param, const int def, const std::string mu)
{
    std::string param_str = info_.hardware_parameters[param];
    int param_int;
    if (param_str == "") {
        RCLCPP_INFO(get_logger(), "parameter %s not specified, will default to %i%s", param.c_str(), def, mu.c_str());
        return def;
    }
    try
    {
        param_int = stod(param_str);
    }
    catch(std::invalid_argument const& ex)
    {
        RCLCPP_WARN(get_logger(), "failed to parse parameter %s; will default to %i%s", param_str.c_str(), def, mu.c_str());
        return def;
    }
    RCLCPP_INFO(get_logger(), "using %s: %i%s", param.c_str(), param_int, mu.c_str());
    return param_int;
}


bool MH5DynamixelBus::setupDynamixelLoops()
{
    // dynamixel state (error, led, active, temperature, voltage)
    info1_read_ = std::make_unique<dynamixel::GroupSyncRead>(portHandler_, packetHandler_, 144, 3);
    info2_read_ = std::make_unique<dynamixel::GroupSyncRead>(portHandler_, packetHandler_, 224, 4);
    for (auto & joint : joints_) {
        if (joint.available_) {
            if(!info1_read_->addParam(joint.id_)) {
                RCLCPP_ERROR(get_logger(), "info_read(temp, volt) failed to add joint id %i. This should not happen.", joint.id_);
                return false;
            }
            if (!info2_read_->addParam(joint.id_)) {
                RCLCPP_ERROR(get_logger(), "info_read(torque, hwerr, led, moving) failed to add joint id %i. This should not happen.", joint.id_);
                return false;
            }
            RCLCPP_DEBUG(get_logger(), "info_read loop added joint %s[%i]", joint.name_.c_str(), joint.id_);
        }
    }
    int info_rate = parseIntParam("info_read_rate", 1, "Hz");
    int horiz = parseIntParam("info_read_horizon", 60, "s");
    // we use info_rate*2 to account for the swapping between the two SyncReads
    info_read_stats_ = PacketCounter(info_rate*2, horiz, get_clock()->now());
    RCLCPP_INFO(get_logger(), "info_read loop configured");
    RCLCPP_INFO(get_logger(), "technical rate: %i", info_read_stats_.rate_);

    // position, velocity, effort
    pve_read_ = std::make_unique<dynamixel::GroupSyncRead>(portHandler_, packetHandler_, 126, 10);
    for (auto & joint : joints_) {
        if (joint.available_) {
            pve_read_->addParam(joint.id_);
            RCLCPP_DEBUG(get_logger(), "pve_read loop added joint %s[%i]", joint.name_.c_str(), joint.id_);
        }
    }
    int pve_rate = parseIntParam("pve_read_rate", 100, "Hz");
    horiz = parseIntParam("pve_read_horizon", 60, "s");
    // we increase the rate to accomodate the loops skipped due to info_read
    pve_read_stats_ = PacketCounter(pve_rate + info_rate * 2, horiz, get_clock()->now());
    RCLCPP_INFO(get_logger(), "pve_read loop configured");
    RCLCPP_INFO(get_logger(), "technical rate: %i", pve_read_stats_.rate_);

    int torque_rate = parseIntParam("torque_write_rate", 1, "Hz");
    horiz = parseIntParam("torque_write_horizon", 60, "s");
    torque_write_stats_ = PacketCounter(torque_rate, horiz, get_clock()->now());

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
            state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name_, "moving", &joint.moving_));
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

        for (auto & joint : joints_) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(joint.name_, "torque_enable", &joint.torque_command_));
        }
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
MH5DynamixelBus::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // status: torque, temperature, voltage, error, led
    if (info_read_stats_.shouldRun(time, period)) {
        // to reduce the impact on other read loops we use the counter_ to only run half of the SyncReads
        if (info_read_stats_.slice_ % 2 == 0) {
            info_read_stats_.addRun(time);
            bool result = read_info1();
            if (!result) {
                info_read_stats_.addErr();
            }
         }
        else {
            info_read_stats_.addRun(time);
            bool result = read_info2();
            if (!result) {
                info_read_stats_.addErr();
            }
        }
        info_read_stats_.slice_++;
        if (info_read_stats_.shouldReset(time)) {
            info_read_stats_.reset(time);
            RCLCPP_INFO(get_logger(), "info_read stats: (%i, %i, %5.2f%%) (%i, %i, %5.2f%%, %5.2fHz)",
                            info_read_stats_.total_packets_, info_read_stats_.total_errors_, info_read_stats_.error_rate_,
                            info_read_stats_.last_packets_, info_read_stats_.last_errors_, info_read_stats_.last_error_rate_,
                            (float) info_read_stats_.last_packets_ / info_read_stats_.horizon_);
        }
        // to avoid overrunning the time alloted for the loop we will return here and skip the (most more frequent) pve_read
        // that will be run next time
        return hardware_interface::return_type::OK;
    }

    // position, velocity, effort
    if (pve_read_stats_.shouldRun(time, period)) {
        pve_read_stats_.addRun(time);
        bool result = read_pve();
        if (! result) {
            pve_read_stats_.addErr();
        }
        if (pve_read_stats_.shouldReset(time)) {
            pve_read_stats_.reset(time);
            RCLCPP_INFO(get_logger(), "pve_read stats: (%i, %i, %5.2f%%) (%i, %i, %5.2f%%, %5.2fHz)",
                            pve_read_stats_.total_packets_, pve_read_stats_.total_errors_, pve_read_stats_.error_rate_,
                            pve_read_stats_.last_packets_, pve_read_stats_.last_errors_, pve_read_stats_.last_error_rate_,
                            (float) pve_read_stats_.last_packets_ / pve_read_stats_.horizon_);
        }
    }

    return hardware_interface::return_type::OK;
}


bool MH5DynamixelBus::read_pve()
{
    int dxl_comm_result = pve_read_->txRxPacket();

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_DEBUG(get_logger(), "pve SyncRead communication failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }

    for (auto  & joint : joints_) {
        if (joint.available_) {
            if (! pve_read_->isAvailable(joint.id_, 132, 4)) {
                RCLCPP_DEBUG(get_logger(), "pve SyncRead getting position for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
            }
            else {
                int32_t pos = pve_read_->getData(joint.id_, 132, 4);
                joint.position_ = (pos - 2047) * 0.001533980787886;
                if (!joint.torque_enable_) {
                    joint.position_command_ = joint.position_;
                }
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

    return true;
}


bool MH5DynamixelBus::read_info1()
{
    int dxl_comm_result = info1_read_->txRxPacket();

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_DEBUG(get_logger(), "info_read(temp, volt) communication failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }

    for (auto  & joint : joints_) {
        if (joint.available_) {
            // voltage
            if (! info1_read_->isAvailable(joint.id_, 144, 2)) {
                RCLCPP_DEBUG(get_logger(), "info_read getting voltage for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
            }
            else {
                joint.voltage_ = info1_read_->getData(joint.id_, 144, 2);
            }
            // temp
            if (! info1_read_->isAvailable(joint.id_, 146, 1)) {
                RCLCPP_DEBUG(get_logger(), "info_read getting temp for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
            }
            else {
                joint.temperature_ = info1_read_->getData(joint.id_, 146, 1);
            }
        }
    }

        return true;
}


bool MH5DynamixelBus::read_info2()
{
    int dxl_comm_result = info2_read_->txRxPacket();

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_DEBUG(get_logger(), "info_read(torque, hwerr, led, moving) communication failed: %s", packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }

    for (auto  & joint : joints_) {
        if (joint.available_) {
            // torque
            if (! info2_read_->isAvailable(joint.id_, 224, 1)) {
                RCLCPP_DEBUG(get_logger(), "info_read getting torque for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
            }
            else {
                joint.torque_enable_ = info2_read_->getData(joint.id_, 224, 1);
            }
            // led
            if (! info2_read_->isAvailable(joint.id_, 225, 1)) {
                RCLCPP_DEBUG(get_logger(), "info_read getting led for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
            }
            else {
                joint.led_ = info2_read_->getData(joint.id_, 225, 1);
            }
            // hwerr
            if (! info2_read_->isAvailable(joint.id_,226, 1)) {
                RCLCPP_DEBUG(get_logger(), "info_read getting hwerr for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
            }
            else {
                uint8_t data = info2_read_->getData(joint.id_, 226, 1);
                joint.error_input_voltage_ = data & 1<<0;
                joint.error_overheating_ = data & 1<<2;
                joint.error_motor_encoder_ = data & 1<<3;
                joint.error_electrical_shock_ = data & 1<<4;
                joint.error_overload_ = data & 1<<5;
            }
            // moving
            if (! info2_read_->isAvailable(joint.id_, 227, 1)) {
                RCLCPP_DEBUG(get_logger(), "info_read getting led for joint %s[%i] failed", joint.name_.c_str(), joint.id_);
            }
            else {
                joint.moving_ = info2_read_->getData(joint.id_, 227, 1);
            }
        }
    }

    return true;
}


hardware_interface::return_type
MH5DynamixelBus::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    uint8_t dxl_error;
    // torque enable
    // because this is one register and is very low frequency we simply write with normal write_register
    if (torque_write_stats_.shouldRun(time, period)) {
        for (auto & joint : joints_) {
            if (joint.torque_command_ < 0.0) {
                // this acts like a "leave in place"
                continue;
            }
            if (joint.torque_command_ > 0.0) {
                joint.torque_command_ = 1.0;
            }
            if (joint.torque_command_ != joint.torque_enable_) {
                torque_write_stats_.addRun(time);
                if (packetHandler_->write1ByteTxRx(portHandler_, joint.id_, 64, (uint8_t)joint.torque_command_, &dxl_error) != COMM_SUCCESS) {
                    torque_write_stats_.addErr();
                    RCLCPP_WARN(get_logger(), "failed to toggle torque for joint %s; communication error", joint.name_.c_str());
                }
                else if (dxl_error != 0) {
                    RCLCPP_WARN(get_logger(), "failed to toggle torque for joint %s; device error %u", joint.name_.c_str(), dxl_error);
                    // this does not count as a communication error, hence we do not increment the counters
                }
                else {
                    RCLCPP_INFO(get_logger(), "joint %s[%i] %s", joint.name_.c_str(), joint.id_, joint.torque_command_ == 0 ? "deactivated":"activated" );
                }
            }
        }
    }
    if (torque_write_stats_.shouldReset(time)) {
        torque_write_stats_.reset(time);
        RCLCPP_INFO(get_logger(), "torque_write stats: (%i, %i, %5.2f%%) (%i, %i, %5.2f%%, %5.2fHz)",
                torque_write_stats_.total_packets_, torque_write_stats_.total_errors_, torque_write_stats_.error_rate_,
                torque_write_stats_.last_packets_, torque_write_stats_.last_errors_, torque_write_stats_.last_error_rate_,
                (float) torque_write_stats_.last_packets_ / torque_write_stats_.horizon_);
    }

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