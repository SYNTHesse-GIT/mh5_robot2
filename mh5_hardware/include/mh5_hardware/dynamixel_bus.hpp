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

#ifndef MH5_HARDWARE_DYNAMIXEL_BUS_HPP_
#define MH5_HARDWARE_DYNAMIXEL_BUS_HPP_

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "dynamixel_sdk/packet_handler.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "mh5_hardware/port_handler.hpp"
#include "mh5_hardware/dynamixel_joint.hpp"
#include "mh5_hardware/dynamixel_loop.hpp"

namespace mh5_hardware
{

// typedef struct {
//   int           total_packets;
//   int           total_errors;
//   double        error_rate;
//   int           last_packets;
//   int           last_errors;
//   double        last_error_rate;
//   int           running_packets;
//   int           running_errors;
// } communication_stats_t;

/**
 * @brief Main class implementing the protocol required by ``ros2_control`` for
 * providing access to the robot hardware.
 *
 * This class performs communication with the servos using Dynamixel protocol
 * and manages the state of these servos. It uses for this purpose
 * [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) (specifically the
 * ROS implementation of it) with the only exception that for port communication
 * it uses a custom subclass of ``PortHandler`` in order to be able to configure
 * the communication port with RS485 support, because the interface board used
 * by RH5 robot uses SC16IS762 chips with hardware control flow that needs
 * to be configured in RS485 mode via ``ioctl``.
 *
 * The class uses the information from the URDF to get details about the
 * communication port configuration and the attached servos. For each dynamixel
 * interface the following parameters are read:
 *
 */
class MH5DynamixelBus: public hardware_interface::SystemInterface
{

public:
  // ~MH5DynamixelBus(); // handles the Ctr-C shutdown

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

//   hardware_interface::return_type prepare_command_mode_switch(
//     const std::vector<std::string> & start_interfaces,
//     const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // mh5_port_handler::PortHandlerMH5* getPortHandler() {return portHandler_; };
  // dynamixel::PacketHandler* getPacketHandler() {return packetHandler_; };

  int parseIntParam(const std::string & param, const int def, const std::string mu);

  // friend class DynamixelLoop;
  // friend class PVEReader;

protected:

    // communication
    std::string     port_;
    int             baudrate_;
    bool            rs485_;
    double          protocol_;
    int             latency_;

    // dynamixel communication
    mh5_port_handler::PortHandlerMH5    *portHandler_;
    dynamixel::PacketHandler            *packetHandler_;

    // resources
    int                                 num_joints_;
    std::vector<DynamixelJoint>         joints_{};

    // dynamixel loops
    std::unique_ptr<dynamixel::GroupSyncRead>         pve_read_;
    PacketCounter                                     pve_read_stats_;

    /**
     * for reading information about the state of the device we have two separate
     * SyncReads because the data is spread in the registry and we cannot use the
     * indirect registers for all data because of the different treatment between
     * XL430 and XL330. So the solution is:
     * - for temp and voltage we will use one SyncRead that will read all devices
     * - for torque, led, hwerr and moving we setup indirect registers in the only 4
     * registers that overlap between XL430 and XL330: data regs 224-227. These are
     * configured in .XACRO file differently for XL430 and XL330 servos.
     * Now the only problem is that runing both SyncReads in the same time in a read()
     * loop will most likely break the allotted time and ros control framework will
     * issue overrun warnings everytime the info_read is run. For this we will
     * alternate between the info1_read and info2_read using the slice_ parameter in the
     * info_read_stats.
     */
    std::unique_ptr<dynamixel::GroupSyncRead>         info1_read_;    // temp, voltage
    std::unique_ptr<dynamixel::GroupSyncRead>         info2_read_;    // torque, hwerr, led, moving
    PacketCounter                                     info_read_stats_;

    PacketCounter                                     torque_write_stats_;

    // int                         num_sensors_;
    // std::vector<FootSensor *>   foot_sensors_;

    /**
     * @brief Initializes the Dynamixel port.
     *
     * @return true if initialization was successful
     * @return false if initialization was unsuccessful
     */
    bool initPort();

    /**
     * @brief Initializes the joints.
     *
     * @return true if all joints have been initialized successfully
     * @return false if any of the joints raised errors
     */
    bool initJoints();

    /**
     * @brief Initializes the sensors.
     *
     * @return true
     * @return false
     */
    bool initSensors();

    /**
     * @brief Convenience function that constructs a loop, reads parameters
     * "rates/<loop_name>" from parameter server or, if not found, uses
     * a default rate for initialisation. It also calls prepare() and
     * registers it communication handle (from getCommStatHandle() with the
     * HW communication status inteface)
     *
     * @tparam Loop the class for the loop
     * @param name the name of the loop
     * @param default_rate the default rate to use incase no parameter is
     * found in the parameter server
     * @return Loop* the newly created loop object
     */
    // template <class Loop>
    // Loop* setupLoop(std::string name, const double default_rate);

    /**
     * @brief Creates and initializes all the loops used by the HW interface:
     * - Read: position, velocity, effort (pve_reader)
     * - Read: temperature, voltage (tv_reader)
     * - Write: position, velocity (pv_writer)
     * - Write: torque (t_writer)
     *
     * @return true
     */
    bool setupDynamixelLoops();

    // bool ping(const uint8_t id, const int num_tries=5);

    bool read_pve();
    bool read_info1();
    bool read_info2();

};

} //namespace mh5_hardware

#endif // MH5_HARDWARE_DYNAMIXEL_BUS_HPP_