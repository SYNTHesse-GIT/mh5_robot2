#ifndef MH5_HARDWARE_DYNAMIXEL_JOINT_HPP_
#define MH5_HARDWARE_DYNAMIXEL_JOINT_HPP_


namespace mh5_hardware
{

struct DynamixelJoint
{
    std::string                         name_;
    int                                 id_;
    bool                                available_;

    double                              position_;
    double                              velocity_;
    double                              effort_;

    double                              position_command_;
    double                              velocity_command_;
    double                              effort_command_;

    explicit DynamixelJoint(std::string name, int id)
        : name_(name), id_(id), available_(true),
          position_(0), velocity_(0), effort_(0),
          position_command_(0), velocity_command_(0), effort_command_(0)
        {}

    // void setAvailable(const bool avaialable) {available_ = avaialable;}

};


} // namespace mh5_hardware

#endif // MH5_HARDWARE_DYNAMIXEL_JOINT_HPP_