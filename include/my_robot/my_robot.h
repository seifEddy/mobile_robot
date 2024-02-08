#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot() 
  { 
    // Initialization of the robot's resources (joints, sensors, actuators) and
    // interfaces can be done here or inside init().
    // E.g. parse the URDF for joint names & interfaces, then initialize them
  }
  bool init(ros::NodeHandle &robot_nh, ros::NodeHandle &robot_hw_nh)
  {
    hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_a);
    hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_b);

    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("left_joint"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_a);
    hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("right_joint"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_b);

    registerInterface(&jnt_pos_interface);
    return true;
  }
  void write(){}
  void read(){}
private:

  hardware_interface::JointStateInterface jnt_state_interface;

  hardware_interface::PositionJointInterface jnt_pos_interface;

  double cmd[2];
  
  // Data member arrays to store the state of the robot's resources (joints, sensors)
  double pos[2];
  double vel[2];
  double eff[2];
};