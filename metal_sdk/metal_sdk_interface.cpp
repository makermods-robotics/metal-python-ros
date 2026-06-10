// bindings/wrapper.cpp

#include <pybind11/functional.h> // Reserved for future callbacks
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // std::vector support
// Include numpy.h or buffer.h if numpy arrays or buffers are needed

#include "metal_sdk/metal_sdk_interface.h"

namespace py = pybind11;

PYBIND11_MODULE(metal_sdk, m) {
  m.doc() = "Python binding for MetalSDKInterface";

  py::class_<makermods::metal::MitControlCommand>(m, "MitControlCommand")
      .def(py::init<>())
      .def_readwrite("kp", &makermods::metal::MitControlCommand::kp)
      .def_readwrite("joint_position",
                     &makermods::metal::MitControlCommand::joint_position)
      .def_readwrite("kd", &makermods::metal::MitControlCommand::kd)
      .def_readwrite("joint_velocity",
                     &makermods::metal::MitControlCommand::joint_velocity)
      .def_readwrite("torque", &makermods::metal::MitControlCommand::torque);

  // Bind ControlMode enum
  py::enum_<makermods::metal::MetalSDKInterface::ControlMode>(m, "ControlMode")
      .value("GRAVITY_COMPENSATION",
             makermods::metal::MetalSDKInterface::GRAVITY_COMPENSATION)
      .value("RT_JOINT_POSITION",
             makermods::metal::MetalSDKInterface::RT_JOINT_POSITION)
      .value("NRT_JOINT_POSITION",
             makermods::metal::MetalSDKInterface::NRT_JOINT_POSITION)
      .value("MIT_CONTROL",
             makermods::metal::MetalSDKInterface::MIT_CONTROL)
      .export_values();

  // Bind MetalSDKInterface
  py::class_<makermods::metal::MetalSDKInterface>(m, "MetalSDKInterface")
      // Constructor
      .def(py::init<const std::string &, const std::string &, int, bool>(),
           py::arg("can_id"), py::arg("urdf_path"), py::arg("arm_end_type"),
           py::arg("enable_arm"))
      // Destructor is handled automatically

      // Methods
      .def("Init", &makermods::metal::MetalSDKInterface::Init,
           "Initialize the SDK interface. Returns true if success.")
      .def("GetJointNames",
           &makermods::metal::MetalSDKInterface::GetJointNames,
           "Returns the joint names (6 or 7 including gripper).")
      .def("GetRotorTemperature",
           &makermods::metal::MetalSDKInterface::GetRotorTemperature,
           "Returns rotor (coil) temperature for all joints.")
      .def("GetJointErrorCode",
           &makermods::metal::MetalSDKInterface::GetJointErrorCode,
           "Returns error codes for all joints.")
      .def("GetMotorCurrent",
           &makermods::metal::MetalSDKInterface::GetMotorCurrent,
           "Returns motor current for all joints.")
      .def("GetJointPosition",
           &makermods::metal::MetalSDKInterface::GetJointPosition,
           "Joint positions.")
      .def("GetJointVelocity",
           &makermods::metal::MetalSDKInterface::GetJointVelocity,
           "Joint velocities.")
      .def("GetJointEffort",
           &makermods::metal::MetalSDKInterface::GetJointEffort,
           "Joint torques / efforts.")
      .def("GetArmEndPose",
           &makermods::metal::MetalSDKInterface::GetArmEndPose,
           "End pose of the arm: [x, y, z, roll, pitch, yaw]")

      .def("SetArmControlMode",
           &makermods::metal::MetalSDKInterface::SetArmControlMode,
           "Set control mode", py::arg("mode"))

      // Control functions
      .def("SetArmJointPosition",
           (void (makermods::metal::MetalSDKInterface::*)(
               const std::array<double, 6> &,
               int))&makermods::metal::MetalSDKInterface::SetArmJointPosition,
           "Set joint positions by std::array<double,6> with optional velocity "
           "ratio",
           py::arg("arm_joint_position"), py::arg("velocity_ratio") = 5)

      .def(
          "SetFollowerArmJointPosition",
          (void (makermods::metal::MetalSDKInterface::*)(
              const std::vector<double>
                  &))&makermods::metal::MetalSDKInterface::SetArmJointPosition,
          "Set joint positions by vector<double>",
          py::arg("arm_joint_position"))

      .def("SetArmEndPose",
           &makermods::metal::MetalSDKInterface::SetArmEndPose,
           "Set end pose [x,y,z,roll,pitch,yaw] and return calculated joint "
           "positions from inverse kinematics, with optional velocity ratio",
           py::arg("arm_end_pose"), py::arg("velocity_ratio") = 5)

      .def("SetGripperStroke",
           &makermods::metal::MetalSDKInterface::SetGripperStroke,
           "Set gripper stroke in mm", py::arg("gripper_stroke"),
           py::arg("velocity_ratio") = 5)

      .def("SetGripperMode",
           &makermods::metal::MetalSDKInterface::SetGripperMode,
           "Set gripper control mode in gravity-compensation mode",
           py::arg("torque_compensation"))

      .def("MitControlArm",
           &makermods::metal::MetalSDKInterface::MitControlArm,
           "MIT control command for J1-J6 arm motors",
           py::arg("arm_control_command"))

      .def("MitControlGripper",
           &makermods::metal::MetalSDKInterface::MitControlGripper,
           "MIT control command for the gripper",
           py::arg("gripper_control_command"))

      .def("SetEnableArm", &makermods::metal::MetalSDKInterface::SetEnableArm,
           "Enable or disable arm motors", py::arg("enable_flag"))

      .def("SaveJ6ZeroPosition",
           &makermods::metal::MetalSDKInterface::SaveJ6ZeroPosition,
           "Save zero position for J6 joint when changing end flange.");

  // End module
}
