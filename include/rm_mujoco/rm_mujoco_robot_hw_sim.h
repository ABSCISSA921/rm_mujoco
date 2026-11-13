#pragma once

#include <list>
#include <string>
#include <vector>

#include <mujoco_ros_control/default_robot_hw_sim.h>
#include <rm_common/hardware_interface/rm_imu_sensor_interface.h>
#include <rm_common/hardware_interface/robot_state_interface.h>
#include <std_srvs/Trigger.h>

namespace rm_mujoco
{
struct PassiveJointData
{
  int joint_id;
  double position{};
  double velocity{};
  double effort{};
};

struct ImuData
{
  int quat_adr{ -1 };
  int gyro_adr{ -1 };
  int acc_adr{ -1 };
  ros::Time time_stamp;
  double ori[4]{};
  double ori_cov[9]{};
  double angular_vel[3]{};
  double angular_vel_cov[9]{};
  double linear_acc[3]{};
  double linear_acc_cov[9]{};
};

class RmMujocoRobotHWSim : public mujoco_ros::control::DefaultRobotHWSim
{
public:
  bool initSim(const mjModel* m_ptr, mjData* d_ptr, mujoco_ros::MujocoEnv* mujoco_env_ptr,
               const std::string& robot_namespace, ros::NodeHandle model_nh, const urdf::Model* const urdf_model,
               std::vector<transmission_interface::TransmissionInfo> transmissions) override;

  void readSim(ros::Time time, ros::Duration period) override;

private:
  void parseImu(XmlRpc::XmlRpcValue& imu_datas);

  static bool switchImuStatus(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

  rm_control::RobotStateInterface robot_state_interface_;
  hardware_interface::ImuSensorInterface imu_sensor_interface_;
  rm_control::RmImuSensorInterface rm_imu_sensor_interface_;

  std::list<ImuData> imu_datas_;
  ros::ServiceServer switch_imu_service_;

  // Stable storage for the read-only handles used by RobotStateController.
  std::list<PassiveJointData> passive_joint_datas_;

  static bool disable_imu_;
};

}  // namespace rm_mujoco
