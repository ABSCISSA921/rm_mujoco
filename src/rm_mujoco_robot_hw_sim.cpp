#include "rm_mujoco/rm_mujoco_robot_hw_sim.h"

#include <algorithm>
#include <cmath>

#include <pluginlib/class_list_macros.h>

namespace rm_mujoco
{
namespace
{
int sensorAddress(const mjModel* model, const std::string& sensor_name)
{
  const int id = mj_name2id(model, mjOBJ_SENSOR, sensor_name.c_str());
  return id < 0 ? -1 : static_cast<int>(model->sensor_adr[id]);
}

bool readDiagonal(XmlRpc::XmlRpcValue& value, double out[3])
{
  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray || value.size() != 3)
    return false;
  for (int i = 0; i < 3; ++i)
  {
    const XmlRpc::XmlRpcValue::Type type = value[i].getType();
    if (type != XmlRpc::XmlRpcValue::TypeDouble && type != XmlRpc::XmlRpcValue::TypeInt)
      return false;
    out[i] = type == XmlRpc::XmlRpcValue::TypeInt ? static_cast<int>(value[i]) : static_cast<double>(value[i]);
  }
  return true;
}
}  // namespace

bool RmMujocoRobotHWSim::initSim(const mjModel* m_ptr, mjData* d_ptr, mujoco_ros::MujocoEnv* mujoco_env_ptr,
                                 const std::string& robot_namespace, ros::NodeHandle model_nh,
                                 const urdf::Model* const urdf_model,
                                 std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  if (!mujoco_ros::control::DefaultRobotHWSim::initSim(m_ptr, d_ptr, mujoco_env_ptr, robot_namespace, model_nh,
                                                       urdf_model, transmissions))
    return false;

  const auto active_names = js_interface_.getNames();
  for (int id = 0; id < m_ptr_->njnt; ++id)
  {
    const char* name = mj_id2name(m_ptr_, mjOBJ_JOINT, id);
    if (name == nullptr || (m_ptr_->jnt_type[id] != mjJNT_HINGE && m_ptr_->jnt_type[id] != mjJNT_SLIDE) ||
        std::find(active_names.begin(), active_names.end(), name) != active_names.end() ||
        urdf_model->getJoint(name) == nullptr)
      continue;
    passive_joint_datas_.push_back({ id });
    auto& joint = passive_joint_datas_.back();
    js_interface_.registerHandle(
        hardware_interface::JointStateHandle(name, &joint.position, &joint.velocity, &joint.effort));
  }

  registerInterface(&robot_state_interface_);
  registerInterface(&imu_sensor_interface_);
  registerInterface(&rm_imu_sensor_interface_);

  XmlRpc::XmlRpcValue imu_config;
  if (!model_nh.getParam("imus", imu_config))
    ROS_WARN("No imu specified");
  else
    parseImu(imu_config);

  switch_imu_service_ = model_nh.advertiseService("switch_imu_status", switchImuStatus);
  return true;
}

void RmMujocoRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  mujoco_ros::control::DefaultRobotHWSim::readSim(time, period);

  for (auto& joint : passive_joint_datas_)
  {
    const int dof = m_ptr_->jnt_dofadr[joint.joint_id];
    joint.position = d_ptr_->qpos[m_ptr_->jnt_qposadr[joint.joint_id]];
    joint.velocity = d_ptr_->qvel[dof];
    // Same semantics as DefaultRobotHWSim: externally applied generalized force,
    // not the constraint reaction or a measured motor torque.
    joint.effort = d_ptr_->qfrc_applied[dof];
  }

  if (!disable_imu_)
  {
    for (auto& imu : imu_datas_)
    {
      imu.time_stamp = time;

      if (imu.quat_adr >= 0)
      {
        const mjtNum* quat = d_ptr_->sensordata + imu.quat_adr;
        imu.ori[0] = quat[1];
        imu.ori[1] = quat[2];
        imu.ori[2] = quat[3];
        imu.ori[3] = quat[0];
      }

      if (imu.gyro_adr >= 0)
      {
        const mjtNum* gyro = d_ptr_->sensordata + imu.gyro_adr;
        imu.angular_vel[0] = gyro[0];
        imu.angular_vel[1] = gyro[1];
        imu.angular_vel[2] = gyro[2];
      }

      if (imu.acc_adr >= 0)
      {
        const mjtNum* acc = d_ptr_->sensordata + imu.acc_adr;
        imu.linear_acc[0] = acc[0];
        imu.linear_acc[1] = acc[1];
        imu.linear_acc[2] = acc[2];
      }
    }
  }

  for (auto& cmd : joint_effort_command_)
    cmd = 0.;
  for (auto& cmd : joint_velocity_command_)
    cmd = 0.;
}

void RmMujocoRobotHWSim::parseImu(XmlRpc::XmlRpcValue& imu_datas)
{
  if (imu_datas.getType() != XmlRpc::XmlRpcValue::TypeStruct)
  {
    ROS_ERROR_STREAM("Imu parameter is not a struct.");
    return;
  }

  for (auto it = imu_datas.begin(); it != imu_datas.end(); ++it)
  {
    if (!it->second.hasMember("frame_id"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated frame id.");
      continue;
    }
    else if (!it->second.hasMember("orientation_covariance_diagonal"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated orientation covariance diagonal.");
      continue;
    }
    else if (!it->second.hasMember("angular_velocity_covariance"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated angular velocity covariance.");
      continue;
    }
    else if (!it->second.hasMember("linear_acceleration_covariance"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated linear acceleration covariance.");
      continue;
    }

    const std::string frame_id = imu_datas[it->first]["frame_id"];
    const int body_id = mj_name2id(m_ptr_, mjOBJ_BODY, frame_id.c_str());
    if (body_id < 0)
    {
      ROS_WARN("Imu %s is not specified in the mujoco model.", it->first.c_str());
      continue;
    }

    double ori_cov[3], angular_cov[3], linear_cov[3];
    if (!readDiagonal(imu_datas[it->first]["orientation_covariance_diagonal"], ori_cov) ||
        !readDiagonal(imu_datas[it->first]["angular_velocity_covariance"], angular_cov) ||
        !readDiagonal(imu_datas[it->first]["linear_acceleration_covariance"], linear_cov))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has malformed covariance parameters, expected three values each.");
      continue;
    }

    ImuData imu_data;
    imu_data.quat_adr = sensorAddress(m_ptr_, frame_id + "_quat");
    imu_data.gyro_adr = sensorAddress(m_ptr_, frame_id + "_gyro");
    imu_data.acc_adr = sensorAddress(m_ptr_, frame_id + "_acc");
    if (imu_data.quat_adr < 0 || imu_data.gyro_adr < 0 || imu_data.acc_adr < 0)
      ROS_WARN_STREAM("Imu " << it->first << ": expected sensors '" << frame_id << "_quat', '" << frame_id
                             << "_gyro' and '" << frame_id
                             << "_acc' in the mujoco model, missing entries are "
                                "published as zero.");
    imu_data.ori_cov[0] = ori_cov[0];
    imu_data.ori_cov[4] = ori_cov[1];
    imu_data.ori_cov[8] = ori_cov[2];
    imu_data.angular_vel_cov[0] = angular_cov[0];
    imu_data.angular_vel_cov[4] = angular_cov[1];
    imu_data.angular_vel_cov[8] = angular_cov[2];
    imu_data.linear_acc_cov[0] = linear_cov[0];
    imu_data.linear_acc_cov[4] = linear_cov[1];
    imu_data.linear_acc_cov[8] = linear_cov[2];

    imu_datas_.push_back(imu_data);
    ImuData& back = imu_datas_.back();
    const hardware_interface::ImuSensorHandle imu_sensor_handle(it->first, frame_id, back.ori, back.ori_cov,
                                                                back.angular_vel, back.angular_vel_cov, back.linear_acc,
                                                                back.linear_acc_cov);
    imu_sensor_interface_.registerHandle(imu_sensor_handle);
    rm_imu_sensor_interface_.registerHandle(rm_control::RmImuSensorHandle(imu_sensor_handle, &back.time_stamp));
  }
}

bool RmMujocoRobotHWSim::switchImuStatus(std_srvs::TriggerRequest& /*req*/, std_srvs::TriggerResponse& res)
{
  disable_imu_ = !disable_imu_;
  res.success = true;
  res.message = disable_imu_ ? "disable" : "enable";
  return true;
}

bool RmMujocoRobotHWSim::disable_imu_ = false;
}  // namespace rm_mujoco

PLUGINLIB_EXPORT_CLASS(rm_mujoco::RmMujocoRobotHWSim, mujoco_ros::control::RobotHWSim)
