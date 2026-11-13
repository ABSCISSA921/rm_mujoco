#include <gtest/gtest.h>
#include <rm_mujoco/rm_mujoco_robot_hw_sim.h>
#include <transmission_interface/transmission_parser.h>

#include <cstdlib>
#include <memory>
#include <numeric>
#include <sstream>

TEST(RmMujocoHardware, ResourceMappingAndAppliedEffort)
{
  ros::NodeHandle nh;
  std::string model_path, description;
  ASSERT_TRUE(nh.getParam("test_model", model_path));
  ASSERT_TRUE(nh.getParam("robot_description", description));
  char error[1024]{};
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
      mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error)), mj_deleteModel);
  ASSERT_NE(model, nullptr) << error;
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data(mj_makeData(model.get()), mj_deleteData);
  ASSERT_EQ(model->nu, 0);  // One actuation path: upstream qfrc_applied.
  ASSERT_EQ(model->neq, 4);
  EXPECT_EQ(mj_version(), mjVERSION_HEADER);

  EXPECT_NEAR(std::accumulate(model->body_mass, model->body_mass + model->nbody, 0.0), 20.41884, 1e-8);
  XmlRpc::XmlRpcValue initial;
  ASSERT_TRUE(nh.getParam("initial_joint_positions/joint_map", initial));
  ASSERT_EQ(initial.size(), model->njnt);
  for (int id = 0; id < model->njnt; ++id)
  {
    const std::string name = mj_id2name(model.get(), mjOBJ_JOINT, id);
    ASSERT_TRUE(initial.hasMember(name));
    std::istringstream values(static_cast<std::string>(initial[name]));
    const int count = model->jnt_type[id] == mjJNT_FREE ? 7 : 1;
    for (int i = 0; i < count; ++i)
      ASSERT_TRUE(static_cast<bool>(values >> data->qpos[model->jnt_qposadr[id] + i]));
  }
  mj_forward(model.get(), data.get());
  for (const auto& side : { std::string("left"), std::string("right") })
    for (int i : { 1, 2 })
    {
      const auto name = side + "_connect_joint" + std::to_string(i);
      const int a = mj_name2id(model.get(), mjOBJ_SITE, (name + "_a").c_str());
      const int b = mj_name2id(model.get(), mjOBJ_SITE, (name + "_b").c_str());
      ASSERT_GE(a, 0);
      ASSERT_GE(b, 0);
      for (int axis = 0; axis < 3; ++axis)
        EXPECT_NEAR(data->site_xpos[3 * a + axis], data->site_xpos[3 * b + axis], 1e-9);
    }

  urdf::Model urdf;
  ASSERT_TRUE(urdf.initString(description));
  std::vector<transmission_interface::TransmissionInfo> transmissions;
  ASSERT_TRUE(transmission_interface::TransmissionParser::parse(description, transmissions));
  ASSERT_EQ(transmissions.size(), 6U);
  rm_mujoco::RmMujocoRobotHWSim hw;
  ASSERT_TRUE(hw.initSim(model.get(), data.get(), nullptr, "", nh, &urdf, transmissions));
  auto* states = hw.get<hardware_interface::JointStateInterface>();
  auto* efforts = hw.get<hardware_interface::EffortJointInterface>();
  ASSERT_NE(states, nullptr);
  ASSERT_NE(efforts, nullptr);
  EXPECT_EQ(states->getNames().size(), 14U);
  EXPECT_EQ(efforts->getNames().size(), 6U);
  ASSERT_NE(hw.get<rm_control::RobotStateInterface>(), nullptr);

  // Static I/O test, not a mechanically valid trajectory or an action test.
  for (const auto& name : states->getNames())
  {
    const int id = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
    ASSERT_GE(id, 0);
    data->qpos[model->jnt_qposadr[id]] = 0.01 * id;
    data->qvel[model->jnt_dofadr[id]] = -0.02 * id;
  }
  mj_forward(model.get(), data.get());
  hw.readSim(ros::Time(1), ros::Duration(0.001));
  for (const auto& name : states->getNames())
  {
    const int id = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
    EXPECT_NEAR(states->getHandle(name).getPosition(), 0.01 * id, 1e-12);
    EXPECT_NEAR(states->getHandle(name).getVelocity(), -0.02 * id, 1e-12);
  }
  auto* imus = hw.get<hardware_interface::ImuSensorInterface>();
  ASSERT_NE(imus, nullptr);
  ASSERT_EQ(imus->getNames().size(), 1U);
  const auto imu = imus->getHandle("base_imu");
  EXPECT_EQ(imu.getFrameId(), "base_imu");
  const int body = mj_name2id(model.get(), mjOBJ_BODY, "base_imu");
  ASSERT_GE(body, 0);
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(imu.getOrientation()[i], data->xquat[4 * body + i + 1], 1e-12);
  EXPECT_NEAR(imu.getOrientation()[3], data->xquat[4 * body], 1e-12);
  for (const auto& sensor : { std::string("base_imu_gyro"), std::string("base_imu_acc") })
  {
    const int id = mj_name2id(model.get(), mjOBJ_SENSOR, sensor.c_str());
    ASSERT_GE(id, 0);
    EXPECT_EQ(model->sensor_dim[id], 3);
    const double* value = sensor == "base_imu_gyro" ? imu.getAngularVelocity() : imu.getLinearAcceleration();
    for (int i = 0; i < 3; ++i)
      EXPECT_NEAR(value[i], data->sensordata[model->sensor_adr[id] + i], 1e-12);
  }

  // Regression: the old environment variable must never override wheel effort.
  setenv("RM_MUJOCO_TORQUE_TEST", "1", 1);
  for (double sign : { 1.0, -1.0 })
  {
    for (const auto& name : efforts->getNames())
      efforts->getHandle(name).setCommand(sign * 1000.0);
    hw.writeSim(ros::Time(1), ros::Duration(0.001));
    for (const auto& name : efforts->getNames())
    {
      const int id = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
      EXPECT_DOUBLE_EQ(data->qfrc_applied[model->jnt_dofadr[id]], sign * urdf.getJoint(name)->limits->effort);
    }
  }
  unsetenv("RM_MUJOCO_TORQUE_TEST");
  hw.readSim(ros::Time(2), ros::Duration(0.001));
  hw.writeSim(ros::Time(2), ros::Duration(0.001));
  for (int i = 0; i < model->nv; ++i)
    EXPECT_DOUBLE_EQ(data->qfrc_applied[i], 0.0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rm_mujoco_hardware_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
