#include <gtest/gtest.h>
#include <rm_mujoco/leg_spring.h>
#include <rm_mujoco/rm_mujoco_robot_hw_sim.h>
#include <transmission_interface/transmission_parser.h>

#include <cmath>
#include <memory>
#include <numeric>
#include <sstream>
#include <vector>

class CompleteModel : public testing::Test
{
protected:
  void SetUp() override
  {
    std::string path;
    ASSERT_TRUE(nh.getParam("test_model", path));
    ASSERT_TRUE(nh.getParam("robot_description", description));
    char error[2048]{};
    model.reset(mj_loadXML(path.c_str(), nullptr, error, sizeof(error)));
    ASSERT_NE(model, nullptr) << error;
    data.reset(mj_makeData(model.get()));
    XmlRpc::XmlRpcValue initial;
    ASSERT_TRUE(nh.getParam("initial_joint_positions/joint_map", initial));
    ASSERT_EQ(initial.size(), model->njnt);
    for (int id = 0; id < model->njnt; ++id)
    {
      const std::string name = mj_id2name(model.get(), mjOBJ_JOINT, id);
      ASSERT_TRUE(initial.hasMember(name)) << name;
      std::istringstream values(static_cast<std::string>(initial[name]));
      for (int i = 0; i < (model->jnt_type[id] == mjJNT_FREE ? 7 : 1); ++i)
        ASSERT_TRUE(static_cast<bool>(values >> data->qpos[model->jnt_qposadr[id] + i]));
    }
    mj_forward(model.get(), data.get());
    ASSERT_TRUE(urdf.initString(description));
  }

  double length(const std::string& side) const
  {
    const int a = mj_name2id(model.get(), mjOBJ_SITE, (side + "_spring_hip").c_str());
    const int b = mj_name2id(model.get(), mjOBJ_SITE, (side + "_spring_wheel").c_str());
    return mju_dist3(data->site_xpos + 3 * a, data->site_xpos + 3 * b);
  }

  // Independent integral of the specified force curve. Its negative gradient
  // must equal the plugin's generalized force, including floating-base DOFs.
  double potential() const
  {
    double energy = 0;
    for (const auto& side : { "left", "right" })
    {
      const double l = length(side);
      energy -= 2094.45 / 4 * std::pow(l, 4) - 3091.28 / 3 * std::pow(l, 3) +
                1408.375 / 2 * l * l - 80.91 * l;
    }
    return energy;
  }

  ros::NodeHandle nh;
  std::string description;
  urdf::Model urdf;
  std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model{ nullptr, mj_deleteModel };
  std::unique_ptr<mjData, decltype(&mj_deleteData)> data{ nullptr, mj_deleteData };
};

TEST_F(CompleteModel, ClosedChainsMassLimitsAndImuFrames)
{
  EXPECT_EQ(mj_version(), mjVERSION_HEADER);
  EXPECT_EQ(model->nu, 0);
  EXPECT_EQ(model->neq, 4);
  EXPECT_EQ(model->njnt, 17);
  EXPECT_EQ(model->nq, 23);
  EXPECT_EQ(model->nv, 22);
  EXPECT_NEAR(std::accumulate(model->body_mass, model->body_mass + model->nbody, 0.0), 30.911130235045, 1e-10);
  EXPECT_DOUBLE_EQ(model->opt.timestep, .001);
  EXPECT_DOUBLE_EQ(model->opt.gravity[2], -9.81);
  for (int i = 0; i < data->nefc; ++i)
    if (data->efc_type[i] == mjCNSTR_EQUALITY)
      EXPECT_NEAR(data->efc_pos[i], 0, 1e-9);
  for (const auto& side : { std::string("left"), std::string("right") })
  {
    EXPECT_NEAR(length(side), .20, 1e-10);
    for (const auto& joint : { "hip", "knee" })
    {
      const auto j = urdf.getJoint(side + "_" + joint + "_joint");
      ASSERT_NE(j, nullptr);
      EXPECT_DOUBLE_EQ(j->limits->effort, 40.0);
      EXPECT_DOUBLE_EQ(j->limits->velocity, 10.46);
      EXPECT_EQ(j->parent_link_name, "base_link");
    }
  }
  EXPECT_DOUBLE_EQ(urdf.getJoint("pitch_joint")->limits->lower, -.60);
  EXPECT_DOUBLE_EQ(urdf.getJoint("pitch_joint")->limits->upper, .30);
  for (const auto& name : { std::string("base_imu"), std::string("gimbal_imu") })
  {
    const int body = mj_name2id(model.get(), mjOBJ_BODY, name.c_str());
    ASSERT_GE(body, 0);
    const auto joint = urdf.getLink(name)->parent_joint;
    const auto& pose = joint->parent_to_joint_origin_transform;
    EXPECT_NEAR(model->body_pos[3 * body], pose.position.x, 1e-12);
    EXPECT_NEAR(model->body_pos[3 * body + 1], pose.position.y, 1e-12);
    EXPECT_NEAR(model->body_pos[3 * body + 2], pose.position.z, 1e-12);
    const double expected[] = { pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z };
    double dot = 0;
    for (int i = 0; i < 4; ++i)
      dot += expected[i] * model->body_quat[4 * body + i];
    EXPECT_NEAR(std::abs(dot), 1.0, 1e-12);
  }
}

TEST_F(CompleteModel, EightEffortChannelsTwoImusAndPassiveReadback)
{
  std::vector<transmission_interface::TransmissionInfo> transmissions;
  ASSERT_TRUE(transmission_interface::TransmissionParser::parse(description, transmissions));
  ASSERT_EQ(transmissions.size(), 8U);
  rm_mujoco::RmMujocoRobotHWSim hw;
  ASSERT_TRUE(hw.initSim(model.get(), data.get(), nullptr, "", nh, &urdf, transmissions));
  auto* states = hw.get<hardware_interface::JointStateInterface>();
  auto* efforts = hw.get<hardware_interface::EffortJointInterface>();
  auto* imus = hw.get<hardware_interface::ImuSensorInterface>();
  ASSERT_NE(states, nullptr);
  ASSERT_NE(efforts, nullptr);
  ASSERT_NE(imus, nullptr);
  EXPECT_EQ(states->getNames().size(), 16U);
  EXPECT_EQ(efforts->getNames().size(), 8U);
  EXPECT_EQ(imus->getNames().size(), 2U);
  hw.readSim(ros::Time(1), ros::Duration(.001));
  for (const auto& name : states->getNames())
  {
    const int id = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
    EXPECT_NEAR(states->getHandle(name).getPosition(), data->qpos[model->jnt_qposadr[id]], 1e-12);
  }
  for (const auto& name : imus->getNames())
  {
    const auto imu = imus->getHandle(name);
    const int id = mj_name2id(model.get(), mjOBJ_SENSOR, (name + "_quat").c_str());
    ASSERT_GE(id, 0);
    const double* quat = data->sensordata + model->sensor_adr[id];
    for (int i = 0; i < 3; ++i)
      EXPECT_NEAR(imu.getOrientation()[i], quat[i + 1], 1e-12);
    EXPECT_NEAR(imu.getOrientation()[3], quat[0], 1e-12);
  }
  // Separate channels, with commands below all motor/soft-limit bounds.
  for (const auto& name : efforts->getNames())
    efforts->getHandle(name).setCommand(.01);
  hw.writeSim(ros::Time(1), ros::Duration(.001));
  for (const auto& name : efforts->getNames())
  {
    const int id = mj_name2id(model.get(), mjOBJ_JOINT, name.c_str());
    EXPECT_NEAR(data->qfrc_applied[model->jnt_dofadr[id]], .01, 1e-12) << name;
  }
  for (double sign : { -1.0, 1.0 })
  {
    for (const auto& side : { std::string("left"), std::string("right") })
      for (const auto& joint : { "hip", "knee" })
        efforts->getHandle(side + "_" + joint + "_joint").setCommand(sign * 1000);
    hw.writeSim(ros::Time(1), ros::Duration(.001));
    for (const auto& side : { std::string("left"), std::string("right") })
      for (const auto& joint : { "hip", "knee" })
      {
        const int id = mj_name2id(model.get(), mjOBJ_JOINT, (side + "_" + joint + "_joint").c_str());
        EXPECT_DOUBLE_EQ(data->qfrc_applied[model->jnt_dofadr[id]], sign * 40.0);
      }
  }
}

TEST_F(CompleteModel, SpringIsInternalConservativeAndIndependentOfMotors)
{
  rm_mujoco::LegSpring spring;
  ASSERT_TRUE(spring.initialize(model.get()));
  EXPECT_NEAR(spring.force(.2), 93.8694, 1e-9);
  // Use nonzero pre-existing passive forces to detect accidental overwriting.
  for (int i = 0; i < model->nv; ++i)
    data->qfrc_passive[i] = .25;
  spring.apply(model.get(), data.get());
  std::vector<double> force(model->nv), q(data->qpos, data->qpos + model->nq), velocity(model->nv, 0.0);
  for (int i = 0; i < model->nv; ++i)
  {
    force[i] = data->qfrc_passive[i] - .25;
    EXPECT_DOUBLE_EQ(data->qfrc_applied[i], 0.0);
    if (i < 6)
      EXPECT_NEAR(force[i], 0.0, 1e-10);
  }
  const double epsilon = 1e-6;
  for (int i = 0; i < model->nv; ++i)
  {
    velocity[i] = 1.0;
    mju_copy(data->qpos, q.data(), model->nq);
    mj_integratePos(model.get(), data->qpos, velocity.data(), epsilon);
    mj_kinematics(model.get(), data.get());
    const double plus = potential();
    mju_copy(data->qpos, q.data(), model->nq);
    mj_integratePos(model.get(), data->qpos, velocity.data(), -epsilon);
    mj_kinematics(model.get(), data.get());
    const double minus = potential();
    EXPECT_NEAR(force[i], -(plus - minus) / (2 * epsilon), 1e-7) << "dof " << i;
    velocity[i] = 0.0;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rm_mujoco_complete_model_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
