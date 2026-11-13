#include <mujoco_ros/plugin_utils.h>
#include <pluginlib/class_list_macros.h>
#include <rm_mujoco/leg_spring.h>

namespace rm_mujoco
{
class LegSpringPlugin : public mujoco_ros::MujocoPlugin
{
public:
  void passiveCallback(const mjModel* model, mjData* data) override
  {
    spring_.apply(model, data);
  }

protected:
  bool load(const mjModel* model, mjData* /*data*/) override
  {
    if (!spring_.initialize(model))
    {
      ROS_ERROR("LegSpringPlugin requires leg_spring_coefficients and left/right spring hip/wheel sites");
      return false;
    }
    ROS_INFO("Loaded physical leg springs: F(L)=2094.45*L^3-3091.28*L^2+1408.375*L-80.91 N");
    return true;
  }

  void reset() override
  {
    // No integration state; the force depends only on the current geometry.
  }

private:
  LegSpring spring_;
};
}  // namespace rm_mujoco

PLUGINLIB_EXPORT_CLASS(rm_mujoco::LegSpringPlugin, mujoco_ros::MujocoPlugin)
