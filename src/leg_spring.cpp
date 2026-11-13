#include <rm_mujoco/leg_spring.h>

#include <cmath>
#include <string>

namespace rm_mujoco
{
bool LegSpring::initialize(const mjModel* model)
{
  const int id = mj_name2id(model, mjOBJ_NUMERIC, "leg_spring_coefficients");
  if (id < 0 || model->numeric_size[id] != 4)
    return false;
  for (int i = 0; i < 4; ++i)
    coefficients_[i] = model->numeric_data[model->numeric_adr[id] + i];
  for (int side = 0; side < 2; ++side)
  {
    const std::string prefix = side == 0 ? "left" : "right";
    sites_[side][0] = mj_name2id(model, mjOBJ_SITE, (prefix + "_spring_hip").c_str());
    sites_[side][1] = mj_name2id(model, mjOBJ_SITE, (prefix + "_spring_wheel").c_str());
    if (sites_[side][0] < 0 || sites_[side][1] < 0)
      return false;
  }
  return true;
}

double LegSpring::force(double length) const
{
  return ((coefficients_[3] * length + coefficients_[2]) * length + coefficients_[1]) * length + coefficients_[0];
}

void LegSpring::apply(const mjModel* model, mjData* data) const
{
  const mjtNum zero_torque[3]{};
  for (const auto& sites : sites_)
  {
    const mjtNum* hip = data->site_xpos + 3 * sites[0];
    const mjtNum* wheel = data->site_xpos + 3 * sites[1];
    mjtNum radial[3];
    mju_sub3(radial, wheel, hip);
    const mjtNum length = mju_norm3(radial);
    mju_scl3(radial, radial, force(length) / length);
    // Equal and opposite internal forces; mj_applyFT accumulates J^T F.
    // Use passive generalized forces so motor effort limits do not clip springs.
    mj_applyFT(model, data, radial, zero_torque, wheel, model->site_bodyid[sites[1]], data->qfrc_passive);
    mju_scl3(radial, radial, -1.0);
    mj_applyFT(model, data, radial, zero_torque, hip, model->site_bodyid[sites[0]], data->qfrc_passive);
  }
}
}  // namespace rm_mujoco
