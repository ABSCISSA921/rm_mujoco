#pragma once

#include <array>
#include <mujoco/mujoco.h>

namespace rm_mujoco
{
// Equivalent axial spring at each leg, with L measured from hip to wheel axle.
// Positive force extends the leg. It is independent of motor/controller state.
class LegSpring
{
public:
  bool initialize(const mjModel* model);
  double force(double length) const;
  void apply(const mjModel* model, mjData* data) const;

private:
  std::array<std::array<int, 2>, 2> sites_{};
  std::array<double, 4> coefficients_{};
};
}  // namespace rm_mujoco
