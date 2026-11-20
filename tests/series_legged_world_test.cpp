#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
const std::string package = RM_MUJOCO_SOURCE_DIR;
using Model = std::unique_ptr<mjModel, decltype(&mj_deleteModel)>;
using Data = std::unique_ptr<mjData, decltype(&mj_deleteData)>;
using Vec = std::array<mjtNum, 3>;
using Triangle = std::array<Vec, 3>;

Model load(const std::string& path, const mjVFS* vfs = nullptr)
{
  char error[2048]{};
  Model model(mj_loadXML(path.c_str(), vfs, error, sizeof(error)), mj_deleteModel);
  if (!model) throw std::runtime_error(error);
  return model;
}

std::vector<Triangle> triangles()
{
  std::ifstream source(package + "/worlds/series_legged/meshes/series_legged_world.stl", std::ios::binary);
  source.seekg(80);
  std::uint32_t count = 0;
  source.read(reinterpret_cast<char*>(&count), sizeof(count));
  std::vector<Triangle> result(count);
  for (auto& triangle : result)
  {
    float normal[3], points[9];
    std::uint16_t attribute;
    source.read(reinterpret_cast<char*>(normal), sizeof(normal));
    source.read(reinterpret_cast<char*>(points), sizeof(points));
    source.read(reinterpret_cast<char*>(&attribute), sizeof(attribute));
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) triangle[i][j] = points[3 * i + j];
  }
  if (!source) throw std::runtime_error("Failed reading source STL");
  return result;
}

double sourceHeight(const std::vector<Triangle>& source, double x, double y)
{
  double height = 0.; // The source world also contains a ground plane.
  for (const auto& triangle : source)
  {
    const auto& a = triangle[0];
    const auto& b = triangle[1];
    const auto& c = triangle[2];
    const double determinant = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1]);
    if (std::abs(determinant) < 1e-10) continue;
    const double u = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / determinant;
    const double v = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / determinant;
    if (u >= -1e-9 && v >= -1e-9 && u + v <= 1. + 1e-9)
      height = std::max(height, u * a[2] + v * b[2] + (1. - u - v) * c[2]);
  }
  return height;
}

double collisionHeight(const mjModel* model, const mjData* data, double x, double y)
{
  const mjtNum position[] = { x, y, 2. }, direction[] = { 0., 0., -1. };
  const mjtByte groups[] = { 1, 0, 0, 1, 0, 0 }; // Floor plus collision, excluding the visual STL.
  int hit = -1;
  const double distance = mj_ray(model, data, position, direction, groups, 1, -1, &hit);
  if (hit < 0) throw std::runtime_error("Terrain ray missed floor");
  return 2. - distance;
}
}  // namespace

TEST(SeriesLeggedWorld, TerrainLoadsWithoutGazeboAndKeepsCollisionMasks)
{
  auto model = load(package + "/worlds/series_legged/scene.xml");
  EXPECT_EQ(model->nq, 0);
  EXPECT_EQ(model->ngeom, 28); // 26 convex pieces, original visual mesh, floor.
  int collisions = 0;
  for (int i = 0; i < model->ngeom; ++i)
  {
    if (model->geom_group[i] != 3) continue;
    ++collisions;
    EXPECT_EQ(model->geom_contype[i], 2);
    EXPECT_EQ(model->geom_conaffinity[i], 1);
  }
  EXPECT_EQ(collisions, 26);
}

TEST(SeriesLeggedWorld, CollisionSurfaceMatchesSourceTrianglesAndLeavesGapsOpen)
{
  auto model = load(package + "/worlds/series_legged/scene.xml");
  Data data(mj_makeData(model.get()), mj_deleteData);
  mj_forward(model.get(), data.get());
  const auto source = triangles();
  ASSERT_EQ(source.size(), 172U);
  for (const auto& triangle : source)
  {
    // An interior XY point of each projected source triangle, including slopes.
    const double x = .23 * triangle[0][0] + .31 * triangle[1][0] + .46 * triangle[2][0];
    const double y = .23 * triangle[0][1] + .31 * triangle[1][1] + .46 * triangle[2][1];
    const double area = (triangle[1][0] - triangle[0][0]) * (triangle[2][1] - triangle[0][1]) -
                        (triangle[2][0] - triangle[0][0]) * (triangle[1][1] - triangle[0][1]);
    if (std::abs(area) < 1e-10) continue;
    EXPECT_NEAR(collisionHeight(model.get(), data.get(), x - 2., y - .6), sourceHeight(source, x, y), 1e-6);
  }
  EXPECT_NEAR(collisionHeight(model.get(), data.get(), 0., 0.), .2, 1e-7);
  EXPECT_NEAR(collisionHeight(model.get(), data.get(), -1.6, 0.), 0., 1e-8);
  EXPECT_NEAR(collisionHeight(model.get(), data.get(), 2., 3.), 0., 1e-8);
}

TEST(SeriesLeggedWorld, RobotDynamicsAndSpringConfigurationMatchFlatScene)
{
  auto flat = load(package + "/models/series_legged2/scene.xml");
  auto world = load(package + "/models/series_legged2/series_legged_world.xml");
  EXPECT_EQ(world->nq, flat->nq);
  EXPECT_EQ(world->nv, flat->nv);
  EXPECT_EQ(world->njnt, flat->njnt);
  EXPECT_EQ(world->neq, flat->neq);
  EXPECT_EQ(world->nsensor, flat->nsensor);
  EXPECT_EQ(world->nbody, flat->nbody + 1); // Static terrain body.
  EXPECT_EQ(world->ngeom, flat->ngeom + 27);
  EXPECT_DOUBLE_EQ(world->opt.timestep, flat->opt.timestep);
  EXPECT_NEAR(std::accumulate(world->body_mass, world->body_mass + world->nbody, 0.),
              std::accumulate(flat->body_mass, flat->body_mass + flat->nbody, 0.), 1e-12);
  const int spring = mj_name2id(world.get(), mjOBJ_NUMERIC, "leg_spring_coefficients");
  ASSERT_GE(spring, 0);
  const double expected[] = { -80.91, 1408.375, -3091.28, 2094.45 };
  for (int i = 0; i < 4; ++i) EXPECT_DOUBLE_EQ(world->numeric_data[world->numeric_adr[spring] + i], expected[i]);
}

TEST(SeriesLeggedWorld, DynamicSphereLandsOnPlatform)
{
  const std::string xml = "<mujoco><include file=\"" + package + "/worlds/series_legged/scene.xml\"/>"
      "<worldbody><body name=\"probe\" pos=\"0 0 .5\"><freejoint/>"
      "<inertial pos=\"0 0 0\" mass=\".1\" diaginertia=\".000036 .000036 .000036\"/>"
      "<geom type=\"sphere\" size=\".03\" contype=\"1\" conaffinity=\"2\"/>"
      "</body></worldbody></mujoco>";
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  ASSERT_EQ(mj_addBufferVFS(&vfs, "terrain_probe.xml", xml.data(), xml.size()), 0);
  auto model = load("terrain_probe.xml", &vfs);
  mj_deleteVFS(&vfs);
  Data data(mj_makeData(model.get()), mj_deleteData);
  for (int i = 0; i < 800; ++i) mj_step(model.get(), data.get());
  EXPECT_NEAR(data->qpos[2], .23, .001);
  EXPECT_GT(data->ncon, 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
