/*
 * gtest_intrinsics_odom2d_serialization.cpp
 *
 *  Created on: Jul 16, 2017
 *      Author: Jeremie Deray
 */

#include "core/common/wolf.h"
#include "../../utils_gtest.h"

#include "../../../serialization/cereal/serialization_eigen_geometry.h"

#include "../../../serialization/cereal/io.h"

#include <cereal/types/memory.hpp>
#include <fstream>

class WolfTestCerealSerializationEigenGeo : public testing::Test
{
public:

  WolfTestCerealSerializationEigenGeo()
  {
    iso_2d_ = Eigen::Isometry2d(Eigen::Rotation2Dd(0.17));
    iso_2d_.translation() << 0.5, 1.8;

    q_d_ = Eigen::Vector4d().setRandom().normalized();

    iso_3d_ = Eigen::Isometry3d(q_d_);
    iso_3d_.translation() << -7.245, +3.88, 0.0001;
  }

  const std::string path_to_io = "/tmp/";
  const std::string filename   = "serialization_eigen_geo";

  const std::vector<std::string> exts = {".bin"/*, ".xml", ".json"*/};

  Eigen::Isometry2d  iso_2d_;
  Eigen::Isometry3d  iso_3d_;
  Eigen::Quaterniond q_d_;
};

TEST_F(WolfTestCerealSerializationEigenGeo,
       CerealSerializationEigenIso2d)
{
  for (const auto ext : exts)
  {
    const std::string full_path = path_to_io + filename + ext;

    ASSERT_NO_THROW( wolf::save( full_path, iso_2d_, iso_3d_, q_d_) )
        << "Failed on saving " << full_path;

    Eigen::Isometry2d  iso_2d_loaded;
    Eigen::Isometry3d  iso_3d_loaded;
    Eigen::Quaterniond q_d_loaded;

    ASSERT_NO_THROW( wolf::load( full_path, iso_2d_loaded, iso_3d_loaded, q_d_loaded) )
        << "Failed on loading " << full_path;

    ASSERT_MATRIX_APPROX(iso_2d_.matrix(), iso_2d_loaded.matrix(), wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(iso_3d_.matrix(), iso_3d_loaded.matrix(), wolf::Constants::EPS);
    ASSERT_MATRIX_APPROX(q_d_.coeffs(),    q_d_loaded.coeffs(),    wolf::Constants::EPS);
  }

  PRINT_TEST_FINISHED;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
