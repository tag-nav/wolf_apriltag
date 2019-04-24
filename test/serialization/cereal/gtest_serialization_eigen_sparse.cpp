/*
 * gtest_intrinsics_odom2d_serialization.cpp
 *
 *  Created on: Jul 16, 2017
 *      Author: Jeremie Deray
 */

#include "base/common/wolf.h"
#include "../../utils_gtest.h"

#include "../../../serialization/cereal/serialization_eigen_sparse.h"

#include "../../../serialization/cereal/io.h"

#include <fstream>

class WolfTestCerealSerializationEigenSparse : public testing::Test
{
public:

  using triplet_t = Eigen::Triplet<double>;
  using sparse_mat_t = Eigen::SparseMatrix<double>;

  WolfTestCerealSerializationEigenSparse()
  {
    triplet_list_.reserve(10);

    for(int i=0; i<10; ++i)
      triplet_list_.emplace_back(i,i,i*5);

    m_.resize(10, 10);
    m_.setFromTriplets(triplet_list_.begin(), triplet_list_.end());
  }

  const std::string path_to_io = "/tmp/";
  const std::string filename_t = "serialization_eigen_triplet";
  const std::string filename_m = "serialization_eigen_sparse";

  const std::vector<std::string> exts = {".bin", ".xml", ".json"};

  triplet_t t_ = Eigen::Triplet<double>(1, 2, 5.5);

  std::vector<triplet_t> triplet_list_;
  Eigen::SparseMatrix<double> m_;
};

TEST_F(WolfTestCerealSerializationEigenSparse,
       CerealSerializationEigenTriplet)
{
  for (const auto ext : exts)
  {
    const std::string full_path = path_to_io + filename_t + ext;

    ASSERT_NO_THROW( wolf::save( full_path, t_) )
        << "Failed on saving " << full_path;

    triplet_t t;

    ASSERT_NO_THROW( wolf::load( full_path, t) )
        << "Failed on loading " << full_path;

    ASSERT_EQ(t_.row(),   t.row());
    ASSERT_EQ(t_.col(),   t.col());
    ASSERT_EQ(t_.value(), t.value());
  }

  PRINT_TEST_FINISHED;
}

TEST_F(WolfTestCerealSerializationEigenSparse,
       CerealSerializationEigenSparseMatrix)
{
  for (const auto ext : exts)
  {
    const std::string full_path = path_to_io + filename_m + ext;

    ASSERT_NO_THROW( wolf::save( full_path, m_) )
        << "Failed on saving " << full_path;

    sparse_mat_t m;

    ASSERT_NO_THROW( wolf::load( full_path, m) )
        << "Failed on loading " << full_path;

    ASSERT_EQ(m_.rows(), m.rows());
    ASSERT_EQ(m_.cols(), m.cols());

    std::vector<triplet_t> triplet_list;
    triplet_list.reserve(10);

    for (int k=0; k<m.outerSize(); ++k)
      for (sparse_mat_t::InnerIterator it(m, k); it; ++it)
      {
        triplet_list.emplace_back(it.row(), it.col(), it.value());
      }

    ASSERT_EQ(triplet_list_.size(), triplet_list.size());

    for (int i=0; i<triplet_list_.size(); ++i)
    {
      ASSERT_EQ(triplet_list_[i].row(),   triplet_list[i].row());
      ASSERT_EQ(triplet_list_[i].col(),   triplet_list[i].col());
      ASSERT_EQ(triplet_list_[i].value(), triplet_list[i].value());
    }
  }

  PRINT_TEST_FINISHED;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
