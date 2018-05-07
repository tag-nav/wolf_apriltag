/**
 * \file gtest_constraint_absolute.cpp
 *
 *  Created on: Dec 9, 2017
 *      \author: datchuth
 */


#include "utils_gtest.h"
#include "constraint_block_absolute.h"
#include "constraint_quaternion_absolute.h"
#include "capture_motion.h"

#include "ceres_wrapper/ceres_manager.h"


using namespace Eigen;
using namespace wolf;
using std::cout;
using std::endl;

Vector10s pose9toPose10(Vector9s _pose9)
{
    return (Vector10s() << _pose9.head<3>() , v2q(_pose9.segment<3>(3)).coeffs(), _pose9.tail<3>()).finished();
}

// Input pose9 and covariance
Vector9s pose(Vector9s::Random());
Vector10s pose10 = pose9toPose10(pose);
Vector9s data_var((Vector9s() << 0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2).finished());
Eigen::Matrix<wolf::Scalar, 9, 9> data_cov = data_var.asDiagonal();

// perturbated priors
Vector10s x0 = pose9toPose10(pose + Vector9s::Random()*0.25);

// Problem and solver
ProblemPtr problem = Problem::create("POV 3D");
CeresManager ceres_mgr(problem);

// Two frames
FrameBasePtr frm0 = problem->emplaceFrame(KEY_FRAME, problem->zeroState(), TimeStamp(0));

// Capture, feature and constraint
CaptureBasePtr cap0 = frm0->addCapture(std::make_shared<CaptureMotion>("IMU ABS", 0, nullptr, pose10, 10, 9, nullptr));

////////////////////////////////////////////////////////
/* In the tests below, we check that ConstraintBlockAbsolute and ConstraintQuaternionAbsolute are working fine
 * These are absolute constraints related to a specific part of the frame's state
 * Both features and constraints are created in the TEST(). Hence, tests will not interfere each others.
 */

TEST(ConstraintBlockAbs, ctr_block_abs_p_check)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("POSITION", pose10.head<3>(), data_cov.topLeftCorner<3,3>()));
    ConstraintBlockAbsolutePtr ctr0 = std::static_pointer_cast<ConstraintBlockAbsolute>(
        fea0->addConstraint(std::make_shared<ConstraintBlockAbsolute>(fea0->getFramePtr()->getPPtr()))
        );
    ASSERT_TRUE(problem->check(0));
}

TEST(ConstraintBlockAbs, ctr_block_abs_p_solve)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("POSITION", pose10.head<3>(), data_cov.topLeftCorner<3,3>()));
    ConstraintBlockAbsolutePtr ctr0 = std::static_pointer_cast<ConstraintBlockAbsolute>(
        fea0->addConstraint(std::make_shared<ConstraintBlockAbsolute>(fea0->getFramePtr()->getPPtr()))
        );
    
    // Unfix frame 0, perturb frm0
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(1);

    //only orientation is constrained
    ASSERT_MATRIX_APPROX(frm0->getState().head<3>(), pose10.head<3>(), 1e-6);
}

TEST(ConstraintBlockAbs, ctr_block_abs_v_check)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("VELOCITY", pose10.tail<3>(), data_cov.bottomRightCorner<3,3>()));
    ConstraintBlockAbsolutePtr ctr0 = std::static_pointer_cast<ConstraintBlockAbsolute>(
        fea0->addConstraint(std::make_shared<ConstraintBlockAbsolute>(fea0->getFramePtr()->getVPtr()))
        );
    ASSERT_TRUE(problem->check(0));
}

TEST(ConstraintBlockAbs, ctr_block_abs_v_solve)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("VELOCITY", pose10.tail<3>(), data_cov.bottomRightCorner<3,3>()));
    ConstraintBlockAbsolutePtr ctr0 = std::static_pointer_cast<ConstraintBlockAbsolute>(
        fea0->addConstraint(std::make_shared<ConstraintBlockAbsolute>(fea0->getFramePtr()->getVPtr()))
        );
    
    // Unfix frame 0, perturb frm0
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(1);

    //only velocity is constrained
    ASSERT_MATRIX_APPROX(frm0->getState().tail<3>(), pose10.tail<3>(), 1e-6);
}

TEST(ConstraintQuatAbs, ctr_block_abs_o_check)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("QUATERNION", pose10.segment<4>(3), data_cov.block<3,3>(3,3)));
    ConstraintBlockAbsolutePtr ctr0 = std::static_pointer_cast<ConstraintBlockAbsolute>(
        fea0->addConstraint(std::make_shared<ConstraintQuaternionAbsolute>(fea0->getFramePtr()->getOPtr()))
        );
    ASSERT_TRUE(problem->check(0));
}

TEST(ConstraintQuatAbs, ctr_block_abs_o_solve)
{
    FeatureBasePtr fea0 = cap0->addFeature(std::make_shared<FeatureBase>("QUATERNION", pose10.segment<4>(3), data_cov.block<3,3>(3,3)));
    ConstraintBlockAbsolutePtr ctr0 = std::static_pointer_cast<ConstraintBlockAbsolute>(
        fea0->addConstraint(std::make_shared<ConstraintQuaternionAbsolute>(fea0->getFramePtr()->getOPtr()))
        );
    
    // Unfix frame 0, perturb frm0
    frm0->unfix();
    frm0->setState(x0);

    // solve for frm0
    std::string brief_report = ceres_mgr.solve(1);

    //only velocity is constrained
    ASSERT_MATRIX_APPROX(frm0->getState().segment<4>(3), pose10.segment<4>(3), 1e-6);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

