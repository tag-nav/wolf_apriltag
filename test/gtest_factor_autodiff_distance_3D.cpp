/**
 * \file gtest_factor_autodiff_distance_3D.cpp
 *
 *  Created on: Apr 10, 2018
 *      \author: jsola
 */

#include "base/factor/factor_autodiff_distance_3D.h"
#include "base/problem.h"
#include "base/logging.h"
#include "base/ceres_wrapper/ceres_manager.h"
#include "base/rotations.h"

#include "utils_gtest.h"

using namespace wolf;
using namespace Eigen;

class FactorAutodiffDistance3D_Test : public testing::Test
{
    public:
        Vector3s pos1, pos2;
        Vector3s euler1, euler2;
        Quaternions quat1, quat2;
        Vector4s vquat1, vquat2; // quaternions as vectors
        Vector7s pose1, pose2;

        FrameBasePtr    F1, F2;
        CaptureBasePtr  C2;
        FeatureBasePtr  f2;
        FactorAutodiffDistance3DPtr c2;

        Vector1s dist;
        Matrix1s dist_cov;

        ProblemPtr problem;
        CeresManagerPtr ceres_manager;

        void SetUp()
        {
            pos1   << 1, 0, 0;
            pos2   << 0, 1, 0;
            euler1 << 0, 0, M_PI; //euler1 *= M_TORAD;
            euler2 << 0, 0, -M_PI_2; //euler2 *= M_TORAD;
            quat1  =  e2q(euler1);
            quat2  =  e2q(euler2);
            vquat1 = quat1.coeffs();
            vquat2 = quat2.coeffs();
            pose1  << pos1, vquat1;
            pose2  << pos2, vquat2;

            dist = Vector1s(sqrt(2.0));
            dist_cov = Matrix1s(0.01);

            problem = Problem::create("PO 3D");
            ceres_manager = std::make_shared<CeresManager>(problem);

            F1 = problem->emplaceFrame        (KEY_FRAME, pose1, 1.0);

            F2 = problem->emplaceFrame        (KEY_FRAME, pose2, 2.0);
            C2 = std::make_shared<CaptureBase>("Base", 1.0);
            F2->addCapture(C2);
            f2 = std::make_shared<FeatureBase>("Dist", dist, dist_cov);
            C2->addFeature(f2);
            c2 = std::make_shared<FactorAutodiffDistance3D>(f2, F1, nullptr, false, FAC_ACTIVE);
            f2->addFactor(c2);
            F1->addConstrainedBy(c2);

        }

};

TEST_F(FactorAutodiffDistance3D_Test, ground_truth)
{
    Scalar res;

    c2->operator ()(pos1.data(), pos2.data(), &res);

    ASSERT_NEAR(res, 0.0, 1e-5);
}

TEST_F(FactorAutodiffDistance3D_Test, expected_residual)
{
    Scalar measurement = 1.400;

    f2->setMeasurement(Vector1s(measurement));

    Scalar res_expected = (measurement - (pos2-pos1).norm()) * c2->getMeasurementSquareRootInformationUpper()(0,0);

    Scalar res;
    c2->operator ()(pos1.data(), pos2.data(), &res);

    ASSERT_NEAR(res, res_expected, 1e-5);
}

TEST_F(FactorAutodiffDistance3D_Test, solve)
{
    Scalar measurement = 1.400;
    f2->setMeasurement(Vector1s(measurement));

    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::QUIET);

    // Check distance between F1 and F2 positions -- must match the measurement
    ASSERT_NEAR( (F1->getP()->getState() - F2->getP()->getState()).norm(), measurement, 1e-10);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

