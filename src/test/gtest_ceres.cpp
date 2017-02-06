/**
 * \file gtest_ceres.cpp
 *
 *  Created on: Jan 18, 2017
 *      \author: Dinesh Atchuthan
 */


#include "utils_gtest.h"

#include "wolf.h"
#include "logging.h"

#include "processor_odom_3D.h"
#include "processor_imu.h"
#include "wolf.h"
#include "problem.h"
#include "ceres_wrapper/ceres_manager.h"
#include "state_quaternion.h"
#include "sensor_imu.h"
#include "rotations.h"

// wolf yaml
#include "../yaml/yaml_conversion.h"
// yaml-cpp library
#include <../yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;
using namespace wolf;

//Global variables

//used in pure_translation test
char * filename_pure_tranlation_imu_data;
char * filename_pure_tranlation_odom;
char * filename_pure_rotation_imu_data;
char * filename_pure_rotation_odom;
unsigned int number_of_KF = 2;

TEST(ProcessorOdom3D, static_ceresOptimisation_Odom_PO)
{

    /* Simple odom test including only processorOdom3D :
     * First keyFrame (origin) is fixed (0,0,0, 0,0,0,1). Then the odometry data for 2 second is [0,0,0, 0,0,0,1], meaning that we did not move at all. 
     * We give this simple graph to solver and let it solve.
     * Everything should converge and final keyFrame State should be exactly [0,0,0, 0,0,0,1]
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_3D);

    SensorBasePtr sen = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");

    // We want to create a processorMotion with a max_time_span of 2 seconds. but here we integrate only odometry and there should be no interpolation between
    // Default processorMotionParams is made so that a KeyFrame will be created at each step. This works in this case
    ProcessorOdom3DParamsPtr prc_odom_params = std::make_shared<ProcessorOdom3DParams>();
    wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", sen, prc_odom_params);
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin((Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE ODOM_3D CLASSES  **************/

    VectorXs d(7);
    d << 0,0,0, 0,0,0,1;
    TimeStamp t(2);

    wolf::CaptureMotionPtr odom_ptr = std::make_shared<CaptureMotion>(t, sen, d);
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    // process data in capture
    sen->process(odom_ptr);

    /* We do not need to create features and frames and constraints here. Everything is done in wolf.
    Features and constraint at created automatically when a new Keyframe is generated. Whether a new keyframe should be created or not, this is
    handled by voteForKeyFrame() function for this processorMotion
    */

    if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }

                                             /************** SOLVER PART  **************/
     ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
     std::cout << summary.FullReport() << std::endl;

     //There should be 3 frames : origin KeyFrame, Generated KeyFrame at t = 2s, and another Frame for incoming data to be processed
     ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);
     
     //This is a static test so we are not supposed to have moved from origin to last KeyFrame
     ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different" << std::endl;
     ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different" << std::endl;
     EXPECT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isFixed()) << "origin_frame is not fixed" << std::endl;

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;
}

TEST(ProcessorOdom3D, static_ceresOptimisation_convergenceOdom_PO)
{

    /* Simple odom test including only processorOdom3D :
     * First keyFrame (origin) is fixed (0,0,0, 0,0,0,1). Then the odometry data for 2 second is [0,0,0, 0,0,0,1], meaning that we did not move at all. 
     * We give this simple graph to solver and let it solve. 
     *
     * But before solving, we change the state of final KeyFrame.
     * First we change only Px, then Py, then Pz, then all of them
     * Second : we change Ox, then Oy, then Oz, then all of them
     * Third : we change everything
     * Everything should converge and final keyFrame State should be exactly [0,0,0, 0,0,0,1]
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PO_3D);

    SensorBasePtr sen = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");

    // We want to create a processorMotion with a max_time_span of 2 seconds. but here we integrate only odometry and there should be no interpolation between
    // Default processorMotionParams is made so that a KeyFrame will be created at each step. This works in this case
    ProcessorOdom3DParamsPtr prc_odom_params = std::make_shared<ProcessorOdom3DParams>();
    wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", sen, prc_odom_params);
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin((Vector7s()<<0,0,0,0,0,0,1).finished(), TimeStamp(0));

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE ODOM_3D CLASSES  **************/

    VectorXs d(7);
    d << 0,0,0, 0,0,0,1;
    TimeStamp t(2);

    wolf::CaptureMotionPtr odom_ptr = std::make_shared<CaptureMotion>(t, sen, d);
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    // process data in capture
    sen->process(odom_ptr);

    if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }

                                             /************** SOLVER PART  **************/

     /* ___________________________________________ CHANGING FINAL FRAME BEFORE OPTIMIZATION ___________________________________________*/
    
    //There should be 3 frames : origin KeyFrame, Generated KeyFrame at t = 2s, and another Frame for incoming data to be processed
    ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);
    EXPECT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isFixed()) << "origin_frame is not fixed" << std::endl;

    //get pointer to the last KeyFrame (which is at t = 2s)
    EXPECT_EQ(t.get(),2);
    FrameBasePtr last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);

    // FIRST SOLVER TEST WITHOUT CHANGING ANYTHING - WE DID NOT MOVE

    std::cout << "______ solving...______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved !______" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;


    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different" << std::endl;
    

                                                    /*********************/
                                                    //CHANGE PX AND SOLVE//
                                                    /*********************/

    last_KF->setState((Vector7s()<<30,0,0,0,0,0,1).finished());

    std::cout << "______ solving... Px changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Px changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Px is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Px is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Px changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Px changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE PY AND SOLVE//
                                                    /*********************/

    last_KF->setState((Vector7s()<<0,30,0,0,0,0,1).finished());

    std::cout << "______ solving... Py changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Py changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Py is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Py is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Py changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Py changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE PZ AND SOLVE//
                                                    /*********************/

    last_KF->setState((Vector7s()<<0,0,30,0,0,0,1).finished());

    std::cout << "______ solving... Pz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Pz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Pz is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Pz is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Pz changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Pz changed" << std::endl;


                                                    /********************************/
                                                    //CHANGE PX, Py AND PZ AND SOLVE//
                                                    /********************************/

    last_KF->setState((Vector7s()<<25,20,30,0,0,0,1).finished());

    std::cout << "______ solving... Px, Py and Pz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Px, Py and Pz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Px, Py and Pz are changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Px, Py and Pz are changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Px, Py and Pz changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Px, Py and Pz changed" << std::endl;

                                                    /*********************/
                                                    //CHANGE OX AND SOLVE//
                                                    /*********************/
    Eigen::Vector3s o_initial_guess;
    Eigen::Quaternions q_init_guess;

    o_initial_guess << 40,0,0;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,7,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w()).finished());

    std::cout << "______ solving... Ox changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Ox changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Ox is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Ox is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Ox changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Ox changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE OY AND SOLVE//
                                                    /*********************/
    o_initial_guess << 0,40,0;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,7,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w()).finished());

    std::cout << "______ solving... Oy changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Oy changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Oy is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Oy is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Oy changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Oy changed" << std::endl;

                                                    /*********************/
                                                    //CHANGE OZ AND SOLVE//
                                                    /*********************/
    o_initial_guess << 0,0,40;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,7,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w()).finished());

    std::cout << "______ solving... Oz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Oz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Oz is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Oz is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Oz changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Oz changed" << std::endl;


                                                    /********************************/
                                                    //CHANGE OX, OY AND OZ AND SOLVE//
                                                    /********************************/
    o_initial_guess << 80,50,40;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,7,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w()).finished());

    std::cout << "______ solving... Ox, Oy and Oz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "______ solved ! Ox, Oy and Oz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Ox, Oy and Oz changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().back()->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Ox, Oy and Oz changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Ox, Oy and Oz changed" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Ox, Oy and Oz changed" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);
}


TEST(ProcessorOdom3D, static_ceresOptimisation_convergenceOdom_POV)
{

    /* Simple odom test including only processorOdom3D :
     * First keyFrame (origin) is fixed (0,0,0, 0,0,0,1). Then the odometry data for 2 second is [0,0,0, 0,0,0,1], meaning that we did not move at all. 
     * We give this simple graph to solver and let it solve. 
     *
     * But before solving, we change the state of final KeyFrame.
     * First we change only Px, then Py, then Pz, then all of them
     * Second : we change Ox, then Oy, then Oz, then all of them
     * Third : we change everything
     * Everything should converge and final keyFrame State should be exactly [0,0,0, 0,0,0,1 ,0,0,0] -->Checked with Assertions
     *
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_POV_3D);
    Eigen::VectorXs x0(10);
    x0 << 0,0,0,  0,0,0,1,  0,0,0;
    TimeStamp t(0);
    wolf_problem_ptr_->setOrigin(x0, Eigen::Matrix6s::Identity() * 0.001, t);

    SensorBasePtr sen = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");

    // We want to create a processorMotion with a max_time_span of 2 seconds. but here we integrate only odometry and there should be no interpolation between
    // Default processorMotionParams is made so that a KeyFrame will be created at each step. This works in this case
    ProcessorOdom3DParamsPtr prc_odom_params = std::make_shared<ProcessorOdom3DParams>();
    wolf_problem_ptr_->installProcessor("ODOM 3D", "odometry integrator", sen, prc_odom_params);

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE ODOM_3D CLASSES  **************/

    VectorXs d(7);
    d << 0,0,0, 0,0,0,1;
    t.set(2);

    wolf::CaptureMotionPtr odom_ptr = std::make_shared<CaptureMotion>(t, sen, d);
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();
    // process data in capture
    sen->process(odom_ptr);

    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/
    wolf_problem_ptr_->print(4,1,1,1);

                                             /************** SOLVER PART  **************/

    //If we want the covariances to be computed, then we need to fix all Velocity StateBlocks because they cannot be observed we Odometry measurements only
    /*for(FrameBasePtr Frame_ptr : wolf_problem_ptr_->getTrajectoryPtr()->getFrameList())
    {
        Frame_ptr->getVPtr()->fix();
    }*/

     /* ___________________________________________ CHANGING FINAL FRAME BEFORE OPTIMIZATION ___________________________________________*/
    
    //There should be 3 frames : origin KeyFrame, Generated KeyFrame at t = 2s, and another Frame for incoming data to be processed
    ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);
    EXPECT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isFixed()) << "origin_frame is not fixed" << std::endl;

    //get pointer to the last KeyFrame (which is at t = 2s)
    EXPECT_EQ(t.get(),2);
    FrameBasePtr last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(t);
    Eigen::Matrix<wolf::Scalar, 10, 1> kf2_state = last_KF->getState(); //This state vector will be used to get the velocity state

    // FIRST SOLVER TEST WITHOUT CHANGING ANYTHING - WE DID NOT MOVE

    std::cout << "\n\t\t\t______ solving...______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved !______" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______" << std::endl;


    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different" << std::endl;
    

                                                    /*********************/
                                                    //CHANGE PX AND SOLVE//
                                                    /*********************/

    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<30,0,0,0,0,0,1,kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Px changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Px changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame position are different - problem when Px is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Px is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Px changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Px changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE PY AND SOLVE//
                                                    /*********************/

    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,30,0,0,0,0,1,kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Py changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Py changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame position are different - problem when Py is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Py is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Py changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Py changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE PZ AND SOLVE//
                                                    /*********************/

    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,30,0,0,0,1,kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Pz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Pz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame position are different - problem when Pz is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Pz is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Pz changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Pz changed" << std::endl;


                                                    /********************************/
                                                    //CHANGE PX, Py AND PZ AND SOLVE//
                                                    /********************************/

    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<25,20,30,0,0,0,1,kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Px, Py and Pz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Px, Py and Pz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame position are different - problem when Px, Py and Pz are changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame orientation are different - problem when Px, Py and Pz are changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\n\t\t\t ______computing covariances______ Px, Py and Pz changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Px, Py and Pz changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE OX AND SOLVE//
                                                    /*********************/
    Eigen::Vector3s o_initial_guess;
    Eigen::Quaternions q_init_guess;

    o_initial_guess << 40,0,0;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w(),kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Ox changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Ox changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Ox is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame orientation are different - problem when Ox is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Ox changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Ox changed" << std::endl;


                                                    /*********************/
                                                    //CHANGE OY AND SOLVE//
                                                    /*********************/
    o_initial_guess << 0,40,0;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w(),kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Oy changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Oy changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Oy is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame orientation are different - problem when Oy is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Oy changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Oy changed" << std::endl;

                                                    /*********************/
                                                    //CHANGE OZ AND SOLVE//
                                                    /*********************/
    o_initial_guess << 0,0,40;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w(),kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Oz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Oz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Oz is changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame orientation are different - problem when Oz is changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Oz changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Oz changed" << std::endl;


                                                    /********************************/
                                                    //CHANGE OX, OY AND OZ AND SOLVE//
                                                    /********************************/
    o_initial_guess << 80,50,40;
    q_init_guess = v2q(o_initial_guess);
    last_KF->setState((Eigen::Matrix<wolf::Scalar,10,1>()<<0,0,0,q_init_guess.x(),q_init_guess.y(),q_init_guess.z(),q_init_guess.w(),kf2_state(7),kf2_state(8),kf2_state(9)).finished());

    std::cout << "\n\t\t\t______ solving... Ox, Oy and Oz changed______" << std::endl;
    summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.BriefReport() << std::endl;
    std::cout << "\t\t\t______ solved ! Ox, Oy and Oz changed______" << std::endl;

    //This is a static test so we are not supposed to have moved from origin to last KeyFrame
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getPPtr()->getVector() - last_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS_SMALL) ) <<
                "origin and final frame position are different - problem when Ox, Oy and Oz changed" << std::endl;
    ASSERT_TRUE( (wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->getOPtr()->getVector() - last_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS) ) <<
                "origin and final frame orientation are different - problem when Ox, Oy and Oz changed" << std::endl;

    // COMPUTE COVARIANCES
    //std::cout << "\t\t\t ______computing covariances______ Ox, Oy and Oz changed" << std::endl;
    //ceres_manager_wolf_diff->computeCovariances(ALL_MARGINALS);//ALL_MARGINALS, ALL
    //std::cout << "\t\t\t ______computed!______ Ox, Oy and Oz changed" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);
}


TEST(ProcessorIMU, static_ceresOptimisation_fixBias)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Bias of all frames are fixed before we call the solver
     * 
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances observable Or... we could fix all bias stateBlocks
     * First we will try to fix bias stateBlocks
     */


    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;
                                            /************** SETTING PROBLEM  **************/

    std::string wolf_root = _WOLF_ROOT_DIR;

    // Wolf problem
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);

    SensorBasePtr sen_imu = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");

    // We want to create a processorIMU with a max_time_span of 1 seconds.
    // Default processorIMUParams is made so that a KeyFrame will be created at each step.
    ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
    prc_imu_params->max_time_span = 1;
    prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_imu_params->dist_traveled = 1000000000;
    prc_imu_params->angle_turned = 1000000000;

    ProcessorBasePtr processor_ptr = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen_imu, prc_imu_params);

    //setting origin and fixing origin KeyFrame
    Eigen::VectorXs x0(16);
    TimeStamp t(0);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,0,  0,0,0;
    wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);

                                             /************** USE IMU CLASSES  **************/
    Eigen::Vector6s data;
    data << 0.0,0.0,-wolf::gravity()(2), 0.0,0.0,0.0; //we use exactly the value of gravity defined in wolf.h

    //integrate IMU data until KeyFrame creation (until we reach max_time_span)
    Scalar dt = t.get();
    TimeStamp ts(0);
    while((dt-t.get())<=std::static_pointer_cast<ProcessorIMU>(processor_ptr)->getMaxTimeSpan()){
    // Time and data variables
    dt += 0.001;
    ts.set(dt);

    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);
    // process data in capture
    sen_imu->process(imu_ptr);
    }

    //Fix all biases StateBlocks -> to make covariances computable
    for(FrameBasePtr it : wolf_problem_ptr_->getTrajectoryPtr()->getFrameList()){
        ( std::static_pointer_cast<FrameIMU>(it) )->getAccBiasPtr()->fix();
        ( std::static_pointer_cast<FrameIMU>(it) )->getGyroBiasPtr()->fix();
    }

    //Check and print wolf tree
    if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }
    
                                             /************** SOLVER PART  **************/
     ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
     std::cout << summary.FullReport() << std::endl;

     wolf_problem_ptr_->print(4,1,1,1);

     //We check with assertions if Final KeyFrame has the same state as origin_KF
     FrameBasePtr origin_KF = wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front();
     FrameBasePtr last_KF = wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts);
     ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
     ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
     ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
     
    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;
}

TEST(ProcessorIMU, static_Optim_IMUOdom_2KF)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1
     *     \____constraintOdom3D___/
     *
     *
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances computable
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    //===================================================== SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;

    // WOLF PROBLEM
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs x0(16);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
    TimeStamp t(0);
    wolf_problem_ptr_->setOrigin(x0, Eigen::Matrix6s::Identity() * 0.001, t);

    // CERES WRAPPER
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


    // SENSOR + PROCESSOR IMU
    //We want a processorIMU with a specific max_time_span (1s) forour test
    SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
    prc_imu_params->max_time_span = 1;
    prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_imu_params->dist_traveled = 1000000000;
    prc_imu_params->angle_turned = 1000000000;

    ProcessorBasePtr processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
    SensorIMUPtr sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
    ProcessorIMUPtr processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);


    // SENSOR + PROCESSOR ODOM 3D
    SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
    prc_odom3D_params->max_time_span = 1;
    prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_odom3D_params->dist_traveled = 1000000000;
    prc_odom3D_params->angle_turned = 1000000000;

    ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
    SensorOdom3DPtr sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
    ProcessorOdom3DPtr processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    // FIX ORIGIN KEYFRAME
    //wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();

    // There should be a FrameIMU at origin as KeyFrame + 1 FrameIMU and 1 FrameOdom Non-KeyFrame
    ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);

    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU + CaptureFix due to setting problem origin before installing processors
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),3);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

    Eigen::Vector6s data;
    //data << 0.0019, 0.0001, 9.8122, 0.1022, 0.1171, -0.0413;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);
    //wolf_problem_ptr_->setProcessorMotion(processor_ptr_imu);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()) ){
        
        // Time and data variables
        dt += 0.001;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    // PROCESS ODOM 3D DATA
    Eigen::Vector6s data_odom3D;
    //wolf_problem_ptr_->setProcessorMotion(processor_ptr_odom3D);
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //Check and print wolf tree
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}



TEST(ProcessorIMU, static_Optim_IMUOdom_nKFs_biasUnfixed)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1 ---- constraintIMU ---- KF2 ---- (. . . . . .) ---- KFn
     *     \____________________________________constraintOdom3D___________________________/
     *
     * IMU BIAS UNFIXED
     *
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances computable
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    //===================================================== SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;
    const unsigned int n = 2; //determine the number of final keyframes that will be created (except origin, so n>=1)
    ASSERT_TRUE(n>0) << "n (number of Keyframe created) must be int >0";

    // WOLF PROBLEM
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs x0(16);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
    TimeStamp t(0);
    wolf_problem_ptr_->setOrigin(x0, Eigen::Matrix6s::Identity() * 0.001, t);

    // CERES WRAPPER
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


    // SENSOR + PROCESSOR IMU
    //We want a processorIMU with a specific max_time_span (1s) forour test
    SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
    prc_imu_params->max_time_span = 1;
    prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_imu_params->dist_traveled = 1000000000;
    prc_imu_params->angle_turned = 1000000000;

    ProcessorBasePtr processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
    SensorIMUPtr sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
    ProcessorIMUPtr processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);


    // SENSOR + PROCESSOR ODOM 3D
    SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
    prc_odom3D_params->max_time_span = n;
    prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_odom3D_params->dist_traveled = 1000000000;
    prc_odom3D_params->angle_turned = 1000000000;

    ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
    SensorOdom3DPtr sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
    ProcessorOdom3DPtr processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    // FIX ORIGIN KEYFRAME
    //wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();

    // There should be a FrameIMU at origin as KeyFrame + 1 FrameIMU and 1 FrameOdom Non-KeyFrame
    ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);

    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU + CaptureFix due to setting problem origin before installing processors
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),3);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

    Eigen::Vector6s data;
    //data << 0.0019, 0.0001, 9.8122, 0.1022, 0.1171, -0.0413;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);
    //wolf_problem_ptr_->setProcessorMotion(processor_ptr_imu);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()*n) ){
        
        // Time and data variables
        dt += 0.001;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    // PROCESS ODOM 3D DATA
    Eigen::Vector6s data_odom3D;
    //wolf_problem_ptr_->setProcessorMotion(processor_ptr_odom3D);
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //Check and print wolf tree
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

TEST(ProcessorIMU, static_Optim_IMUOdom_nKFs_biasFixed)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1 ---- constraintIMU ---- KF2 ---- (. . . . . .) ---- KFn
     *     \____________________________________constraintOdom3D___________________________/
     *
     * IMU BIAS UNFIXED
     *
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances computable
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    //===================================================== SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;
    const unsigned int n = 2; //determine the number of final keyframes that will be created (except origin, so n>=1)
    ASSERT_TRUE(n>0) << "n (number of Keyframe created) must be int >0";

    // WOLF PROBLEM
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs x0(16);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
    TimeStamp t(0);
    wolf_problem_ptr_->setOrigin(x0, Eigen::Matrix6s::Identity() * 0.001, t);

    // CERES WRAPPER
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


    // SENSOR + PROCESSOR IMU
    //We want a processorIMU with a specific max_time_span (1s) forour test
    SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
    prc_imu_params->max_time_span = 1;
    prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_imu_params->dist_traveled = 1000000000;
    prc_imu_params->angle_turned = 1000000000;

    ProcessorBasePtr processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
    SensorIMUPtr sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
    ProcessorIMUPtr processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);


    // SENSOR + PROCESSOR ODOM 3D
    SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
    prc_odom3D_params->max_time_span = n;
    prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_odom3D_params->dist_traveled = 1000000000;
    prc_odom3D_params->angle_turned = 1000000000;

    ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
    SensorOdom3DPtr sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
    ProcessorOdom3DPtr processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    // FIX ORIGIN KEYFRAME
    //wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();

    // There should be a FrameIMU at origin as KeyFrame + 1 FrameIMU and 1 FrameOdom Non-KeyFrame
    ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);

    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU + CaptureFix due to setting problem origin before installing processors
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),3);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS IMU DATA

    Eigen::Vector6s data;
    //data << 0.0019, 0.0001, 9.8122, 0.1022, 0.1171, -0.0413;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    Scalar dt = t.get();
    TimeStamp ts(0.001);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);
    //wolf_problem_ptr_->setProcessorMotion(processor_ptr_imu);

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()*n) ){
        
        // Time and data variables
        dt += 0.001;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);
    }

    // PROCESS ODOM 3D DATA
    Eigen::Vector6s data_odom3D;
    //wolf_problem_ptr_->setProcessorMotion(processor_ptr_odom3D);
    data_odom3D << 0,0,0, 0,0,0;
    
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    sen_odom3D->process(mot_ptr);

        //Fix all biases StateBlocks
    for(FrameBasePtr it : wolf_problem_ptr_->getTrajectoryPtr()->getFrameList()){
        ( std::static_pointer_cast<FrameIMU>(it) )->getAccBiasPtr()->fix();
        ( std::static_pointer_cast<FrameIMU>(it) )->getGyroBiasPtr()->fix();
    }

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //Check and print wolf tree
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;

    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}



TEST(ProcessorIMU, static_Optim_IMUOdom_SeveralKFs)
{
    /* In this scenario, we simulate the integration of a perfect IMU that is not moving and we add an odometry measurement.
     * Initial State is [0,0,0, 0,0,0,1, 0,0,0] so we expect the Final State to be exactly the same
     * Origin KeyFrame is fixed
     * 
     * Finally, we can represent the graph as :
     *
     *  KF0 ---- constraintIMU ---- KF1 ---- constraintIMU ---- KF2 ---- (. . . . . .) ---- KFn
     *     \____constraintOdom3D___/   \____constraintOdom3D___/   \____constraintOdom3D___/
     *
     * data integration is done for 10s (10 * max_time_span)
     * IMU data are processed at 1 Khz (every ms)
     * Odom3D data are processed at 100 Hz (every 10 ms)
     *
     * With IMU data only, biases are not observable ! So covariance cannot be computed due to jacobian rank deficiency.
     * We must add an odometry to make covariances computable
     */

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    //===================================================== SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;
    const unsigned int n = 10; //determine the number of final keyframes that will be created (except origin, so n>=1)
    ASSERT_TRUE(n>0) << "n (number of Keyframe created) must be int >0";

    // WOLF PROBLEM
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs x0(16);
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00;
    TimeStamp t(0);
    wolf_problem_ptr_->setOrigin(x0, Eigen::Matrix6s::Identity() * 0.001, t);

    // CERES WRAPPER
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


    // SENSOR + PROCESSOR IMU
    //We want a processorIMU with a specific max_time_span (1s) forour test
    SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorIMUParamsPtr prc_imu_params = std::make_shared<ProcessorIMUParams>();
    prc_imu_params->max_time_span = 1;
    prc_imu_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_imu_params->dist_traveled = 1000000000;
    prc_imu_params->angle_turned = 1000000000;

    ProcessorBasePtr processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", sen0_ptr, prc_imu_params);
    SensorIMUPtr sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
    ProcessorIMUPtr processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);


    // SENSOR + PROCESSOR ODOM 3D
    SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    ProcessorOdom3DParamsPtr prc_odom3D_params = std::make_shared<ProcessorOdom3DParams>();
    prc_odom3D_params->max_time_span = 1;
    prc_odom3D_params->max_buff_length = 1000000000; //make it very high so that this condition will not pass
    prc_odom3D_params->dist_traveled = 1000000000;
    prc_odom3D_params->angle_turned = 1000000000;

    ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", sen1_ptr, prc_odom3D_params);
    SensorOdom3DPtr sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
    ProcessorOdom3DPtr processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    // FIX ORIGIN KEYFRAME
    //wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin
    wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();

    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU + CaptureFix due to setting problem origin before installing processors
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),3);
    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data, data_odom3D;
    data << 0.00, 0.000, -wolf::gravity()(2), 0.0, 0.0, 0.0;
    data_odom3D << 0,0,0, 0,0,0;
    Scalar dt = t.get();
    TimeStamp ts(0.000);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(ts, sen_odom3D, data_odom3D);
    wolf_problem_ptr_->setProcessorMotion(processor_ptr_imu);
    unsigned int iter = 0;
    const unsigned int odom_freq = 10; //processing odometry data every 10 ms

    while( (dt-t.get()) < (std::static_pointer_cast<ProcessorIMU>(processor_ptr_)->getMaxTimeSpan()*n) ){
        
        // PROCESS IMU DATA
        // Time and data variables
        dt += 0.001;
        iter++;
        ts.set(dt);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);

        if(iter == odom_freq) //every 100 ms
        {
            // PROCESS ODOM 3D DATA
            mot_ptr->setTimeStamp(ts);
            mot_ptr->setData(data_odom3D);
            sen_odom3D->process(mot_ptr);

            iter = 0;
        }
    }

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    FrameIMUPtr origin_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front());
    FrameIMUPtr last_KF = std::static_pointer_cast<FrameIMU>(wolf_problem_ptr_->getTrajectoryPtr()->closestKeyFrameToTimeStamp(ts));

    //Check and print wolf tree
    //wolf_problem_ptr_->print(4,1,1,1);
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;
    
    wolf_problem_ptr_->print(4,1,1,1);

    ASSERT_TRUE( (last_KF->getPPtr()->getVector() - origin_KF->getPPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getOPtr()->getVector() - origin_KF->getOPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getVPtr()->getVector() - origin_KF->getVPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS ));
    ASSERT_TRUE( (last_KF->getAccBiasPtr()->getVector() - origin_KF->getAccBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU
    ASSERT_TRUE( (last_KF->getGyroBiasPtr()->getVector() - origin_KF->getGyroBiasPtr()->getVector()).isMuchSmallerThan(1, wolf::Constants::EPS )); //because we simulate a perfect IMU

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

TEST(ProcessorIMU, Pure_translation)
{
    //In this test we will process both IMU and Odom3D data at the same time (in a same loop).
    //we use data simulating a perfect IMU doing pure translation

    using std::shared_ptr;
    using std::make_shared;
    using std::static_pointer_cast;

    std::ifstream imu_data_input;
    std::ifstream odom_data_input;

    imu_data_input.open(filename_pure_tranlation_imu_data);
    odom_data_input.open(filename_pure_tranlation_odom);
    std::cout << "pure translation imu file: " << filename_pure_tranlation_imu_data << std::endl;
    std::cout << "pure translation odom: " << filename_pure_tranlation_odom << std::endl;

    //std::string dummy;
    //getline(imu_data_input, dummy); getline(odom_data_input, dummy); //needed only to delete headers or first useless data

    if(!imu_data_input.is_open() || !odom_data_input.is_open()){
        std::cerr << "Failed to open data files... Exiting" << std::endl;
        ADD_FAILURE();
    }

    //prepare creation of file if DEBUG_RESULTS activated
#ifdef DEBUG_RESULTS
    std::ofstream debug_results;
    debug_results.open("debug_results.dat");
    if(debug_results)
        debug_results << "%%TimeStamp\t"
                      << "dp_x\t" << "dp_y\t" << "dp_z\t" << "dq_x\t" << "dq_y\t" << "dq_z\t" << "dq_w\t" << "dv_x\t" << "dv_y\t" << "dv_z\t"
                      << "Dp_x\t" << "Dp_y\t" << "Dp_z\t" << "Dq_x\t" << "Dq_y\t" << "Dq_z\t" << "Dq_w\t" << "Dv_x\t" << "Dv_y\t" << "Dv_z\t"
                      << "X_x\t" << "X_y\t" << "X_z\t" << "Xq_x\t" << "Xq_y\t" << "Xq_z\t" << "Xq_w\t" << "Xv_x\t" << "Xv_y\t" << "Xv_z\t" << std::endl;
#endif


    //===================================================== SETTING PROBLEM
    std::string wolf_root = _WOLF_ROOT_DIR;

    // WOLF PROBLEM
    ProblemPtr wolf_problem_ptr_ = Problem::create(FRM_PQVBB_3D);
    Eigen::VectorXs x0(16);
    //x0 << 0,0,0,  0,0,0,1,  1,2,2,  0,0,.00,  0,0,.00; //INITIAL CONDITIONS
    x0 << 0,0,0,  0,0,0,1,  0,0,0,  0,0,.00,  0,0,.00; //INITIAL CONDITIONS
    TimeStamp t(0);
    wolf_problem_ptr_->setOrigin(x0, Eigen::Matrix6s::Identity() * 0.001, t);

    // CERES WRAPPER
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;ceres::LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_ptr_, ceres_options, true);


    // SENSOR + PROCESSOR IMU
    SensorBasePtr sen0_ptr = wolf_problem_ptr_->installSensor("IMU", "Main IMU", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_imu.yaml");
    ProcessorBasePtr processor_ptr_ = wolf_problem_ptr_->installProcessor("IMU", "IMU pre-integrator", "Main IMU", wolf_root + "/src/examples/processor_imu.yaml");
    SensorIMUPtr sen_imu = std::static_pointer_cast<SensorIMU>(sen0_ptr);
    ProcessorIMUPtr processor_ptr_imu = std::static_pointer_cast<ProcessorIMU>(processor_ptr_);

    // SET ORIGIN AND FIX ORIGIN KEYFRAME
    //wolf_problem_ptr_->getProcessorMotionPtr()->setOrigin(x0, t); //this also creates a keyframe at origin
    //wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->fix();


    // SENSOR + PROCESSOR ODOM 3D
    SensorBasePtr sen1_ptr = wolf_problem_ptr_->installSensor("ODOM 3D", "odom", (Vector7s()<<0,0,0,0,0,0,1).finished(), wolf_root + "/src/examples/sensor_odom_3D.yaml");
    ProcessorBasePtr processor_ptr_odom = wolf_problem_ptr_->installProcessor("ODOM 3D", "odom", "odom", wolf_root + "/src/examples/processor_odom_3D.yaml");
    SensorOdom3DPtr sen_odom3D = std::static_pointer_cast<SensorOdom3D>(sen1_ptr);
    ProcessorOdom3DPtr processor_ptr_odom3D = std::static_pointer_cast<ProcessorOdom3D>(processor_ptr_odom);

    // There should be a FrameIMU at origin as KeyFrame + 1 FrameIMU and 1 FrameOdom Non-KeyFrame
    ASSERT_EQ(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().size(),3);

    //There should be 3 captures at origin_frame : CaptureOdom, captureIMU + CaptureFix due to setting problem origin before installing processors
    EXPECT_EQ((wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front())->getCaptureList().size(),3);

    ASSERT_TRUE(wolf_problem_ptr_->getTrajectoryPtr()->getFrameList().front()->isKey()) << "origin_frame is not a KeyFrame..." << std::endl;

    //===================================================== END{SETTING PROBLEM}

    //===================================================== PROCESS DATA
    // PROCESS DATA

    Eigen::Vector6s data_imu, data_odom3D;
    data_imu << 0,0,-wolf::gravity()(2), 0,0,0;
    //data_imu << 0.00, 0.000, 9.81, 0.0, 0.0, 0.0;
    data_odom3D << 0,0,0, 0,0,0;
    //Scalar dt = t.get();
    Scalar input_clock;
    TimeStamp ts(0.001);
    TimeStamp t_odom(0);
    wolf::CaptureIMUPtr imu_ptr = std::make_shared<CaptureIMU>(ts, sen_imu, data_imu);
    wolf::CaptureMotionPtr mot_ptr = std::make_shared<CaptureMotion>(t, sen_odom3D, data_odom3D);
    wolf_problem_ptr_->setProcessorMotion(processor_ptr_imu);
    
    //read first odom data from file
    odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
    t_odom.set(input_clock);
    //when we find a IMU timestamp corresponding with this odometry timestamp then we process odometry measurement

    while( !imu_data_input.eof() && !odom_data_input.eof() )
    {
        // PROCESS IMU DATA
        // Time and data variables
        imu_data_input >> input_clock >> data_imu[0] >> data_imu[1] >> data_imu[2] >> data_imu[3] >> data_imu[4] >> data_imu[5]; //Ax, Ay, Az, Gx, Gy, Gz
        //data_imu[2] += 9.806;
        //9.81 added in Az because gravity was not added in the perfect imu simulation
        ts.set(input_clock);
        imu_ptr->setTimeStamp(ts);
        imu_ptr->setData(data_imu);

        // process data in capture
        imu_ptr->getTimeStamp();
        sen_imu->process(imu_ptr);

        if(ts == t_odom) //every 100 ms
        {
            // PROCESS ODOM 3D DATA
            mot_ptr->setTimeStamp(t_odom);
            mot_ptr->setData(data_odom3D);
            sen_odom3D->process(mot_ptr);

            //prepare next odometry measurement if there is any
            odom_data_input >> input_clock >> data_odom3D[0] >> data_odom3D[1] >> data_odom3D[2] >> data_odom3D[3] >> data_odom3D[4] >> data_odom3D[5];
            t_odom.set(input_clock);
        }
    }

    //===================================================== END{PROCESS DATA}

    //===================================================== SOLVER PART

    //Check and print wolf tree
    //wolf_problem_ptr_->print(4,1,1,1);
    /*if(wolf_problem_ptr_->check(1)){
        wolf_problem_ptr_->print(4,1,1,1);
    }*/

    std::cout << "print...\n" << std::endl;
    wolf_problem_ptr_->print(4,1,1,1);
     
    std::cout << "\t\t\t ______solving______" << std::endl;
    ceres::Solver::Summary summary = ceres_manager_wolf_diff->solve();
    std::cout << summary.FullReport() << std::endl;
    std::cout << "\t\t\t ______solved______" << std::endl;
    
    wolf_problem_ptr_->print(4,1,1,1);

    // COMPUTE COVARIANCES
    std::cout << "\t\t\t ______computing covariances______" << std::endl;
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS, ALL
    std::cout << "\t\t\t ______computed!______" << std::endl;

    //===================================================== END{SOLVER PART}
}

int main(int argc, char **argv)
{
    /*  LIST OF TESTS :
     *      - ProcessorOdom3D:
     *          - static_ceresOptimisation_Odom_PO
     *          - static_ceresOptimisation_convergenceOdom_PO
     *          - static_ceresOptimisation_convergenceOdom_POV
     *      - ProcessorIMU:
     *          - static_ceresOptimisation_fixBias
     *          - static_Optim_IMUOdom_2KF
     *          - static_Optim_IMUOdom_nKFs_biasUnfixed
     *          - static_Optim_IMUOdom_nKFs_biasFixed
     *          - static_Optim_IMUOdom_SeveralKFs
     *          - Pure_translation
     */

     /* Running simply ./gtest_ceres will run all tests that do not need any input file
     ** If you want to run specific test, just modify booleans in 'gtest_ceres.yaml' to specify the tests you want to run
      * In this file you can also modify some values that will be used in the tests (example : number of keyframes)
      * to make it simple, we modify global variables given values in this file
     */

     //We use parser to determine from yaml which tests will be executed
     //initialize all variables
     //Default is to only run tests which do not need any file input
     bool static_ceresOptimisation_Odom_PO = true;
     bool static_ceresOptimisation_convergenceOdom_PO = true;
     bool static_ceresOptimisation_convergenceOdom_POV = true;
     bool static_ceresOptimisation_fixBias = true;
     bool static_Optim_IMUOdom_2KF = true;
     bool static_Optim_IMUOdom_nKFs_biasUnfixed = true;
     bool static_Optim_IMUOdom_nKFs_biasFixed = true;
     bool static_Optim_IMUOdom_SeveralKFs = true;
     bool Pure_translation = false;

     std::string wolf_root = _WOLF_ROOT_DIR;
     std::string gtest_ceres_yaml_path;
     gtest_ceres_yaml_path = wolf_root + "/src/examples/gtest_ceres.yaml";
     //parser used only if first argument is a valid file ending in '.yaml.'
     std::string tests_to_run = "";

     if (argc == 2)
     {
         const char * option = argv[1];
         std:: string option_string(option);
         
         if(option_string == "--use_yaml") // use yaml option detected
         {
             YAML::Node gtest_config = YAML::LoadFile(gtest_ceres_yaml_path);

             static_ceresOptimisation_Odom_PO               = gtest_config["ProcessorOdom3D"]["static_ceresOptimisation_Odom_PO"]                  .as<bool>();
             static_ceresOptimisation_convergenceOdom_PO    = gtest_config["ProcessorOdom3D"]["static_ceresOptimisation_convergenceOdom_PO"]       .as<bool>();
             static_ceresOptimisation_convergenceOdom_POV   = gtest_config["ProcessorOdom3D"]["static_ceresOptimisation_convergenceOdom_POV"]      .as<bool>();

             static_ceresOptimisation_fixBias               = gtest_config["ProcessorIMU"]["static_ceresOptimisation_fixBias"]                  .as<bool>();
             static_Optim_IMUOdom_2KF                       = gtest_config["ProcessorIMU"]["static_Optim_IMUOdom_2KF"]                          .as<bool>();
             static_Optim_IMUOdom_nKFs_biasUnfixed          = gtest_config["ProcessorIMU"]["static_Optim_IMUOdom_nKFs_biasUnfixed"]             .as<bool>();
             static_Optim_IMUOdom_nKFs_biasFixed            = gtest_config["ProcessorIMU"]["static_Optim_IMUOdom_nKFs_biasFixed"]               .as<bool>();
             static_Optim_IMUOdom_SeveralKFs                = gtest_config["ProcessorIMU"]["static_Optim_IMUOdom_SeveralKFs"]                   .as<bool>();
             Pure_translation                               = gtest_config["ProcessorIMU"]["Pure_translation"]                                  .as<bool>();
             number_of_KF                                   = gtest_config["ProcessorIMU"]["n_KF"]                                              .as<unsigned int>();

             std::string pure_tranlation_imu_data_string    = gtest_config["ProcessorIMU"]["Pure_translation_IMU_filepath"]                     .as<std::string>();
             std::string pure_tranlation_odom_string        = gtest_config["ProcessorIMU"]["Pure_translation_Odom_filepath"]                    .as<std::string>();
             std::string pure_rotation_imu_data_string      = gtest_config["ProcessorIMU"]["Pure_rotation_IMU_filepath"]                        .as<std::string>();
             std::string pure_rotation_odom_string          = gtest_config["ProcessorIMU"]["Pure_rotation_Odom_filepath"]                       .as<std::string>();

             //store filename in global variables
            filename_pure_tranlation_imu_data   = new char[pure_tranlation_imu_data_string.length() + 1];
            filename_pure_tranlation_odom       = new char[pure_tranlation_odom_string.length()     + 1];
            filename_pure_rotation_imu_data     = new char[pure_rotation_imu_data_string.length()   + 1];
            filename_pure_rotation_odom         = new char[pure_rotation_odom_string.length()       + 1];

            std::strcpy(filename_pure_tranlation_imu_data, pure_tranlation_imu_data_string.c_str());
            std::strcpy(filename_pure_tranlation_odom, pure_tranlation_odom_string.c_str());
            std::strcpy(filename_pure_rotation_imu_data, pure_rotation_imu_data_string.c_str());
            std::strcpy(filename_pure_rotation_odom, pure_rotation_odom_string.c_str());
         }
     }

    if(static_ceresOptimisation_Odom_PO)
        tests_to_run +=":ProcessorOdom3D.static_ceresOptimisation_Odom_PO";
    if(static_ceresOptimisation_convergenceOdom_PO)
        tests_to_run +=":ProcessorOdom3D.static_ceresOptimisation_convergenceOdom_PO";
    if(static_ceresOptimisation_convergenceOdom_POV)
        tests_to_run +=":ProcessorOdom3D.static_ceresOptimisation_convergenceOdom_POV";
    
    if(static_ceresOptimisation_fixBias)
        tests_to_run +=":ProcessorIMU.static_ceresOptimisation_fixBias";
    if(static_Optim_IMUOdom_2KF)
        tests_to_run +=":ProcessorIMU.static_Optim_IMUOdom_2KF";
    if(static_Optim_IMUOdom_nKFs_biasUnfixed)
        tests_to_run +=":ProcessorIMU.static_Optim_IMUOdom_nKFs_biasUnfixed";
    if(static_Optim_IMUOdom_nKFs_biasFixed)
        tests_to_run +=":ProcessorIMU.static_Optim_IMUOdom_nKFs_biasFixed";
    if(static_Optim_IMUOdom_SeveralKFs)
        tests_to_run +=":ProcessorIMU.static_Optim_IMUOdom_SeveralKFs";
    if(Pure_translation)
        tests_to_run +=":ProcessorIMU.Pure_translation";


    std::string final_tests =  tests_to_run;
  ::testing::InitGoogleTest(&argc, argv);
  //::testing::GTEST_FLAG(filter) = "*static_ceresOptimisation*"; //default : use all test for static optimisation (not using any input)
  //::testing::GTEST_FLAG(filter) = "*static_Optim_IMUOdom_2KF*";
  ::testing::GTEST_FLAG(filter) = final_tests;

  if (argc < 3)
    {
        std::cout << "Missing input argument to run pure_translation test! : needs 2 arguments (path to accelerometer file and path to gyroscope data)." << std::endl;
        ADD_FAILURE(); //Generates a non fatal failure
    }
 else{
     filename_pure_tranlation_imu_data = argv[1];
     filename_pure_tranlation_odom = argv[2];
     //::testing::GTEST_FLAG(filter) = "*static_ceresOptimisation*:*Pure_translation*"; //if arguments given, run test for static_optimisation + pure_translation
    ::testing::GTEST_FLAG(filter) = "*Pure_translation*";
 }
  //google::InitGoogleLogging(argv[0]);
  return RUN_ALL_TESTS();
}