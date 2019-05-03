/*
 * test_kf_callback.cpp
 *
 *  Created on: Nov 6, 2016
 *      Author: jsola
 */

#include "core/sensor/sensor_odom_2D.h"
#include "core/processor/processor_odom_2D.h"
#include "core/processor/processor_tracker_feature_dummy.h"
#include "core/capture/capture_void.h"

int main()
{
    using namespace wolf;
    using namespace Eigen;
    using namespace std;

    ProblemPtr problem = Problem::create("PO 2D");

    SensorBasePtr sen_odo    = problem->installSensor   ("ODOM 2D", "main odometer", (Vector3s()<<0,0,0).finished(),"");
    ProcessorParamsOdom2DPtr params_odo = std::make_shared<ProcessorParamsOdom2D>();
    params_odo->max_time_span = 2;
    params_odo->angle_turned = M_PI; // 180 degrees turn
    ProcessorBasePtr prc_odo = problem->installProcessor("ODOM 2D", "odometry integrator", sen_odo, params_odo);
    prc_odo->setTimeTolerance(0.1);

    SensorBasePtr sen_ftr    = problem->installSensor   ("ODOM 2D", "other odometer", (Vector3s()<<0,0,0).finished(),"");
    ProcessorParamsTrackerFeaturePtr params_trk = std::make_shared<ProcessorParamsTrackerFeature>();
    params_trk->max_new_features = 4;
    params_trk->min_features_for_keyframe = 7;
    params_trk->time_tolerance = 0.5;
    shared_ptr<ProcessorTrackerFeatureDummy> prc_ftr = make_shared<ProcessorTrackerFeatureDummy>(params_trk);
    prc_ftr->setName("tracker");
    sen_ftr->addProcessor(prc_ftr);
    prc_ftr->setTimeTolerance(0.1);

    cout << "Motion sensor    : " << problem->getProcessorMotion()->getSensor()->getName() << endl;
    cout << "Motion processor : " << problem->getProcessorMotion()->getName() << endl;

    TimeStamp t(0);
    cout << "=======================\n>> TIME: " << t.get() << endl;
    Vector3s x({0,0,0});
    Matrix3s P; P.setZero();
    problem->setPrior(x, P, t, 0.01);

    cout << "x(" << t.get() << ") = " << problem->getCurrentState().transpose() << endl;

    Vector2s odo_data;  odo_data << .1, (M_PI / 10);

    problem->print(2, false, true, true); // print(level, constr_by, metric, state_blocks)

    Scalar dt = 1;
    for (auto i = 0; i < 4; i++)
    {

        cout << "Tracker----------------" << endl;
        sen_ftr->process(make_shared<CaptureVoid>(t, sen_ftr));
        problem->print(2, false, true, true); // print(level, constr_by, metric, state_blocks)

        t += dt;
        cout << "=======================\n>> TIME: " << t.get() << endl;

        cout << "Motion-----------------" << endl;
        sen_odo->process(make_shared<CaptureMotion>("ODOM 2D", t, sen_odo, odo_data, 3, 3, nullptr));
        cout << "x(" << t.get() << ") = " << problem->getCurrentState().transpose() << endl;
        problem->print(2, false, true, true); // print(level, constr_by, metric, state_blocks)

    }

    return 0;
}
