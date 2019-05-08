/*
 * hello_plugin.cpp
 *
 *  Created on: Nov 12, 2018
 *      Author: jcasals
 */
#include "base/sensor/sensor_base.h"
#include "base/common/wolf.h"
// #include "sensor_odom_2D.cpp"
#include <yaml-cpp/yaml.h>
#include "base/yaml/parser_yaml.hpp"
#include "base/utils/params_server.hpp"

#include "../hello_wolf/capture_range_bearing.h"
#include "../hello_wolf/feature_range_bearing.h"
#include "../hello_wolf/factor_range_bearing.h"
#include "../hello_wolf/landmark_point_2D.h"
#include "base/utils/loader.hpp"
#include "base/processor/processor_odom_2D.h"

#include "base/solver/solver_factory.h"
#include "base/ceres_wrapper/ceres_manager.h"

using namespace std;
using namespace wolf;
using namespace Eigen;

int main(int argc, char** argv) {
    string file = "";
    if(argc > 1) file = argv[1];
    parserYAML parser = parserYAML(file);
    parser.parse();
    paramsServer server = paramsServer(parser.getParams(), parser.sensorsSerialization(), parser.processorsSerialization());
    cout << "PRINTING SERVER MAP" << endl;
    server.print();
    cout << "-----------------------------------" << endl;
    /**
       It seems to be a requirement that each file is loaded by its own ClassLoader object, otherwise I get
       a segmentation fault. Likewise, it seems that these ClassLoaders must be allocated at the heap, because
       the constructor refuses to build an object if I try to do local (stack) allocation, i.e `ClassLoader(it)` is not allowed but `new ClassLoader(it)` is.
     **/
    // vector<ClassLoader*> class_loaders = vector<ClassLoader*>();
    // for(auto it : parser.getFiles()) {
    //     auto c = new ClassLoader(it);
    //     class_loaders.push_back(c);
    // }
    auto loaders = vector<Loader*>();
    for(auto it : parser.getFiles()) {
        auto l = new LoaderRaw(it);
        loaders.push_back(l);
    }
    ProblemPtr problem = Problem::create("PO", 2);
    auto sensorMap = map<string, SensorBasePtr>();
    auto procesorMap = map<string, ProcessorBasePtr>();
    for(auto s : server.getSensors()){
        cout << s._name << " " << s._type << endl;
        sensorMap.insert(pair<string, SensorBasePtr>(s._name,problem->installSensor(s._type, s._name, server)));
    }
    for(auto s : server.getProcessors()){
        cout << s._name << " " << s._type << " " << s._name_assoc_sensor << endl;
        procesorMap.insert(pair<string, ProcessorBasePtr>(s._name,problem->installProcessor(s._type, s._name, s._name_assoc_sensor, server)));
    }

    problem->print(4,1,1,1);
    Vector2s motion_data(1.0, 0.0);                     // Will advance 1m at each keyframe, will not turn.
    Matrix2s motion_cov = 0.1 * Matrix2s::Identity();

    // landmark observations data
    VectorXi ids;
    VectorXs ranges, bearings;


    // SET OF EVENTS =======================================================
    std::cout << std::endl;
    WOLF_TRACE("======== BUILD PROBLEM =======");

    // ceres::Solver::Options options;
    // options.max_num_iterations              = 1000; // We depart far from solution, need a lot of iterations
    // CeresManagerPtr ceres                   = std::make_shared<CeresManager>(problem, options);
    auto ceres = SolverFactory::get().create("Solver", problem, server);
    // We'll do 3 steps of motion and landmark observations.

    // STEP 1 --------------------------------------------------------------

    // initialize
    TimeStamp   t(0.0);                     // t : 0.0
    Vector3s    x(0,0,0);
    Matrix3s    P = Matrix3s::Identity() * 0.1;
    problem->setPrior(x, P, t, 0.5);             // KF1 : (0,0,0)
    auto sensor_rb = sensorMap.find("rb")->second;
    // observe lmks
    ids.resize(1); ranges.resize(1); bearings.resize(1);
    ids         << 1;                       // will observe Lmk 1
    ranges      << 1.0;                     // see drawing
    bearings    << M_PI/2;
    CaptureRangeBearingPtr cap_rb = std::make_shared<CaptureRangeBearing>(t, sensor_rb, ids, ranges, bearings);
    sensor_rb   ->process(cap_rb);          // L1 : (1,2)

    // STEP 2 --------------------------------------------------------------
    t += 1.0;                     // t : 1.0

    // motion
    auto sensor_odometry = sensorMap.find("odom")->second;
    CaptureOdom2DPtr cap_motion = std::make_shared<CaptureOdom2D>(t, sensor_odometry, motion_data, motion_cov);
    sensor_odometry ->process(cap_motion);      // KF2 : (1,0,0)

    // observe lmks
    ids.resize(2); ranges.resize(2); bearings.resize(2);
    ids         << 1, 2;                    // will observe Lmks 1 and 2
    ranges      << sqrt(2.0), 1.0;          // see drawing
    bearings    << 3*M_PI/4, M_PI/2;
    cap_rb      = std::make_shared<CaptureRangeBearing>(t, sensor_rb, ids, ranges, bearings);
    sensor_rb   ->process(cap_rb);          // L1 : (1,2), L2 : (2,2)

    // STEP 3 --------------------------------------------------------------
    t += 1.0;                     // t : 2.0

    // motion
    cap_motion  ->setTimeStamp(t);
    sensor_odometry  ->process(cap_motion);      // KF3 : (2,0,0)
    // observe lmks
    ids.resize(2); ranges.resize(2); bearings.resize(2);
    ids         << 2, 3;                    // will observe Lmks 2 and 3
    ranges      << sqrt(2.0), 1.0;          // see drawing
    bearings    << 3*M_PI/4, M_PI/2;
    cap_rb      = std::make_shared<CaptureRangeBearing>(t, sensor_rb, ids, ranges, bearings);
    sensor_rb   ->process(cap_rb);          // L1 : (1,2), L2 : (2,2), L3 : (3,2)
    problem->print(1,0,1,0);


    // SOLVE ================================================================

    // SOLVE with exact initial guess
    WOLF_TRACE("======== SOLVE PROBLEM WITH EXACT PRIORS =======")
    std::string report = ceres->solve(wolf::SolverManager::ReportVerbosity::FULL);
    WOLF_TRACE(report);                     // should show a very low iteration number (possibly 1)
    problem->print(1,0,1,0);

    // PERTURB initial guess
    WOLF_TRACE("======== PERTURB PROBLEM PRIORS =======")
    for (auto sen : problem->getHardware()->getSensorList())
        for (auto sb : sen->getStateBlockVec())
            if (sb && !sb->isFixed())
                sb->setState(sb->getState() + VectorXs::Random(sb->getSize()) * 0.5);       // We perturb A LOT !
    for (auto kf : problem->getTrajectory()->getFrameList())
        if (kf->isKey())
            for (auto sb : kf->getStateBlockVec())
                if (sb && !sb->isFixed())
                    sb->setState(sb->getState() + VectorXs::Random(sb->getSize()) * 0.5);   // We perturb A LOT !
    for (auto lmk : problem->getMap()->getLandmarkList())
        for (auto sb : lmk->getStateBlockVec())
            if (sb && !sb->isFixed())
                sb->setState(sb->getState() + VectorXs::Random(sb->getSize()) * 0.5);       // We perturb A LOT !
    problem->print(1,0,1,0);

    // SOLVE again
    WOLF_TRACE("======== SOLVE PROBLEM WITH PERTURBED PRIORS =======")
    report = ceres->solve(wolf::SolverManager::ReportVerbosity::FULL);
    WOLF_TRACE(report);                     // should show a very high iteration number (more than 10, or than 100!)
    problem->print(1,0,1,0);

    // GET COVARIANCES of all states
    WOLF_TRACE("======== COVARIANCES OF SOLVED PROBLEM =======")
    ceres->computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL_MARGINALS);
    for (auto kf : problem->getTrajectory()->getFrameList()){
        if (kf->isKey())
            {
                Eigen::MatrixXs cov;
                WOLF_TRACE("KF", kf->id(), "_cov = \n", kf->getCovariance(cov));
            }
        for (auto lmk : problem->getMap()->getLandmarkList()) {
            Eigen::MatrixXs cov;
            WOLF_TRACE("L", lmk->id(), "_cov = \n", lmk->getCovariance(cov));
        }
    }
    std::cout << std::endl;

    WOLF_TRACE("======== FINAL PRINT FOR INTERPRETATION =======")
    problem->print(4,1,1,1);

    return 0;
}
