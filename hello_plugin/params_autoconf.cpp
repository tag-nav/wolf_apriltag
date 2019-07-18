/*
 *  params_autoconf.cpp
 *
 *  Created on: Feb 15, 2019
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
    ParamsServer server = ParamsServer(parser.getParams(), parser.sensorsSerialization(), parser.processorsSerialization());
    cout << "PRINTING SERVER MAP" << endl;
    server.print();
    cout << "-----------------------------------" << endl;
    /**
       It seems to be a requirement that each file is loaded by its own ClassLoader object, otherwise I get
       a segmentation fault. Likewise, it seems that these ClassLoaders must be allocated at the heap, because
       the constructor refuses to build an object if I try to do local (stack) allocation, i.e `ClassLoader(it)` is not allowed but `new ClassLoader(it)` is.
     **/
    auto loaders = vector<Loader*>();
    for(auto it : parser.getFiles()) {
        auto l = new LoaderRaw(it);
        l->load();
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
    auto prc = ProcessorParamsOdom2D("my_proc_test", server);

    std::cout << "prc.cov_det " << prc.cov_det << std::endl;
    std::cout << "prc.unmeasured_perturbation_std " << prc.unmeasured_perturbation_std << std::endl;
    std::cout << "prc.angle_turned " << prc.angle_turned << std::endl;
    std::cout << "prc.dist_traveled " << prc.dist_traveled << std::endl;
    std::cout << "prc.max_buff_length " << prc.max_buff_length << std::endl;
    std::cout << "prc.max_time_span " << prc.max_time_span << std::endl;
    std::cout << "prc.time_tolerance " << prc.time_tolerance << std::endl;
    std::cout << "prc.voting_active " << prc.voting_active << std::endl;

    return 0;
}
