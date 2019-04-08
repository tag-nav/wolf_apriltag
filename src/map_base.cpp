
// wolf
#include "base/map_base.h"
#include "base/landmark/landmark_base.h"
#include "base/factory.h"

// YAML
#include <yaml-cpp/yaml.h>

// stl
#include <fstream>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace wolf {

MapBase::MapBase() :
    NodeBase("MAP", "Base")
{
//    std::cout << "constructed M"<< std::endl;
}

MapBase::~MapBase()
{
//	std::cout << "destructed -M" << std::endl;
}

LandmarkBasePtr MapBase::addLandmark(LandmarkBasePtr _landmark_ptr)
{
    landmark_list_.push_back(_landmark_ptr);
    _landmark_ptr->setMapPtr(shared_from_this());
    _landmark_ptr->setProblem(getProblem());
    _landmark_ptr->registerNewStateBlocks();
    return _landmark_ptr;
}

void MapBase::addLandmarkList(LandmarkBasePtrList& _landmark_list)
{
    for (auto lmk : _landmark_list)
        addLandmark(lmk);
//	LandmarkBasePtrList lmk_list_copy = _landmark_list; //since _landmark_list will be empty after addDownNodeList()
//    for (LandmarkBasePtr landmark_ptr : lmk_list_copy)
//    {
//        landmark_ptr->setMapPtr(shared_from_this());
//        landmark_ptr->setProblem(getProblem());
//        landmark_ptr->registerNewStateBlocks();
//    }
//    landmark_list_.splice(landmark_list_.end(), _landmark_list);
}

void MapBase::load(const std::string& _map_file_dot_yaml)
{
    YAML::Node map = YAML::LoadFile(_map_file_dot_yaml);

    unsigned int nlandmarks = map["nlandmarks"].as<unsigned int>();

    assert(nlandmarks == map["landmarks"].size() && "Number of landmarks in map file does not match!");

    for (unsigned int i = 0; i < nlandmarks; i++)
    {
        YAML::Node lmk_node = map["landmarks"][i];
        LandmarkBasePtr lmk_ptr = LandmarkFactory::get().create(lmk_node["type"].as<std::string>(), lmk_node);
        addLandmark(lmk_ptr);
    }

}

void MapBase::save(const std::string& _map_file_yaml, const std::string& _map_name)
{
    YAML::Emitter emitter;

    emitter << YAML::BeginMap;
    emitter << "map name"   << _map_name;
    emitter << "date-time" << dateTimeNow(); // Get date and time for archiving purposes

    emitter << "nlandmarks" << getLandmarkList().size();

    emitter << "landmarks"  << YAML::BeginSeq;

    for (LandmarkBasePtr lmk_ptr : getLandmarkList())
    {
        emitter << YAML::Flow << lmk_ptr->saveToYaml();
    }
    emitter << YAML::EndSeq << YAML::EndMap;

    std::ofstream fout(_map_file_yaml);
    fout << emitter.c_str();
    fout.close();
}

std::string MapBase::dateTimeNow()
{
    // Get date and time for archiving purposes
    std::time_t rawtime;
    std::time(&rawtime);
    const std::tm* timeinfo = std::localtime(&rawtime);
    char time_char[30];
    std::strftime(time_char, sizeof(time_char), "%d/%m/%Y at %H:%M:%S", timeinfo);
    std::string date_time(time_char);
    return date_time;
}

} // namespace wolf
