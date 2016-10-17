/**
 * \file test_map_yaml.cpp
 *
 *  Created on: Jul 27, 2016
 *      \author: jsola
 */

#include "wolf.h"
#include "problem.h"
#include "map_base.h"
#include "landmark_polyline_2D.h"
#include "landmark_AHP.h"
#include "state_block.h"
#include "../yaml/yaml_conversion.h"

#include <iostream>
using namespace wolf;

void print(MapBase& _map)
{
    for (auto lmk_ptr : *(_map.getLandmarkListPtr()))
    {
        std::cout << "Lmk ID:    " << lmk_ptr->id();
        std::cout << "\nLmk type:  " << lmk_ptr->getType();
        switch (lmk_ptr->getTypeId())
        {
            case LANDMARK_POLYLINE_2D:
            {
                std::shared_ptr<LandmarkPolyline2D> poly_ptr = std::static_pointer_cast<LandmarkPolyline2D>(lmk_ptr);
                std::cout << "\npos:       " << poly_ptr->getPPtr()->getVector().transpose() << " -- fixed: " << poly_ptr->getPPtr()->isFixed();
                std::cout << "\nori:       " << poly_ptr->getOPtr()->getVector().transpose() << " -- fixed: " << poly_ptr->getOPtr()->isFixed();
                std::cout << "\nn points:  " << poly_ptr->getNPoints();
                std::cout << "\nFirst idx: " << poly_ptr->getFirstId();
                std::cout << "\nFirst def: " << poly_ptr->isFirstDefined();
                std::cout << "\nLast  def: " << poly_ptr->isLastDefined();
                for (int idx = poly_ptr->getFirstId(); idx <= poly_ptr->getLastId(); idx++)
                    std::cout << "\n  point: " << idx << ": " << poly_ptr->getPointStateBlockPtr(idx)->getVector().transpose();
                break;
            }
            case LANDMARK_AHP:
            {
                std::shared_ptr<LandmarkAHP> ahp_ptr = std::static_pointer_cast<LandmarkAHP>(lmk_ptr);
                std::cout << "\npos:       " << ahp_ptr->getPPtr()->getVector().transpose() << " -- fixed: " << ahp_ptr->getPPtr()->isFixed();
                std::cout << "\ndescript:  " << ahp_ptr->getCvDescriptor().t();
                break;
            }
            default:
                break;
        }
        std::cout << std::endl;
    }
}

int main()
{
    using namespace Eigen;

    std::cout << "\nTesting Lmk creator and node saving..." << std::endl;
    Vector4s v;
    v << 1, 2, 3, 4;
    cv::Mat d = (cv::Mat_<int>(8,1) << 1, 2, 3, 4, 5, 6, 7, 8);
    LandmarkAHP lmk_1(v, nullptr, nullptr, d);
    std::cout << "Pos 1 = " << lmk_1.getPPtr()->getVector().transpose() << std::endl;
    std::cout << "Des 1 = " << lmk_1.getCvDescriptor().t() << std::endl;

    YAML::Node n = lmk_1.saveToYaml();
    std::cout << "Pos n = " << n["position"].as<VectorXs>().transpose() << std::endl;
    std::cout << "Des n = " << n["descriptor"].as<VectorXs>().transpose() << std::endl;

    LandmarkAHP lmk_2 = *(std::static_pointer_cast<LandmarkAHP>(LandmarkAHP::create(n)));
    std::cout << "Pos 2 = " << lmk_2.getPPtr()->getVector().transpose() << std::endl;
    std::cout << "Des 2 = " << lmk_2.getCvDescriptor().t() << std::endl;

    std::string filename;

    char* w = std::getenv("WOLF_ROOT");
    if (w == NULL)
        throw std::runtime_error("Environment variable WOLF_ROOT not found");
    std::string WOLF_ROOT       = w;
    std::string WOLF_CONFIG     = WOLF_ROOT + "/src/examples";
    std::cout << "\nWolf directory for configuration files: " << WOLF_CONFIG << std::endl;

    Problem problem(FRM_PO_2D);
    filename = WOLF_CONFIG + "/map_polyline_example.yaml";
    std::cout << "Reading map from file: " << filename << std::endl;
    problem.loadMap(filename);

    std::cout << "Printing map..." << std::endl;
    print(*(problem.getMapPtr()));

    filename = WOLF_CONFIG + "/map_polyline_example_write.yaml";
    std::cout << "Writing map to file: " << filename << std::endl;
    std::string thisfile = __FILE__;
    problem.getMapPtr()->save(filename, "Example generated by test file " + thisfile);


    std::cout << "Clearing map... " << std::endl;
    problem.getMapPtr()->getLandmarkListPtr()->clear();

    std::cout << "Re-reading map from file: " << filename << std::endl;
    problem.loadMap(filename);

    std::cout << "Printing map..." << std::endl;
    print(*(problem.getMapPtr()));

    return 0;
}
