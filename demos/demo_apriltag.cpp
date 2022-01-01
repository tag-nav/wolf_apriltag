//--------LICENSE_START--------
//
// Copyright (C) 2020,2021,2022 Institut de Robòtica i Informàtica Industrial, CSIC-UPC.
// Authors: Joan Solà Ortega (jsola@iri.upc.edu)
// All rights reserved.
//
// This file is part of WOLF
// WOLF is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//--------LICENSE_END--------
/**
 * \file test_apriltag.cpp
 *
 *  Created on: Dec 14, 2018
 *      \author: Dinesh Atchtuhan
 */

//Wolf
#include "core/wolf.h"
#include "core/rotations.h"
#include "core/problem.h"
#include "core/ceres_wrapper/ceres_manager.h"
#include "core/sensor/sensor_camera.h"
#include "core/processor/processor_tracker_landmark_apriltag.h"
#include "core/capture/capture_image.h"
#include "core/feature/feature_apriltag.h"

// opencv
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/opencv.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// std
#include <iostream>
#include <stdlib.h>


void draw_apriltag(cv::Mat image, std::vector<cv::Point2d> corners, int thickness=1, bool draw_corners=false);


int main(int argc, char *argv[])
{
    /*
     * HOW TO USE ?
     * For now, just call the executable and append the list of images to be processed.
     * The images must be placed in the root folder of your wolf project.
     * Ex:
     * ./test_apriltag frame1.jpg frame2.jpg frame3.jpg
     */

    using namespace wolf;


    // General execution options
    const bool APPLY_CONTRAST = false;
    const bool IMAGE_OUTPUT   = true;
    const bool USEMAP         = false;


    WOLF_INFO( "==================== processor apriltag test ======================" )

    std::string wolf_root = _WOLF_ROOT_DIR;
    // Wolf problem
    ProblemPtr problem              = Problem::create("PO", 3);
    ceres::Solver::Options options;
    options.function_tolerance = 1e-6;
    options.max_num_iterations = 100;
    CeresManagerPtr ceres_manager   = std::make_shared<CeresManager>(problem, options);


    WOLF_INFO( "====================    Configure Problem      ======================" )
    Eigen::Vector7d cam_extrinsics; cam_extrinsics << 0,0,0,  0,0,0,1;
    SensorBasePtr sen       = problem->installSensor("CAMERA", "camera", cam_extrinsics, wolf_root + "/src/examples/camera_logitech_c300_640_480.yaml");
//    SensorBasePtr sen       = problem->installSensor("CAMERA", "camera", cam_extrinsics, wolf_root + "/src/examples/camera_Dinesh_LAAS_params_notangentrect.yaml");
    SensorCameraPtr sen_cam = std::static_pointer_cast<SensorCamera>(sen);
    ProcessorBasePtr prc    = problem->installProcessor("TRACKER LANDMARK APRILTAG", "apriltags", "camera", wolf_root + "/src/examples/processor_tracker_landmark_apriltag.yaml");

    if (USEMAP){
        problem->loadMap(wolf_root + "/src/examples/maps/map_apriltag_logitech_1234.yaml");
        for (auto lmk : problem->getMap()->getLandmarkList()){
            lmk->fix();
        }
    }

    // set prior
    Eigen::Matrix6d covariance = Matrix6d::Identity();
    double std_m;
    double std_deg;
    if (USEMAP){
        std_m   = 100;  // standard deviation on original translation
        std_deg = 180;  // standard deviation on original rotation
    }
    else {
        std_m   = 0.00001;  // standard deviation on original translation
        std_deg = 0.00001;  // standard deviation on original rotation
    }

    covariance.topLeftCorner(3,3)       =  std_m*std_m * covariance.topLeftCorner(3,3);
    covariance.bottomRightCorner(3,3)   = (M_TORAD*std_deg)*(M_TORAD*std_deg) * covariance.bottomRightCorner(3,3);

    if (USEMAP){
        FrameBasePtr F1 = problem->setPrior((Vector7d()<<0.08, 0.15, -0.75, 0, 0, 0, 1).finished(), covariance, 0.0, 0.1);
    }
    else {
        FrameBasePtr F1 = problem->setPrior((Vector7d()<<0,0,0,0,0,0,1).finished(), covariance, 0.0, 0.1);
        F1->fix();
    }

    // first argument is the name of the program.
    // following arguments are path to image (from wolf_root)
    const int inputs = argc -1;
    WOLF_DEBUG("nb of images: ", inputs);
    cv::Mat frame;
    double ts(0);
    double dt = 1;

    WOLF_INFO( "====================        Main loop       ======================" )
    for (int input = 1; input <= inputs; input++) {
        std::string path = wolf_root + "/" + argv[input];
        frame = cv::imread(path, CV_LOAD_IMAGE_COLOR);

        if( frame.data ){ //if imread succeeded

            if (APPLY_CONTRAST){
                double alpha = 2.0; // to tune contrast  [1-3]
                int beta = 0;       // to tune lightness [0-100]
                // Do the operation new_image(i,j) = alpha*image(i,j) + beta
                for( int y = 0; y < frame.rows; y++ ){
                    for( int x = 0; x < frame.cols; x++ ){
                        for( int c = 0; c < 3; c++ ){
                            frame.at<cv::Vec3b>(y,x)[c] = cv::saturate_cast<uchar>( alpha*( frame.at<cv::Vec3b>(y,x)[c] ) + beta );
                        }
                    }
                }
            }

            CaptureImagePtr cap = std::make_shared<CaptureImage>(ts, sen_cam, frame);
    //       cap->setType(argv[input]); // only for problem->print() to show img filename
            cap->setName(argv[input]);
            WOLF_DEBUG("Processing image...", path);
            sen->process(cap);

            if (IMAGE_OUTPUT){
                cv::namedWindow( cap->getName(), cv::WINDOW_NORMAL );// Create a window for display.
            }

        }
        else
            WOLF_WARN("could not load image ", path);

        ts += dt;
    }


    if (IMAGE_OUTPUT){
        WOLF_INFO( "====================    Draw all detections    ======================" )
        for (auto F : problem->getTrajectory()->getFrameList())
        {
            if (F->isKey())
            {
                for (auto cap : F->getCaptureList())
                {
                    if (cap->getType() == "IMAGE")
                    {
                        auto img = std::static_pointer_cast<CaptureImage>(cap);
                        for (FeatureBasePtr f : img->getFeatureList())
                        {
                            FeatureApriltagPtr fa = std::static_pointer_cast<FeatureApriltag>(f);
                            draw_apriltag(img->getImage(), fa->getTagCorners(), 1);
                        }
                        cv::imshow( img->getName(), img->getImage() );  // display original image.
                        cv::waitKey(1);
                    }
                }
            }
        }
    }



//    WOLF_INFO( "====================    Provide perturbed prior    ======================" )
//    for (auto kf : problem->getTrajectory()->getFrameList())
//    {
//        Vector7d x;
//        if (kf->isKey())
//        {
//            x.setRandom();
//            x.tail(4).normalize();
//            kf->setState(x);
//        }
//    }

    WOLF_INFO( "====================    Solve problem    ======================" )
    std::string report = ceres_manager->solve(SolverManager::ReportVerbosity::FULL); // 0: nothing, 1: BriefReport, 2: FullReport
    WOLF_DEBUG(report);
    problem->print(3,0,1,1);



    WOLF_INFO("============= SOLVED PROBLEM : POS | EULER (DEG) ===============")
    for (auto kf : problem->getTrajectory()->getFrameList())
    {
        if (kf->isKey())
            for (auto cap : kf->getCaptureList())
            {
                if (cap->getType() != "POSE")
                {
                    Vector3d T = kf->getP()->getState();
                    Vector4d qv= kf->getO()->getState();
                    Vector3d e = M_TODEG * R2e(q2R(qv));
                    WOLF_DEBUG("KF", kf->id(), " => ", T.transpose(), " | ", e.transpose());
                }
            }
    }
    for (auto lmk : problem->getMap()->getLandmarkList())
    {
        Vector3d T = lmk->getP()->getState();
        Vector4d qv= lmk->getO()->getState();
        Vector3d e = M_TODEG * R2e(q2R(qv));
        WOLF_DEBUG(" L", lmk->id(), " => ", T.transpose(), " | ", e.transpose());
    }


    // ===============================================
    // COVARIANCES ===================================
    // ===============================================
    // Print COVARIANCES of all states
    WOLF_INFO("======== COVARIANCES OF SOLVED PROBLEM : POS | QUAT =======")
    ceres_manager->computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL_MARGINALS);
    for (auto kf : problem->getTrajectory()->getFrameList())
        if (kf->isKey())
        {
            Eigen::MatrixXd cov = kf->getCovariance();
            WOLF_DEBUG("KF", kf->id(), "_std (sigmas) = ", cov.diagonal().transpose().array().sqrt());
        }
    for (auto lmk : problem->getMap()->getLandmarkList())
    {
        Eigen::MatrixXd cov = lmk->getCovariance();
        WOLF_DEBUG(" L", lmk->id(), "_std (sigmas) = ", cov.diagonal().transpose().array().sqrt());
    }
    std::cout << std::endl;


    // ===============================================
    // SAVE MAP TO YAML ==============================
    // ===============================================
    //
    //    problem->saveMap(wolf_root + "/src/examples/map_apriltag_set3_HC.yaml", "set3");

    if (IMAGE_OUTPUT){
        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    return 0;

}


void draw_apriltag(cv::Mat image, std::vector<cv::Point2d> corners,
                  int thickness, bool draw_corners) {
  cv::line(image, corners[0], corners[1], CV_RGB(255, 0, 0), thickness);
  cv::line(image, corners[1], corners[2], CV_RGB(0, 255, 0), thickness);
  cv::line(image, corners[2], corners[3], CV_RGB(0, 0, 255), thickness);
  cv::line(image, corners[3], corners[0], CV_RGB(255, 0, 255), thickness);

  ///////
  // Leads to implement other displays
  ///////

//  const auto line_type = cv::LINE_AA;
//  if (draw_corners) {
//    int r = thickness;
//    cv::circle(image, cv::Point2i(p[0].x, p[0].y), r, CV_RGB(255, 0, 0), -1,
//               line_type);
//    cv::circle(image, cv::Point2i(p[1].x, p[1].y), r, CV_RGB(0, 255, 0), -1,
//               line_type);
//    cv::circle(image, cv::Point2i(p[2].x, p[2].y), r, CV_RGB(0, 0, 255), -1,
//               line_type);
//    cv::circle(image, cv::Point2i(p[3].x, p[3].y), r, CV_RGB(255, 0, 255), -1,
//               line_type);
//  }

//  cv::putText(image, std::to_string(apriltag.id),
//              cv::Point2f(apriltag.center.x - 5, apriltag.center.y + 5),
//              cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 255), 2, line_type);


}

//void DrawApriltags(cv::Mat &image, const ApriltagVec &apriltags) {
//  for (const auto &apriltag : apriltags) {
////    DrawApriltag(image, apriltag);
//    DrawApriltag(image, apriltag, 1);
//  }
//}

