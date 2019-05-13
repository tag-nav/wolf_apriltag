//std includes
#include <iostream>

// Vision utils
#include <vision_utils.h>
#include <sensors.h>
#include <common_class/buffer.h>
#include <common_class/frame.h>
#include <detectors/orb/detector_orb.h>
#include <descriptors/orb/descriptor_orb.h>
#include <matchers/bruteforce_hamming_2/matcher_bruteforce_hamming_2.h>

//Wolf
#include "core/processor/processor_tracker_landmark_image.h"

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << std::endl << "==================== tracker ORB test ======================" << std::endl;

    //=====================================================
    // Environment variable for configuration files
    std::string wolf_root = _WOLF_ROOT_DIR;
    //=====================================================

    //=====================================================

    // Sensor or sensor recording
    vision_utils::SensorCameraPtr sen_ptr = vision_utils::askUserSource(argc, argv);
    if (sen_ptr==NULL)
        return 0;

    // Detector
    vision_utils::DetectorParamsORBPtr params_det = std::make_shared<vision_utils::DetectorParamsORB>();

    params_det->nfeatures = 500;        // The maximum number of features to retain.
    params_det->scaleFactor = 2;        // Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.
    params_det->nlevels = 8;            // The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels).
    params_det->edgeThreshold = 16;     // This is size of the border where the features are not detected. It should roughly match the patchSize parameter.
    params_det->firstLevel = 0;         // It should be 0 in the current implementation.
    params_det->WTA_K = 2;              // The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).
    params_det->scoreType = cv::ORB::HARRIS_SCORE; //#enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };
    params_det->patchSize = 31;

    vision_utils::DetectorBasePtr det_b_ptr = vision_utils::setupDetector("ORB", "ORB detector", params_det);
    vision_utils::DetectorORBPtr det_ptr = std::static_pointer_cast<vision_utils::DetectorORB>(det_b_ptr);

    // Descriptor
    vision_utils::DescriptorParamsORBPtr params_des = std::make_shared<vision_utils::DescriptorParamsORB>();

    params_des->nfeatures = 500;        // The maximum number of features to retain.
    params_des->scaleFactor = 2;        // Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.
    params_des->nlevels = 8;            // The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels).
    params_des->edgeThreshold = 16;     // This is size of the border where the features are not detected. It should roughly match the patchSize parameter.
    params_des->firstLevel = 0;         // It should be 0 in the current implementation.
    params_des->WTA_K = 2;              // The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).
    params_des->scoreType = cv::ORB::HARRIS_SCORE; //#enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };
    params_des->patchSize = 31;

    vision_utils::DescriptorBasePtr des_b_ptr = vision_utils::setupDescriptor("ORB", "ORB descriptor", params_des);
    vision_utils::DescriptorORBPtr des_ptr = std::static_pointer_cast<vision_utils::DescriptorORB>(des_b_ptr);

    // Matcher
    vision_utils::MatcherParamsBRUTEFORCE_HAMMING_2Ptr params_mat = std::make_shared<vision_utils::MatcherParamsBRUTEFORCE_HAMMING_2>();
    vision_utils::MatcherBasePtr mat_b_ptr = vision_utils::setupMatcher("BRUTEFORCE_HAMMING_2", "BRUTEFORCE_HAMMING_2 matcher", params_mat);
    vision_utils::MatcherBRUTEFORCE_HAMMING_2Ptr mat_ptr = std::static_pointer_cast<vision_utils::MatcherBRUTEFORCE_HAMMING_2>(mat_b_ptr);

    //=====================================================

    unsigned int buffer_size = 8;
    vision_utils::Buffer<vision_utils::FramePtr> frame_buff(buffer_size);
    frame_buff.add( vision_utils::setFrame(sen_ptr->getImage(), 0) );

    unsigned int img_width  = frame_buff.back()->getImage().cols;
    unsigned int img_height = frame_buff.back()->getImage().rows;
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;

    cv::namedWindow("Feature tracker");    // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);
    cv::startWindowThread();
    cv::imshow("Feature tracker", frame_buff.back()->getImage());
    cv::waitKey(1);

    KeyPointVector target_keypoints;
    KeyPointVector tracked_keypoints_;
    KeyPointVector tracked_keypoints_2;
    KeyPointVector current_keypoints;
    cv::Mat target_descriptors;
    cv::Mat tracked_descriptors;
    cv::Mat tracked_descriptors2;
    cv::Mat current_descriptors;
    cv::Mat image_original = frame_buff.back()->getImage();
    cv::Mat image_graphics;

    unsigned int roi_width = 200;
    unsigned int roi_heigth = 200;

    int n_first_1 = 0;
    int n_second_1 = 0;

    // Initial detection
    target_keypoints = det_ptr->detect(image_original);
    target_descriptors = des_ptr->getDescriptor(image_original, target_keypoints);

    for (unsigned int f_num=0; f_num < 1000; ++f_num)
    {
        frame_buff.add( vision_utils::setFrame(sen_ptr->getImage(), f_num) );

        KeyPointVector keypoints;
        cv::Mat descriptors;
        DMatchVector cv_matches;
        cv::Mat image = frame_buff.back()->getImage();
        image_graphics = image.clone();
        bool matched = false;
        n_first_1 = n_second_1 = 0;

        unsigned int tracked_keypoints = 0;

        for(unsigned int target_idx = 0; target_idx < target_keypoints.size(); target_idx++)
        {
            std::cout << "\npixel: " << target_keypoints[target_idx].pt << std::endl;
            std::cout << "target_descriptor[" << target_idx << "]:\n" << target_descriptors.row(target_idx) << std::endl;

            matched = false;

            cv::Rect roi = vision_utils::setRoi(target_keypoints[target_idx].pt.x, target_keypoints[target_idx].pt.y, roi_width, roi_heigth);

            cv::Point2f roi_up_left_corner;
            roi_up_left_corner.x = roi.x;
            roi_up_left_corner.y = roi.y;

            for(unsigned int fr = 0; fr < 2; fr++)
            {
                keypoints = det_ptr->detect(image, roi);
                descriptors = des_ptr->getDescriptor(image, keypoints);

                cv::Mat target_descriptor; //B(cv::Rect(0,0,vec_length,1));
                target_descriptor = target_descriptors(cv::Rect(0,target_idx,target_descriptors.cols,1));

                if(keypoints.size() != 0)
                {
                    mat_ptr->match(target_descriptor, descriptors, des_ptr->getSize(), cv_matches);
                    Scalar normalized_score = 1 - (Scalar)(cv_matches[0].distance)/(des_ptr->getSize()*8);
                    std::cout << "pixel: " << keypoints[cv_matches[0].trainIdx].pt + roi_up_left_corner << std::endl;
                    std::cout << "normalized score: " << normalized_score << std::endl;
                    if(normalized_score < 0.8)
                    {
                        std::cout << "not tracked" << std::endl;
                    }
                    else
                    {
                        std::cout << "tracked" << std::endl;

                        matched = true;

                        cv::Point2f point,t_point;
                        point.x = keypoints[cv_matches[0].trainIdx].pt.x;
                        point.y = keypoints[cv_matches[0].trainIdx].pt.y;
                        t_point.x = target_keypoints[target_idx].pt.x;
                        t_point.y = target_keypoints[target_idx].pt.y;

                        cv::circle(image_graphics, t_point, 4, cv::Scalar(51.0, 51.0, 255.0), -1, 3, 0);
                        cv::circle(image_graphics, point, 2, cv::Scalar(255.0, 255.0, 0.0), -1, 8, 0);
                        cv::putText(image_graphics, std::to_string(target_idx), point, cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 255.0, 0.0));

                        //introduce in list - tracked point
                        cv::KeyPoint tracked_kp = keypoints[cv_matches[0].trainIdx];
                        tracked_kp.pt.x = tracked_kp.pt.x + roi.x;
                        tracked_kp.pt.y = tracked_kp.pt.y + roi.y;
                        if(fr==0)
                            tracked_keypoints_.push_back(tracked_kp);
                        else
                            tracked_keypoints_2.push_back(tracked_kp);

                        cv::Mat tracked_desc;
                        tracked_desc = descriptors(cv::Rect(0,cv_matches[0].trainIdx,target_descriptors.cols,1));
                        if(fr==0)
                            tracked_descriptors.push_back(tracked_desc);
                        else
                            tracked_descriptors2.push_back(tracked_desc);

                        //introduce in list - target point
                        if(fr==0)
                        {
                            current_keypoints.push_back(target_keypoints[target_idx]);
                            current_descriptors.push_back(target_descriptor);
                        }

                        if (fr == 0 && normalized_score == 1)n_first_1++;
                        if (fr == 1 && normalized_score == 1)n_second_1++;
                    }
                }
                else
                    std::cout << "not tracked" << std::endl;

            }
            if (matched) tracked_keypoints++;
        }

        std::cout << "\ntracked keypoints: " << tracked_keypoints << "/" << target_keypoints.size() << std::endl;
        std::cout << "percentage first: " << ((float)((float)tracked_keypoints/(float)target_keypoints.size()))*100 << "%" << std::endl;
        std::cout << "Number of perfect first matches: " << n_first_1 << std::endl;
        std::cout << "Number of perfect second matches: " << n_second_1 << std::endl;

        if(tracked_keypoints == 0)
        {
            target_keypoints = det_ptr->detect(image);
            target_descriptors = des_ptr->getDescriptor(image, target_keypoints);
            std::cout << "number of new keypoints to be tracked: " << target_keypoints.size() << std::endl;
        }
        else
        {
            std::cout << "\n\nADVANCE" << std::endl;
//            for(unsigned int i = 0; i < target_keypoints.size(); i++)
//            {
//                std::cout << "\ntarget keypoints";
//                std::cout << target_keypoints[i].pt;
//            }
//            for(unsigned int i = 0; i < current_keypoints.size(); i++)
//            {
//                std::cout << "\ncurrent keypoints";
//                std::cout << current_keypoints[i].pt;
//            }
//            for( int i = 0; i < target_descriptors.rows; i++)
//            {
//                std::cout << "\ntarget descriptors";
//                std::cout << target_descriptors.row(i);
//            }
//            for( int i = 0; i < current_descriptors.rows; i++)
//            {
//                std::cout << "\ncurrent descriptors";
//                std::cout << current_descriptors.row(i);
//            }
//            for( int i = 0; i < current_descriptors2.rows; i++)
//            {
//                std::cout << "\ncurrent descriptors2";
//                std::cout << current_descriptors2.row(i);
//            }

            //target_keypoints.clear();
            target_keypoints = current_keypoints;
            current_descriptors.copyTo(target_descriptors);
            current_keypoints.clear();
            current_descriptors.release();

            std::cout << "\nAFTER THE ADVANCE" << std::endl;
//            for(unsigned int i = 0; i < target_keypoints.size(); i++)
//            {
//                std::cout << "target keypoints";
//                std::cout << target_keypoints[i].pt << "\t" ;
//            }
//            for( int i = 0; i < target_descriptors.rows; i++)
//            {
//                std::cout << "\ntarget descriptors";
//                std::cout << target_descriptors.row(i);
//            }

            std::cout << "\nEND OF ADVANCE\n";

        }

        tracked_keypoints = 0;
        cv::imshow("Feature tracker", image_graphics);
        cv::waitKey(1);
    }
}
