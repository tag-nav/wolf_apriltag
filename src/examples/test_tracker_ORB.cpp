//std includes
#include <iostream>

//opencv includes
#include "opencv2/features2d/features2d.hpp"

#include "processor_image_landmark.h"

int main(int argc, char** argv)
{
    using namespace wolf;

    std::cout << std::endl << "==================== tracker ORB test ======================" << std::endl;

    cv::VideoCapture capture;
    const char * filename;
    if (argc == 1)
    {
        filename = "/home/jtarraso/Vídeos/gray.mp4";
        capture.open(filename);
    }
    else if (std::string(argv[1]) == "0")
    {
        //camera
        filename = "0";
        capture.open(0);
    }
    else
    {
        filename = argv[1];
        capture.open(filename);
    }
    std::cout << "Input video file: " << filename << std::endl;
    if(!capture.isOpened()) std::cout << "failed" << std::endl; else std::cout << "succeded" << std::endl;
    capture.set(CV_CAP_PROP_POS_MSEC, 3000);

    unsigned int img_width  = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    unsigned int img_height = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
    std::cout << "Image size: " << img_width << "x" << img_height << std::endl;



    cv::Feature2D* detector_descriptor_ptr_;
    cv::DescriptorMatcher* matcher_ptr_;

    unsigned int nfeatures = 500;
    float scaleFactor = 1.2;
    unsigned int nlevels = 1;
    unsigned int edgeThreshold = 4;
    unsigned int firstLevel = 0;
    unsigned int WTA_K = 2;                  //# See: http://docs.opencv.org/trunk/db/d95/classcv_1_1ORB.html#a180ae17d3300cf2c619aa240d9b607e5
    unsigned int scoreType = 0;              //#enum { kBytes = 32, HARRIS_SCORE=0, FAST_SCORE=1 };
    unsigned int patchSize = 31;

    detector_descriptor_ptr_ = new cv::ORB(nfeatures, //
                                           scaleFactor, //
                                           nlevels, //
                                           edgeThreshold, //
                                           firstLevel, //
                                           WTA_K, //
                                           scoreType, //
                                           patchSize);

    unsigned int nominal_pattern_radius = 0;
    unsigned int pattern_radius = (unsigned int)( (nominal_pattern_radius) * pow(scaleFactor, nlevels-1));

//    std::cout << "nominal pattern radius: " << _dd_params->nominal_pattern_radius << std::endl;
//    std::cout << "scale factor: " << params_orb->scaleFactor << std::endl;
//    std::cout << "nlevels: " << params_orb->nlevels << std::endl;

    unsigned int size_bits = detector_descriptor_ptr_->descriptorSize() * 8;

    matcher_ptr_ = new cv::BFMatcher(6);



    // CAPTURES
    SensorCamera* camera_ptr_;
    CaptureImage* image_ptr;
    TimeStamp t = 1;

    unsigned int buffer_size = 20;
    std::vector<cv::Mat> frame(buffer_size);
    unsigned int f  = 1;
    capture >> frame[f % buffer_size];

    cv::namedWindow("Feature tracker");    // Creates a window for display.
    cv::moveWindow("Feature tracker", 0, 0);

//    image_ptr = new CaptureImage(t, camera_ptr_, frame[f % buffer_size]);

    cv::imshow("Feature tracker", frame[f % buffer_size]);
    cv::waitKey(0);

    std::vector<cv::KeyPoint> target_keypoints;
    cv::Mat target_descriptors;
    cv::Mat image_roi_ = frame[f % buffer_size];

    detector_descriptor_ptr_->detect(image_roi_, target_keypoints);
    detector_descriptor_ptr_->compute(image_roi_, target_keypoints, target_descriptors);




    while(!(frame[f % buffer_size].empty()))
    {

        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        cv::Mat image_roi = frame[f % buffer_size];
        std::vector<cv::DMatch> cv_matches;

        unsigned int tracked_keypoints = 0;
        detector_descriptor_ptr_->detect(image_roi, keypoints);
        detector_descriptor_ptr_->compute(image_roi, keypoints, descriptors);

        for(int j = 0; j < target_keypoints.size(); j++)
        {
            cv::Mat target_descriptor; //B(cv::Rect(0,0,vec_length,1));
            target_descriptor = target_descriptors(cv::Rect(0,j,target_descriptors.cols,1));

            matcher_ptr_->match(target_descriptor, descriptors, cv_matches);
            Scalar normalized_score = 1 - (Scalar)(cv_matches[0].distance)/size_bits;
            std::cout << "normalized score: " << normalized_score << std::endl;
            if(normalized_score < 0.8)
            {
                std::cout << "not tracked" << std::endl;
            }
            else
            {
                std::cout << "tracked" << std::endl;
                tracked_keypoints++;
                cv::Point point,t_point;
                point.x = keypoints[cv_matches[0].trainIdx].pt.x;
                point.y = keypoints[cv_matches[0].trainIdx].pt.y;
                t_point.x = target_keypoints[j].pt.x;
                t_point.y = target_keypoints[j].pt.y;

                cv::circle(image_roi, t_point, 4, cv::Scalar(51.0, 51.0, 255.0), -1, 3, 0);
                cv::circle(image_roi, point, 2, cv::Scalar(255.0, 255.0, 0.0), -1, 8, 0);
                cv::putText(image_roi, std::to_string(j), point, cv:: FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255.0, 255.0, 0.0));
            }

        }

        std::cout << "tracked keypoints: " << tracked_keypoints << "/" << target_keypoints.size() << std::endl;
        std::cout << "percentage: " << ((float)((float)tracked_keypoints/(float)target_keypoints.size()))*100 << "%" << std::endl;


        cv::imshow("Feature tracker", image_roi);
        cv::waitKey(0);


        target_keypoints = keypoints;
        target_descriptors = descriptors;

        f++;
        capture >> frame[f % buffer_size];
    }
}
