
// wolf
#include "processor_tracker_feature_trifocal.h"

#include "sensor_camera.h"
#include "feature_point_image.h"
#include "constraints/constraint_autodiff_trifocal.h"
#include "capture_image.h"
#include "feature_point_image.h"

// vision_utils
#include <vision_utils/vision_utils.h>
#include <vision_utils/detectors.h>
#include <vision_utils/descriptors.h>
#include <vision_utils/matchers.h>
#include <vision_utils/algorithms.h>

#include <memory>

namespace wolf {

// Constructor
ProcessorTrackerFeatureTrifocal::ProcessorTrackerFeatureTrifocal(ProcessorParamsTrackerFeatureTrifocalPtr _params_tracker_feature_trifocal) :
        ProcessorTrackerFeature("TRACKER FEATURE TRIFOCAL", _params_tracker_feature_trifocal ),
        //cell_width_(0), cell_height_(0), // These will be initialized to proper values taken from the sensor via function configure()
        params_tracker_feature_trifocal_(_params_tracker_feature_trifocal),
        prev_origin_ptr_(nullptr),
        initialized_(false)
{
    setName(_params_tracker_feature_trifocal->name);
    assert(!(params_tracker_feature_trifocal_->yaml_file_params_vision_utils.empty()) && "Missing YAML file with vision_utils parameters!");
    assert(file_exists(params_tracker_feature_trifocal_->yaml_file_params_vision_utils) && "Cannot setup processor: vision_utils' YAML file does not exist.");

    // Detector
    std::string det_name = vision_utils::readYamlType(params_tracker_feature_trifocal_->yaml_file_params_vision_utils, "detector");
    det_ptr_ = vision_utils::setupDetector(det_name, det_name + " detector", params_tracker_feature_trifocal_->yaml_file_params_vision_utils);

    if (det_name.compare("ORB") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorORB>(det_ptr_);
    else if (det_name.compare("FAST") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorFAST>(det_ptr_);
    else if (det_name.compare("SIFT") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorSIFT>(det_ptr_);
    else if (det_name.compare("SURF") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorSURF>(det_ptr_);
    else if (det_name.compare("BRISK") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorBRISK>(det_ptr_);
    else if (det_name.compare("MSER") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorMSER>(det_ptr_);
    else if (det_name.compare("GFTT") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorGFTT>(det_ptr_);
    else if (det_name.compare("HARRIS") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorHARRIS>(det_ptr_);
    else if (det_name.compare("SBD") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorSBD>(det_ptr_);
    else if (det_name.compare("KAZE") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorKAZE>(det_ptr_);
    else if (det_name.compare("AKAZE") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorAKAZE>(det_ptr_);
    else if (det_name.compare("AGAST") == 0)
        det_ptr_ = std::static_pointer_cast<vision_utils::DetectorAGAST>(det_ptr_);

    // Descriptor
    std::string des_name = vision_utils::readYamlType(params_tracker_feature_trifocal_->yaml_file_params_vision_utils, "descriptor");
    des_ptr_ = vision_utils::setupDescriptor(des_name, des_name + " descriptor", params_tracker_feature_trifocal_->yaml_file_params_vision_utils);

    if (des_name.compare("ORB") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorORB>(des_ptr_);
    else if (des_name.compare("SIFT") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorSIFT>(des_ptr_);
    else if (des_name.compare("SURF") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorSURF>(des_ptr_);
    else if (des_name.compare("BRISK") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorBRISK>(des_ptr_);
    else if (des_name.compare("KAZE") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorKAZE>(des_ptr_);
    else if (des_name.compare("AKAZE") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorAKAZE>(des_ptr_);
    else if (des_name.compare("LATCH") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorLATCH>(des_ptr_);
    else if (des_name.compare("FREAK") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorFREAK>(des_ptr_);
    else if (des_name.compare("BRIEF") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorBRIEF>(des_ptr_);
    else if (des_name.compare("DAISY") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorDAISY>(des_ptr_);
    else if (des_name.compare("LUCID") == 0)
        des_ptr_ = std::static_pointer_cast<vision_utils::DescriptorLUCID>(des_ptr_);

    // Matcher
    std::string mat_name = vision_utils::readYamlType(params_tracker_feature_trifocal_->yaml_file_params_vision_utils, "matcher");
    mat_ptr_ = vision_utils::setupMatcher(mat_name, mat_name + " matcher", params_tracker_feature_trifocal_->yaml_file_params_vision_utils);

    if (mat_name.compare("FLANNBASED") == 0)
        mat_ptr_ = std::static_pointer_cast<vision_utils::MatcherFLANNBASED>(mat_ptr_);
    if (mat_name.compare("BRUTEFORCE") == 0)
        mat_ptr_ = std::static_pointer_cast<vision_utils::MatcherBRUTEFORCE>(mat_ptr_);
    if (mat_name.compare("BRUTEFORCE_L1") == 0)
        mat_ptr_ = std::static_pointer_cast<vision_utils::MatcherBRUTEFORCE_L1>(mat_ptr_);
    if (mat_name.compare("BRUTEFORCE_HAMMING") == 0)
        mat_ptr_ = std::static_pointer_cast<vision_utils::MatcherBRUTEFORCE_HAMMING>(mat_ptr_);
    if (mat_name.compare("BRUTEFORCE_HAMMING_2") == 0)
        mat_ptr_ = std::static_pointer_cast<vision_utils::MatcherBRUTEFORCE_HAMMING_2>(mat_ptr_);

//    // Active search grid
//    vision_utils::AlgorithmBasePtr alg_ptr = vision_utils::setupAlgorithm("ACTIVESEARCH", "ACTIVESEARCH algorithm", params_tracker_feature_trifocal_->yaml_file_params_vision_utils);
//    active_search_ptr_ = std::static_pointer_cast<vision_utils::AlgorithmACTIVESEARCH>(alg_ptr);

//    // DEBUG VIEW
//    cv::startWindowThread();
//    cv::namedWindow("DEBUG VIEW", cv::WINDOW_NORMAL);
}

// Destructor
ProcessorTrackerFeatureTrifocal::~ProcessorTrackerFeatureTrifocal()
{
//    cv::destroyAllWindows();
}

void matchFeatures(const cv::Mat &query, const cv::Mat &target,
                   std::vector<cv::DMatch> &goodMatches) {
    std::vector<std::vector<cv::DMatch>> matches;
    cv::Ptr<cv::FlannBasedMatcher> matcher = cv::FlannBasedMatcher::create();
    // Find 2 best matches for each descriptor to make later the second neighbor test.
    matcher->knnMatch(query, target, matches, 2);
    // Second neighbor ratio test.
    for (unsigned int i = 0; i < matches.size(); ++i) {
        if (matches[i][0].distance < matches[i][1].distance * 0.75)
            goodMatches.push_back(matches[i][0]);
    }
}

void ProcessorTrackerFeatureTrifocal::fillGrid(const vision_utils::FeatureIdxGridPtr _grid, const unsigned int& _max_new_features, const std::vector<cv::KeyPoint>& _kps, const cv::Mat& _desc, std::vector<cv::KeyPoint>& _new_kps, cv::Mat& _new_desc)
{
    // Get KeyPoints and descriptors sorted by response and cutting at min_response_new_feature
    std::vector<cv::KeyPoint> kpsSorted;
    cv::Mat desSorted;
    vision_utils::sortByResponse(_kps, kpsSorted, _desc, desSorted, CV_SORT_DESCENDING, params_tracker_feature_trifocal_->min_response_new_feature);

    // Set number of point to keep depending on max: (existing num. kps) or (max new features)
    int numRetPoints;
    if (_kps.size() >= _max_new_features)
        numRetPoints = _max_new_features;
    else
        numRetPoints = _kps.size();

    float tolerance = 1e-6; // tolerance of the number of return points

    // ANMS via SSC: Adaptive Non-Maximal Suppression via Square Covering
    // Bailo, Oleksandr, et al. "Efficient adaptive non-maximal suppression algorithms for homogeneous spatial keypoint distribution"
    // Pattern Recognition Letters 2018.
    vision_utils::Ssc(kpsSorted, _new_kps, desSorted, _new_desc, numRetPoints, tolerance, image_last_.cols, image_last_.rows);

    // Fill grid
    _grid->insert(_new_kps);
}

void ProcessorTrackerFeatureTrifocal::getCandidatesFromGrid(const int& _cell_col, const int& _cell_row, const vision_utils::FeatureIdxGridPtr _grid, const std::vector<cv::KeyPoint>& _kps, const cv::Mat& _desc, std::vector<cv::KeyPoint>& _candidate_kps, cv::Mat& _candidate_desc)
{
    WOLF_TRACE("INside getCandidatesFromGrid");
    // Four surrounding cells
    std::vector<int> idx_rows(3), idx_cols(3);
    for (int ii=-1;ii<2;++ii)
    {
        WOLF_TRACE("ii: ",ii);
        idx_cols.at(ii+1) = _cell_col+ii;
        idx_rows.at(ii+1) = _cell_row+ii;
    }

    for (auto col : idx_cols)
    {
        for (auto row : idx_rows)
        {
            if (col>=0 && row>=0 &&
                col<params_tracker_feature_trifocal_->n_cells_h && row<params_tracker_feature_trifocal_->n_cells_v)
            {
                WOLF_TRACE("Doing CELL: ",col,"x",row);

                // Select candidate cell
                vision_utils::FeatureIdxMap cell_fm_xy = grid_incoming_-> getFeatureIdxMap(col,row);

                WOLF_TRACE("Feature Map in CELL, size:", cell_fm_xy.size());

                WOLF_TRACE("size of input kps: ", _kps.size());

                // Push candidate KeyPoints and descriptors
                for (auto fm : cell_fm_xy)
                {
                    WOLF_TRACE("pushing ",fm.second);
                    _candidate_kps.push_back(_kps[fm.second]);
                    _candidate_desc.push_back(_desc.row(fm.second));
                }
            }
        }
    }
}

unsigned int ProcessorTrackerFeatureTrifocal::detectNewFeatures(const unsigned int& _max_new_features, FeatureBaseList& _feature_list_out)
{
    // DEBUG =====================================
    debug_tStart = clock();
    WOLF_TRACE("======== DetectNewFeatures =========");
    // ===========================================

    KeyPointVector kps;
    cv::Mat desc;

    // DEBUG =====================================
    debug_tTmp = clock();
    // ===========================================

    // Detect KeyPoints
    kps  = det_ptr_->detect(image_last_);

    // DEBUG =====================================
    debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
    WOLF_TRACE("--> TIME: Detect New Features: detect ",debug_comp_time_);
    // ===========================================

    // New classes to be filled
    KeyPointVector new_kps;
    cv::Mat new_desc;

    if (kps.size()>0)
    {
        // DEBUG =====================================
        debug_tTmp = clock();
        // ===========================================

        // Get descriptors
        desc = des_ptr_->getDescriptor(image_last_, kps);

        // DEBUG =====================================
        debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
        WOLF_TRACE("--> TIME: Detect New Features: descript ",debug_comp_time_);
        // ===========================================

        // DEBUG =====================================
        debug_tTmp = clock();
        // ===========================================

        // Get KeyPoints and descriptors
        // sort them by response
        // cut them at min_response_new_feature
        // And finally filling the grid
        fillGrid(grid_last_, _max_new_features, kps, desc, new_kps, new_desc);

        // Create all WOLF features
        for (int ii=0; ii<new_kps.size(); ++ii)
        {
            FeaturePointImagePtr point = std::make_shared<FeaturePointImage>(
                                         new_kps.at(ii),
                                         new_desc.row(0),
                                         Eigen::Matrix2s::Identity() * params_tracker_feature_trifocal_->pixel_noise_std * params_tracker_feature_trifocal_->pixel_noise_std);
                                         point->setIsKnown(true);
            _feature_list_out.push_back(point);
        }

        // DEBUG =====================================
        debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
        WOLF_TRACE("--> TIME: Detect New Features: Sort, Cut and Fill grid ",debug_comp_time_);
        // ===========================================
    }

    return new_kps.size();

//    KeyPointVector kps;
//    cv::Mat desc;
//    cv::Rect roi;
//    KeyPointVector new_keypoints;
//    cv::Mat new_descriptors;
//    cv::KeyPointsFilter keypoint_filter;
//
//    debug_tTmp = clock();
//    KeyPointVector kps_test;
//    cv::Mat desc_test;
//    kps_test  = det_ptr_->detect(image_last_);
//    debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
//    WOLF_TRACE("--> FLIPA '''''' TIME: detect new features: detect ",debug_comp_time_);
//    debug_tTmp = clock();
//    desc_test = des_ptr_->getDescriptor(image_last_, kps_test);
//    debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
//    WOLF_TRACE("--> FLIPA '''''' TIME: detect new features: descript ",debug_comp_time_);
//    WOLF_TRACE("kps_test_size: ", kps_test.size(), " ", desc_test.rows);
//
//    debug_tTmp = clock();
//    KeyPointVector kps_test_i;
//    cv::Mat desc_test_i;
//    kps_test_i  = det_ptr_->detect(image_incoming_);
//    debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
//    WOLF_TRACE("--> FLIPA MANDARINES '''''' TIME: detect new features: detect ",debug_comp_time_);
//    debug_tTmp = clock();
//    desc_test_i = des_ptr_->getDescriptor(image_incoming_, kps_test_i);
//    debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
//    WOLF_TRACE("--> FLIPA MANDARINES '''''' TIME: detect new features: descript ",debug_comp_time_);
//    WOLF_TRACE("kps_test_size: ", kps_test_i.size(), " ", desc_test_i.rows);
//
//    debug_tTmp = clock();
//    //Matching descriptor vectors using FLANN matcher
//    cv::FlannBasedMatcher matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
//    std::vector< cv::DMatch > matches;
//
//    if(desc_test.type()!=CV_32F)
//        desc_test.convertTo(desc_test, CV_32F);
//
//    if(desc_test_i.type()!=CV_32F)
//        desc_test_i.convertTo(desc_test_i, CV_32F);
//
//    matchFeatures( desc_test, desc_test_i, matches );
//    debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
//    WOLF_TRACE("--> FLIPA LLIMONES '''''' TIME: detect new features: match ",debug_comp_time_);
//    WOLF_TRACE("--> num matches ",matches.size());
//
//    unsigned int n_new_features = 0;
//
//    for (unsigned int n_iterations = 0; _max_new_features == 0 || n_iterations < _max_new_features; n_iterations++)
//    {
//        if (active_search_ptr_->pickEmptyRoi(roi))
//        {
//            // DEBUG
//            debug_tTmp = clock();
//
//            kps  = det_ptr_->detect       (image_last_, roi);
//
//            // DEBUG
//            debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
//            WOLF_TRACE("--> TIME: detect new features: detect ",debug_comp_time_);
//
//            if (kps.size() > 0)
//            {
//                keypoint_filter.retainBest(kps,1);
//
//                // DEBUG
//                WOLF_TRACE("response: " , kps[0].response );
//
//                if(kps[0].response > params_tracker_feature_trifocal_activesearch_ptr_->min_response_new_feature)
//                {
//                    // DEBUG
//                    debug_tTmp = clock();
//
//                    desc = des_ptr_->getDescriptor(image_last_, kps);
//
//                    // DEBUG
//                    debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
//                    WOLF_TRACE("--> TIME: detect new features: descript ",debug_comp_time_);
//
//                    FeaturePointImagePtr point = std::make_shared<FeaturePointImage>(
//                            kps[0],
//                            desc.row(0),
//                            Eigen::Matrix2s::Identity() * params_tracker_feature_trifocal_->pixel_noise_std * params_tracker_feature_trifocal_->pixel_noise_std);
//                    point->setIsKnown(true);
//                    _feature_list_out.push_back(point);
//
//                    active_search_ptr_->hitCell(kps[0]);
//
//                    // Debug ROI
//                    vision_utils::trimRoi(image_incoming_.cols, image_incoming_.rows, roi);
//                    vision_utils::ROIEnhanced roi_e(roi,true);
//                    debug_roi_enh_.push_back(roi_e);
//
//                    n_new_features++;
//                }
//                else
//                {
//                    active_search_ptr_->blockCell(roi);
////
////                    // Debug ROI
////                    vision_utils::trimRoi(image_incoming_.cols, image_incoming_.rows, roi);
////                    vision_utils::ROIEnhanced roi_e(roi,false);
////                    debug_roi_enh_.push_back(roi_e);
//                }
//            }
//            else
//            {
//                active_search_ptr_->blockCell(roi);
////
////                // Debug ROI
////                vision_utils::trimRoi(image_incoming_.cols, image_incoming_.rows, roi);
////                vision_utils::ROIEnhanced roi_e(roi,false);
////                debug_roi_enh_.push_back(roi_e);
//            }
//        }
//        else
//        {
//            WOLF_TRACE("Cannot pick roi. n_iterations: ", n_iterations);
//            break;
//        }
//    }
//
//    WOLF_TRACE("DetectNewFeatures - Number of new features detected: " , n_new_features );
//
//    // DEBUG
//    debug_comp_time_ = (double)(clock() - debug_tStart) / CLOCKS_PER_SEC;
//    WOLF_TRACE("--> TIME: detect new features: TOTAL ",debug_comp_time_);
//    WOLF_TRACE("======== ========= =========");
//
//    return n_new_features;
}

bool ProcessorTrackerFeatureTrifocal::voteForKeyFrame()
{
    // List of conditions

	// A. crossing voting threshold with ascending number of features
	bool vote_up = true;
	// 1. vote if we did not have enough features before
	vote_up = vote_up && (getNewFeaturesListLast().size() < params_tracker_feature_trifocal_->min_features_for_keyframe);
    // 2. vote if we have enough features now
    vote_up = vote_up && (getNewFeaturesListIncoming().size() >= params_tracker_feature_trifocal_->min_features_for_keyframe);

	// B. crossing voting threshold with descending number of features
    bool vote_down = true;
    // 1. vote if we had enough features before
    vote_down = vote_down && (getNewFeaturesListLast().size() >= params_tracker_feature_trifocal_->min_features_for_keyframe);
    // 2. vote if we have not enough features now
    vote_down = vote_down && (getNewFeaturesListIncoming().size() < params_tracker_feature_trifocal_->min_features_for_keyframe);

    return vote_up || vote_down;
}

bool ProcessorTrackerFeatureTrifocal::correctFeatureDrift(const FeatureBasePtr _origin_feature, const FeatureBasePtr _last_feature, FeatureBasePtr _incoming_feature)
{
    return true;
}

unsigned int ProcessorTrackerFeatureTrifocal::trackFeatures(const FeatureBaseList& _feature_list_in, FeatureBaseList& _feature_list_out, FeatureMatchMap& _feature_matches)
{
    // DEBUG =====================================
    debug_tStart = clock();
    WOLF_TRACE("======== TrackFeatures =========");
    // ===========================================

    // DEBUG =====================================
    debug_tTmp = clock();
    // ===========================================

    KeyPointVector kps;
    cv::Mat desc;

    // Detect KeyPoints
    kps  = det_ptr_->detect(image_incoming_);

    // DEBUG =====================================
    debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
    WOLF_TRACE("--> TIME: Track: detect ",debug_comp_time_);
    // ===========================================

    if (kps.size() > 0)
    {
        // DEBUG =====================================
        debug_tTmp = clock();
        // ===========================================

        // Get descriptors
        desc = des_ptr_->getDescriptor(image_incoming_, kps);

        // DEBUG =====================================
        debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
        WOLF_TRACE("--> TIME: Track: descript ",debug_comp_time_);
        // ===========================================

        // DEBUG =====================================
        debug_tTmp = clock();
        // ===========================================

        KeyPointVector new_kps;
        cv::Mat new_desc;

        // Get KeyPoints and descriptors
        // sort them by response
        // cut them at min_response_new_feature
        // And finally filling the grid
        fillGrid(grid_incoming_, params_tracker_feature_trifocal_->max_new_features, kps, desc, new_kps, new_desc);

        // DEBUG =====================================
        debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
        WOLF_TRACE("--> TIME: Track: Fill grid ",debug_comp_time_);
        // ===========================================

        // DEBUG =====================================
        debug_tTmp = clock();
        // ===========================================

        for (auto feature_base_ptr : _feature_list_in)
        {
            WOLF_TRACE("");

            // Feature and descriptor
            FeaturePointImagePtr target_feature = std::static_pointer_cast<FeaturePointImage>(feature_base_ptr);
            cv::KeyPoint target_keypoint = target_feature->getKeypoint();
            cv::Mat target_descriptor = target_feature->getDescriptor();

            WOLF_TRACE("target feature ",target_keypoint.pt.x, "x", target_keypoint.pt.y);

            // get target cell
            Eigen::Vector2i target_cell = grid_incoming_->getCell(target_keypoint);

            WOLF_TRACE("target_cell: ",target_cell);

            // Augment candidates features to surrounding cells;
            // Get candidate KeyPoints and descriptors of surroinding cells
            KeyPointVector candidate_kps;
            cv::Mat candidate_desc;

            WOLF_TRACE("PREV get candidates");


            getCandidatesFromGrid(target_cell(0), target_cell(1), grid_incoming_, kps, desc, candidate_kps, candidate_desc);

            WOLF_TRACE("AFTER get candidates");
            WOLF_TRACE("Size candidates kps: ", candidate_kps.size(), "  desc: ", candidate_desc.rows);

            if (candidate_kps.size() > 0)
            {
                // Try to match with candidates

                cv::DMatch cv_match;
                Scalar normalized_score = mat_ptr_->match(target_descriptor, candidate_desc, des_ptr_->getSize(), cv_match);

                WOLF_TRACE("MATCH");

                if ( normalized_score > mat_ptr_->getParams()->min_norm_score )
                {
                    WOLF_TRACE("");

                    cv::KeyPoint tracked_kp = candidate_kps[cv_match.trainIdx];

                    WOLF_TRACE("candidate cols", candidate_desc.cols);

                    cv::Mat tracked_desc = candidate_desc(cv::Rect(0,cv_match.trainIdx,candidate_desc.cols,1));

                    WOLF_TRACE("");

                    FeaturePointImagePtr incoming_point_ptr = std::make_shared<FeaturePointImage>(
                            tracked_kp, tracked_desc,
                            Eigen::Matrix2s::Identity() * params_tracker_feature_trifocal_->pixel_noise_std * params_tracker_feature_trifocal_->pixel_noise_std);

                    WOLF_TRACE("");

                    incoming_point_ptr->setIsKnown(target_feature->isKnown());

                    _feature_list_out.push_back(incoming_point_ptr);

                    _feature_matches[incoming_point_ptr] = std::make_shared<FeatureMatch>(FeatureMatch({feature_base_ptr, normalized_score}));

                    //                // Debug ROI
                    //                vision_utils::trimRoi(image_incoming_.cols, image_incoming_.rows, roi);
                    //                vision_utils::ROIEnhanced roi_e(roi,true);
                    //                debug_roi_enh_.push_back(roi_e);
                }
                WOLF_TRACE("DONE WOLF struct FEAT");
            }
        }

        // DEBUG =====================================
        debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
        WOLF_TRACE("--> TIME: Track: Match selecting surr. cells",debug_comp_time_);
        // ===========================================
    }
    return _feature_list_out.size();

//    KeyPointVector kps;
//    cv::Mat desc;
//
////    // Debug
////    debug_roi_enh_.clear();
//    debug_tStart = clock();
//    WOLF_TRACE("======== trackFeatures =========");
//
//    for (auto feature_base_ptr : _feature_list_in)
//    {
//        // Feature and descriptor
//        FeaturePointImagePtr target_feature = std::static_pointer_cast<FeaturePointImage>(feature_base_ptr);
//        cv::Mat target_descriptor = target_feature->getDescriptor();
//
//        // Hit cell
//        active_search_ptr_->hitCell(target_feature->getKeypoint());
//
//        // Set ROI
//        cv::Rect roi = vision_utils::setRoi(target_feature->getKeypoint().pt.x, target_feature->getKeypoint().pt.y, cell_width_, cell_height_);
//
//        // DEBUG
//        debug_tTmp = clock();
//
//        // Detect features in ROI
//        kps = det_ptr_->detect(image_incoming_, roi);
//
//        // DEBUG
//        debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
//        WOLF_TRACE("--> TIME: track: detect ",debug_comp_time_);
//
//        if (kps.size() > 0)
//        {
//            // DEBUG
//            debug_tTmp = clock();
//
//            // New descriptors in ROI
//            desc = des_ptr_->getDescriptor(image_incoming_, kps);
//
//            // DEBUG
//            debug_comp_time_ = (double)(clock() - debug_tTmp) / CLOCKS_PER_SEC;
//            WOLF_TRACE("--> TIME: track: descript ",debug_comp_time_);
//
//            // Match
//            cv::DMatch cv_match;
//            Scalar normalized_score = mat_ptr_->match(target_descriptor, desc, des_ptr_->getSize(), cv_match);
//
//            if ( normalized_score > mat_ptr_->getParams()->min_norm_score )
//            {
//                cv::KeyPoint tracked_kp = kps[cv_match.trainIdx];
//                cv::Mat tracked_desc = desc(cv::Rect(0,cv_match.trainIdx,desc.cols,1));
//
//                FeaturePointImagePtr incoming_point_ptr = std::make_shared<FeaturePointImage>(
//                        tracked_kp, tracked_desc,
//                        Eigen::Matrix2s::Identity() * params_tracker_feature_trifocal_->pixel_noise_std * params_tracker_feature_trifocal_->pixel_noise_std);
//
//                incoming_point_ptr->setIsKnown(target_feature->isKnown());
//
//                _feature_list_out.push_back(incoming_point_ptr);
//
//                _feature_matches[incoming_point_ptr] = std::make_shared<FeatureMatch>(FeatureMatch({feature_base_ptr, normalized_score}));
//
////                // Debug ROI
////                vision_utils::trimRoi(image_incoming_.cols, image_incoming_.rows, roi);
////                vision_utils::ROIEnhanced roi_e(roi,true);
////                debug_roi_enh_.push_back(roi_e);
//            }
////            else
////            {
////                // Debug ROI
////                vision_utils::trimRoi(image_incoming_.cols, image_incoming_.rows, roi);
////                vision_utils::ROIEnhanced roi_e(roi,false);
////                debug_roi_enh_.push_back(roi_e);
////            }
//        }
////        else
////        {
////            // Debug ROI
////            vision_utils::trimRoi(image_incoming_.cols, image_incoming_.rows, roi);
////            vision_utils::ROIEnhanced roi_e(roi,false);
////            debug_roi_enh_.push_back(roi_e);
////        }
//    }
//
//    // DEBUG
//    debug_comp_time_ = (double)(clock() - debug_tStart) / CLOCKS_PER_SEC;
//    WOLF_TRACE("--> TIME: track: ",debug_comp_time_);
//    WOLF_TRACE("======== ========= =========");
//
//    return _feature_list_out.size();
}

void ProcessorTrackerFeatureTrifocal::advanceDerived()
{
    ProcessorTrackerFeature::advanceDerived();
    image_last_ = image_incoming_;
    grid_last_ = grid_incoming_;
}

void ProcessorTrackerFeatureTrifocal::resetDerived()
{
    // Call parent class method
    ProcessorTrackerFeature::resetDerived();

    image_last_ = image_incoming_;
    grid_last_ = grid_incoming_;

    // Conditionally assign the prev_origin pointer
    if (initialized_)
        prev_origin_ptr_ = origin_ptr_;

//    // Print resulting list
//    TrackMatches matches_prevorig_origin = track_matrix_.matches(prev_origin_ptr_, origin_ptr_);
//    for (auto const & pair_trkid_pair : matches_prevorig_origin)
//    {
//        FeatureBasePtr feature_in_prev   = pair_trkid_pair.second.first;
//        FeatureBasePtr feature_in_origin = pair_trkid_pair.second.second;
//
//        WOLF_DEBUG("Matches reset prev <-- orig: track: ", pair_trkid_pair.first,
//                   " prev: ", feature_in_prev->id(),
//                   " origin: ", feature_in_origin->id());
//    }
}

void ProcessorTrackerFeatureTrifocal::preProcess()
{
    if (!initialized_)
    {
        if (origin_ptr_ && last_ptr_ && (last_ptr_ != origin_ptr_) && prev_origin_ptr_ == nullptr)
            prev_origin_ptr_ = origin_ptr_;

        if (prev_origin_ptr_ && origin_ptr_ && last_ptr_ && prev_origin_ptr_ != origin_ptr_)
            initialized_ = true;
    }

    image_incoming_ = std::static_pointer_cast<CaptureImage>(incoming_ptr_)->getImage();

    grid_incoming_ = std::make_shared<vision_utils::FeatureIdxGrid>(image_incoming_.cols, image_incoming_.rows, params_tracker_feature_trifocal_->n_cells_h, params_tracker_feature_trifocal_->n_cells_v);

//    active_search_ptr_->renew();

//    // Debug
//    debug_roi_enh_.clear();
}

void ProcessorTrackerFeatureTrifocal::postProcess()
{
    // DEBUG
    std::vector<vision_utils::KeyPointEnhanced> kps_e;
    for (auto const & feat_base : last_ptr_->getFeatureList())
    {
        FeaturePointImagePtr feat = std::static_pointer_cast<FeaturePointImage>(feat_base);
        unsigned int id = feat->id();
        unsigned int track_id = feat->trackId();
        vision_utils::KeyPointEnhanced kp_e(feat->getKeypoint(), id, track_id, track_matrix_.trackSize(track_id), feat->getMeasurementCovariance());
        kps_e.push_back(kp_e);
    }

    // DEBUG
//    cv::Mat img = (std::static_pointer_cast<CaptureImage>(last_ptr_))->getImage();
//    cv::Mat img_proc = vision_utils::buildImageProcessed(img, kps_e, debug_roi_enh_);
//
//    cv::imshow("DEBUG VIEW", img_proc);
//    cv::waitKey(1);
}

ConstraintBasePtr ProcessorTrackerFeatureTrifocal::createConstraint(FeatureBasePtr _feature_ptr, FeatureBasePtr _feature_other_ptr)
{
    // NOTE: This function cannot be implemented because the API lacks an input to feature_prev_origin.
    // Therefore, we implement establishConstraints() instead and do all the job there.
    // This function remains empty, and with a warning or even an error issued in case someone calls it.
    std::cout << "033[1;33m [WARN]:033[0m ProcessorTrackerFeatureTrifocal::createConstraint is empty." << std::endl;
    ConstraintBasePtr return_var{}; //TODO: fill this variable
    return return_var;
}

void ProcessorTrackerFeatureTrifocal::establishConstraints()
{
    if (initialized_)
    {
        // get tracks between prev, origin and last
        TrackMatches matches = track_matrix_.matches(prev_origin_ptr_, last_ptr_); // it's guaranteed by construction that the track also includes origin

        for (auto pair_trkid_match : matches) // OMG this will add potentially a loooot of constraints! TODO see a smarter way of adding constraints
        {
            // get track ID
            size_t trk_id = pair_trkid_match.first;

            // get the three features for this track
            // FeatureBasePtr ftr_prev = track_matrix_.feature(trk_id, prev_origin_ptr_); // left here for ref, but implemented in a quicker way below
            // FeatureBasePtr ftr_last = track_matrix_.feature(trk_id, last_ptr_); // same here
            FeatureBasePtr ftr_prev = pair_trkid_match.second.first;
            FeatureBasePtr ftr_orig = track_matrix_.feature(trk_id, origin_ptr_); // because it's a tracker, this feature in the middle of prev and last exists for sure!
            FeatureBasePtr ftr_last = pair_trkid_match.second.second;

            // make constraint
            ConstraintAutodiffTrifocalPtr ctr = std::make_shared<ConstraintAutodiffTrifocal>(ftr_prev, ftr_orig, ftr_last, shared_from_this(), false, CTR_ACTIVE);

            // link to wolf tree
            ftr_last->addConstraint(ctr);
            ftr_orig->addConstrainedBy(ctr);
            ftr_prev->addConstrainedBy(ctr);
        }
    }
}

void ProcessorTrackerFeatureTrifocal::configure(SensorBasePtr _sensor)
{
    SensorCameraPtr camera = std::static_pointer_cast<SensorCamera>(_sensor);

    camera->setNoiseStd(Vector2s::Ones() * params_tracker_feature_trifocal_->pixel_noise_std);

//    active_search_ptr_->initAlg(camera->getImgWidth(), camera->getImgHeight() , det_ptr_->getPatternRadius());
//
//    params_tracker_feature_trifocal_activesearch_ptr_ = std::static_pointer_cast<vision_utils::AlgorithmParamsACTIVESEARCH>( active_search_ptr_->getParams() );
//
//    cell_width_  = camera->getImgWidth()  / params_tracker_feature_trifocal_activesearch_ptr_->n_cells_h;
//    cell_height_ = camera->getImgHeight() / params_tracker_feature_trifocal_activesearch_ptr_->n_cells_v;
}

ProcessorBasePtr ProcessorTrackerFeatureTrifocal::create(const std::string& _unique_name,
                                                         const ProcessorParamsBasePtr _params,
                                                         const SensorBasePtr _sensor_ptr)
{
  const auto params = std::static_pointer_cast<ProcessorParamsTrackerFeatureTrifocal>(_params);

  ProcessorBasePtr prc_ptr = std::make_shared<ProcessorTrackerFeatureTrifocal>(params);
  prc_ptr->setName(_unique_name);
  prc_ptr->configure(_sensor_ptr);
  return prc_ptr;
}

} // namespace wolf

// Register in the ProcessorFactory
#include "processor_factory.h"
namespace wolf {
WOLF_REGISTER_PROCESSOR("TRACKER FEATURE TRIFOCAL", ProcessorTrackerFeatureTrifocal)
} // namespace wolf
