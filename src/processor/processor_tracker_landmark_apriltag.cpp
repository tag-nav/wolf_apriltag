#include "apriltag/processor/processor_tracker_landmark_apriltag.h"
#include "apriltag/feature/feature_apriltag.h"
#include "apriltag/landmark/landmark_apriltag.h"
#include "apriltag/factor/factor_autodiff_apriltag.h"
#include "apriltag/processor/ippe.h"

#include "vision/capture/capture_image.h"

#include "core/math/rotations.h"
#include "core/state_block/state_quaternion.h"
#include "vision/math/pinhole_tools.h"

// April tags
#include "apriltag/common/homography.h"
#include "apriltag/common/zarray.h"

#include <apriltag/tag16h5.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>


// #include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

namespace wolf {


// Constructor
ProcessorTrackerLandmarkApriltag::ProcessorTrackerLandmarkApriltag( ProcessorParamsTrackerLandmarkApriltagPtr _params_tracker_landmark_apriltag) :
        ProcessorTrackerLandmark("TRACKER LANDMARK APRILTAG",  _params_tracker_landmark_apriltag ),
        tag_widths_(_params_tracker_landmark_apriltag->tag_widths_),
        tag_width_default_(_params_tracker_landmark_apriltag->tag_width_default_),
        std_xy_ (_params_tracker_landmark_apriltag->std_xy_ ),
        std_z_  (_params_tracker_landmark_apriltag->std_z_  ),
        std_rpy_(_params_tracker_landmark_apriltag->std_rpy_),
        std_pix_(_params_tracker_landmark_apriltag->std_pix_),
        ippe_min_ratio_(_params_tracker_landmark_apriltag->ippe_min_ratio_),
        ippe_max_rep_error_(_params_tracker_landmark_apriltag->ippe_max_rep_error_),
        cv_K_(3,3),
        reestimate_last_frame_(_params_tracker_landmark_apriltag->reestimate_last_frame_),
        n_reset_(0),
        min_time_vote_(_params_tracker_landmark_apriltag->min_time_vote_),
        max_time_vote_(_params_tracker_landmark_apriltag->max_time_vote_),
        min_features_for_keyframe_(_params_tracker_landmark_apriltag->min_features_for_keyframe),
        max_features_diff_(_params_tracker_landmark_apriltag->max_features_diff_),
        nb_vote_for_every_first_(_params_tracker_landmark_apriltag->nb_vote_for_every_first_),
        enough_info_necessary_(_params_tracker_landmark_apriltag->enough_info_necessary_),
        add_3D_cstr_(_params_tracker_landmark_apriltag->add_3D_cstr_),
        nb_vote_(0)

{
    // configure apriltag detector
    std::string famname(_params_tracker_landmark_apriltag->tag_family_);
    if (famname == "tag16h5")
        tag_family_ = *tag16h5_create();
    else if (famname == "tag25h9")
        tag_family_ = *tag25h9_create();
    else if (famname == "tag36h11")
        tag_family_ = *tag36h11_create();
    else if (famname == "tagCircle21h7")
        tag_family_ = *tagCircle21h7_create();
    else if (famname == "tagCircle49h12")
        tag_family_ = *tagCircle49h12_create();
    else if (famname == "tagCustom48h12")
        tag_family_ = *tagCustom48h12_create();
    else if (famname == "tagStandard41h12")
        tag_family_ = *tagStandard41h12_create();
    else if (famname == "tagStandard52h13")
        tag_family_ = *tagStandard52h13_create();
    else {
        WOLF_ERROR(famname, ": Unrecognized tag family name. Use e.g. \"tag36h11\".");
        exit(-1);
    }

    // tag_family_.black_border     = _params_tracker_landmark_apriltag->tag_black_border_;  // not anymore in apriltag 3

    detector_ = *apriltag_detector_create();
    apriltag_detector_add_family(&detector_, &tag_family_);

    detector_.quad_decimate     = _params_tracker_landmark_apriltag->quad_decimate_;
    detector_.quad_sigma        = _params_tracker_landmark_apriltag->quad_sigma_;
    detector_.nthreads          = _params_tracker_landmark_apriltag->nthreads_;
    detector_.debug             = _params_tracker_landmark_apriltag->debug_;
    detector_.refine_edges      = _params_tracker_landmark_apriltag->refine_edges_;
}

// Destructor
ProcessorTrackerLandmarkApriltag::~ProcessorTrackerLandmarkApriltag()
{
    // destroy raw pointers in detector_
    //apriltag_detector_destroy(&detector_); cannot be used because it is trying to free() the detector_ itself that is not implemented as a raw pointer in our case
    timeprofile_destroy(detector_.tp);
    apriltag_detector_clear_families(&detector_);
    zarray_destroy(detector_.tag_families);
    workerpool_destroy(detector_.wp);

    //free raw pointers in tag_family_
    free(tag_family_.name);
    free(tag_family_.codes);
}


ProcessorBasePtr ProcessorTrackerLandmarkApriltag::create(const std::string& _unique_name, const ProcessorParamsBasePtr _params, const SensorBasePtr _sen_ptr)
{
    std::shared_ptr<ProcessorParamsTrackerLandmarkApriltag> prc_apriltag_params;
    if (_params)
        prc_apriltag_params = std::static_pointer_cast<ProcessorParamsTrackerLandmarkApriltag>(_params);
    else
        prc_apriltag_params = std::make_shared<ProcessorParamsTrackerLandmarkApriltag>();

    ProcessorTrackerLandmarkApriltagPtr prc_ptr = std::make_shared<ProcessorTrackerLandmarkApriltag>(prc_apriltag_params);
    prc_ptr->setName(_unique_name);
    return prc_ptr;
}

void ProcessorTrackerLandmarkApriltag::preProcess()
{
    //clear wolf detections so that new ones will be stored inside
    detections_incoming_.clear();

    // The image is assumed to be of color BGR2 type
    cv::cvtColor(std::static_pointer_cast<CaptureImage>(incoming_ptr_)->getImage(), grayscale_image_, cv::COLOR_BGR2GRAY);   
    
    //detect tags in incoming image
    // Make an image_u8_t header for the Mat data
    image_u8_t im = {   .width  = grayscale_image_.cols,
                        .height = grayscale_image_.rows,
                        .stride = grayscale_image_.cols,
                        .buf    = grayscale_image_.data
                    };

    // run Apriltag detector
    zarray_t *detections = apriltag_detector_detect(&detector_, &im);
    // loop all detections
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        int    tag_id     = det->id;
        Scalar tag_width  = getTagWidth(tag_id);   // tag width in meters

        Eigen::Affine3ds c_M_t;
        bool use_rotation = true;
        //////////////////
        // IPPE (Infinitesimal Plane-based Pose Estimation)
        //////////////////
        Eigen::Affine3ds M_ippe1, M_ippe2, M_april, M_PnP;
        Scalar rep_error1;
        Scalar rep_error2;
        ippePoseEstimation(det, cv_K_, tag_width, M_ippe1, rep_error1, M_ippe2, rep_error2);
        // If not so sure about whether we have the right solution or not, do not create a feature
        use_rotation = ((rep_error2 / rep_error1 > ippe_min_ratio_) && rep_error1 < ippe_max_rep_error_);
        //////////////////
        c_M_t = M_ippe1;

        // set the measured pose vector
        Eigen::Vector3s translation ( c_M_t.translation() ); // translation vector in apriltag meters
        Eigen::Vector7s pose;
        pose << translation, R2q(c_M_t.linear()).coeffs();

        // compute the covariance
        // Eigen::Matrix6s cov = getVarVec().asDiagonal() ;  // fixed dummy covariance
        Eigen::Matrix6s info = computeInformation(translation, c_M_t.linear(), K_, tag_width, std_pix_);  // Lie jacobians covariance

        if (!use_rotation){
            // Put a very high covariance on angles measurements (low info matrix)
            info.bottomLeftCorner(3,3) = Eigen::Matrix3s::Zero();
            info.topRightCorner(3,3)    = Eigen::Matrix3s::Zero();
            info.bottomRightCorner(3,3) = 0.0001 * Eigen::Matrix3s::Identity();
        }

        // add to detected features list
        detections_incoming_.push_back(std::make_shared<FeatureApriltag>(pose, info, tag_id, *det, rep_error1, rep_error2, use_rotation, 
                                                                         FeatureBase::UncertaintyType::UNCERTAINTY_IS_INFO));
    }

    apriltag_detections_destroy(detections);
}

void ProcessorTrackerLandmarkApriltag::ippePoseEstimation(apriltag_detection_t *_det, cv::Mat_<Scalar> _K, double _tag_width,
                            Eigen::Affine3d &_M1,
                            double &_rep_error1,
                            Eigen::Affine3d &_M2,
                            double &_rep_error2){

    // get corners from det
    std::vector<cv::Point2d> corners_pix(4);
    for (int i = 0; i < 4; i++)
    {
        corners_pix[i].x = _det->p[i][0];
        corners_pix[i].y = _det->p[i][1];
    }
    std::vector<cv::Point3d> obj_pts;
    // Same order as the 2D corners (anti clockwise, looking at the tag).
    // Looking at the tag, the reference frame is
    // X = Right, Y = Down, Z = Inside the plane
    Scalar s = _tag_width/2;
    obj_pts.emplace_back(-s,  s, 0); // bottom left
    obj_pts.emplace_back( s,  s, 0); // bottom right
    obj_pts.emplace_back( s, -s, 0); // top right
    obj_pts.emplace_back(-s, -s, 0); // top left

    cv::Mat rvec1, tvec1, rvec2, tvec2;
    float err1, err2;
    IPPE::PoseSolver pose_solver;

    pose_solver.solveSquare(_tag_width, corners_pix, _K, cv::Mat(),
                            rvec1, tvec1, err1,
                            rvec2, tvec2, err2);

    _rep_error1 = err1;
    _rep_error2 = err2;

    // Puts the result in a Eigen affine Transform
    cv::Matx33d rmat1;
    cv::Rodrigues(rvec1, rmat1);
    Eigen::Matrix3s R_eigen1; cv2eigen(rmat1, R_eigen1);
    Eigen::Vector3s t_eigen1; cv2eigen(tvec1, t_eigen1);
    _M1.matrix().block(0,0,3,3) = R_eigen1;
    _M1.matrix().block(0,3,3,1) = t_eigen1;

    cv::Matx33d rmat2;
    cv::Rodrigues(rvec2, rmat2);
    Eigen::Matrix3s R_eigen2; cv2eigen(rmat2, R_eigen2);
    Eigen::Vector3s t_eigen2; cv2eigen(tvec2, t_eigen2);
    _M2.matrix().block(0,0,3,3) = R_eigen2;
    _M2.matrix().block(0,3,3,1) = t_eigen2;
}


void ProcessorTrackerLandmarkApriltag::postProcess()
{

}

FactorBasePtr ProcessorTrackerLandmarkApriltag::createFactor(FeatureBasePtr _feature_ptr,
                                                                     LandmarkBasePtr _landmark_ptr)
{
    FactorAutodiffApriltagPtr constraint = std::make_shared<FactorAutodiffApriltag>(
            getSensor(),
            getLast()->getFrame(),
            std::static_pointer_cast<LandmarkApriltag>(_landmark_ptr),
            std::static_pointer_cast<FeatureApriltag> (_feature_ptr ),
            true,
            FAC_ACTIVE
    );
    return constraint;
}

LandmarkBasePtr ProcessorTrackerLandmarkApriltag::createLandmark(FeatureBasePtr _feature_ptr)
{
    // world to rob
    Vector3s pos = getLast()->getFrame()->getP()->getState();
    Quaternions quat (getLast()->getFrame()->getO()->getState().data());
    Eigen::Affine3ds w_M_r = Eigen::Translation<Scalar,3>(pos.head(3)) * quat;

    // rob to camera
    pos = getSensor()->getP()->getState();
    quat.coeffs() = getSensor()->getO()->getState();
    Eigen::Affine3ds r_M_c = Eigen::Translation<Scalar,3>(pos.head(3)) * quat;

    // camera to lmk (tag)
    pos = _feature_ptr->getMeasurement().head(3);
    quat.coeffs() = _feature_ptr->getMeasurement().tail(4);
    Eigen::Affine3ds c_M_t   = Eigen::Translation<Scalar,3>(pos) * quat;

    // world to lmk (tag)
    Eigen::Affine3ds w_M_t = w_M_r * r_M_c * c_M_t;

    // make 7-vector for lmk (tag) pose
    pos  = w_M_t.translation();
    quat = w_M_t.linear();
    Vector7s w_pose_t;
    w_pose_t << pos, quat.coeffs();

    FeatureApriltagPtr feat_april = std::static_pointer_cast<FeatureApriltag>(_feature_ptr);
    int tag_id = feat_april->getTagId();

    // LandmarkApriltagPtr new_landmark = std::make_shared<LandmarkApriltag>(w_pose_t, tag_id, getTagWidth(tag_id));
    // LandmarkApriltagPtr new_landmark = std::static_pointer_cast<LandmarkApriltag>(LandmarkBase::emplace<LandmarkApriltag>(nullptr, w_pose_t, tag_id, getTagWidth(tag_id)));
    return LandmarkBase::emplace<LandmarkApriltag>(nullptr, w_pose_t, tag_id, getTagWidth(tag_id));
}

unsigned int ProcessorTrackerLandmarkApriltag::detectNewFeatures(const int& _max_features, FeatureBasePtrList& _new_features_last)
{
    LandmarkBasePtrList& landmark_list = getProblem()->getMap()->getLandmarkList();
    for (auto feature_in_image : detections_last_)
    {
        bool feature_already_found(false);
        // list of landmarks in the map

        //Loop over the landmark to find is one is associated to  feature_in_image
        for(auto it = landmark_list.begin(); it != landmark_list.end(); ++it){
            if(std::static_pointer_cast<LandmarkApriltag>(*it)->getTagId() == std::static_pointer_cast<FeatureApriltag>(feature_in_image)->getTagId()){
                feature_already_found = true;
                break;
            }
        }

        if (!feature_already_found)
        {
            for (FeatureBasePtrList::iterator it=_new_features_last.begin(); it != _new_features_last.end(); ++it)
                if (std::static_pointer_cast<FeatureApriltag>(*it)->getTagId() == std::static_pointer_cast<FeatureApriltag>(feature_in_image)->getTagId())
                {
                    //we have a detection with the same id as the currently processed one. We remove the previous feature from the list for now
                    _new_features_last.erase(it);
                    //it should not be possible two detection with the same id before getting there so we can stop here.
                    break; 
                }
            // discard features that do not have orientation information
            if (!std::static_pointer_cast<FeatureApriltag>(feature_in_image)->getUserotation())
                continue;

            // If the feature is not in the map & not in the list of newly detected features yet then we add it.
            _new_features_last.push_back(feature_in_image); 
        } //otherwise we check the next feature
    }

    return _new_features_last.size();
}

bool ProcessorTrackerLandmarkApriltag::voteForKeyFrame()
{   
    // Necessary conditions
    bool more_in_last = getLast()->getFeatureList().size() >= min_features_for_keyframe_;
    // Vote for every image processed at the beginning, bypasses the others
    if (more_in_last && (nb_vote_ < nb_vote_for_every_first_)){
        nb_vote_++;
        return true;
    }
    // Check if enough information is provided by the measurements to determine uniquely the position of the KF
    // Is only activated when enough_info_necessary_ is set to true
    bool enough_info;
    if (enough_info_necessary_){
        int nb_userot = 0;
        int nb_not_userot = 0;
        for (auto it_feat = getLast()->getFeatureList().begin(); it_feat != getLast()->getFeatureList().end(); it_feat++){
            FeatureApriltagPtr feat_apr_ptr = std::static_pointer_cast<FeatureApriltag>(*it_feat);
            if (feat_apr_ptr->getUserotation()){
                nb_userot++;
            }
            else{
                nb_not_userot++;
            }
        }
        // Condition on wether enough information is provided by the measurements:
        // Position + rotation OR more that 3 3D translations (3 gives 2 symmetric solutions)
        enough_info = (nb_userot > 0) || (nb_not_userot > 3);
    }
    else{
        enough_info = true;
    }
    Scalar dt_incoming_origin = getIncoming()->getTimeStamp().get() - getOrigin()->getTimeStamp().get();
    bool more_than_min_time_vote = dt_incoming_origin > min_time_vote_; 

    // Possible decision factors
    bool too_long_since_last_KF = dt_incoming_origin > max_time_vote_;
    bool less_in_incoming = getIncoming()->getFeatureList().size() <  min_features_for_keyframe_;
    int diff = getOrigin()->getFeatureList().size() - getIncoming()->getFeatureList().size();
    bool too_big_feature_diff = (abs(diff) >=  max_features_diff_);

    // Final decision logic
    bool vote = enough_info && more_than_min_time_vote && more_in_last && (less_in_incoming || too_long_since_last_KF || too_big_feature_diff);
    return vote;
}

unsigned int ProcessorTrackerLandmarkApriltag::findLandmarks(const LandmarkBasePtrList& _landmark_list_in,
                                                             FeatureBasePtrList& _feature_list_out,
                                                             LandmarkMatchMap& _feature_landmark_correspondences)
{   
    for (auto feature_in_image : detections_incoming_)
    {
        int tag_id(std::static_pointer_cast<FeatureApriltag>(feature_in_image)->getTagId());

        for (auto landmark_in_ptr : _landmark_list_in)
        {
            if(std::static_pointer_cast<LandmarkApriltag>(landmark_in_ptr)->getTagId() == tag_id)
            {
                _feature_list_out.push_back(feature_in_image);
                Scalar score(1.0);
                LandmarkMatchPtr matched_landmark = std::make_shared<LandmarkMatch>(landmark_in_ptr, score); //TODO: smarter score
                _feature_landmark_correspondences.emplace ( feature_in_image, matched_landmark );
                break;
            }
        }
    }

    return _feature_list_out.size();
}

wolf::Scalar ProcessorTrackerLandmarkApriltag::getTagWidth(int _id) const
{
    if (tag_widths_.find(_id) != tag_widths_.end())
        return tag_widths_.at(_id);
    else
        return tag_width_default_;
}

Eigen::Vector6s ProcessorTrackerLandmarkApriltag::getVarVec()
{
    Eigen::Vector6s var_vec;
    var_vec << std_xy_*std_xy_, std_xy_*std_xy_, std_z_*std_z_, std_rpy_*std_rpy_, std_rpy_*std_rpy_, std_rpy_*std_rpy_;

    return var_vec;
}

Eigen::Matrix6s ProcessorTrackerLandmarkApriltag::computeInformation(Eigen::Vector3s const &t, Eigen::Matrix3s const &R, Eigen::Matrix3s const &K, Scalar const &tag_width, double const &sig_q)
{
    // Same order as the 2D corners (anti clockwise, looking at the tag).
    // Looking at the tag, the reference frame is
    // X = Right, Y = Down, Z = Inside the plane
    Scalar s = tag_width/2;
    Eigen::Vector3s p1; p1 << -s,  s, 0; // bottom left
    Eigen::Vector3s p2; p2 <<  s,  s, 0; // bottom right
    Eigen::Vector3s p3; p3 <<  s, -s, 0; // top right
    Eigen::Vector3s p4; p4 << -s, -s, 0; // top left
    std::vector<Eigen::Vector3s> pvec = {p1, p2, p3, p4};
    std::vector<Eigen::Vector2s> proj_pix_vec; proj_pix_vec.resize(4);

    // Initialize jacobian matrices
    Eigen::Matrix<Scalar, 8, 6> J_u_TR = Eigen::Matrix<Scalar, 8, 6>::Zero();  // 2N x 6 jac
    Eigen::Vector3s h;
    Eigen::Matrix3s J_h_T, J_h_R;
    Eigen::Vector2s eu;  // 2D pixel coord, not needed
    Eigen::Matrix<Scalar, 3, 6> J_h_TR;
    Eigen::Matrix<Scalar, 2, 3> J_u_h;
    for (int i=0; i < pvec.size(); i++){
        // Pinhole projection to non normalized homogeneous coordinates in pixels (along with jacobians)
        pinholeHomogeneous(K, t, R, pvec[i], h, J_h_T, J_h_R);
        // 3 x 6 tag to camera translation|rotation jacobian
        J_h_TR << J_h_T, J_h_R;
        // Euclidianization Jacobian
        pinhole::projectPointToNormalizedPlane(h, eu, J_u_h);
        // Fill jacobian for ith corner
        J_u_TR.block(2*i, 0, 2, 6) = J_u_h * J_h_TR;
        proj_pix_vec[i] = eu;
    }

    // Pixel uncertainty covariance matrix
    Eigen::Matrix<Scalar, 8, 8> pixel_cov = pow(sig_q, 2) * Eigen::Matrix<Scalar, 8, 8>::Identity();
    // 6 x 6 translation|rotation information matrix computed with covariance propagation formula (inverted)
    Eigen::Matrix6s transformation_info  = (J_u_TR.transpose() * pixel_cov.inverse() * J_u_TR);  // Wolf jac

    return transformation_info;

}

void ProcessorTrackerLandmarkApriltag::pinholeHomogeneous(Eigen::Matrix3s const & K, Eigen::Vector3s const & t,
                                                          Eigen::Matrix3s const & R, Eigen::Vector3s const & p,
                                                          Eigen::Vector3s &h, Eigen::Matrix3s &J_h_T, Eigen::Matrix3s &J_h_R)
{
    // Pinhole projection + jacobians
    h =  K * (t + R * p);
    J_h_T = K;
    Eigen::Matrix3s p_hat;
    p_hat << 0, -p(2), p(1),
             p(2), 0, -p(0),
            -p(1), p(0), 0;
    J_h_R = -K * R * p_hat;
}

FeatureBasePtrList ProcessorTrackerLandmarkApriltag::getIncomingDetections() const
{
    return detections_incoming_;
}

FeatureBasePtrList ProcessorTrackerLandmarkApriltag::getLastDetections() const
{
    return detections_last_;
}

void ProcessorTrackerLandmarkApriltag::configure(SensorBasePtr _sensor)
{
    SensorCameraPtr sen_cam_ptr = std::static_pointer_cast<SensorCamera>(_sensor);
    sen_cam_ptr->useRectifiedImages();

    // get camera intrinsic parameters
    Eigen::Vector4s k(_sensor->getIntrinsic()->getState()); //[cx cy fx fy]

    // Intrinsic matrices for opencv and eigen:

    cv_K_ << k(2),    0, k(0),
                0, k(3), k(1),
                0,    0,    1;

    K_ << k(2),    0, k(0),
             0, k(3), k(1),
             0,    0,    1;

}

void ProcessorTrackerLandmarkApriltag::advanceDerived()
{
    ProcessorTrackerLandmark::advanceDerived();
    detections_last_ = std::move(detections_incoming_);
}

void ProcessorTrackerLandmarkApriltag::resetDerived()
{   
    // Add 3D distance constraint between 2 frames
    if (getProblem()->getProcessorMotion() == nullptr && add_3D_cstr_){
        if ((getOrigin() != nullptr) && 
            (getOrigin()->getFrame() != nullptr) && 
            (getOrigin() != getLast()) &&
            (getOrigin()->getFrame() != getLast()->getFrame()) 
            )
        {

            FrameBasePtr ori_frame = getOrigin()->getFrame();
            Eigen::Vector1s dist_meas; dist_meas << 0.0;
            double dist_std = 0.5;
            Eigen::Matrix1s cov0(dist_std*dist_std);

            CaptureBasePtr capt3D = std::make_shared<CaptureBase>("Dist", getLast()->getTimeStamp());
            getLast()->getFrame()->addCapture(capt3D);
            FeatureBasePtr feat_dist = capt3D->addFeature(std::make_shared<FeatureBase>("Dist", dist_meas, cov0));
            FactorAutodiffDistance3DPtr cstr = std::make_shared<FactorAutodiffDistance3D>(feat_dist, ori_frame, nullptr, false, FAC_ACTIVE);
            feat_dist->addFactor(cstr);
            ori_frame->addConstrainedBy(cstr);    
        }
    }
    
    if ((getProblem()->getProcessorMotion() == nullptr) && reestimate_last_frame_){
        reestimateLastFrame();
    }

    ProcessorTrackerLandmark::resetDerived();
    detections_last_ = std::move(detections_incoming_);
}

void ProcessorTrackerLandmarkApriltag::reestimateLastFrame(){
    // Rewrite the last frame state and landmark state initialised during last frame creation to account
    // for a better estimate of the current state using the last landmark detection.
    // Otherwise, default behaviour is to take the last KF as an initialization of the new KF which can lead
    // to the solver finding local minima

    // When called for the first time, no feature list initialized in ori/last(?)
    if (n_reset_ < 1){
        n_reset_ += 1;
        return;
    }

    // A TESTER
    // (getOrigin() != nullptr)

    // Retrieve camera extrinsics
    Eigen::Quaternions last_q_cam(getSensor()->getO()->getState().data());
    Eigen::Affine3ds last_M_cam = Eigen::Translation3ds(getSensor()->getP()->getState()) * last_q_cam;

    // get features list of KF linked to origin capture and last capture
    FeatureBasePtrList ori_feature_list = getOrigin()->getFeatureList();
    FeatureBasePtrList last_feature_list = getLast()->getFeatureList();

    // std::cout << "last_feature_list.size(): " << last_feature_list.size() << std::endl;
    // std::cout << "ori_feature_list.size(): " << ori_feature_list.size() << std::endl;
    if (last_feature_list.size() == 0 || ori_feature_list.size() == 0){
        return;
    }
    
    // Among landmarks detected in origin and last, find the one that has the smallest error ratio (best confidence)
    Scalar lowest_ration = 1;  // rep_error1/rep_error2 cannot be higher than 1
    FeatureApriltagPtr best_feature;
    bool useable_feature = false;
    for (auto it_last = last_feature_list.begin(); it_last != last_feature_list.end(); it_last++){
        FeatureApriltagPtr last_feat_ptr = std::static_pointer_cast<FeatureApriltag>(*it_last);
        for (auto it_ori = ori_feature_list.begin(); it_ori != ori_feature_list.end(); it_ori++){
            FeatureApriltagPtr ori_feat_ptr =  std::static_pointer_cast<FeatureApriltag>(*it_ori);
            if (ori_feat_ptr->getTagId() == last_feat_ptr->getTagId()){
                Scalar ratio = ori_feat_ptr->getRepError1() / ori_feat_ptr->getRepError2();
                //if (ratio < lowest_ration){
                if (last_feat_ptr->getUserotation() && (ratio < lowest_ration)){
                    useable_feature = true;
                    lowest_ration = ratio;
                    best_feature = last_feat_ptr;
                    // std::cout << "Best feature id: " << best_feature->getTagId() << std::endl;
                }
            }
        }
    }
    // If there is no common feature between the two images, the continuity is broken and 
    // nothing can be estimated. In the case of aprilslam alone, this result in a broken factor map
    if (!useable_feature){
        return;
    }
    
    // std::cout << "Best feature id after: " << best_feature->getTagId() << std::endl;
    // Retrieve cam to landmark transform
    Eigen::Vector7s cam_pose_lmk = best_feature->getMeasurement();
    Eigen::Quaternions cam_q_lmk(cam_pose_lmk.segment<4>(3).data());
    Eigen::Affine3ds cam_M_lmk = Eigen::Translation3ds(cam_pose_lmk.head(3)) * cam_q_lmk;
    
    // Get corresponding landmarks in origin/last landmark list
    Eigen::Affine3ds w_M_lmk;
    LandmarkBasePtrList lmk_list = getProblem()->getMap()->getLandmarkList();
    // Iterate in reverse order because landmark detected in last are at the end of the list (while still landmarks to discovers)
    for (std::list<LandmarkBasePtr>::reverse_iterator it_lmk = lmk_list.rbegin(); it_lmk != lmk_list.rend(); ++it_lmk){
        LandmarkApriltagPtr lmk_ptr = std::dynamic_pointer_cast<LandmarkApriltag>(*it_lmk);
        // the map might contain other types of landmarks so check if the cast is valid
        if (lmk_ptr == nullptr){
            continue;
        }
    
        if (lmk_ptr->getTagId() == best_feature->getTagId()){
            Eigen::Vector3s w_t_lmk = lmk_ptr->getP()->getState();
            Eigen::Quaternions w_q_lmk(lmk_ptr->getO()->getState().data());
            w_M_lmk = Eigen::Translation<Scalar,3>(w_t_lmk) * w_q_lmk;
        }
    }

    // Compute last frame estimate
    Eigen::Affine3ds w_M_last = w_M_lmk * (last_M_cam * cam_M_lmk).inverse();

    // Use the w_M_last to overide last key frame state estimation and keyframe estimation
    Eigen::Vector3s pos_last  = w_M_last.translation();
    Eigen::Quaternions quat_last(w_M_last.linear());
    getLast()->getFrame()->getP()->setState(pos_last);
    getLast()->getFrame()->getO()->setState(quat_last.coeffs());

    // if (!best_feature->getUserotation()){
    //     return;
    // }
    ///////////////////
    // Reestimate position of landmark new in Last
    ///////////////////
    for (auto it_feat = new_features_last_.begin(); it_feat != new_features_last_.end(); it_feat++){
        FeatureApriltagPtr new_feature_last = std::static_pointer_cast<FeatureApriltag>(*it_feat);
       
        Eigen::Vector7s cam_pose_lmk = new_feature_last->getMeasurement();
        Eigen::Quaternions cam_q_lmk(cam_pose_lmk.segment<4>(3).data());
        Eigen::Affine3ds cam_M_lmk_new = Eigen::Translation3ds(cam_pose_lmk.head(3)) * cam_q_lmk;
        Eigen::Affine3ds w_M_lmk =  w_M_last * last_M_cam * cam_M_lmk_new;

        for (auto it_lmk = new_landmarks_.begin(); it_lmk != new_landmarks_.end(); ++it_lmk){
            LandmarkApriltagPtr lmk_ptr = std::dynamic_pointer_cast<LandmarkApriltag>(*it_lmk);
            if (lmk_ptr == nullptr){
                continue;
            }
            if (lmk_ptr->getTagId() == new_feature_last->getTagId()){
                Eigen::Vector3s pos_lmk_last  = w_M_lmk.translation();
                Eigen::Quaternions quat_lmk_last(w_M_lmk.linear());
                lmk_ptr->getP()->setState(pos_lmk_last);
                lmk_ptr->getO()->setState(quat_lmk_last.coeffs());
                break;
            }
        }
    }
}

std::string ProcessorTrackerLandmarkApriltag::getTagFamily() const
{
    return tag_family_.name;
}

} // namespace wolf

// Register in the SensorFactory
#include "core/processor/processor_factory.h"

namespace wolf
{
WOLF_REGISTER_PROCESSOR("TRACKER LANDMARK APRILTAG", ProcessorTrackerLandmarkApriltag)
}

