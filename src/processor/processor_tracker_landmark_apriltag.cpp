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

#include "apriltag/processor/processor_tracker_landmark_apriltag.h"

#include "core/math/SE3.h"

#include <chrono>
#include <ctime>
namespace wolf {


// Constructor
ProcessorTrackerLandmarkApriltag::ProcessorTrackerLandmarkApriltag( ParamsProcessorTrackerLandmarkApriltagPtr _params_tracker_landmark_apriltag) :
        ProcessorTrackerLandmark("ProcessorTrackerLandmarkApriltag", "PO", _params_tracker_landmark_apriltag),
        MotionProvider("PO", _params_tracker_landmark_apriltag),
        tag_widths_(_params_tracker_landmark_apriltag->tag_widths_),
        tag_width_default_(_params_tracker_landmark_apriltag->tag_width_default_),
        std_pix_(_params_tracker_landmark_apriltag->std_pix_),
        use_proj_factor_(_params_tracker_landmark_apriltag->use_proj_factor_),
        ippe_min_ratio_(_params_tracker_landmark_apriltag->ippe_min_ratio_),
        ippe_max_rep_error_(_params_tracker_landmark_apriltag->ippe_max_rep_error_),
        cv_K_(3,3),
        n_reset_(0),
        min_time_span_(_params_tracker_landmark_apriltag->min_time_span_),
        max_time_span_(_params_tracker_landmark_apriltag->max_time_span_),
        min_features_for_keyframe_(_params_tracker_landmark_apriltag->min_features_for_keyframe),
        nb_vote_for_every_first_(_params_tracker_landmark_apriltag->nb_vote_for_every_first_),
        nb_vote_(0)
{
    // configure apriltag detector
    std::string famname(_params_tracker_landmark_apriltag->tag_family_);
    if (famname == "tag16h5")
        tag_family_ = tag16h5_create();
    else if (famname == "tag25h9")
        tag_family_ = tag25h9_create();
    else if (famname == "tag36h11")
        tag_family_ = tag36h11_create();
    else if (famname == "tagCircle49h12")
        tag_family_ = tagCircle49h12_create();
    else if (famname == "tagCustom48h12")
        tag_family_ = tagCustom48h12_create();
    else if (famname == "tagStandard41h12")
        tag_family_ = tagStandard41h12_create();
    else if (famname == "tagStandard52h13")
        tag_family_ = tagStandard52h13_create();
    else {
        WOLF_ERROR("Unrecognized tag family name: ", famname, ". Use e.g. \"tag36h11\".");
        exit(-1);
    }

    detector_ = apriltag_detector_create();
    apriltag_detector_add_family(detector_, tag_family_);

    detector_->quad_decimate     = _params_tracker_landmark_apriltag->quad_decimate_;
    detector_->quad_sigma        = _params_tracker_landmark_apriltag->quad_sigma_;
    detector_->nthreads          = _params_tracker_landmark_apriltag->nthreads_;
    detector_->debug             = _params_tracker_landmark_apriltag->debug_;
    detector_->refine_edges      = _params_tracker_landmark_apriltag->refine_edges_;
}

// Destructor
ProcessorTrackerLandmarkApriltag::~ProcessorTrackerLandmarkApriltag()
{
    // destroy family
    std::string famname(tag_family_->name);
    if (famname == "tag16h5")
        tag16h5_destroy(tag_family_);
    else if (famname == "tag25h9")
        tag25h9_destroy(tag_family_);
    else if (famname == "tag36h11")
        tag36h11_destroy(tag_family_);
    else if (famname == "tagCircle49h12")
        tagCircle49h12_destroy(tag_family_);
    else if (famname == "tagCustom48h12")
        tagCustom48h12_destroy(tag_family_);
    else if (famname == "tagStandard41h12")
        tagStandard41h12_destroy(tag_family_);
    else if (famname == "tagStandard52h13")
        tagStandard52h13_destroy(tag_family_);
    else {
        WOLF_ERROR("This is impossible");
    }

    // destroy detector
    apriltag_detector_destroy(detector_);
}

void ProcessorTrackerLandmarkApriltag::preProcess()
{
    auto t1 = std::chrono::system_clock::now();

    //clear wolf detections so that new ones will be stored inside
    detections_incoming_.clear();

    auto incoming_ptr = std::dynamic_pointer_cast<CaptureImage>(incoming_ptr_);
    assert(incoming_ptr != nullptr && "Capture type mismatch. ProcessorTrackerLandmarkApriltag can only process captures of type CaptureImage");

    // If the image has 3 channels, grayscale it
    if (incoming_ptr->getImage().channels() == 3) {
        cv::cvtColor(incoming_ptr->getImage(), grayscale_image_, cv::COLOR_BGR2GRAY);
    }
    else {
        grayscale_image_ = incoming_ptr->getImage();
    }

    
    //detect tags in incoming image
    // Make an image_u8_t header for the Mat data
    image_u8_t im = {   .width  = grayscale_image_.cols,
                        .height = grayscale_image_.rows,
                        .stride = grayscale_image_.cols,
                        .buf    = grayscale_image_.data
                    };

    // run Apriltag detector
    auto t2 = std::chrono::system_clock::now();
    zarray_t *detections = apriltag_detector_detect(detector_, &im);
    auto __attribute__((unused)) dt_detection = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - t2).count();
    // WOLF_INFO( "ProcessorTrackerLandmarkApriltag dt_detection (ms): " , dt_detection);

    // loop all detections
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        int    tag_id     = det->id;
        double tag_width  = getTagWidth(tag_id);   // tag width in meters

        //////////////////
        // IPPE (Infinitesimal Plane-based Pose Estimation)
        // TODO: Quite bad for reprojection factor: we only need to do this question once
        // when the landmark is newly discovered. Would require to change the whole logic 
        // or to check if this landmark is already in the map HERE (which is usually not the logic of preProcess) 
        //////////////////
        Eigen::Isometry3d M_ippe1, M_ippe2;
        double rep_error1, rep_error2;
        ippePoseEstimation(det, cv_K_, tag_width, M_ippe1, rep_error1, M_ippe2, rep_error2);
        // If not so sure about whether we have the right solution or not, do not create a feature
        bool use_rotation = ((rep_error2 / rep_error1 > ippe_min_ratio_) && rep_error1 < ippe_max_rep_error_);
        //////////////////

        // set the measured pose vector
        Eigen::Vector3d p_c_t ( M_ippe1.translation() ); // translation vector in apriltag meters
        Eigen::Matrix3d R_c_t = M_ippe1.linear(); 
        Eigen::Vector7d pose; pose << p_c_t, R2q(R_c_t).coeffs();

        // Order of the 2d corners (anti clockwise, looking at the tag).
        // Looking at the tag, the reference frame is
        // X = Right, Y = Down, Z = Inside the plane        
        Vector8d corners_vec; corners_vec << det->p[0][0], det->p[0][1],  // bottom left
                                             det->p[1][0], det->p[1][1],  // bottom right
                                             det->p[2][0], det->p[2][1],  // top right
                                             det->p[3][0], det->p[3][1];  // top left

        if (use_proj_factor_)
        {
            // add to detected features list
            detections_incoming_.push_back(std::make_shared<FeatureApriltagProj>(corners_vec,
                                                                                 std_pix_*std_pix_*Eigen::Matrix8d::Identity(),
                                                                                 tag_id,
                                                                                 tag_width,
                                                                                 pose,
                                                                                 use_rotation));
        }
        else
        {
            // compute the covariance
            // Eigen::Matrix6d cov = getVarVec().asDiagonal() ;  // fixed dummy covariance
            Eigen::Matrix6d info = computeInformation(p_c_t, R_c_t, K_, tag_width, std_pix_);  // Lie jacobians covariance

            if (!use_rotation){
                // Put a very high covariance on angles measurements (low info matrix)
                info.bottomLeftCorner(3,3) = Eigen::Matrix3d::Zero();
                info.topRightCorner(3,3) = Eigen::Matrix3d::Zero();
                info.bottomRightCorner(3,3) = M_1_PI*M_1_PI * Eigen::Matrix3d::Identity();  // 180 degrees standar deviation
            }

            // add to detected features list
            detections_incoming_.push_back(std::make_shared<FeatureApriltagPose>(pose,
                                                                                 info,
                                                                                 tag_id,
                                                                                 tag_width,
                                                                                 corners_vec,
                                                                                 use_rotation,
                                                                                 FeatureBase::UncertaintyType::UNCERTAINTY_IS_INFO));
        }
    }

    apriltag_detections_destroy(detections);

    auto __attribute__((unused)) dt_preprocess = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - t1).count();
    // WOLF_INFO( "ProcessorTrackerLandmarkApriltag dt_preprocess (ms): " , dt_preprocess );
}

void ProcessorTrackerLandmarkApriltag::ippePoseEstimation(apriltag_detection_t *_det,
                                                          cv::Mat_<double> _K,
                                                          double _tag_width,
                                                          Eigen::Isometry3d &_M1,
                                                          double &_rep_error1,
                                                          Eigen::Isometry3d &_M2,
                                                          double &_rep_error2){

    // get corners from det
    // Order of the 2d corners (anti clockwise, looking at the tag).
    // Looking at the tag, the reference frame is
    // X = Right, Y = Down, Z = Inside the plane
    // bottom left, bottom right, top right ,top left
    std::vector<cv::Point2d> corners_pix(4);
    for (int i = 0; i < 4; i++)
    {
        corners_pix[i].x = _det->p[i][0];
        corners_pix[i].y = _det->p[i][1];
    }


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
    Eigen::Matrix3d R_eigen1; cv2eigen(rmat1, R_eigen1);
    Eigen::Vector3d t_eigen1; cv2eigen(tvec1, t_eigen1);
    _M1.matrix().block(0,0,3,3) = R_eigen1;
    _M1.matrix().block(0,3,3,1) = t_eigen1;

    cv::Matx33d rmat2;
    cv::Rodrigues(rvec2, rmat2);
    Eigen::Matrix3d R_eigen2; cv2eigen(rmat2, R_eigen2);
    Eigen::Vector3d t_eigen2; cv2eigen(tvec2, t_eigen2);
    _M2.matrix().block(0,0,3,3) = R_eigen2;
    _M2.matrix().block(0,3,3,1) = t_eigen2;
}


void ProcessorTrackerLandmarkApriltag::postProcess()
{
    nb_vote_++;
}

FactorBasePtr ProcessorTrackerLandmarkApriltag::emplaceFactor(FeatureBasePtr _feature_ptr,
                                                              LandmarkBasePtr _landmark_ptr)
{
    auto feat_pose = std::dynamic_pointer_cast<FeatureApriltagPose>(_feature_ptr);
    if (feat_pose)
    {
        return FactorBase::emplace<FactorRelativePose3dWithExtrinsics>(_feature_ptr,
                                                _feature_ptr,
                                                _landmark_ptr,
                                                shared_from_this(),
                                                params_->apply_loss_function);
    }
    else
    {
        auto feat_proj = std::dynamic_pointer_cast<FeatureApriltagProj>(_feature_ptr);
        return FactorBase::emplace<FactorApriltagProj>(feat_proj,
                                        feat_proj,
                                        getSensor(),
                                        feat_proj->getFrame(),
                                        _landmark_ptr,
                                        shared_from_this(),
                                        params_->apply_loss_function);
    }

}

LandmarkBasePtr ProcessorTrackerLandmarkApriltag::emplaceLandmark(FeatureBasePtr _feature_ptr)
{
    Quaterniond quat_tmp;

    // world to rob
    Vector3d pos            = getLast()->getFrame()->getP()->getState();
    quat_tmp.coeffs()       = getLast()->getFrame()->getO()->getState();
    Eigen::Isometry3d w_M_r = Eigen::Translation<double,3>(pos.head(3)) * quat_tmp;

    // rob to camera
    pos                     = getSensor()->getP()->getState();
    quat_tmp.coeffs()       = getSensor()->getO()->getState();
    Eigen::Isometry3d r_M_c = Eigen::Translation<double,3>(pos.head(3)) * quat_tmp;

    // camera to lmk (tag)
    if (use_proj_factor_)
    {
        auto feat_proj = std::dynamic_pointer_cast<FeatureApriltagProj>(_feature_ptr);
        pos                     = feat_proj->getPosePnp().head(3);
        quat_tmp.coeffs()       = feat_proj->getPosePnp().tail(4);
    }
    else
    {
        pos                     = _feature_ptr->getMeasurement().head(3);
        quat_tmp.coeffs()       = _feature_ptr->getMeasurement().tail(4);
    }
    Eigen::Isometry3d c_M_t = Eigen::Translation<double,3>(pos) * quat_tmp;

    // world to lmk (tag)
    Eigen::Isometry3d w_M_t = w_M_r * r_M_c * c_M_t;

    // make 7-vector for lmk (tag) pose
    pos  = w_M_t.translation();
    Eigen::Matrix3d wRt = w_M_t.linear();
    quat_tmp.coeffs() = R2q(wRt).coeffs().transpose();
    Vector7d w_pose_t;
    w_pose_t << pos, quat_tmp.coeffs();

    FeatureApriltagPosePtr feat_april = std::static_pointer_cast<FeatureApriltagPose>(_feature_ptr);
    int tag_id = feat_april->getTagId();

    return LandmarkBase::emplace<LandmarkApriltag>(getProblem()->getMap(), w_pose_t, tag_id, getTagWidth(tag_id));
}

unsigned int ProcessorTrackerLandmarkApriltag::detectNewFeatures(const int& _max_new_features,
                                                                 const CaptureBasePtr& _capture,
                                                                 FeatureBasePtrList& _features_out)
{
    // _max_new_features: max number of new features to be detected
    // _capture: capture in which are stored the features
    // _features_out: filled with new features in last. Empty at the beginning of the call

    // detectNewFeatures is called by processNew() in ProcessorTrackerLandmark
    // processNew is only called when a KF is created by the processor or another 
    // (except for the SECOND_TIME_WITHOUT_KEYFRAME case)
    // -> use detections in last frame/capture to detect features that cannot be associated to any landmark, create one accordingly

    // Loop over the detections in last and check
    for (auto feat: detections_last_)
    {
        // max_new_features reached
        if (_max_new_features != -1 && _features_out.size() == _max_new_features)
            break;

        auto feat_april = std::static_pointer_cast<FeatureApriltag>(feat);
        bool match_found = false;
        // Loop over the landmarks to find if one is associated to the 
        for(auto lmk: getProblem()->getMap()->getLandmarkList()){
            auto lmk_april = std::dynamic_pointer_cast<LandmarkApriltag>(lmk);
            if (lmk_april and lmk_april->getTagId() == feat_april->getTagId())
            {
                match_found = true;
                break;
            }
        }

        // if the feature was not matched to any existing landmark, add it to the new features list
        if (!match_found)
        {
            // remove duplicate features (detections that correspond to the same landmark, even though this should never happen)
            for (FeatureBasePtrList::iterator it=_features_out.begin(); it != _features_out.end(); ++it)
            {
                if (std::static_pointer_cast<FeatureApriltag>(*it)->getTagId() == feat_april->getTagId())
                {
                    //we have a detection with the same id as the currently processed one. We remove the previous feature from the list for now
                    _features_out.erase(it);
                    //it should not be possible two detection with the same id before getting there so we can stop here.
                    break; 
                }
            }

            // If the feature is not in the map & not in the list of newly detected features yet then we add it.
            _features_out.push_back(feat);

            // link feature (they are created unlinked in preprocess())
            feat->link(_capture);

        } //otherwise we check the next feature
    }

    return _features_out.size();
}

bool ProcessorTrackerLandmarkApriltag::voteForKeyFrame() const
{   
    // if no detections in last capture, no case where it is useful to create a KF from last
    if (detections_last_.empty())
        return false;

    double dt_incoming_origin = getIncoming()->getTimeStamp().get() - getOrigin()->getTimeStamp().get();
    bool more_than_min_time_vote = dt_incoming_origin > min_time_span_; 
    bool too_long_since_last_KF = dt_incoming_origin > max_time_span_ + 1e-5;

    bool enough_features_in_last = detections_last_.size() >= min_features_for_keyframe_;
    bool enough_features_in_incoming = detections_incoming_.size() >= min_features_for_keyframe_;
    
    // not enough detection in incoming capture and a minimum time since last KF has past
    if (enough_features_in_last and !enough_features_in_incoming and more_than_min_time_vote)
        return true;

    // the elapsed time since last KF is too long 
    if (too_long_since_last_KF and enough_features_in_last){
        return true;
    }

    // Vote for every image processed at the beginning if possible
    if (nb_vote_ < nb_vote_for_every_first_){
        return true;
    }

    return false;
}

unsigned int ProcessorTrackerLandmarkApriltag::findLandmarks(const LandmarkBasePtrList& _landmarks_in,
                                                             const CaptureBasePtr& _capture,
                                                             FeatureBasePtrList& _features_out,
                                                             LandmarkMatchMap& _feature_landmark_correspondences)
{   
    // In current implementation of ProcessorTrackerLandmark, _landmarks_in is the whole 
    // Would be great to split in different maps
    for (auto feat : detections_incoming_)
    {
        int tag_id(std::static_pointer_cast<FeatureApriltag>(feat)->getTagId());

        // Linear search in all landmarks (Apriltag and others) -> BAD
        for (auto lmk : _landmarks_in)
        {
            auto lmk_april = std::dynamic_pointer_cast<LandmarkApriltag>(lmk);

            // correspondance check in 2 steps:
            // 1. the cast gives a non-nullptr -> right class type
            // 2. the tag id is the same (assumed unique tag id for all tags)
            if(lmk_april and lmk_april->getTagId() == tag_id)
            {
                _features_out.push_back(feat);
                double score(1.0);
                LandmarkMatchPtr matched_landmark = std::make_shared<LandmarkMatch>(lmk, score);
                _feature_landmark_correspondences.emplace ( feat, matched_landmark );
                feat->link(_capture); // since all features created in preProcess() are not linked yet
                break;
            }
        }
    }

    return _features_out.size();
}

double ProcessorTrackerLandmarkApriltag::getTagWidth(int _id) const
{
    if (tag_widths_.find(_id) != tag_widths_.end())
        return tag_widths_.at(_id);
    else
        return tag_width_default_;
}

Eigen::Matrix6d ProcessorTrackerLandmarkApriltag::computeInformation(Eigen::Vector3d const &t,
                                                                     Eigen::Matrix3d const &R,
                                                                     Eigen::Matrix3d const &K,
                                                                     double const &tag_width,
                                                                     double const &sig_q)
{
    // Same order as the 2d corners (anti clockwise, looking at the tag).
    // Looking at the tag, the reference frame is
    // X = Right, Y = Down, Z = Inside the plane
    double s = tag_width/2;
    Eigen::Vector3d p1; p1 << -s,  s, 0; // bottom left
    Eigen::Vector3d p2; p2 <<  s,  s, 0; // bottom right
    Eigen::Vector3d p3; p3 <<  s, -s, 0; // top right
    Eigen::Vector3d p4; p4 << -s, -s, 0; // top left
    std::vector<Eigen::Vector3d> pvec = {p1, p2, p3, p4};
    std::vector<Eigen::Vector2d> proj_pix_vec(4); //proj_pix_vec.resize(4);

    // Initialize jacobian matrices
    Eigen::Matrix<double, 8, 6> J_u_TR = Eigen::Matrix<double, 8, 6>::Zero();  // 2N x 6 jac
    Eigen::Vector3d h;
    Eigen::Matrix3d J_h_T, J_h_R;
    Eigen::Vector2d eu;  // 2d pixel coord, not needed
    Eigen::Matrix<double, 3, 6> J_h_TR;
    Eigen::Matrix<double, 2, 3> J_u_h;
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
    Eigen::Matrix<double, 8, 8> pixel_cov = pow(sig_q, 2) * Eigen::Matrix<double, 8, 8>::Identity();
    // 6 x 6 translation|rotation information matrix computed with covariance propagation formula (inverted)
    Eigen::Matrix6d transformation_info  = (J_u_TR.transpose() * pixel_cov.inverse() * J_u_TR);  // Wolf jac

    return transformation_info;

}

void ProcessorTrackerLandmarkApriltag::pinholeHomogeneous(Eigen::Matrix3d const & K,
                                                          Eigen::Vector3d const & t,
                                                          Eigen::Matrix3d const & R,
                                                          Eigen::Vector3d const & p,
                                                          Eigen::Vector3d &h,
                                                          Eigen::Matrix3d &J_h_T,
                                                          Eigen::Matrix3d &J_h_R)
{
    // Pinhole projection + jacobians
    h =  K * (t + R * p);
    J_h_T = K;
    Eigen::Matrix3d p_hat;
    p_hat << 0, -p(2), p(1),
             p(2), 0, -p(0),
            -p(1), p(0), 0;
    J_h_R = -K * R * p_hat;
}

FeatureBasePtrList ProcessorTrackerLandmarkApriltag::getIncomingDetections() const
{
    FeatureBasePtrList list_const;
    for (auto && obj : detections_incoming_)
        list_const.push_back(obj);
    return list_const;
}

FeatureBasePtrList ProcessorTrackerLandmarkApriltag::getIncomingDetections()
{
    return detections_incoming_;
}

FeatureBasePtrList ProcessorTrackerLandmarkApriltag::getLastDetections() const
{
    FeatureBasePtrList list_const;
    for (auto && obj : detections_last_)
        list_const.push_back(obj);
    return list_const;
}

FeatureBasePtrList ProcessorTrackerLandmarkApriltag::getLastDetections()
{
    return detections_last_;
}

void ProcessorTrackerLandmarkApriltag::configure(SensorBasePtr _sensor)
{
    SensorCameraPtr sen_cam_ptr = std::static_pointer_cast<SensorCamera>(_sensor);
    sen_cam_ptr->useRectifiedImages();

    // get camera intrinsic parameters
    Eigen::Vector4d k(_sensor->getIntrinsic()->getState()); //[cx cy fx fy]

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
    ProcessorTrackerLandmark::resetDerived();
    detections_last_ = std::move(detections_incoming_);
}

std::string ProcessorTrackerLandmarkApriltag::getTagFamily() const
{
    return tag_family_->name;
}


////////////////////////////////////////
// MotionProvider methods implementation
////////////////////////////////////////


TimeStamp ProcessorTrackerLandmarkApriltag::getTimeStamp() const 
{
    if ( last_ptr_ == nullptr )
        return TimeStamp::Invalid();
    else
        return last_ptr_->getTimeStamp();
}

VectorComposite ProcessorTrackerLandmarkApriltag::getState(const StateStructure& _structure) const
{
    // compute the state of the frame corresponding to last capture
    // matches_landmark_from_last_ contains matches between landmarks already present in the map when
    // processing last capture and detections in last capture.
    // From the map current estimate and apriltag last features, we can extract the last pose of the robot 

    Vector7d pose_c_r = SE3::inverse(getSensor()->getStateVector("PO"));

    // Compute average SE3 pose between using the hyperbolic average (mean of the se(3) twist 6D vectors)
    // Neglected:
    // - Uncertainty on the landmark estimate
    // - Uncertainty on the camera->landmark pose measurements
    int nb_use_rot = 0;
    // 1. hyperbolic avg on se(3)
    Vector6d nu_w_r_avg = Vector6d::Zero();
    // 2. hyperbolic metric avg on so(3) (http://lucaballan.altervista.org/pdfs/IK.pdf slide)
    Vector3d t_w_r_avg = Vector3d::Zero();
    Vector3d omg_w_r_avg = Vector3d::Zero();
    // 3. Frobenius/chordal metric avg followed by projection on SO(3) -> not really a diff
    // Matrix3d R_w_r_avg = Matrix3d::Zero();
    for (auto match: matches_landmark_from_last_){
        auto feat_a = std::static_pointer_cast<FeatureApriltag>(match.first);
        // use the pose only if the rotation is not ambiguous
        if (feat_a->getUseRotation()){
            auto lmk_a = std::static_pointer_cast<LandmarkApriltag> (match.second->landmark_ptr_);

            Vector7d pose_l_c = SE3::inverse(feat_a->getPosePnp());
            Vector7d pose_w_l = lmk_a->getStateVector();
            Vector7d pose_w_r = SE3::compose(SE3::compose(pose_w_l, pose_l_c), pose_c_r);
            // 1.
            nu_w_r_avg += SE3::log(pose_w_r);
            // 2.
            t_w_r_avg += pose_w_r.head<3>();
            omg_w_r_avg += q2v(Quaterniond(pose_w_r.tail<4>()));
            // 3.
            // R_w_r_avg += q2R(pose_w_r.tail<4>());
            nb_use_rot += 1;
        }
    }
    if (nb_use_rot == 0){
        return VectorComposite();
    }
    // // 1.
    // nu_w_r_avg /= nb_use_rot;
    // Vector7d pose_avg = SE3::exp(nu_w_r_avg);

    // 2. 
    t_w_r_avg /= nb_use_rot;
    omg_w_r_avg /= nb_use_rot;
    Vector7d pose_avg; pose_avg << t_w_r_avg, v2q(omg_w_r_avg).coeffs();

    // // 3.
    // t_w_r_avg /= nb_use_rot;
    // R_w_r_avg /= nb_use_rot;
    

    return VectorComposite(pose_avg, "PO", {3,4});

}

VectorComposite ProcessorTrackerLandmarkApriltag::getState(const TimeStamp& _ts, const StateStructure& _structure) const
{
    if (true) {
        return getState(_structure);
    }

    // empty state should be handled by the receiver
    return VectorComposite();
}




} // namespace wolf

// Register in the FactorySensor
#include "core/processor/factory_processor.h"

namespace wolf
{
WOLF_REGISTER_PROCESSOR(ProcessorTrackerLandmarkApriltag)
WOLF_REGISTER_PROCESSOR_AUTO(ProcessorTrackerLandmarkApriltag)
}