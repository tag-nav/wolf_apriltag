#ifndef FEATURE_BASE_H_
#define FEATURE_BASE_H_

// Forward declarations for node templates
namespace wolf{
class CaptureBase;
class FactorBase;
}

//Wolf includes
#include "base/wolf.h"
#include "base/node_base.h"

//std includes

namespace wolf {

//class FeatureBase
class FeatureBase : public NodeBase, public std::enable_shared_from_this<FeatureBase>
{
    private:
        CaptureBaseWPtr capture_ptr_;
        FactorBasePtrList factor_list_;
        FactorBasePtrList constrained_by_list_;

        static unsigned int feature_id_count_;

    protected:
        unsigned int feature_id_;
        unsigned int track_id_; // ID of the feature track
        unsigned int landmark_id_; // ID of the landmark
        Eigen::VectorXs measurement_;                   ///<  the measurement vector
        Eigen::MatrixXs measurement_covariance_;        ///<  the measurement covariance matrix
        Eigen::MatrixXs measurement_sqrt_information_upper_;  ///<  the squared root information matrix
        Eigen::VectorXs expectation_;                   ///<  expectation
        
    public:

        typedef enum
        {
            UNCERTAINTY_IS_COVARIANCE,
            UNCERTAINTY_IS_INFO,
            UNCERTAINTY_IS_STDDEV
        } UncertaintyType;

        /** \brief Constructor from capture pointer and measure
         * \param _tp type of feature -- see wolf.h
         * \param _measurement the measurement
         * \param _meas_covariance the noise of the measurement
         */
        FeatureBase(const std::string& _type, const Eigen::VectorXs& _measurement, const Eigen::MatrixXs& _meas_uncertainty, UncertaintyType _uncertainty_type = UNCERTAINTY_IS_COVARIANCE);

        virtual ~FeatureBase();
        virtual void remove();

        virtual void setProblem(ProblemPtr _problem) final;

        // properties
        unsigned int id();
        unsigned int trackId(){return track_id_;}
        void setTrackId(unsigned int _tr_id){track_id_ = _tr_id;}
        unsigned int landmarkId(){return landmark_id_;}
        void setLandmarkId(unsigned int _lmk_id){landmark_id_ = _lmk_id;}

        // values
        /* \brief Returns _ii component of measurement vector
         *
         * WARNING: To be fast, it does not check that index _ii is smaller than dimension.
         */
        Scalar getMeasurement(unsigned int _ii) const;
        const Eigen::VectorXs& getMeasurement() const;
        void setMeasurement(const Eigen::VectorXs& _meas);
        void setMeasurementCovariance(const Eigen::MatrixXs & _meas_cov);
        void setMeasurementInformation(const Eigen::MatrixXs & _meas_info);
        const Eigen::MatrixXs& getMeasurementCovariance() const;
        Eigen::MatrixXs getMeasurementInformation() const;
        const Eigen::MatrixXs& getMeasurementSquareRootInformationUpper() const;

        const Eigen::VectorXs& getExpectation() const;
        void setExpectation(const Eigen::VectorXs& expectation);

        // wolf tree access
        FrameBasePtr getFrame() const;

        CaptureBasePtr getCapture() const;
        void setCapture(CaptureBasePtr _cap_ptr){capture_ptr_ = _cap_ptr;}

        FactorBasePtr addFactor(FactorBasePtr _co_ptr);
        FactorBasePtrList& getFactorList();

        virtual FactorBasePtr addConstrainedBy(FactorBasePtr _fac_ptr);
        unsigned int getHits() const;
        FactorBasePtrList& getConstrainedByList();

        // all factors
        void getFactorList(FactorBasePtrList & _fac_list);

    private:
        Eigen::MatrixXs computeSqrtUpper(const Eigen::MatrixXs& _M) const;
};

}

// IMPLEMENTATION

#include "base/factor/factor_base.h"

namespace wolf{

inline unsigned int FeatureBase::getHits() const
{
    return constrained_by_list_.size();
}

inline FactorBasePtrList& FeatureBase::getConstrainedByList()
{
    return constrained_by_list_;
}

inline unsigned int FeatureBase::id()
{
    return feature_id_;
}

inline CaptureBasePtr FeatureBase::getCapture() const
{
    return capture_ptr_.lock();
}

inline Scalar FeatureBase::getMeasurement(unsigned int _ii) const
{
    return measurement_(_ii);
}

inline const Eigen::VectorXs& FeatureBase::getMeasurement() const
{
    return measurement_;
}

inline const Eigen::MatrixXs& FeatureBase::getMeasurementCovariance() const
{
    return measurement_covariance_;
}

inline Eigen::MatrixXs FeatureBase::getMeasurementInformation() const
{
    return measurement_sqrt_information_upper_.transpose() * measurement_sqrt_information_upper_;
}

inline const Eigen::MatrixXs& FeatureBase::getMeasurementSquareRootInformationUpper() const
{
    return measurement_sqrt_information_upper_;
}

inline void FeatureBase::setMeasurement(const Eigen::VectorXs& _meas)
{
    measurement_ = _meas;
}

inline const Eigen::VectorXs& FeatureBase::getExpectation() const
{
    return expectation_;
}

inline void FeatureBase::setExpectation(const Eigen::VectorXs& expectation)
{
    expectation_ = expectation;
}

} // namespace wolf

#endif
