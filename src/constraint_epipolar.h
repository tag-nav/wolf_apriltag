#ifndef CONSTRAINT_EPIPOLAR_H
#define CONSTRAINT_EPIPOLAR_H

#include "constraint_base.h"

namespace wolf {

WOLF_PTR_TYPEDEFS(ConstraintEpipolar);
    

class ConstraintEpipolar : public ConstraintBase
{
    public:
        ConstraintEpipolar(const ProcessorBasePtr& _processor_ptr, const FeatureBasePtr& _feature_ptr,
                           const FeatureBasePtr& _feature_other_ptr, bool _apply_loss_function = false,
                           ConstraintStatus _status = CTR_ACTIVE);

        virtual ~ConstraintEpipolar() = default;

        /** \brief Returns the jacobians computation method
         **/
        virtual JacobianMethod getJacobianMethod() const override {return JAC_ANALYTIC;}

        /** \brief Returns a vector of scalar pointers to the first element of all state blocks involved in the constraint
         **/
        virtual const std::vector<Scalar*> getStateScalarPtrVector() override {return std::vector<Scalar*>(0);}

        /** \brief Returns a vector of pointers to the states in which this constraint depends
         **/
        virtual const std::vector<StateBlockPtr> getStateBlockPtrVector() const override {return std::vector<StateBlockPtr>(0);}

        /** \brief Returns the constraint residual size
         **/
        virtual unsigned int getSize() const override {return 0;}

    public:
        static wolf::ConstraintBasePtr create(const ProcessorBasePtr& _processor_ptr,
                                              const FeatureBasePtr& _feature_ptr,
                                              const NodeBasePtr& _correspondant_ptr);

};

inline ConstraintEpipolar::ConstraintEpipolar(const ProcessorBasePtr& _processor_ptr, const FeatureBasePtr& _feature_ptr,
                                              const FeatureBasePtr& _feature_other_ptr, bool _apply_loss_function, ConstraintStatus _status) :
        ConstraintBase(CTR_EPIPOLAR, _processor_ptr, nullptr, _feature_other_ptr, nullptr, _apply_loss_function, _status)
{
    setType("EPIPOLAR");
}

inline wolf::ConstraintBasePtr ConstraintEpipolar::create(const ProcessorBasePtr& _processor_ptr,
                                                          const FeatureBasePtr& _feature_ptr,
                                                          const NodeBasePtr& _correspondant_ptr)
{
    return std::make_shared<ConstraintEpipolar>(_processor_ptr, _feature_ptr, std::static_pointer_cast<FeatureBase>(_correspondant_ptr));
}

} // namespace wolf

#endif // CONSTRAINT_EPIPOLAR_H
