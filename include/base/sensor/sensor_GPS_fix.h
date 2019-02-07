
#ifndef SENSOR_GPS_FIX_H_
#define SENSOR_GPS_FIX_H_

//wolf includes
#include "sensor_base.h"

// std includes




namespace wolf {

WOLF_STRUCT_PTR_TYPEDEFS(IntrinsicsGPSFix);
struct IntrinsicsGPSFix : public IntrinsicsBase
{
        Eigen::Vector3s noise_std;
        // Empty -- it acts only as a typedef for IntrinsicsBase, but allows future extension with parameters
        virtual ~IntrinsicsGPSFix() = default;
};

WOLF_PTR_TYPEDEFS(SensorGPSFix);

class SensorGPSFix : public SensorBase
{
    public:
        /** \brief Constructor with arguments
         * 
         * Constructor with arguments
         * \param _p_ptr StateBlock pointer to the sensor position
         * \param _o_ptr StateBlock pointer to the sensor orientation
         * \param _noise noise standard deviation
         * 
         **/
		SensorGPSFix(StateBlockPtr _p_ptr, StateBlockPtr _o_ptr, const double& _noise);
        SensorGPSFix(const Eigen::VectorXs & _extrinsics, const IntrinsicsGPSFix& _intrinsics);
        SensorGPSFix(const Eigen::VectorXs & _extrinsics, IntrinsicsGPSFixPtr _intrinsics_ptr);

        virtual ~SensorGPSFix();

    public:
        static SensorBasePtr create(const std::string& _unique_name, const Eigen::VectorXs& _extrinsics_pq, const IntrinsicsBasePtr _intrinsics);

};

} // namespace wolf

#endif
