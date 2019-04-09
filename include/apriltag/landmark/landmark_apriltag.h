
#ifndef LANDMARK_APRILTAG_H_
#define LANDMARK_APRILTAG_H_

//Wolf includes
#include "base/landmark/landmark_base.h"

#include "base/state_quaternion.h"

// Std includes


namespace wolf {

WOLF_PTR_TYPEDEFS(LandmarkApriltag);
    
//class LandmarkApriltag
class LandmarkApriltag : public LandmarkBase
{
    public:
        /** \brief Constructor with type, time stamp and the position state pointer
         *
         * Constructor with type, and state pointer
         * \param _p_ptr StateBlock shared pointer to the position
         * \param _o_ptr StateBlock shared pointer to the orientation
         * \param _tagid descriptor of the landmark: id of the tag
         * \param _tag_width : total width of the tag (in metres)
         *
         **/
		LandmarkApriltag(Eigen::Vector7s& pose, const int& _tagid, const Scalar& _tag_width);

        virtual ~LandmarkApriltag();
        
        /** \brief Returns tag id
         * 
         * Returns id of the tag
         * 
         **/
        int getTagId() const;
        
        /** \brief Returns tag width
         * 
         * Returns width of the tag
         * 
         **/
        Scalar getTagWidth() const;

        /** Factory method to create new landmarks from YAML nodes
         */
        static LandmarkBasePtr create(const YAML::Node& _lmk_node);

        YAML::Node saveToYaml() const;


    private:
        const int tag_id_;
        const Scalar tag_width_;         
        
};

} // namespace wolf

#endif
