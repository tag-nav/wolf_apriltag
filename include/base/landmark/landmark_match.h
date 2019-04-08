#ifndef LANDMARK_MATCH_H_
#define LANDMARK_MATCH_H_

// Wolf includes
#include "base/wolf.h"

//wolf nampseace
namespace wolf {
    
// Map of Feature - Landmark matches
WOLF_STRUCT_PTR_TYPEDEFS(LandmarkMatch);
typedef std::map<FeatureBasePtr, LandmarkMatchPtr> LandmarkMatchMap;

/** \brief Match between a feature and a landmark
 *
 * Match between a feature and a landmark
 *
 **/
struct LandmarkMatch
{
    LandmarkBasePtr landmark_ptr_;  ///< Pointer to the matched landmark
    Scalar normalized_score_;       ///< Similarity measure: 0: no match -- 1: perfect match
    
    LandmarkMatch() :
            landmark_ptr_(nullptr), normalized_score_(0)
    {

    }
    LandmarkMatch(LandmarkBasePtr _landmark_ptr, Scalar _normalized_score) :
            landmark_ptr_(_landmark_ptr), normalized_score_(_normalized_score)
    {

    }
};

}//end namespace

#endif
