#ifndef NODE_BASE_H_
#define NODE_BASE_H_

// Wolf includes
#include "wolf.h"


namespace wolf {

/** \brief Base class for Nodes
 *
 * Base class for all Nodes in the Wolf tree. Each node has
 *
 *  - A unique ID. The class implements the ID factory.
 *
 *  - A unique class name, strictly within this range of possibilities:
 *    - "UNDEFINED" -- used for NodeTerminus
 *    - "PROBLEM"
 *    - "HARDWARE"
 *    - "SENSOR"
 *    - "PROCESSOR"
 *    - "TRAJECTORY"
 *    - "FRAME"
 *    - "CAPTURE"
 *    - "FEATURE"
 *    - "CONSTRAINT"
 *    - "MAP"
 *    - "LANDMARK"
 *
 *  - A unique type, which is a subclass of the above. The list here cannot be exhaustive, but a few examples follow:
 *    - "Camera"
 *    - "LIDAR 2D"
 *    - "Point 3D"
 *    - "Lidar 2D processor"
 *
 *    please refer to each base class derived from NodeLinked for better examples of their types.
 *
 *  - A name, defined in each application, which is specific of each object. Again, just some examples:
 *    - "Front-left camera"
 *    - "Main wheel odometer"
 *    - "IMU delta pre-integrator processor"
 *
 *    These names are only required for objects being defined by the user at configuration time, such as:
 *    - Sensors
 *    - Processors
 *
 *    Additionally, but not necessarily, one could also give names to:
 *    - Pre-defined maps or landmarks (provided to the problem)
 *    - Each of the generated captures --> these names could, for example, be a copy of the sensor name.
 *
 *    Finally, all the other node names in the Wolf Tree are not required, and in fact they are not used for anything.
 *
 **/
class NodeBase
{
    private:
        static unsigned int node_id_count_; ///< Object counter (acts as simple ID factory)

    protected:
        ProblemWPtr problem_ptr_;
        bool is_removing_;

        unsigned int node_id_;   ///< Node id. It is unique over the whole Wolf Tree
        std::string node_class_; ///< Text label identifying the class of node ("SENSOR", "FEATURE", etc)
        std::string node_type_;  ///< Text label identifying the type or subclass of node ("Pin Hole", "Point 2D", etc)
        std::string node_name_;  ///< Text label identifying each specific object ("left camera", "LIDAR 1", "PointGrey", "Andrew", etc)

    public: 

        NodeBase(const std::string& _class, const std::string& _type = "Undefined", const std::string& _name = "");
        virtual ~NodeBase();

        unsigned int nodeId() const;
        std::string getClass() const;
        std::string getType() const {return node_type_;}
        std::string getName() const;

        void setType(const std::string& _name){node_type_ = _name;};
        void setName(const std::string& _name);

        virtual ProblemPtr getProblem(){return problem_ptr_;}
        void setProblem(ProblemPtr _prob_ptr) {problem_ptr_ = _prob_ptr;}

};

} // namespace wolf

#include <iostream>

namespace wolf{

inline NodeBase::NodeBase(const std::string& _class, const std::string& _type, const std::string& _name) :
        problem_ptr_(nullptr), is_removing_(false), node_id_(++node_id_count_), node_class_(_class), node_type_(_type), node_name_(_name)
{
    //
}

inline NodeBase::~NodeBase()
{
    //
}

inline unsigned int NodeBase::nodeId() const
{
    return node_id_;
}

inline std::string NodeBase::getClass() const
{
    return node_class_;
}

inline std::string NodeBase::getName() const
{
    return node_name_;
}

inline void NodeBase::setName(const std::string& _name)
{
    node_name_ = _name;
}

} // namespace wolf

#endif /* NODE_BASE_H_ */
