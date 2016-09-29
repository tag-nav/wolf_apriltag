//wolf
#include "node_terminus.h"
#include "node_linked.h"
#include "problem.h"

//namespaces
namespace wolf {
using namespace std;

//id count init
// unsigned int NodeBase::node_id_count_ = 0; 

//forward declarations
class TrajectoryN;
class FrameN;
class MeasurementN;
class Problem;

//class TrajectoryN
class TrajectoryN : public NodeLinked<Problem,FrameN>
{
    protected:
        unsigned int length_; //just something to play
        
    public:
        TrajectoryN(const unsigned int _len) :
            NodeLinked(MID, "TRAJECTORY"),
            length_(_len)
        {
            //
        };
        
        ~TrajectoryN()
        {
            
        };
        
//         virtual void printLabel(ostream & _ost = cout) const
//         {
//             _ost <<"TRAJECTORY";
//         }        
};

//class FrameN
class FrameN : public NodeLinked<TrajectoryN,MeasurementN>
{
    protected:
        double time_stamp_; //just something to play
        
    public:
        FrameN(double _ts) :
            NodeLinked(MID, "FRAME"),
            time_stamp_(_ts)
        {
            //
        };
        
        ~FrameN()
        {
            
        };
 
//         virtual void printLabel(ostream & _ost = cout) const
//         {
//             _ost <<"FRAME";
//         }

};

//class MeasurementN
class MeasurementN : public NodeLinked<FrameN,NodeTerminus>
{
    protected:
        unsigned int size_; //just something to play
        
    public:
        MeasurementN(const unsigned int _sz) :
            NodeLinked(BOTTOM, "MEASUREMENT"),
            size_(_sz)
        {
            //
        };
        
        ~MeasurementN()
        {
            
        };
        
//         virtual void printLabel(ostream & _ost = cout) const
//         {
//             _ost <<"MEASUREMENT";
//         }
        
};

} // namespace wolf

int main()
{
    using namespace wolf;

    cout << endl << "Node class test" << endl;
    cout << "========================================================" << endl;

    cout << endl << "TEST 1. Constructors" << endl;
    Problem* problem_(new Problem(FRM_PO_2D));
    TrajectoryN* trajectory_(new TrajectoryN(2));
    FrameN* frame_1_(new FrameN(1.011));
    FrameN* frame_2_(new FrameN(2.022));
    MeasurementN* sensor_data_cam_1_(new MeasurementN(640));
    MeasurementN* sensor_data_laser_(new MeasurementN(180));
    MeasurementN* sensor_data_cam_2_(new MeasurementN(480));
    MeasurementN* sensor_data_radar_(new MeasurementN(90));
    cout << "========================================================" << endl;    

    cout << endl << "TEST 2. Build tree dependencies" << endl;
    frame_1_->addDownNode(sensor_data_cam_1_);
    frame_1_->addDownNode(sensor_data_laser_);
    frame_2_->addDownNode(sensor_data_cam_2_);
    trajectory_->addDownNode(frame_1_);    
    trajectory_->addDownNode(frame_2_);
    trajectory_->linkToUpperNode(problem_);
    cout << "========================================================" << endl;
    
    cout << endl << "TEST 3. Modify one of the nodes (add new node), once tree has been constructed" << endl;
    frame_2_->addDownNode(sensor_data_radar_);    
    cout << "========================================================" << endl;    
    
    cout << endl << "TEST 4. Remove nodes" << endl;
    //check if resetting ptr previously, effectively removes object when calling removeDownNode()
    unsigned int f1_id_ = frame_1_->nodeId();
    frame_1_ = nullptr;
    sensor_data_cam_1_ = nullptr;
    sensor_data_laser_ = nullptr;;
    trajectory_->removeDownNode(f1_id_);
    cout << "========================================================" << endl;    
    
    cout << endl << "TEST 5. getWolfProblem()" << endl;
    Problem* nb_ptr = sensor_data_radar_->getProblem();
    //shared_ptr<TrajectoryN> nb_shptr((TrajectoryN*)nb_ptr);
    //    cout << "TOP node is: " << nb_ptr->nodeId() << endl;
    //cout << "nb_shptr.use_count(): " << nb_shptr.use_count() << "; value: " << nb_shptr.get() << endl;
    //cout << "trajectory_.use_count(): " << trajectory_.use_count() << "; value: " << trajectory_.get() << endl;
    //nb_shptr.reset();
    //COMMENTS: It seems that if shared pointer is used here, since type is NodeBase, there is no sharing ownership with trajectory_, which has type TrajectoryN, so the program crashes at the end: two shared pointers, pointing to the same object but each one with use_count()=1"
    cout << "========================================================" << endl;        
    
    cout << endl << "End NodeLinked test" << endl;
    return 0;
}

