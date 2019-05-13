/*
 * test_virtual_hierarchy.cpp
 *
 *  Created on: Sep 8, 2014
 *      Author: jsola
 */

#include <list>

namespace wolf{

// BASE CLASSES

/**
 * \brief Node class.
 *
 * The Node class has only an ID and an ID factory built in the constructor.
 */
class N
{
    private:
        unsigned int id_;
        static unsigned int id_count_;
    protected:
        N() : id_(++id_count_) { }
        virtual ~N() { }
    public:
        unsigned int id() { return id_; }
};

/**
 * \brief Base class for children.
 *
 * It has a pointer to parent.
 *
 * NOTE: The virtual inheritance solves the "diamond of death" problem.
 */
template<class Parent>
class ChildOf : virtual public N
{
    private:
        Parent* up_ptr_;
    protected:
        ChildOf(Parent* _up_ptr) : up_ptr_(_up_ptr) { }
        virtual ~ChildOf() { }
        Parent* up() { return up_ptr_; }
};

/**
 * \brief Base class for parents
 *
 * It has a list of pointers to children, and a dumy method 'print' that is recursive to all children.
 *
 * NOTE: The virtual inheritance solves the "diamond of death" problem.
 */
template<class Child>
class ParentOf : virtual public N
{
    private:
        std::list<Child*> down_list_;
    protected:
        ParentOf() { }
        virtual ~ParentOf() { }
    public:
        void addToList(Child* _down_ptr) { down_list_.push_back(_down_ptr); }
        std::list<Child*> downList() { return down_list_; }
        virtual void print();
};

///*
// * Virtual inheritance solves the "diamond of death" problem.
// */
//template<class Sibling>
//class SiblingOf : virtual public N
//{
//    private:
//        std::list<Sibling*> side_list_;
//    protected:
//        SiblingOf() { }
//        virtual ~SiblingOf() { }
//    public:
//        void addToList(Sibling* _side_ptr) { side_list_.push_back(_side_ptr); }
//        std::list<Sibling*> sideList() { return side_list_; }
//};

/**
 * \brief Base class for bottom nodes.
 *
 * This class is for children that are no parents - they are bottom nodes
 *
 * It overloads the dummy 'print' function so that this is is not recursive any more.
 *
 * NOTE: The virtual inheritance solves the "diamond of death" problem.
 */
class Bot : virtual public N
{
    protected:
        virtual ~Bot() { }
    public:
        virtual void print();
};

//template<class Child>
//class ExplorerOf : public ParentOf<Child>
//{
//    protected:
//        ExplorerOf() { }
//        virtual ~ExplorerOf() { }
//    public:
//        virtual void explore();
//};

//class Explored : public Bot
//{
//    protected:
//        virtual ~Explored() { }
//    public:
//        virtual void explore() { }
//};

// DERIVED ISOLATED CLASSES

// a bunch of fwd_decs
class VehNode;
class FrmNode;
class CapNode;
class FeaNode;
class CorNode;
class TrSNode;
class SenNode;

class Veh
{
    public:
        Veh() : v_(1){}
        void duplicate(){v_ *= 2;}
        double v(){return v_;}
    private:
        double v_;
};
class Frm
{
    public:
        Frm() : f_(1){}
        void duplicate(){f_ *= 2;}
        double f(){return f_;}
    private:
        double f_;
};
class Cap
{
    public:
        Cap() : c_(1){}
        void duplicate(){c_ *= 2;}
        double c(){return c_;}
    private:
        double c_;
};
class Fea
{
    public:
        Fea() : ft_(1){}
        void duplicate(){ft_ *= 2;}
        double ft(){return ft_;}
    private:
        double ft_;
};
class Cor
{
public:
	Cor() : co_(1){}
	void duplicate(){co_ *= 2;}
	double co(){return co_;}
private:
	double co_;
};
class Sen
{
    public:
        Sen() : s_(1){}
        void duplicate(){s_ *= 2;}
        double s(){return s_;}
    private:
        double s_;
};

// Derived classes for all levels of the tree

class VehNode : public Veh, public ParentOf<FrmNode>, public ParentOf<SenNode>
{
    public:
        VehNode() { }
        virtual ~VehNode() { }
        void print(); // Overload because I am Top and have both Down and Explorer children.
};
class FrmNode : public Frm, public ChildOf<VehNode>, public ParentOf<CapNode>
{
    public:
        FrmNode(VehNode* _veh_ptr) : ChildOf<VehNode>(_veh_ptr) { }
        virtual ~FrmNode() { }
};
class CapNode : public Cap, public ChildOf<FrmNode>, public ParentOf<FeaNode>//, public SiblingOf<TrSNode>
{
    public:
        CapNode(FrmNode* _frm_ptr) : ChildOf<FrmNode>(_frm_ptr) { }
        virtual ~CapNode() { }
        void explore(); // Overload because I have both Explorer and Side children
};
class FeaNode : public Fea, public ChildOf<CapNode>, public ParentOf<CorNode>
{
    public:
        FeaNode(CapNode* _cap_ptr) : ChildOf<CapNode>(_cap_ptr) { }
        virtual ~FeaNode() { }
};
class CorNode : public Cor, public ChildOf<FeaNode>, public Bot//, public Explored
{
    public:
        CorNode(FeaNode* _fea_ptr) : ChildOf<FeaNode>(_fea_ptr) { }
        virtual ~CorNode() { }
};
//class TrSNode : public virtual N
//{
//    public:
//        TrSNode() { }
//        virtual ~TrSNode() { }
//};
class SenNode : public Sen, public ChildOf<VehNode>, public Bot
{
    public:
        SenNode(VehNode* _veh_ptr) : ChildOf<VehNode>(_veh_ptr) { }
        virtual ~SenNode() { }
};

} // namespace wolf

/////////////////////
// IMPLEMENTATIONS, here to avoid incomplete types and unwanted #includes
/////////////////////

#include <iostream>

namespace wolf {
using namespace std;

template<class Child>
void ParentOf<Child>::print()
{
    cout << this->id() << ":( ";
    for (auto const & it_ptr : this->downList())
        cout << it_ptr->id() << " ";
    cout << ")" << endl;
    for (auto const & it_ptr : this->downList())
        it_ptr->print();
}

//template<class Child>
//void ExplorerOf<Child>::explore()
//{
//    cout << this->id() << ":( "; // Yes I look sad but I'm OK.
//    for (auto const & it_ptr : ParentOf<Child>::downList())
//        cout << it_ptr->id() << " ";
//    cout << ")" << endl;
//    for (auto const & it_ptr : ParentOf<Child>::downList())
//        it_ptr->explore();
//}

void Bot::print(){
	cout << this->id() << ":( Bottom )" << endl;
}

void VehNode::print()
{
    cout << "Vehicle Own Field: v_ = " << v() << endl;
    cout << "Vehicle Hardware:" << endl;
    ParentOf < SenNode > ::print();
    cout << "Vehicle Data:" << endl;
    ParentOf < FrmNode > ::print();
}

//void CapNode::explore()
//{
//    cout << this->id() << ":( "; // Yes I look sad but I'm OK.
//    for (auto const & it_ptr : ExplorerOf<FeaNode>::downList())
//        cout << it_ptr->id() << " ";
//    cout << "/ ";
//    for (auto const & it_ptr : SiblingOf<TrSNode>::sideList())
//        cout << it_ptr->id() << " ";
//    cout << ")" << endl;
//    for (auto const & it_ptr : ExplorerOf<FeaNode>::downList())
//        it_ptr->explore();
//}

///////////////////////
// START APPLICATION
///////////////////////

unsigned int N::id_count_ = 0;

} // namespace wolf

int main()
{
    using namespace wolf;

    // Create all nodes with up-pointers already set up
    VehNode V;
    SenNode S0(&V), S1(&V);
    FrmNode F0(&V), F1(&V);
    CapNode C00(&F0), C01(&F0), C10(&F1), C11(&F1);
    FeaNode f000(&C00), f001(&C00), f010(&C01), f011(&C01), f100(&C10), f101(&C10), f110(&C11), f111(&C11);
//    TrSNode T0001, T0010, T0011, T0110, T0111, T1011;

    // Add sensors to vehicle
    V.ParentOf < SenNode > ::addToList(&S0);
    V.ParentOf < SenNode > ::addToList(&S1);

    // Add frames to vehicle
    V.ParentOf < FrmNode > ::addToList(&F0);
    V.ParentOf < FrmNode > ::addToList(&F1);

    // Add captures to frames
    F0.ParentOf<CapNode>::addToList(&C00);
    F0.ParentOf<CapNode>::addToList(&C01);
    F1.ParentOf<CapNode>::addToList(&C10);
    F1.ParentOf<CapNode>::addToList(&C11);

    // Add features to captures
    C00.ParentOf<FeaNode>::addToList(&f000);
    C00.ParentOf<FeaNode>::addToList(&f001);
    C01.ParentOf<FeaNode>::addToList(&f010);
    C01.ParentOf<FeaNode>::addToList(&f011);
    C10.ParentOf<FeaNode>::addToList(&f100);
    C10.ParentOf<FeaNode>::addToList(&f101);
    C11.ParentOf<FeaNode>::addToList(&f110);
    C11.ParentOf<FeaNode>::addToList(&f111);

//    // Add trans-sensors to captures
//    C00.SiblingOf<TrSNode>::addToList(&T0001);
//    C00.SiblingOf<TrSNode>::addToList(&T0010);
//    C00.SiblingOf<TrSNode>::addToList(&T0011);
//    C01.SiblingOf<TrSNode>::addToList(&T0110);
//    C01.SiblingOf<TrSNode>::addToList(&T0111);
//    C10.SiblingOf<TrSNode>::addToList(&T1011);

    // explore() : means we are calling advanced functionality from Explorer classes. Here, we just fake.
    // print()   : means we just print linkage info.
    cout << "V.explore():" << endl;
//    V.explore();
    V.duplicate();
    cout << "V.print():" << endl;
    V.print();

    return 0;
}
