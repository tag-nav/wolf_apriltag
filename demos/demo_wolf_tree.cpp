// Testing create and delete full wolf tree with odometry captures

//std includes
#include <iostream>
#include <memory>
#include <random>
#include <cmath>
#include <queue>

//Wolf includes
#include "wolf_manager.h"

int main(int argc, char** argv) 
{
    using namespace wolf;

    //Welcome message
    std::cout << std::endl << " ========= WOLF TREE test ===========" << std::endl << std::endl;

    SensorOdom2D* odom_sensor_ptr_ = new SensorOdom2D(std::make_shared<StateBlock>(Eigen::Vector3s::Zero()),
                                                      std::make_shared<StateBlock>(Eigen::Vector1s::Zero()), 0.1, 0.1);
    //std::cout << " odom sensor created!" << std::endl;

    WolfManager* wolf_manager_ = new WolfManager(FRM_PO_2D,                             //frame structure
                                                 odom_sensor_ptr_,                  //odom sensor
                                                 Eigen::Vector3s::Zero(),           //prior
                                                 Eigen::Matrix3s::Identity()*0.01,  //prior cov
                                                 5,                                 //window size
                                                 1);                                //time for new keyframe
    //std::cout << " wolf_manager_ created!" << std::endl;

    wolf_manager_->addSensor(odom_sensor_ptr_);
    //std::cout << " odom sensor added!" << std::endl;

    //main loop
    for (unsigned int ii = 0; ii<1000; ii++)
    {
        // Add sintetic odometry
        wolf_manager_->addCapture(new CaptureOdom2D(TimeStamp(ii*0.1),
                                                    TimeStamp(ii*0.1+0.01),
                                                    odom_sensor_ptr_,
                                                    Eigen::Vector3s(0.1, 0. ,0.05)));
        //std::cout << " capture added!" << std::endl;

        // update wolf tree
        wolf_manager_->update();
        //std::cout << " updated!" << std::endl;
    }
    //std::cout << " end for!" << std::endl;

    delete wolf_manager_; //not necessary to delete anything more, wolf will do it!

    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;
       
    //exit
    return 0;
}

