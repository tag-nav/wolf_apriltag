// Testing creating wolf tree from imported .graph file

//C includes for sleep, time and main args
#include "unistd.h"

//std includes
#include <iostream>
#include <memory>
#include <random>
#include <cmath>
#include <queue>

//Wolf includes
#include "wolf_manager.h"
#include "core/capture/capture_void.h"
#include "core/ceres_wrapper/ceres_manager.h"

// EIGEN
//#include <Eigen/CholmodSupport>

namespace wolf {
// inserts the sparse matrix 'ins' into the sparse matrix 'original' in the place given by 'row' and 'col' integers
void insertSparseBlock(const Eigen::SparseMatrix<Scalar>& ins, Eigen::SparseMatrix<Scalar>& original, const unsigned int& row, const unsigned int& col)
{
  for (int k=0; k<ins.outerSize(); ++k)
    for (Eigen::SparseMatrix<Scalar>::InnerIterator iti(ins,k); iti; ++iti)
      original.coeffRef(iti.row() + row, iti.col() + col) = iti.value();

  original.makeCompressed();
}

int main(int argc, char** argv) 
{
    using namespace wolf;

    //Welcome message
    std::cout << std::endl << " ========= WOLF IMPORTED .graph TEST ===========" << std::endl << std::endl;

    if (argc != 3 || atoi(argv[2]) < 0 )
    {
        std::cout << "Please call me with: [./test_wolf_imported_graph FILE_PATH MAX_VERTICES], where:" << std::endl;
        std::cout << "    FILE_PATH is the .graph file path" << std::endl;
        std::cout << "    MAX_VERTICES max edges to be loaded (0: ALL)" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }

    // auxiliar variables
    std::string file_path_ = argv[1];
    unsigned int MAX_VERTEX = atoi(argv[2]);
    if (MAX_VERTEX == 0) MAX_VERTEX = 1e6;
    std::ifstream offLineFile_;
    ceres::Solver::Summary summary_autodiff, summary_analytic;

    // loading variables
    std::map<unsigned int, FrameBasePtr> index_2_frame_ptr_autodiff;
    std::map<unsigned int, FrameBasePtr> index_2_frame_ptr_analytic;

    // Wolf problem
    ProblemPtr wolf_problem_autodiff = new Problem(FRM_PO_2D);
    ProblemPtr wolf_problem_analytic = new Problem(FRM_PO_2D);
    SensorBasePtr sensor = new SensorBase("ODOM 2D", std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), std::make_shared<StateBlock>(Eigen::VectorXs::Zero(1)), std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), 2);

    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_autodiff = new CeresManager(wolf_problem_autodiff, ceres_options);
    CeresManager* ceres_manager_analytic = new CeresManager(wolf_problem_analytic, ceres_options);

    // load graph from .txt
    offLineFile_.open(file_path_.c_str(), std::ifstream::in);
    if (offLineFile_.is_open())
    {
        std::string buffer;
        unsigned int j = 0;
        // Line by line
        while (std::getline(offLineFile_, buffer))
        {
            //std::cout << "new line:" << buffer << std::endl;
            std::string bNum;
            unsigned int i = 0;

            // VERTEX
            if (buffer.at(0) == 'V')
            {
                //skip rest of VERTEX word
                while (buffer.at(i) != ' ') i++;
                //skip white spaces
                while (buffer.at(i) == ' ') i++;

                //vertex index
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                unsigned int vertex_index = atoi(bNum.c_str());
                bNum.clear();

                if (vertex_index <= MAX_VERTEX+1)
                {
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;

                    // vertex pose
                    Eigen::Vector3s vertex_pose;
                    // x
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    vertex_pose(0) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // y
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    vertex_pose(1) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // theta
                    while (i < buffer.size() && buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    vertex_pose(2) = atof(bNum.c_str());
                    bNum.clear();

                    // add frame to problem
                    FrameBasePtr vertex_frame_ptr_autodiff = new FrameBase(TimeStamp(0), std::make_shared<StateBlock>(vertex_pose.head(2)), std::make_shared<StateBlock>(vertex_pose.tail(1)));
                    FrameBasePtr vertex_frame_ptr_analytic = new FrameBase(TimeStamp(0), std::make_shared<StateBlock>(vertex_pose.head(2)), std::make_shared<StateBlock>(vertex_pose.tail(1)));
                    wolf_problem_autodiff->getTrajectory()->addFrame(vertex_frame_ptr_autodiff);
                    wolf_problem_analytic->getTrajectory()->addFrame(vertex_frame_ptr_analytic);
                    // store
                    index_2_frame_ptr_autodiff[vertex_index] = vertex_frame_ptr_autodiff;
                    index_2_frame_ptr_analytic[vertex_index] = vertex_frame_ptr_analytic;

                    //std::cout << "Added vertex! index: " << vertex_index << " id: " << vertex_frame_ptr_analytic->id() << std::endl << "pose: " << vertex_pose.transpose() << std::endl;
                }
            }
            // EDGE
            else if (buffer.at(0) == 'E')
            {
                j++;
                //skip rest of EDGE word
                while (buffer.at(i) != ' ') i++;
                //skip white spaces
                while (buffer.at(i) == ' ') i++;

                //from
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                unsigned int edge_old = atoi(bNum.c_str());
                bNum.clear();
                //skip white spaces
                while (buffer.at(i) == ' ') i++;

                //to index
                while (buffer.at(i) != ' ')
                    bNum.push_back(buffer.at(i++));
                unsigned int edge_new = atoi(bNum.c_str());
                bNum.clear();

                if (edge_new <= MAX_VERTEX+1 && edge_old <= MAX_VERTEX+1 )
                {
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;

                    // edge vector
                    Eigen::Vector3s edge_vector;
                    // x
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_vector(0) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // y
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_vector(1) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // theta
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_vector(2) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;

                    // edge covariance
                    Eigen::Matrix3s edge_information;
                    // xx
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_information(0,0) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // xy
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_information(0,1) = atof(bNum.c_str());
                    edge_information(1,0) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // yy
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_information(1,1) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // thetatheta
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_information(2,2) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // xtheta
                    while (buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_information(0,2) = atof(bNum.c_str());
                    edge_information(2,0) = atof(bNum.c_str());
                    bNum.clear();
                    //skip white spaces
                    while (buffer.at(i) == ' ') i++;
                    // ytheta
                    while (i < buffer.size() && buffer.at(i) != ' ')
                        bNum.push_back(buffer.at(i++));
                    edge_information(1,2) = atof(bNum.c_str());
                    edge_information(2,1) = atof(bNum.c_str());
                    bNum.clear();

                    // add capture, feature and factor to problem
                    FeatureBasePtr feature_ptr_autodiff = new FeatureBase("POSE", edge_vector, edge_information.inverse());
                    CaptureVoid* capture_ptr_autodiff = new CaptureVoid(TimeStamp(0), sensor);
                    assert(index_2_frame_ptr_autodiff.find(edge_old) != index_2_frame_ptr_autodiff.end() && "edge from vertex not added!");
                    FrameBasePtr frame_old_ptr_autodiff = index_2_frame_ptr_autodiff[edge_old];
                    assert(index_2_frame_ptr_autodiff.find(edge_new) != index_2_frame_ptr_autodiff.end() && "edge to vertex not added!");
                    FrameBasePtr frame_new_ptr_autodiff = index_2_frame_ptr_autodiff[edge_new];
                    frame_new_ptr_autodiff->addCapture(capture_ptr_autodiff);
                    capture_ptr_autodiff->addFeature(feature_ptr_autodiff);
                    FactorOdom2D* factor_ptr_autodiff = new FactorOdom2D(feature_ptr_autodiff, frame_old_ptr_autodiff);
                    feature_ptr_autodiff->addFactor(factor_ptr_autodiff);
                    //std::cout << "Added autodiff edge! " << factor_ptr_autodiff->id() << " from vertex " << factor_ptr_autodiff->getCapture()->getFrame()->id() << " to " << factor_ptr_autodiff->getFrameOther()->id() << std::endl;

                    // add capture, feature and factor to problem
                    FeatureBasePtr feature_ptr_analytic = new FeatureBase("POSE", edge_vector, edge_information.inverse());
                    CaptureVoid* capture_ptr_analytic = new CaptureVoid(TimeStamp(0), sensor);
                    assert(index_2_frame_ptr_analytic.find(edge_old) != index_2_frame_ptr_analytic.end() && "edge from vertex not added!");
                    FrameBasePtr frame_old_ptr_analytic = index_2_frame_ptr_analytic[edge_old];
                    assert(index_2_frame_ptr_analytic.find(edge_new) != index_2_frame_ptr_analytic.end() && "edge to vertex not added!");
                    FrameBasePtr frame_new_ptr_analytic = index_2_frame_ptr_analytic[edge_new];
                    frame_new_ptr_analytic->addCapture(capture_ptr_analytic);
                    capture_ptr_analytic->addFeature(feature_ptr_analytic);
                    FactorOdom2DAnalytic* factor_ptr_analytic = new FactorOdom2DAnalytic(feature_ptr_analytic, frame_old_ptr_analytic);
                    feature_ptr_analytic->addFactor(factor_ptr_analytic);
                    //std::cout << "Added analytic edge! " << factor_ptr_analytic->id() << " from vertex " << factor_ptr_analytic->getCapture()->getFrame()->id() << " to " << factor_ptr_analytic->getFrameOther()->id() << std::endl;
                    //std::cout << "vector " << factor_ptr_analytic->getMeasurement().transpose() << std::endl;
                    //std::cout << "information " << std::endl << edge_information << std::endl;
                    //std::cout << "covariance " << std::endl << factor_ptr_analytic->getMeasurementCovariance() << std::endl;
                }
            }
            else
                assert("unknown line");
        }
        printf("\nGraph loaded!\n");
    }
    else
        printf("\nError opening file\n");

    // PRIOR
    FrameBasePtr first_frame_autodiff = wolf_problem_autodiff->getTrajectory()->getFrameList().front();
    FrameBasePtr first_frame_analytic = wolf_problem_analytic->getTrajectory()->getFrameList().front();
    CaptureFix* initial_covariance_autodiff = new CaptureFix(TimeStamp(0), new SensorBase("ABSOLUTE POSE", nullptr, nullptr, nullptr, 0), first_frame_autodiff->getState(), Eigen::Matrix3s::Identity() * 0.01);
    CaptureFix* initial_covariance_analytic = new CaptureFix(TimeStamp(0), new SensorBase("ABSOLUTE POSE", nullptr, nullptr, nullptr, 0), first_frame_analytic->getState(), Eigen::Matrix3s::Identity() * 0.01);
    first_frame_autodiff->addCapture(initial_covariance_autodiff);
    first_frame_analytic->addCapture(initial_covariance_analytic);
    initial_covariance_autodiff->emplaceFeatureAndFactor();
    initial_covariance_analytic->emplaceFeatureAndFactor();

    // SOLVING PROBLEMS
    std::cout << "solving..." << std::endl;
    std::cout << "ANALYTIC -----------------------------------" << std::endl;
    summary_analytic = ceres_manager_analytic->solve();
    std::cout << summary_analytic.FullReport() << std::endl;
    std::cout << "AUTODIFF -----------------------------------" << std::endl;
    summary_autodiff = ceres_manager_autodiff->solve();
    std::cout << summary_autodiff.FullReport() << std::endl;

    // COMPUTE COVARIANCES
    std::cout << "computing covariances..." << std::endl;
    std::cout << "ANALYTIC -----------------------------------" << std::endl;
    clock_t t1 = clock();
    ceres_manager_analytic->computeCovariances(ALL);//ALL_MARGINALS
    std::cout << "Time: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;
    std::cout << "AUTODIFF -----------------------------------" << std::endl;
    t1 = clock();
    ceres_manager_autodiff->computeCovariances(ALL);//ALL_MARGINALS
    std::cout << "Time: " << ((double) clock() - t1) / CLOCKS_PER_SEC << "s" << std::endl;

    delete wolf_problem_autodiff; //not necessary to delete anything more, wolf will do it!
    std::cout << "wolf_problem_ deleted!" << std::endl;
    delete ceres_manager_autodiff;
    delete ceres_manager_analytic;
    std::cout << "ceres_manager deleted!" << std::endl;
    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;
       
    //exit
    return 0;
}
