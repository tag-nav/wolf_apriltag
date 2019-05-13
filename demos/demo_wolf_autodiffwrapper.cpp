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
    clock_t t1;
    ceres::Solver::Summary summary_ceres_diff, summary_wolf_diff;

    // loading variables
    std::map<unsigned int, FrameBasePtr> index_2_frame_ptr_ceres_diff;
    std::map<unsigned int, FrameBasePtr> index_2_frame_ptr_wolf_diff;
    std::map<FrameBasePtr, unsigned int> frame_ptr_2_index_wolf_diff;

    // Wolf problem
    ProblemPtr wolf_problem_ceres_diff = new Problem(FRM_PO_2D);
    ProblemPtr wolf_problem_wolf_diff = new Problem(FRM_PO_2D);
    SensorBasePtr sensor = new SensorBase("ODOM 2D", std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), std::make_shared<StateBlock>(Eigen::VectorXs::Zero(1)), std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), 2);

    // Ceres wrappers
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_ceres_diff = new CeresManager(wolf_problem_ceres_diff, ceres_options, false);
    CeresManager* ceres_manager_wolf_diff = new CeresManager(wolf_problem_wolf_diff, ceres_options, true);

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
                    FrameBasePtr vertex_frame_ptr_ceres_diff = new FrameBase(TimeStamp(0), std::make_shared<StateBlock>(vertex_pose.head(2)), std::make_shared<StateBlock>(vertex_pose.tail(1)));
                    FrameBasePtr vertex_frame_ptr_wolf_diff = new FrameBase(TimeStamp(0), std::make_shared<StateBlock>(vertex_pose.head(2)), std::make_shared<StateBlock>(vertex_pose.tail(1)));
                    wolf_problem_ceres_diff->getTrajectory()->addFrame(vertex_frame_ptr_ceres_diff);
                    wolf_problem_wolf_diff->getTrajectory()->addFrame(vertex_frame_ptr_wolf_diff);
                    // store
                    index_2_frame_ptr_ceres_diff[vertex_index] = vertex_frame_ptr_ceres_diff;
                    index_2_frame_ptr_wolf_diff[vertex_index] = vertex_frame_ptr_wolf_diff;
                    frame_ptr_2_index_wolf_diff[vertex_frame_ptr_wolf_diff] = vertex_index;

                    //std::cout << "Added vertex! index: " << vertex_index << " id: " << vertex_frame_ptr_wolf_diff->id() << std::endl << "pose: " << vertex_pose.transpose() << std::endl;
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
                    FeatureBasePtr feature_ptr_ceres_diff = new FeatureBase("POSE", edge_vector, edge_information.inverse());
                    FeatureBasePtr feature_ptr_wolf_diff = new FeatureBase("POSE", edge_vector, edge_information.inverse());
                    CaptureVoid* capture_ptr_ceres_diff = new CaptureVoid(TimeStamp(0), sensor);
                    CaptureVoid* capture_ptr_wolf_diff = new CaptureVoid(TimeStamp(0), sensor);
                    assert(index_2_frame_ptr_ceres_diff.find(edge_old) != index_2_frame_ptr_ceres_diff.end() && "edge from vertex not added!");
                    assert(index_2_frame_ptr_wolf_diff.find(edge_old) != index_2_frame_ptr_wolf_diff.end() && "edge from vertex not added!");
                    FrameBasePtr frame_old_ptr_ceres_diff = index_2_frame_ptr_ceres_diff[edge_old];
                    FrameBasePtr frame_old_ptr_wolf_diff = index_2_frame_ptr_wolf_diff[edge_old];
                    assert(index_2_frame_ptr_ceres_diff.find(edge_new) != index_2_frame_ptr_ceres_diff.end() && "edge to vertex not added!");
                    assert(index_2_frame_ptr_wolf_diff.find(edge_new) != index_2_frame_ptr_wolf_diff.end() && "edge to vertex not added!");
                    FrameBasePtr frame_new_ptr_ceres_diff = index_2_frame_ptr_ceres_diff[edge_new];
                    FrameBasePtr frame_new_ptr_wolf_diff = index_2_frame_ptr_wolf_diff[edge_new];
                    frame_new_ptr_ceres_diff->addCapture(capture_ptr_ceres_diff);
                    frame_new_ptr_wolf_diff->addCapture(capture_ptr_wolf_diff);
                    capture_ptr_ceres_diff->addFeature(feature_ptr_ceres_diff);
                    capture_ptr_wolf_diff->addFeature(feature_ptr_wolf_diff);
                    FactorOdom2D* factor_ptr_ceres_diff = new FactorOdom2D(feature_ptr_ceres_diff, frame_old_ptr_ceres_diff);
                    FactorOdom2D* factor_ptr_wolf_diff = new FactorOdom2D(feature_ptr_wolf_diff, frame_old_ptr_wolf_diff);
                    feature_ptr_ceres_diff->addFactor(factor_ptr_ceres_diff);
                    feature_ptr_wolf_diff->addFactor(factor_ptr_wolf_diff);
                    //std::cout << "Added edge! " << factor_ptr_wolf_diff->id() << " from vertex " << factor_ptr_wolf_diff->getCapture()->getFrame()->id() << " to " << factor_ptr_wolf_diff->getFrameTo()->id() << std::endl;
                    //std::cout << "vector " << factor_ptr_wolf_diff->getMeasurement().transpose() << std::endl;
                    //std::cout << "information " << std::endl << edge_information << std::endl;
                    //std::cout << "covariance " << std::endl << factor_ptr_wolf_diff->getMeasurementCovariance() << std::endl;
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
    FrameBasePtr first_frame_ceres_diff = wolf_problem_ceres_diff->getTrajectory()->getFrameList().front();
    FrameBasePtr first_frame_wolf_diff = wolf_problem_wolf_diff->getTrajectory()->getFrameList().front();
    CaptureFix* initial_covariance_ceres_diff = new CaptureFix(TimeStamp(0), new SensorBase("ABSOLUTE POSE", nullptr, nullptr, nullptr, 0), first_frame_ceres_diff->getState(), Eigen::Matrix3s::Identity() * 0.01);
    CaptureFix* initial_covariance_wolf_diff = new CaptureFix(TimeStamp(0), new SensorBase("ABSOLUTE POSE", nullptr, nullptr, nullptr, 0), first_frame_wolf_diff->getState(), Eigen::Matrix3s::Identity() * 0.01);
    first_frame_ceres_diff->addCapture(initial_covariance_ceres_diff);
    first_frame_wolf_diff->addCapture(initial_covariance_wolf_diff);
    initial_covariance_ceres_diff->process();
    initial_covariance_wolf_diff->process();
    //std::cout << "initial covariance: factor " << initial_covariance_wolf_diff->getFeatureList().front()->getFactorFromList().front()->id() << std::endl << initial_covariance_wolf_diff->getFeatureList().front()->getMeasurementCovariance() << std::endl;

    // COMPUTE COVARIANCES
    std::cout << "computing covariances..." << std::endl;
    t1 = clock();
    ceres_manager_ceres_diff->computeCovariances(ALL);//ALL_MARGINALS
    double t_sigma_ceres = ((double) clock() - t1) / CLOCKS_PER_SEC;
    t1 = clock();
    ceres_manager_wolf_diff->computeCovariances(ALL);//ALL_MARGINALS
    double t_sigma_wolf = ((double) clock() - t1) / CLOCKS_PER_SEC;
    std::cout << "computed!" << std::endl;

    // SOLVING PROBLEMS
    std::cout << "solving..." << std::endl;
    Eigen::VectorXs prev_ceres_state = wolf_problem_ceres_diff->getTrajectory()->getFrameList().back()->getState();
    summary_ceres_diff = ceres_manager_ceres_diff->solve();
    Eigen::VectorXs post_ceres_state = wolf_problem_ceres_diff->getTrajectory()->getFrameList().back()->getState();
    //std::cout << summary_ceres_diff.BriefReport() << std::endl;
    Eigen::VectorXs prev_wolf_state = wolf_problem_wolf_diff->getTrajectory()->getFrameList().back()->getState();
    summary_wolf_diff = ceres_manager_wolf_diff->solve();
    Eigen::VectorXs post_wolf_state = wolf_problem_wolf_diff->getTrajectory()->getFrameList().back()->getState();
    //std::cout << summary_wolf_diff.BriefReport() << std::endl;
    std::cout << "solved!" << std::endl;

    std::cout << "CERES AUTODIFF:" << std::endl;
    std::cout << "Covariance computation: " << t_sigma_ceres << "s" << std::endl;
    std::cout << "Solving:                " << summary_ceres_diff.total_time_in_seconds << "s" << std::endl;
    std::cout << "Prev:                   " << prev_ceres_state.transpose() << std::endl;
    std::cout << "Post:                   " << post_ceres_state.transpose() << std::endl;

    std::cout << std::endl << "WOLF AUTODIFF:" << std::endl;
    std::cout << "Covariance computation: " << t_sigma_wolf << "s" << std::endl;
    std::cout << "Solving:                " << summary_wolf_diff.total_time_in_seconds << "s" << std::endl;
    std::cout << "Prev:                   " << prev_wolf_state.transpose() << std::endl;
    std::cout << "Post:                   " << post_wolf_state.transpose() << std::endl;

    delete wolf_problem_ceres_diff; //not necessary to delete anything more, wolf will do it!
    delete wolf_problem_wolf_diff; //not necessary to delete anything more, wolf will do it!
    std::cout << "wolf_problem_ deleted!" << std::endl;
    delete ceres_manager_ceres_diff;
    delete ceres_manager_wolf_diff;
    std::cout << "ceres_manager deleted!" << std::endl;
    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;
       
    //exit
    return 0;
}
