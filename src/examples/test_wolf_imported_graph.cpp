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

namespace wolf{
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
    clock_t t1;
    ceres::Solver::Summary summary_full, summary_prun;
    Eigen::MatrixXs Sigma_ii(3,3), Sigma_ij(3,3), Sigma_jj(3,3), Sigma_z(3,3), Ji(3,3), Jj(3,3);
    Scalar xi, yi, thi, si, ci, xj, yj;

    // loading variables
    std::map<unsigned int, FrameBasePtr> index_2_frame_ptr_full;
    std::map<unsigned int, FrameBasePtr> index_2_frame_ptr_prun;
    std::map<FrameBasePtr, unsigned int> frame_ptr_2_index_prun;

    // Wolf problem
    ProblemPtr wolf_problem_full = new Problem(FRM_PO_2D);
    ProblemPtr wolf_problem_prun = new Problem(FRM_PO_2D);
    SensorBasePtr sensor = new SensorBase("ODOM 2D", std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), std::make_shared<StateBlock>(Eigen::VectorXs::Zero(1)), std::make_shared<StateBlock>(Eigen::VectorXs::Zero(2)), 2);

    Eigen::SparseMatrix<Scalar> Lambda(0,0);

    // Ceres wrapper
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 1e4;
    CeresManager* ceres_manager_full = new CeresManager(wolf_problem_full, ceres_options);
    CeresManager* ceres_manager_prun = new CeresManager(wolf_problem_prun, ceres_options);

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
                    FrameBasePtr vertex_frame_ptr_full = new FrameBase(KEY_FRAME, TimeStamp(0), std::make_shared<StateBlock>(vertex_pose.head(2)), std::make_shared<StateBlock>(vertex_pose.tail(1)));
                    FrameBasePtr vertex_frame_ptr_prun = new FrameBase(KEY_FRAME, TimeStamp(0), std::make_shared<StateBlock>(vertex_pose.head(2)), std::make_shared<StateBlock>(vertex_pose.tail(1)));
                    wolf_problem_full->getTrajectory()->addFrame(vertex_frame_ptr_full);
                    wolf_problem_prun->getTrajectory()->addFrame(vertex_frame_ptr_prun);
                    // store
                    index_2_frame_ptr_full[vertex_index] = vertex_frame_ptr_full;
                    index_2_frame_ptr_prun[vertex_index] = vertex_frame_ptr_prun;
                    frame_ptr_2_index_prun[vertex_frame_ptr_prun] = vertex_index;
                    // Information matrix
                    Lambda.conservativeResize(Lambda.rows()+3,Lambda.cols()+3);

                    //std::cout << "Added vertex! index: " << vertex_index << " id: " << vertex_frame_ptr_prun->id() << std::endl << "pose: " << vertex_pose.transpose() << std::endl;
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
                    FeatureBasePtr feature_ptr_full = new FeatureBase("POSE", edge_vector, edge_information.inverse());
                    FeatureBasePtr feature_ptr_prun = new FeatureBase("POSE", edge_vector, edge_information.inverse());
                    CaptureVoid* capture_ptr_full = new CaptureVoid(TimeStamp(0), sensor);
                    CaptureVoid* capture_ptr_prun = new CaptureVoid(TimeStamp(0), sensor);
                    assert(index_2_frame_ptr_full.find(edge_old) != index_2_frame_ptr_full.end() && "edge from vertex not added!");
                    assert(index_2_frame_ptr_prun.find(edge_old) != index_2_frame_ptr_prun.end() && "edge from vertex not added!");
                    FrameBasePtr frame_old_ptr_full = index_2_frame_ptr_full[edge_old];
                    FrameBasePtr frame_old_ptr_prun = index_2_frame_ptr_prun[edge_old];
                    assert(index_2_frame_ptr_full.find(edge_new) != index_2_frame_ptr_full.end() && "edge to vertex not added!");
                    assert(index_2_frame_ptr_prun.find(edge_new) != index_2_frame_ptr_prun.end() && "edge to vertex not added!");
                    FrameBasePtr frame_new_ptr_full = index_2_frame_ptr_full[edge_new];
                    FrameBasePtr frame_new_ptr_prun = index_2_frame_ptr_prun[edge_new];
                    frame_new_ptr_full->addCapture(capture_ptr_full);
                    frame_new_ptr_prun->addCapture(capture_ptr_prun);
                    capture_ptr_full->addFeature(feature_ptr_full);
                    capture_ptr_prun->addFeature(feature_ptr_prun);
                    FactorOdom2D* factor_ptr_full = new FactorOdom2D(feature_ptr_full, frame_old_ptr_full);
                    FactorOdom2D* factor_ptr_prun = new FactorOdom2D(feature_ptr_prun, frame_old_ptr_prun);
                    feature_ptr_full->addFactor(factor_ptr_full);
                    feature_ptr_prun->addFactor(factor_ptr_prun);
                    //std::cout << "Added edge! " << factor_ptr_prun->id() << " from vertex " << factor_ptr_prun->getCapture()->getFrame()->id() << " to " << factor_ptr_prun->getFrameTo()->id() << std::endl;
                    //std::cout << "vector " << factor_ptr_prun->getMeasurement().transpose() << std::endl;
                    //std::cout << "information " << std::endl << edge_information << std::endl;
                    //std::cout << "covariance " << std::endl << factor_ptr_prun->getMeasurementCovariance() << std::endl;

                    Scalar xi = *(frame_old_ptr_prun->getP()->get());
                    Scalar yi = *(frame_old_ptr_prun->getP()->get()+1);
                    Scalar thi = *(frame_old_ptr_prun->getO()->get());
                    Scalar si = sin(thi);
                    Scalar ci = cos(thi);
                    Scalar xj = *(frame_new_ptr_prun->getP()->get());
                    Scalar yj = *(frame_new_ptr_prun->getP()->get()+1);
                    Eigen::MatrixXs Ji(3,3), Jj(3,3);
                    Ji << -ci,-si,-(xj-xi)*si+(yj-yi)*ci,
                           si,-ci,-(xj-xi)*ci-(yj-yi)*si,
                            0,  0,                    -1;
                    Jj <<  ci, si, 0,
                          -si, ci, 0,
                            0,  0, 1;
                    //std::cout << "Ji" << std::endl << Ji << std::endl;
                    //std::cout << "Jj" << std::endl << Jj << std::endl;
                    Eigen::SparseMatrix<Scalar> DeltaLambda(Lambda.rows(), Lambda.cols());
                    insertSparseBlock((Ji.transpose() * edge_information * Ji).sparseView(), DeltaLambda, edge_old*3, edge_old*3);
                    insertSparseBlock((Jj.transpose() * edge_information * Jj).sparseView(), DeltaLambda, edge_new*3, edge_new*3);
                    insertSparseBlock((Ji.transpose() * edge_information * Jj).sparseView(), DeltaLambda, edge_old*3, edge_new*3);
                    insertSparseBlock((Jj.transpose() * edge_information * Ji).sparseView(), DeltaLambda, edge_new*3, edge_old*3);
                    Lambda = Lambda + DeltaLambda;
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
    FrameBasePtr first_frame_full = wolf_problem_full->getTrajectory()->getFrameList().front();
    FrameBasePtr first_frame_prun = wolf_problem_prun->getTrajectory()->getFrameList().front();
    CaptureFix* initial_covariance_full = new CaptureFix(TimeStamp(0), new SensorBase("ABSLOUTE POSE", nullptr, nullptr, nullptr, 0), first_frame_full->getState(), Eigen::Matrix3s::Identity() * 0.01);
    CaptureFix* initial_covariance_prun = new CaptureFix(TimeStamp(0), new SensorBase("ABSLOUTE POSE", nullptr, nullptr, nullptr, 0), first_frame_prun->getState(), Eigen::Matrix3s::Identity() * 0.01);
    first_frame_full->addCapture(initial_covariance_full);
    first_frame_prun->addCapture(initial_covariance_prun);
    initial_covariance_full->process();
    initial_covariance_prun->process();
    //std::cout << "initial covariance: factor " << initial_covariance_prun->getFeatureList().front()->getFactorFromList().front()->id() << std::endl << initial_covariance_prun->getFeatureList().front()->getMeasurementCovariance() << std::endl;
    Eigen::SparseMatrix<Scalar> DeltaLambda(Lambda.rows(), Lambda.cols());
    insertSparseBlock((Eigen::Matrix3s::Identity() * 100).sparseView(), DeltaLambda, 0, 0);
    Lambda = Lambda + DeltaLambda;

    // COMPUTE COVARIANCES
    FactorBasePtrList factors;
    wolf_problem_prun->getTrajectory()->getFactorList(factors);
    // Manual covariance computation
    t1 = clock();
    Eigen::SimplicialLLT<Eigen::SparseMatrix<Scalar>> chol(Lambda);  // performs a Cholesky factorization of A
    Eigen::MatrixXs Sigma = chol.solve(Eigen::MatrixXs::Identity(Lambda.rows(), Lambda.cols()));
    double t_sigma_manual = ((double) clock() - t1) / CLOCKS_PER_SEC;
    //std::cout << "Lambda" << std::endl << Lambda << std::endl;
    //std::cout << "Sigma" << std::endl << Sigma << std::endl;

    std::cout << " ceres is computing covariances..." << std::endl;
    t1 = clock();
    ceres_manager_prun->computeCovariances(ALL);//ALL_MARGINALS
    double t_sigma_ceres = ((double) clock() - t1) / CLOCKS_PER_SEC;
    std::cout << "computed!" << std::endl;

    t1 = clock();

    for (auto c_it=factors.begin(); c_it!=factors.end(); c_it++)
    {
        if ((*c_it)->getCategory() != FAC_FRAME) continue;

        // ii (old)
        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameOther()->getP(), (*c_it)->getFrameOther()->getP(), Sigma_ii, 0, 0);
        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameOther()->getP(), (*c_it)->getFrameOther()->getO(), Sigma_ii, 0, 2);
        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameOther()->getO(), (*c_it)->getFrameOther()->getP(), Sigma_ii, 2, 0);
        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameOther()->getO(), (*c_it)->getFrameOther()->getO(), Sigma_ii, 2, 2);
//        std::cout << "Sigma_ii" << std::endl << Sigma_ii << std::endl;
//        std::cout << "Sigma(i,i)" << std::endl << Sigma.block<3,3>(frame_ptr_2_index_prun[(*c_it)->getFrameTo()]*3, frame_ptr_2_index_prun[(*c_it)->getFrameTo()]*3) << std::endl;
        // jj (new)
        wolf_problem_prun->getCovarianceBlock((*c_it)->getCapture()->getFrame()->getP(), (*c_it)->getCapture()->getFrame()->getP(), Sigma_jj, 0, 0);
        wolf_problem_prun->getCovarianceBlock((*c_it)->getCapture()->getFrame()->getP(), (*c_it)->getCapture()->getFrame()->getO(), Sigma_jj, 0, 2);
        wolf_problem_prun->getCovarianceBlock((*c_it)->getCapture()->getFrame()->getO(), (*c_it)->getCapture()->getFrame()->getP(), Sigma_jj, 2, 0);
        wolf_problem_prun->getCovarianceBlock((*c_it)->getCapture()->getFrame()->getO(), (*c_it)->getCapture()->getFrame()->getO(), Sigma_jj, 2, 2);
//        std::cout << "Sigma_jj" << std::endl << Sigma_jj << std::endl;
//        std::cout << "Sigma(j,j)" << std::endl << Sigma.block<3,3>(frame_ptr_2_index_prun[(*c_it)->getCapture()->getFrame()]*3, frame_ptr_2_index_prun[(*c_it)->getCapture()->getFrame()]*3) << std::endl;
        // ij (old-new)
        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameOther()->getP(), (*c_it)->getCapture()->getFrame()->getP(), Sigma_ij, 0, 0);
        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameOther()->getP(), (*c_it)->getCapture()->getFrame()->getO(), Sigma_ij, 0, 2);
        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameOther()->getO(), (*c_it)->getCapture()->getFrame()->getP(), Sigma_ij, 2, 0);
        wolf_problem_prun->getCovarianceBlock((*c_it)->getFrameOther()->getO(), (*c_it)->getCapture()->getFrame()->getO(), Sigma_ij, 2, 2);
//        std::cout << "Sigma_ij" << std::endl << Sigma_ij << std::endl;
//        std::cout << "Sigma(i,j)" << std::endl << Sigma.block<3,3>(frame_ptr_2_index_prun[(*c_it)->getFrameTo()]*3, frame_ptr_2_index_prun[(*c_it)->getCapture()->getFrame()]*3) << std::endl;

        //jacobian
        xi = *(*c_it)->getFrameOther()->getP()->get();
        yi = *((*c_it)->getFrameOther()->getP()->get()+1);
        thi = *(*c_it)->getFrameOther()->getO()->get();
        si = sin(thi);
        ci = cos(thi);
        xj = *(*c_it)->getCapture()->getFrame()->getP()->get();
        yj = *((*c_it)->getCapture()->getFrame()->getP()->get()+1);

        Ji << -ci,-si,-(xj-xi)*si+(yj-yi)*ci,
               si,-ci,-(xj-xi)*ci-(yj-yi)*si,
                0,  0,                    -1;
        Jj <<  ci, si, 0,
              -si, ci, 0,
                0,  0, 1;
        //std::cout << "Ji" << std::endl << Ji << std::endl;
        //std::cout << "Jj" << std::endl << Jj << std::endl;

        // Measurement covariance
        Sigma_z = (*c_it)->getFeature()->getMeasurementCovariance();
        //std::cout << "Sigma_z" << std::endl << Sigma_z << std::endl;
        //std::cout << "Sigma_z.determinant() = " << Sigma_z.determinant() << std::endl;

        //std::cout << "denominador : " << std::endl << Sigma_z - (Ji * Sigma_ii * Ji.transpose() + Jj * Sigma_jj * Jj.transpose() + Ji * Sigma_ij * Jj.transpose() + Jj * Sigma_ij.transpose() * Ji.transpose()) << std::endl;
        // Information gain
        Scalar IG = 0.5 * log( Sigma_z.determinant() / (Sigma_z - (Ji * Sigma_ii * Ji.transpose() + Jj * Sigma_jj * Jj.transpose() + Ji * Sigma_ij * Jj.transpose() + Jj * Sigma_ij.transpose() * Ji.transpose())).determinant() );
        //std::cout << "IG = " << IG << std::endl;

        if (IG < 2)
            (*c_it)->setStatus(FAC_INACTIVE);
    }
    double t_ig = ((double) clock() - t1) / CLOCKS_PER_SEC;
    std::cout << "manual sigma computation " << t_sigma_manual << "s" << std::endl;
    std::cout << "ceres sigma computation " << t_sigma_ceres << "s" << std::endl;
    std::cout << "information gain computation " << t_ig << "s" << std::endl;

    // SOLVING PROBLEMS
    std::cout << "solving..." << std::endl;
    summary_full = ceres_manager_full->solve();
    std::cout << summary_full.FullReport() << std::endl;
    summary_prun = ceres_manager_prun->solve();
    std::cout << summary_prun.FullReport() << std::endl;

    delete wolf_problem_full; //not necessary to delete anything more, wolf will do it!
    std::cout << "wolf_problem_ deleted!" << std::endl;
    delete ceres_manager_full;
    delete ceres_manager_prun;
    std::cout << "ceres_manager deleted!" << std::endl;
    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;
       
    //exit
    return 0;
}
