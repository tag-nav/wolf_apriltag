// Sparsification example creating wolf tree from imported graph from .txt file

//C includes for sleep, time and main args
#include "unistd.h"

//std includes
#include <cstdlib>
#include <fstream>
#include <string>
#include <iostream>
#include <memory>
#include <random>
#include <cmath>
#include <queue>

//Wolf includes
#include "core/capture/capture_void.h"
#include "core/feature/feature_odom_2D.h"
#include "core/factor/factor_base.h"
#include "core/ceres_wrapper/ceres_manager.h"

// EIGEN
//#include <Eigen/CholmodSupport>
#include <Eigen/StdVector> // Eigen in std vector

namespace wolf{
// inserts the sparse matrix 'ins' into the sparse matrix 'original' in the place given by 'row' and 'col' integers

void insertSparseBlock(const Eigen::SparseMatrix<Scalar>& ins, Eigen::SparseMatrix<Scalar>& original, const unsigned int& row, const unsigned int& col)
{
  for (int k=0; k<ins.outerSize(); ++k)
    for (Eigen::SparseMatrix<Scalar>::InnerIterator iti(ins,k); iti; ++iti)
      original.coeffRef(iti.row() + row, iti.col() + col) = iti.value();

  original.makeCompressed();
}

void decodeEdge(const std::string& buffer, unsigned int& edge_from, unsigned int& edge_to, Eigen::Vector3s& measurement, Eigen::Matrix3s& covariance)
{
	std::string str_num;

	unsigned int i = 0;

	// only decode edges
	if (buffer.at(0) == 'E')
	{
		//skip rest of EDGE word
		while (buffer.at(i) != ' ') i++;
		//skip white spaces
		while (buffer.at(i) == ' ') i++;

		// FROM ID
		while (buffer.at(i) != ' ')
			str_num.push_back(buffer.at(i++));
		edge_from = atoi(str_num.c_str())+1;
		str_num.clear();

		//skip white spaces
		while (buffer.at(i) == ' ') i++;

		// TO ID
		while (buffer.at(i) != ' ')
			str_num.push_back(buffer.at(i++));
		edge_to = atoi(str_num.c_str())+1;
		str_num.clear();

		//skip white spaces
		while (buffer.at(i) == ' ') i++;

		// MEASUREMENT
		// X
		while (buffer.at(i) != ' ')
			str_num.push_back(buffer.at(i++));
		measurement(0) = atof(str_num.c_str());
		str_num.clear();
		//skip white spaces
		while (buffer.at(i) == ' ') i++;
		// Y
		while (buffer.at(i) != ' ')
			str_num.push_back(buffer.at(i++));
		measurement(1) = atof(str_num.c_str());
		str_num.clear();
		//skip white spaces
		while (buffer.at(i) == ' ') i++;
		// THETA
		while (buffer.at(i) != ' ')
			str_num.push_back(buffer.at(i++));
		measurement(2) = atof(str_num.c_str());
		str_num.clear();
		//skip white spaces
		while (buffer.at(i) == ' ') i++;

		// INFORMATION
		Eigen::Matrix3s information;
		// XX
		while (buffer.at(i) != ' ')
			str_num.push_back(buffer.at(i++));
		information(0,0) = atof(str_num.c_str());
		str_num.clear();
		//skip white spaces
		while (buffer.at(i) == ' ') i++;
		// XY
		while (buffer.at(i) != ' ')
			str_num.push_back(buffer.at(i++));
		information(0,1) = atof(str_num.c_str());
		information(1,0) = atof(str_num.c_str());
		str_num.clear();
		//skip white spaces
		while (buffer.at(i) == ' ') i++;
		// YY
		while (buffer.at(i) != ' ')
			str_num.push_back(buffer.at(i++));
		information(1,1) = atof(str_num.c_str());
		str_num.clear();
		//skip white spaces
		while (buffer.at(i) == ' ') i++;
		// THETATHETA
		while (buffer.at(i) != ' ')
			str_num.push_back(buffer.at(i++));
		information(2,2) = atof(str_num.c_str());
		str_num.clear();
		//skip white spaces
		while (buffer.at(i) == ' ') i++;
		// XTHETA
		while (buffer.at(i) != ' ')
			str_num.push_back(buffer.at(i++));
		information(0,2) = atof(str_num.c_str());
		information(2,0) = atof(str_num.c_str());
		str_num.clear();
		//skip white spaces
		while (buffer.at(i) == ' ') i++;
		// YTHETA
		while (i < buffer.size() && buffer.at(i) != ' ')
			str_num.push_back(buffer.at(i++));
		information(1,2) = atof(str_num.c_str());
		information(2,1) = atof(str_num.c_str());
		str_num.clear();

		// COVARIANCE
		covariance = information.inverse();
	}
	else
	{
		edge_from = 0;
		edge_to   = 0;
	}
}

}

int main(int argc, char** argv) 
{
    using namespace wolf;

    //Welcome message
    std::cout << std::endl << " ========= WOLF IMPORTED .graph TEST ===========" << std::endl << std::endl;

    bool wrong_input = false;
    if (argc < 3)
    	wrong_input = true;
    else if (argc > 4)
    	wrong_input = true;
    else if (argc > 2 && (atoi(argv[2]) < 2 || atoi(argv[2]) > 5))
    	wrong_input = true;
    else if (argc > 3 && atoi(argv[3]) < 0 )
    	wrong_input = true;

    if (wrong_input)
    {
        std::cout << "Please call me with: [./test_wolf_imported_graph DATASET T (MAX_VERTICES)], where:" << std::endl;
        std::cout << "    DATASET: manhattan, killian or intel" << std::endl;
        std::cout << "    T keep one node each T: 2, 3, 4 or 5" << std::endl;
        std::cout << "    optional: MAX_VERTICES max edges to be loaded" << std::endl;
        std::cout << "EXIT due to bad user input" << std::endl << std::endl;
        return -1;
    }

    // input variables
    char const * dataset_path = std::getenv("DATASET_PATH");
	unsigned int pruning_T = atoi(argv[2]);
    std::string file_path(dataset_path);
	file_path = file_path + "/graphs/redirected_" + std::to_string(pruning_T) + "_" + argv[1] + ".graph";
	unsigned int MAX_VERTEX = 1e9;
	if (argc > 3 && atoi(argv[3]) != 0)
    	MAX_VERTEX = atoi(argv[3]);

    // Wolf problem
    FrameBasePtr last_frame_ptr, frame_from_ptr, frame_to_ptr;
    ProblemPtr bl_problem_ptr = Problem::create("PO_2D");
    SensorBasePtr sensor_ptr = bl_problem_ptr->installSensor("ODOM 2D", "Odometry", Eigen::VectorXs::Zero(3), IntrinsicsBasePtr());

    // Ceres wrapper
    std::string bl_summary, sp_summary;
    ceres::Solver::Options ceres_options;
    ceres_options.minimizer_type = ceres::TRUST_REGION; //ceres::TRUST_REGION;LINE_SEARCH
    ceres_options.max_line_search_step_contraction = 1e-3;
    ceres_options.max_num_iterations = 10;
    CeresManagerPtr bl_ceres_manager = std::make_shared<CeresManager>(bl_problem_ptr, ceres_options);

    // load graph from .txt
    std::ifstream graph_file;
    graph_file.open(file_path.c_str(), std::ifstream::in);
    if (!graph_file.is_open())
    {
    	printf("\nError opening file: %s\n",file_path.c_str());
    	return -1;
    }

    // auxiliar variables
	std::string line_string;
	unsigned int edge_from, edge_to;
	Eigen::Vector3s meas;
	Eigen::Matrix3s meas_cov;
	Eigen::Matrix3s R = Eigen::Matrix3s::Identity();
	//clock_t t1;

	// ------------------------ START EXPERIMENT ------------------------
	// First frame FIXED
	last_frame_ptr = bl_problem_ptr->emplaceFrame(KEY_FRAME, Eigen::Vector3s::Zero(),TimeStamp(0));
	last_frame_ptr->fix();
	bl_problem_ptr->print(4, true, false, true);

	while (std::getline(graph_file, line_string) && last_frame_ptr->id() <= MAX_VERTEX)
	{
		std::cout << "new line:" << line_string << std::endl;
		decodeEdge(line_string, edge_from, edge_to, meas, meas_cov);

		// only factors
		if (edge_from != 0)
		{

			// ODOMETRY -------------------
			if (edge_to > last_frame_ptr->id())
			{
				frame_from_ptr = last_frame_ptr;

				// NEW KEYFRAME
				Eigen::Vector3s from_pose = frame_from_ptr->getState();
				R.topLeftCorner(2,2) = Eigen::Rotation2Ds(from_pose(2)).matrix();
				Eigen::Vector3s new_frame_pose = from_pose + R*meas;
				last_frame_ptr = bl_problem_ptr->emplaceFrame(KEY_FRAME, new_frame_pose, TimeStamp(double(edge_to)));

				frame_to_ptr = last_frame_ptr;

				std::cout << "NEW FRAME " << last_frame_ptr->id() << " - ts = " << last_frame_ptr->getTimeStamp().get() << std::endl;

				// REMOVE PREVIOUS NODES
			}
			// LOOP CLOSURE ---------------
			else
			{
				if (edge_from == last_frame_ptr->id())
					frame_from_ptr = last_frame_ptr;
				else
					for (auto frm_rit = bl_problem_ptr->getTrajectory()->getFrameList().rbegin(); frm_rit != bl_problem_ptr->getTrajectory()->getFrameList().rend(); frm_rit++)
						if ((*frm_rit)->id() == edge_from)
						{
							frame_from_ptr = *frm_rit;
							break;
						}
				if (edge_to == last_frame_ptr->id())
					frame_to_ptr = last_frame_ptr;
				else
					for (auto frm_rit = bl_problem_ptr->getTrajectory()->getFrameList().rbegin(); frm_rit != bl_problem_ptr->getTrajectory()->getFrameList().rend(); frm_rit++)
						if ((*frm_rit)->id() == edge_to)
						{
							frame_to_ptr = *frm_rit;
							break;
						}
			}
//			std::cout << "frame_from " << frame_from_ptr->id() << std::endl;
//			std::cout << "edge_from " << edge_from << std::endl;
//			std::cout << "frame_to " << frame_to_ptr->id() << std::endl;
//			std::cout << "edge_to " << edge_to << std::endl;

			assert(frame_from_ptr->id() == edge_from && "frame from id and edge from idx must be the same");
			assert(frame_to_ptr->id() == edge_to && "frame to id and edge to idx must be the same");

			// CAPTURE
			CaptureVoidPtr capture_ptr = std::make_shared<CaptureVoid>(TimeStamp(0), sensor_ptr);
			frame_from_ptr->addCapture(capture_ptr);

			// FEATURE
			FeatureBasePtr feature_ptr = std::make_shared<FeatureOdom2D>(meas, meas_cov);
			capture_ptr->addFeature(feature_ptr);

			// CONSTRAINT
			FactorOdom2DPtr factor_ptr = std::make_shared<FactorOdom2D>(feature_ptr, frame_to_ptr);
			feature_ptr->addFactor(factor_ptr);
			frame_to_ptr->addConstrainedBy(factor_ptr);

			// SOLVE
			// solution
      bl_summary = bl_ceres_manager->solve(SolverManager::ReportVerbosity::FULL);
		    std::cout << bl_summary << std::endl;

			// covariance
        bl_ceres_manager->computeCovariances(SolverManager::CovarianceBlocksToBeComputed::ALL);//ALL_MARGINALS

	//		t1 = clock();
	//		double t_sigma_manual = 0;
	//		t_sigma_manual += ((double) clock() - t1) / CLOCKS_PER_SEC;

		}
	}

	//bl_problem_ptr->print(4, true, false, true);

    //End message
    std::cout << " =========================== END ===============================" << std::endl << std::endl;
       
    //exit
    return 0;
}
