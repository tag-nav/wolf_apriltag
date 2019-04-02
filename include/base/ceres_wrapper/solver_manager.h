#ifndef SOLVER_MANAGER_H_
#define SOLVER_MANAGER_H_

//wolf includes
#include "base/wolf.h"
#include "base/state_block.h"
#include "base/factor/factor_base.h"

namespace wolf {

/** \brief Enumeration of covariance blocks to be computed
 *
 * Enumeration of covariance blocks to be computed
 *
 */
typedef enum
{
    ALL, ///< All blocks and all cross-covariances
    ALL_MARGINALS, ///< All marginals
    ROBOT_LANDMARKS ///< marginals of landmarks and current robot pose plus cross covariances of current robot and all landmarks
} CovarianceBlocksToBeComputed;

WOLF_PTR_TYPEDEFS(SolverManager);

/** \brief Solver manager for WOLF
 *
 */

class SolverManager
{
	protected:
		ProblemPtr wolf_problem_;

	public:
        SolverManager(ProblemPtr _wolf_problem);

		virtual ~SolverManager();

		virtual std::string solve(const unsigned int& _report_level) = 0;

		virtual void computeCovariances(CovarianceBlocksToBeComputed _blocks = ROBOT_LANDMARKS) = 0;

		virtual void computeCovariances(const StateBlockPtrList& st_list) = 0;

		virtual void update();

        virtual ProblemPtr getProblem();

	private:

		virtual void addFactor(FactorBasePtr _fac_ptr) = 0;

		virtual void removeFactor(FactorBasePtr _fac_ptr) = 0;

		virtual void addStateBlock(StateBlockPtr _st_ptr) = 0;

		virtual void removeStateBlock(StateBlockPtr _st_ptr) = 0;

		virtual void updateStateBlockStatus(StateBlockPtr _st_ptr) = 0;
};

} // namespace wolf

#endif
