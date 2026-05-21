#pragma once


class DebugGizmosRenderer;
namespace vx {

	struct Linear1DRow;
	class ConstraintSolver;


	class Constraint
	{
	public:
		//virtual bool PrepSolver(SolverBuilder*) = 0;
		virtual bool PrepSolver(ConstraintSolver*, float dt) = 0;
		/// essentailly used for commiting back accumulated lambda
		/// based on constraints policy
		virtual void CommitSolverState(const Linear1DRow& row) = 0;

		virtual void DebugGizmos(DebugGizmosRenderer* debug_renderer) const = 0;
	};

} //namespace vx