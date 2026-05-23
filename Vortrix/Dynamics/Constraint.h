#pragma once

#include "EConstraintFlags.h"

class DebugGizmosRenderer;
namespace vx {

	struct Linear1DRow;
	class ConstraintSolver;
	struct PhysicsStepContext;

	class Constraint
	{
	public:
		//virtual bool PrepSolver(SolverBuilder*) = 0;
		virtual bool PrepSolver(ConstraintSolver*, const PhysicsStepContext&) = 0;
		/// essentailly used for commiting back accumulated lambda
		/// based on constraints policy
		virtual void CommitSolverState(const Linear1DRow& row) = 0;

		virtual void DebugGizmos(DebugGizmosRenderer* debug_renderer) const = 0;

		virtual void SolvePositionConstraint(float dt, float baumgarte) = 0;
	protected:
		/// could be used, in PrepSolver/BuildSolver or BuildIsland
		/// or even SolverPositionConstraint
		/// or even SolverGeometryConstraint to reject Solve pass
		/// 
		/// and couple setup in Constraint creatrion, 
		/// 
		/// preventing bloating of solver data (Linear1DRow) and is flag 
		/// is checked during PrepSolver/BuildSolver or BuildIsland
		/// then Constraint could be added to list/graph for Geomteric/Position 
		/// correction, i.e reducing the number of constraint loop is most are not 
		/// hard/rigid constraint
		EConstraintFlags mFlags = EConstraintFlags::SolveVelocity | EConstraintFlags::SolvePosition;

		virtual bool RequiresPositionCorrection() = 0;
	};

} //namespace vx