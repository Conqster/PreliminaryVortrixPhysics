#pragma once

#include "EConstraintFlags.h"

namespace vx {

	enum class EConstraintType : uint8
	{
		Distance,
		Point,
	};


	class ConstraintSolver;
	class ConstraintCoordinator;
	class Body;
	struct PhysicsStepContext;
	struct Linear1DRow;

	struct NonContactConstraintDrawSettings;
	
	class DebugGizmosRenderer;

	class Constraint
	{
	public:

		Constraint(Body* bodyA, Body* bodyB) : mBodyA(bodyA), mBodyB(bodyB) {}
		virtual EConstraintType Type() const = 0;
		virtual const char* TypeName() const = 0;

		///assume all constraint are two bodies for now, until required not to be
		Body* BodyA() const { return mBodyA; }
		Body* BodyB() const { return mBodyB; }
		//virtual bool PrepSolver(SolverBuilder*) = 0;
		virtual uint32 PrepSolver(ConstraintSolver*, const PhysicsStepContext&) = 0;
		/// essentailly used for commiting back accumulated lambda
		/// based on constraints policy
		virtual void CommitSolverState(const Linear1DRow& row) = 0;

		virtual void DebugGizmos(DebugGizmosRenderer* debug_renderer, const NonContactConstraintDrawSettings& draw_settings) const = 0;

		virtual void SolvePositionConstraint(float dt, float baumgarte) = 0;

		/// idx in coordinate constraint vector
		using Idx = uint32;
		/// idx in coordinate constraint vector
		static constexpr Idx kInvalidIdx = 0xffffffff;
		Idx ConstraintIdx()const { return mConstraintIdx; }
		void ConstraintIdx(uint32 idx) { mConstraintIdx = idx; }



		virtual void GetRowCounts(/*const PhysicsStepContext& ctx, */uint32& o_1D_rows, uint32& o_3D_rows) = 0;
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

		friend ConstraintCoordinator;
		Idx mConstraintIdx = kInvalidIdx;

		virtual bool RequiresPositionCorrection() = 0;

		Body* mBodyA = nullptr;
		Body* mBodyB = nullptr;
	};

} //namespace vx