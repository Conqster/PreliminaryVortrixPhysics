#include "DistanceConstraint.h"
#include "PhysicsWorldSettings.h"
#include "Core/Profiler.h"

bool vx::DistanceConstraint::PrepSolver(ConstraintSolver* solver, const PhysicsStepContext& ctx)
{
	VX_PROFILE_FUNCTION();
	Linear1DRow row = BuildDistanceJacobian(ctx.stepDeltaTime);

	if (row.effMass == 0.0f)
		return false;

	row.bodyAidx = solver->GetOrCreateSolverBody(mBodyA->GetID(), ctx);
	row.bodyBidx = solver->GetOrCreateSolverBody(mBodyB->GetID(), ctx);

	//if (mResolvePosition == ESolvePosition::Projection)
	//	solver->AppendPositionCorrection(this);// Queue

	row.user = this;

	solver->AddLinearRow(row);
	return true;
}
