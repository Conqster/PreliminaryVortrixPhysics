#include "DistanceConstraint.h"
#include "PhysicsWorldSettings.h"

bool vx::DistanceConstraint::PrepSolver(ConstraintSolver* solver, const PhysicsStepContext& ctx)
{
	Linear1DRow row = BuildDistanceJacobian(ctx.stepDeltaTime);

	row.bodyAidx = solver->GetOrCreateSolverBody(mBodyA->GetID(), ctx);
	row.bodyBidx = solver->GetOrCreateSolverBody(mBodyB->GetID(), ctx);

	row.user = this;

	solver->AddLinearRow(row);
	return true;
}
