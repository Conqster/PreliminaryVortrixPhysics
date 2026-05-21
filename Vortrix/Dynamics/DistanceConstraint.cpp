#include "DistanceConstraint.h"

bool vx::DistanceConstraint::PrepSolver(ConstraintSolver* solver, float dt)
{
	Linear1DRow row = SetupDistanceJacobian2(dt);
	solver->AddLinearRow(row);
	return true;
}
