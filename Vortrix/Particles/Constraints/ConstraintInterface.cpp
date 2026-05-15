#include "ConstraintInterface.h"


namespace vx::Particles
{

	void ConstraintSolverPool::Add(IConstraintInterface* force)
	{
		mSolverPool.push_back(force);
	}

	void ConstraintSolverPool::UpdateSolvers(float time_step)
	{
		for (auto& solver : mSolverPool)
			solver->UpdateSolver(time_step);
	}

	void ConstraintSolverPool::OnDebugGizmos(DebugGizmosRenderer* debug_renderer)
	{
		for (auto& solver : mSolverPool)
			solver->DebugGizmos(debug_renderer);
	}

	void ConstraintSolverPool::Clear()
	{
		mSolverPool.clear();
	}
}