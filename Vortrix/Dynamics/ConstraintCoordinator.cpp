#include "ConstraintCoordinator.h"

#include "PhysicsWorldSettings.h"
#include "Core/Profiler.h"


namespace vx{

	ConstraintCoordinator::~ConstraintCoordinator()
	{
		for (auto& c : mConstraints)
			delete c; //<-- fix this
	}

	void ConstraintCoordinator::Add(Constraint** constraints, uint32 count)
	{
		mConstraints.reserve(mConstraints.size() + count);
		for (Constraint** c = constraints, **c_end = constraints + count; c < c_end; ++c)
		{
			(*c)->mConstraintIdx = mConstraints.size();
			mConstraints.push_back((*c));
		}
	}

	void ConstraintCoordinator::Remove(Constraint** constraints, uint32 count)
	{
		for (Constraint** c = constraints, **c_end = constraints + count; c < c_end; ++c)
		{
			VX_ASSERT((*c), "Attempting to remove null constraint");
			Constraint::Idx c_idx = (*c)->mConstraintIdx;
			VX_ASSERT(c_idx != Constraint::kInvalidIdx, "Attempting to remove invalid constraint");

			Constraint::Idx c_idx_end = mConstraints.back()->mConstraintIdx;

			//swap 
			if (c_idx < c_idx_end)
			{
				std::swap(mConstraints[c_idx], mConstraints[c_idx_end]);
				mConstraints[c_idx]->mConstraintIdx = c_idx;
			}

			(*c)->mConstraintIdx = Constraint::kInvalidIdx;
			mConstraints.pop_back();
		}
	}

	void ConstraintCoordinator::PrepConstraintSolving(ConstraintSolver& solver, const PhysicsStepContext& ctx)
	{
		VX_PROFILE_FUNCTION();
		for (auto& c : mConstraints)
			c->PrepSolver(&solver, ctx);
	}

	void ConstraintCoordinator::DebugGizmos(DebugGizmosRenderer* debug_renderer, const NonContactConstraintDrawSettings& draw_settings)
	{
		for (auto& c : mConstraints)
			c->DebugGizmos(debug_renderer, draw_settings);
	}
}
