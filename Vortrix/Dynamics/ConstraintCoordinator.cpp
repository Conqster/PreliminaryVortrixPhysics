#include "ConstraintCoordinator.h"

#include "Vortrix/PhysicsWorldSettings.h"
#include "Vortrix/Core/Profiler.h"

#include "ConstraintSolver.h"
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

			uint32 pred_1d_rows, pred_3d_rows;
			(*c)->GetRowCounts(pred_1d_rows, pred_3d_rows);
			mTotalPredicted1DRow += pred_1d_rows;

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
			uint32 pred_1d_rows, pred_3d_rows;
			(*c)->GetRowCounts(pred_1d_rows, pred_3d_rows);
			mTotalPredicted1DRow -= pred_1d_rows;
			mConstraints.pop_back();
		}
	}

	uint32 ConstraintCoordinator::PrepConstraintSolving(ConstraintSolver& solver, const PhysicsStepContext& ctx)
	{
		VX_PROFILE_FUNCTION();
		//for now just all constraint might need position correction 
		uint32 req_position_correction = mConstraints.size();
		solver.PrepareSolver(mTotalPredicted1DRow, req_position_correction, ctx);


		uint32 active_constraints = 0;
		for (auto& c : mConstraints)
			active_constraints += c->PrepSolver(&solver, ctx);
		//{
		//	if (c->PrepSolver(&solver, ctx))
		//		active_constraints++;
		//}

		return active_constraints;
	}

	void ConstraintCoordinator::DebugGizmos(DebugGizmosRenderer* debug_renderer, const NonContactConstraintDrawSettings& draw_settings)
	{
		for (auto& c : mConstraints)
			c->DebugGizmos(debug_renderer, draw_settings);
	}
}
