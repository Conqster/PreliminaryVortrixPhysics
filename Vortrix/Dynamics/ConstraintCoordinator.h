#pragma once
#include "Constraints/Constraint.h"



namespace vx {

	class DebugGizmosRenderer;
	class ConstraintSolver;
	struct NonContactConstraintDrawSettings;

	///Constraints 
	using Constraints = std::vector<Constraint*>;

	class ConstraintCoordinator
	{
	public:
		ConstraintCoordinator() = default;
		~ConstraintCoordinator();

		void Add(Constraint** constraints, uint32 count);
		void Remove(Constraint** constraints, uint32 count);

		template<typename T>
		T* CreateT(const T& constraint)
		{
			T* c = new T(constraint);
			Add((Constraint**)(&c), 1);
			return c;
		}

		///Limit 
		template<typename T>
		void CreateT(const T* constraint_Ts, uint32 count)
		{
			constexpr uint32 dyn_stack_allo_limit = 64;
			//uint32 _count = VxMin(count, dyn_stack_allo_limit);

			VX_ASSERT(count < dyn_stack_allo_limit);
			Constraint** c = (Constraint**)VX_STACK_ALLOC(count * sizeof(Constraint*));
		
			for (int i = 0; i < count; ++i)
				c[i] = new T(constraint_Ts[i]);

			Add((Constraint**)c, count);
		}


		Constraints& GetConstraints() { return mConstraints; }
		uint32 ConstraintCount() const { return mConstraints.size(); }
		uint32 GetTotalPredicted1DRow() const { return mTotalPredicted1DRow; }
		uint32 PrepConstraintSolving(ConstraintSolver& solver, PhysicsStepContext& ctx);
		void DebugGizmos(DebugGizmosRenderer* debug_renderer, const NonContactConstraintDrawSettings& draw_settings);

	private: 
		//for now vector 
		Constraints mConstraints;

		uint32 mTotalPredicted1DRow = 0;
		uint32 mTotalPredicted3DRow = 0;
	};

} //namespace vx