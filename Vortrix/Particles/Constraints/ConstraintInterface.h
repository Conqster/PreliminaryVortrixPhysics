#pragma once

#include <Vortrix/Vortrix.h>
#include "Vortrix/Core/NonCopyable.h"

#include <vector>


class DebugGizmosRenderer; //TODO(Conqster): later have Vortrix debug render with module
namespace vx::Particles
{
	class IConstraintInterface// : public NonCopyable
	{
	public:
		virtual void UpdateSolver(float time_step) = 0;
		virtual void DebugGizmos(DebugGizmosRenderer* debug_renderer = nullptr) = 0;
		virtual ~IConstraintInterface() = default;
	};



	class ConstraintSolverPool
	{
	private:
		using ConstraintPool = std::vector<IConstraintInterface*>;
		ConstraintPool mSolverPool;
	public:
		ConstraintSolverPool() = default;
		~ConstraintSolverPool() { Clear(); }
		void Add(IConstraintInterface* force);
		void UpdateSolvers(float time_step);
		void OnDebugGizmos(DebugGizmosRenderer* debug_renderer = nullptr);
		void Clear();
	};
}