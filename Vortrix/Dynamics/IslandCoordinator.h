#pragma once
#include "Vortrix/Core/Core.h"

#include "ConstraintSolver.h"

namespace vx {


	/// Requirements 
	///
	/// availabe parameters
	///	- BodyID/actual body
	/// - SolverBody & SolverIndex
	/// 
	/// - ContactConstraint
	/// - Constraint (Non-Contact)
	/// 
	/// 
	/// CONTACT section
	/// - broadphase pair processed in narrowphase
	/// - constructs contact constraint + solver body + LinkBodies() &/LinkContact()
	/// 
	/// requirement for solving
	/// * get constraint to solve for (this helps for selective island specific solving) task based
	/// 
	/// 
	/// * sort bodies, for instance 
	/// | 0 | 1 | 1 | 1 | 0 | 1 | 0 | 0 | 0 
	///  - to become 
	/// | 0 | 0 | 0 | 0 | 0 | 1 | 1 | 1 | 1
	///  - for easy GetIslandBodies
	/// * sort constraint, similar to bodies for GetIslandNonContactConstraints & GetIslandContactConstraints
	/// 
	/// * importantly its important to sort SolverBodyIdx to adhere with the island it leaves in
	/// 

	class IslandCoordinator
	{
	public:
		~IslandCoordinator();

		void Init(uint32 max_bodies, uint32 max_contact_constraint);

		void PrepareIslands(uint32 body_count);


		uint32 ComputeActiveBodyLowestIdx(uint32 idx);


		void LinkBodies(uint32 body_activeA, uint32 body_activeB);
		void LinkContactConstraint(uint32 constraint_idx, uint32 min_active_body_idx);
		void LinkNonConstactConstraint(uint32 constraint_idx, uint32 min_active_body_idx);
		
		void FinaliseIslands(const ConstraintSolver& constraint_solver, uint32 contact_constraint_count, BodyManager& body_manager, ScratchAllocator* scratchAllocator);
		void FinaliseBodyIslands(const ConstraintSolver& constraint_solver, BodyManager& body_manager, const uint32 active_bodies_count, uint32* io_temp_active_bodies_island_indices, ScratchAllocator* scratchAllocator);
		/// const i_temp_active_bodies_island_indices used to determine which island 
		/// constraubt fall into basecd on its body
		void FinaliseContactConstraint(const uint32 constraint_count, const uint32 island_count, 
			const uint32* i_temp_active_bodies_island_indices, ScratchAllocator* scratchAllocator);

		void SortIslands();

	//private:
		/// list would be invalid if not participanting or static 
		//uint32* mBodiesIdxs = nullptr;
		std::atomic<uint32>* mActiveBodyLinkIndices = nullptr; /// mBodyLinkIndexs
		/// this is correspond to the step 
		/// contact constraint in buffer 
		/// link to active body
		uint32* mContactConstraintBodyLinkIndices = nullptr;
		uint32 mMaxContactConstraint = 256;


		std::atomic<uint32> mValidateStepMaxConstraint{ 0 };


		//uint32* mIslandIdxs = nullptr;
		std::vector<uint32> mIslandIdxs;

		uint32 mActiveCount;


		uint32 mIslandCount{ 0 };


		template<typename T>
		struct IslandRange
		{
			IslandRange(T* _begin, T* _end) : begin(_begin), end(_end) {}
			T* begin;
			T* end;
		};


		IslandRange<BodyID> GetIslandBodyIDs(uint32 island_idx);
		IslandRange<uint32> GetIslandContactConstraintIndices(uint32 island_idx);

		/// sorted per island data

		/// convert to solver body index
		BodyID* mBodyIDIslands = nullptr;
		SolverBodyIndex* mSolverBodyIndexIslands = nullptr;
		//Solver body index end is equal tobody id end
		//uint32* mSolverBodyIndexPerIslandIndexEnds = nullptr;
		uint32* mBodyIDPerIslandIndexEnds = nullptr;


		uint32* mConstraintIndicesIslands = nullptr;
		uint32* mConstraintIslandIndexEnds = nullptr;



	public:
		struct Island
		{
			std::vector<BodyID> bodyIds;
		};
		std::vector<Island> islands;
	};

} /// namespace vx 