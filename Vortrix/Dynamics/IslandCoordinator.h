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

		void Init(uint32 max_bodies, uint32 max_contact_constraint, uint32 max_noncontact_constraint);

		void PrepareIslands(uint32 body_count);


		uint32 ComputeActiveBodyLowestIdx(uint32 idx);


		void LinkBodies(uint32 body_activeA, uint32 body_activeB);
		void LinkContactConstraint(uint32 constraint_idx, uint32 min_active_body_idx);

		/// using this for now 
		/// might later convert to a pipeline 
		/// where Row are not allocated during prep'ing
		/// so first loop; prep constraint (i.e active constraint) 
		/// then after building islands; jacobian rows are allocated 
		/// contigious to each other due island 
		struct ConstraintRowEdge //data
		{
			uint32 bodyLink; ///body index
			uint32 rowStart;
			uint32 rowCount;
		};
		/// so this would held 
		/// on building island Linking
		/// it links to the lowest body sort island allocation
		/// the row start is the pointer index to the constraint row in constraint solver
		/// row count is the jacobian row for constraint 
		/// 
		/// by the end of island building constraint island indices would contain 
		/// tje constraint in order ends for constraint is determined per constraintRowEdge start + count 
		void LinkNonConstactConstraint(uint32 constraint_row_idx, uint32 row_count, uint32 min_active_body_idx);

		
		void FinaliseIslands(const ConstraintSolver& constraint_solver, uint32 contact_constraint_count,
			uint32 active_non_contact_constraint_count, BodyManager& body_manager, ScratchAllocator* scratchAllocator);
		void FinaliseBodyIslands(const ConstraintSolver& constraint_solver, BodyManager& body_manager, const uint32 active_bodies_count, uint32* io_temp_active_bodies_island_indices, ScratchAllocator* scratchAllocator);
		/// const i_temp_active_bodies_island_indices used to determine which island 
		/// constraubt fall into basecd on its body
		void FinaliseContactConstraint(const uint32 constraint_count, const uint32 island_count, 
			const uint32* i_temp_active_bodies_island_indices, ScratchAllocator* scratchAllocator);
		void FinaliseNonContactConstraint(const uint32 constraint_count, const uint32 island_count,
			const uint32* i_temp_active_bodies_island_indices, ScratchAllocator* scratchAllocator);

		void SortIslands();

		uint32 NumIslands() const { return mIslandCount; }

	//private:
		/// list would be invalid if not participanting or static 
		//uint32* mBodiesIdxs = nullptr;
		std::atomic<uint32>* mActiveBodyLinkIndices = nullptr; /// mBodyLinkIndexs
		/// this is correspond to the step 
		/// contact constraint in buffer 
		/// link to active body
		uint32* mContactConstraintBodyLinkIndices = nullptr;
		uint32 mMaxContactConstraint = 256;

		/// using this for now 
		/// might later convert to a pipeline 
		/// where Row are not allocated during prep'ing
		/// so first loop; prep constraint (i.e active constraint) 
		/// then after building islands; jacobian rows are allocated 
		/// contigious to each other due island 
		ConstraintRowEdge* mNonContactConstraintRowBodyLinkIndices = nullptr; ///later make this temporary step data; using scratch allocation
		uint32 mMaxNonContactConstraint = 256;

		std::atomic<uint32> mValidateStepMaxConstraint{ 0 };
		std::atomic<uint32> mValidateStepMaxNonConstraint{ 0 };


		//uint32* mIslandIdxs = nullptr;
		std::vector<uint32> mIslandIdxs;


		uint32 mIslandCount{ 0 };


		template<typename T>
		struct IslandRange
		{
			IslandRange(T* _begin, T* _end) : begin(_begin), end(_end) {}
			const T* begin;
			const T* end;

			bool Valid() const 
			{ 
				return begin != nullptr && 
					end != nullptr &&
					end > begin; /// invariant; a valid range should contain at least one element
			}
			uint32 Size() const 
			{ 
				VX_ASSERT(Valid()); 
				return static_cast<uint32>(end - begin);
			}
		};



		IslandRange<BodyID> IslandBodyIDsRange(uint32 island_idx) const;
		IslandRange<SolverBodyIndex> IslandSolverBodyIndicesRange(uint32 island_idx) const;
		IslandRange<uint32> IslandContactConstraintIndicesRange(uint32 island_idx) const;
		IslandRange<uint32> IslandNonContactConstraintRowIndicesRange(uint32 island_idx) const;

		/// sorted per island data

		/// convert to solver body index
		BodyID* mBodyIDIslands = nullptr;
		SolverBodyIndex* mSolverBodyIndexIslands = nullptr;
		//Solver body index end is equal tobody id end
		//uint32* mSolverBodyIndexPerIslandIndexEnds = nullptr;
		uint32* mBodyIDPerIslandIndexEnds = nullptr;


		uint32* mConstraintIndicesIslands = nullptr;
		uint32* mConstraintIslandIndexEnds = nullptr;


		uint32* mNonConstactConstraintRowIndicesIslands = nullptr;
		uint32* mNonConstraintRowIslandIndexEnds = nullptr;
	};

} /// namespace vx 