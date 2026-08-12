#include "PhysicsWorld.h"

#include "Core/Profiler.h"

#include "Dynamics/Body/Body.h"
#include "Dynamics/Body/BodySimStats.h"

#include "Collision/Shapes/Shape.h"
#include "Collision/Shapes/SphereShape.h"
#include "Collision/Shapes/BoxShape.h"
#include "Collision/Shapes/PlaneShape.h"
#include "Collision/Shapes/CapsuleShape.h"

#include "Collision/BoxBoxContactDebug.h"

#include "Geometry/AABB.h"
#include "Geometry/OBB.h"
#include "Collision/Broadphase/BVHBroadphase.h"

#include "Vortrix/Visuals/Renderers.h"


#include "Collision/Broadphase/BruteforceBroadphase.h"

#include "Collision/Narrowphase/NarrowphaseQuery.h"
#include "Collision/Narrowphase/NarrowphaseQueryStat.h"

#include "Visuals/ERenderInstanceFlags.h"
#include "Visuals/RenderSettings.h"

#include "SimulationContexts.h"

#include "Dynamics/ConstraintCoordinator.h"
#include "Dynamics/ConstraintSolver.h"

#include "Core/ScratchAllocator.h"

#include "Core/TaskCoordinator.h"
#include "Dynamics/IslandCoordinator.h"


namespace vx
{

	uint64 PhysicsWorld::mFrameIdx = 0;

	PhysicsWorld::PhysicsWorld(const PhysicsWorldSettings& in_settings) :
		mSettings(in_settings)
	{
	}

	PhysicsWorld::~PhysicsWorld()
	{
		delete mBroadphase;

		delete mConstraintSolver;

		delete mIslandCoordinator;

		mScratchAllocator->Free(testAllocation, testAllocationSize);
		delete mScratchAllocator;

		mTaskCoordinator->Quit();
		delete mTaskCoordinator;
	}

	void PhysicsWorld::CreateSimpleWorld(PhysicsWorld* io_world)
	{
		VX_ASSERT(io_world, "Physics World is null");

		static float no_static_bodies = 0.0f;

		bool create_capsules = true;
		bool create_boxes = true;

		BodySettings dyn_bodies_settings = BodySettings::DefaultDynamicConstruct();
		BodySettings static_bodies_settings = BodySettings::DefaultStaticConstruct();
		if (create_boxes)
		{
			Ref<BoxShape> unit_box = MakeRef<BoxShape>(0.5f);
			dyn_bodies_settings.position = Vec3(-2.0f, 5.5f, 0.0f);
			dyn_bodies_settings.debug_name = "box";
			dyn_bodies_settings.shape = MakeRef<BoxShape>(0.5f, 0.25f, 0.5f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(2.0f, 3.0f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(0.15f, 0.6f, 0.35f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.shape = unit_box;
			dyn_bodies_settings.position = Vec3(-4.0f, 5.5f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(-4.0f, 10.0f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(1.0f, 7.5f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(0.0f, 12.5f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(1.5f, 0.15f, 0.15f);
			io_world->CreateBody(dyn_bodies_settings);
		}

		vx::SphereShapeSettings shape_settings(0.5f);
		shape_settings.SetDensity(0.0f);
		Ref<SphereShape> static_unit_sphere = MakeRef<SphereShape>(shape_settings);
		static_bodies_settings = BodySettings::DefaultStaticConstruct();
		static_bodies_settings.position = Vec3(3.0f, 5.5f, 0.0f);
		static_bodies_settings.debug_name = "sphere";
		static_bodies_settings.shape = static_unit_sphere;
		io_world->CreateBody(static_bodies_settings);

		io_world->CreateBody(static_bodies_settings);
		static_bodies_settings.position = Vec3(6.0f, 5.5f, 0.1f);
		io_world->CreateBody(static_bodies_settings);
		static_bodies_settings.position = Vec3(4.5f, 8.35f, 0.0f);
		io_world->CreateBody(static_bodies_settings);
		static_bodies_settings.position = Vec3(4.5f, 3.3f, -0.1f);
		io_world->CreateBody(static_bodies_settings);


		/// Ground plane
		vx::PlaneShapeSettings plane_shape_settings(vx::Vec3::Up(), 100.0f);
		plane_shape_settings.SetDensity(0.0f);
		static_bodies_settings = BodySettings::DefaultStaticConstruct();
		static_bodies_settings.debug_name = "ground";
		static_bodies_settings.shape = MakeRef<PlaneShape>(plane_shape_settings);
		io_world->CreateBody(static_bodies_settings);

		/// Create side planes as well
		///front 
		/// back
		/// right
		/// left
		/// 
		/// front offset along z and face -z (rotate around -x)
		BodySettings body_settings = BodySettings::DefaultStaticConstruct();
		body_settings.debug_name = "front plane";
		body_settings.shape = static_bodies_settings.shape;///*MakeRef<PlaneShape>(Vec3::Up(), 100.0f);*/
		
		float world_size = 100.0f;
		body_settings.position = Vec3(0.0f, 0.0f, world_size);
		body_settings.orientation.SetAxisAngle(Vec3::Right(), DegToRad(-90.0f));
		io_world->CreateBody(body_settings);

		/// front offset along -z and face z
		body_settings.debug_name = "back plane";
		body_settings.position = Vec3(0.0f, 0.0f, -world_size);
		body_settings.orientation.SetAxisAngle(Vec3::Right(), DegToRad(90.0f));
		io_world->CreateBody(body_settings);

		/// front offset along x and face -x (rotate around z
		body_settings.debug_name = "right plane";
		body_settings.position = Vec3(world_size, 0.0f, 0.0f);
		body_settings.orientation.SetAxisAngle(Vec3::Forward(), DegToRad(90.0f));
		io_world->CreateBody(body_settings);

		/// front offset along -x and face x (rotate around z)
		body_settings.debug_name = "left plane";
		body_settings.position = Vec3(-world_size, 0.0f, 0.0f);
		body_settings.orientation.SetAxisAngle(Vec3::Forward(), DegToRad(-90.0f));
		io_world->CreateBody(body_settings);

		if(create_capsules)
		{
			dyn_bodies_settings = BodySettings::DefaultDynamicConstruct();
			dyn_bodies_settings.debug_name = "Capsule";
			dyn_bodies_settings.shape = MakeRef<CapsuleShape>(0.5f, 0.5f);
			dyn_bodies_settings.position = Vec3(4.0f, 10.0f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(7.5f, 10.0f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(-4.5f, 10.0f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(-7.5f, 12.0f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(0.5f, 12.0f, 0.0f);
			io_world->CreateBody(dyn_bodies_settings);
		}


		
		if(create_boxes)
		{
			dyn_bodies_settings.position = Vec3(-2.0f, 5.5f, 0.0f);
			dyn_bodies_settings.debug_name = "box";
			dyn_bodies_settings.shape = MakeRef<BoxShape>(0.5f, 0.25f, 0.5f);
			io_world->CreateBody(dyn_bodies_settings);


			dyn_bodies_settings.position = Vec3(1.0f, 7.0f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(0.15f, 0.6f, 0.35f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(0.0f, 5.5f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(0.25f, 0.125, 0.25f);
			io_world->CreateBody(dyn_bodies_settings);


			dyn_bodies_settings.position = Vec3(3.0f, 7.5f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(1.0f, 0.25f, 0.25f);
			io_world->CreateBody(dyn_bodies_settings);


			dyn_bodies_settings.position = Vec3(0.0f, 12.5f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(1.5f, 0.15f, 0.15f);
			io_world->CreateBody(dyn_bodies_settings);

			dyn_bodies_settings.position = Vec3(0.0f, 10.0f, 0.0f);
			dyn_bodies_settings.shape = MakeRef<BoxShape>(1.0f, 0.25f, 1.0f);
			io_world->CreateBody(dyn_bodies_settings);
		}

	}

	void PhysicsWorld::GenerateWorldDefaultConfig(int& o_max_bodies, int& o_max_body_pairs, int& o_max_contact_constraint)
	{
		o_max_bodies = 10240;// 16384;

		int avg_max_pair = 4;//8;
		o_max_body_pairs = avg_max_pair * o_max_bodies;

		int avg_contact_per_body = 4;
		o_max_contact_constraint = avg_contact_per_body * o_max_bodies;
	}

	void PhysicsWorld::Init(float max_bodies, float max_body_pairs, float max_contact_constraint)
	{
		VX_PROFILE_FUNCTION();
		//heavily using point to data in vector in Broadphase 
		//at the moment so let reseve 
		mBodyManager.Init(max_bodies);
		mContext.bodyManager = &mBodyManager;

		BroadphaseInitInfo broad_init;
		broad_init.maxDirtyBodies = mBodyManager.MaxBodies() * 0.6f;
		mBroadphase = new BVHBroadphase<AABB>();
		mBroadphase->SetBoundThreshold(mSettings.collision.boundsMargin);
		mBroadphase->Init(&mBodyManager, broad_init);

		//mBroadphasePairs.reserve(max_body_pairs);
		//mBroadphaseBuffer.data = new BroadphasePair[max_body_pairs];
		mBroadphaseBuffer.maxPairs = max_body_pairs;

		///later pass body manager as via ctx
		mNarrowphaseQuery.Init(&mBodyManager);

		mContactConstraintSolver.Init(max_contact_constraint);
		mContactConstraintSolver.SetPhysicsContext(&mContext);

		mConstraintSolver = new ConstraintSolver;
		mConstraintSolver->Init(mBodyManager);

		mWorldQuery.Init(mBroadphase);

		mScratchAllocator = new ScratchAllocator(1 * 1024 * 1024);

		testAllocationSize =
			sizeof(Linear1DRow) * 2000 +
			//sizeof(BroadphasePair) * max_body_pairs +
			sizeof(Constraint*) * 250;
		testAllocation = mScratchAllocator->Allocate(testAllocationSize);

		int num_threads = 0;
		num_threads = std::thread::hardware_concurrency() - 1;
		mTaskCoordinator = new TaskCoordinatorMt(-1);


		mIslandCoordinator = new IslandCoordinator;
		mIslandCoordinator->Init(mBodyManager.MaxBodies(), mContactConstraintSolver.MaxConstraints(), 512);
	}

	Body* PhysicsWorld::CreateBody(const BodySettings& body_setting, bool activate_body)
	{
		const BodyID body_id = mBodyManager.AddBody(body_setting);
		
		if (!body_id.IsValid())
			return nullptr;

		if (body_setting.inBroadphase)
		{
			VX_ASSERT(mBroadphase);
			mBroadphase->InsertBody(&mBodyManager.GetBody(body_id));

			if (activate_body && body_setting.motionType != EMotionType::Static)
				ActivateBodies(&body_id, 1);

			mBodyManager.GetBodyDebugInfo(body_id).bodyInBroadphase = true;
		}
		else
			VX_ASSERT(!activate_body, "To activate Body has to participate in broadphase");


		return &mBodyManager.GetBody(body_id);
	}

	void PhysicsWorld::RemoveBody(const BodyID& id)
	{
		mBroadphase->RemoveBody(id);
		mBodyManager.RemoveBody(id);
	}


	void PhysicsWorld::StepSimulation(float dt)
	{
		VX_PROFILE_FUNCTION();
		Vec3 sample_gravity_vel = mSettings.gravity * mSettings.gravityScale;
		sample_gravity_vel *= dt;
		mSettings.frameGravityVelocity = sample_gravity_vel.ToFloat3();
		mContext.stepDeltaTime = dt;
		mContext.gravity = mSettings.gravity;
		mContext.gravityScale = mSettings.gravityScale;
		mContext.forceBVHRebuild = mSettings.forceBVHRebuild;
		mContext.BVH_rebuild_SAH = mSettings.collision.BVH_rebuild_SAH;
		mContext.rebuildBVH_ImbalanceRatioTreshold = mSettings.collision.rebuildBVH_ImbalanceRatioTreshold;
		mContext.mScratchAllocator = mScratchAllocator;
		mContext.mIslandCoordinator = mIslandCoordinator;
		mContext.maxBroadphasePair = mBroadphaseBuffer.maxPairs;

		mWorldQuery.SetDebugRender(mHackDebugRenderer);
		mWorldQuery.SetDrawBroadphaseNodesWalked(mSettings.drawSettings.drawWalkedTreeQuery);

#if VX_DEBUG_ALLOCATOR
		mScratchAllocator->ResetDebugAlloc();
#endif // VX_DEBUG_ALLOCATOR

		{
			VX_PROFILE_SCOPE("Reset Simulation State");
			for (auto& body_debug : mBodyManager.GetBodiesDebug())
				body_debug.simulationStats.Reset();
		}


		{
			VX_PROFILE_SCOPE("Integrate bodies acceleration");

			uint32 num_active_bodies = GetNumActiveBodies();
			BodyID* active_bodies = GetActiveBodies();

			for (int i = 0; i < num_active_bodies; ++i)
			{
				Body& body = mBodyManager.GetBody(active_bodies[i]);
				body.IntegrateAcceleration(dt, mSettings.gravity * mSettings.gravityScale);
				body.ClearAccumulatedForces();
			}
		}

		mBroadphaseBuffer.data = reinterpret_cast<BroadphasePair*>(mScratchAllocator->Allocate(sizeof(BroadphasePair) * mBroadphaseBuffer.maxPairs));
		auto& mBroadpairCount = mBroadphaseBuffer.count;
		mBroadpairCount = 0;
		//uint32& broadpair_count = 0; later wrap in pointer
		/// Broadphase collsion
		//VX_ASSERT_WARN(mBroadphase, "Broadphase is null.");
		if (mBroadphase != nullptr)
			mBroadphase->ComputeCollidingPair(mContext, mBroadphaseBuffer.data, mBroadpairCount);

		mContactConstraintSolver.PreFrameSetup(mSettings); //for per frame transient allcation for now


		/// for now need to invalidate previous frame local bodies 
		/// so the bodies could be update for use by narrowphase handshake 
		/// with contact constraint, fix later 
		{
			VX_PROFILE_SCOPE("Clear Non contact constraint");
			mConstraintSolver->HackClear();
		}
		CollisionContext collision_ctx
		{
			mSettings.collision, mHackDebugRenderer,
			mSettings.drawSettings.drawContactConstraintSolverTBNs,
			mFrameIdx, mBroadpairCount, mConstraintSolver,
			mIslandCoordinator, &mBodyManager
		};
		//Narrowphase: collision detection & contact generations
		
		//mNarrowphaseQuery.ProcessPairs(mBroadphaseBuffer.data, mStepManifolds, mContactConstraintSolver, collision_ctx);


		mIslandCoordinator->PrepareIslands((uint32)mBodyManager.GetBodies().size());

#if USE_MULTITHREAD
		{
			VX_PROFILE_SCOPE("Processing and contact constraint setup multithreading");
			///multithreading
			struct QuickPairProcessAndConstraintSetupContext
			{
				BroadphasePair* pairs;
				NarrowphaseQuery& narrowphase_query;
				ContactConstraintSolver& contact_solver;
				const CollisionContext& collision_ctx;
			};

			QuickPairProcessAndConstraintSetupContext pair_process_and_constraint_setup_ctx =
			{
				mBroadphaseBuffer.data,
				mNarrowphaseQuery,
				mContactConstraintSolver,
				collision_ctx
			};


			//mProcessPairAndTrySetupContactConstraintTasks.clear();

			//for (BroadphasePair* bp = mBroadphaseBuffer.data,
			//	*bp_end = mBroadphaseBuffer.data + collision_ctx.broadphasePairCount;
			//	bp < bp_end; ++bp)
			//{
			//	/// create tasks 
			//	mProcessPairAndTrySetupContactConstraintTasks.push_back(mTaskCoordinator->ConstructTask([body_a = (*bp).a, body_b = (*bp).b, ctx = &pair_process_and_constraint_setup_ctx]()
			//		{
			//			ctx->narrowphase_query.ProcessPairAndTrySetupContactConstraint(body_a, body_b, ctx->contact_solver, ctx->collision_ctx);
			//		}, 0)); /// no dependancies
			//}

			constexpr uint32 k_pair_per_task = 32;

			mTaskCoordinator->ParallelFor(
				collision_ctx.broadphasePairCount,
				k_pair_per_task,
				///Range Task
				{
					&pair_process_and_constraint_setup_ctx,
					[](void* user_data, uint32 begin, uint32 end)
					{
						VX_PROFILE_SCOPE("Parallel For Broadphase pair; Thread execution");
						auto& ctx = *static_cast<QuickPairProcessAndConstraintSetupContext*>(user_data);
						for (uint32 i = begin; i < end; ++i)
						{
							auto& pair = ctx.pairs[i];

							ctx.narrowphase_query.ProcessPairAndTrySetupContactConstraint(
								pair.a,
								pair.b,
								ctx.contact_solver, ctx.collision_ctx);
						}
					}
				});

			mTaskCoordinator->WaitForTasks();
		}
#else
		mNarrowphaseQuery.ProcessPairs(mBroadphaseBuffer.data, mStepManifolds, mContactConstraintSolver, collision_ctx);
#endif // USE_MULTITHREAD

#if TEST_CONTACT_CONSTRAINT_MT
		mContactConstraintSolver.FinaliseWriteManifoldCache();
#endif // TEST_CONTACT_CONSTRAINT_MT


		/// free broadphase data straight after narrowphase
		mScratchAllocator->Free(mBroadphaseBuffer.data, sizeof(BroadphasePair) * mBroadphaseBuffer.maxPairs);
		mBroadphaseBuffer.data = nullptr;

		/// Prepare non contact constraint 
		/// contact constraint should be done
		/// time to prep joint constraints 
		const uint32 active_joint_constraint_count = mConstraintCoordinator.PrepConstraintSolving(*mConstraintSolver, mContext);


		/// Active Body should have been linked 
		/// Body Contact Constraint 
		/// Body Non Contact Constraint
		mIslandCoordinator->FinaliseIslands(*mConstraintSolver, mContactConstraintSolver.NumContactConstraints(), active_joint_constraint_count, mBodyManager, mScratchAllocator);



	
		{
			VX_PROFILE_SCOPE("Splitting Islands");
			mIslandCoordinator->mSplitter.Prepare(mBodyManager.GetNumActiveBodies(), *mIslandCoordinator, mScratchAllocator);

			for (uint32 i = 0; i < mBodyManager.GetNumActiveBodies(); ++i)
			{
				auto& body = mBodyManager.GetBody(mBodyManager.GetActiveBodyID(i));
				body.mIslandConstraintGroupMask = 0;
			}

			for (uint32 i = 0; i < mIslandCoordinator->IslandCount(); ++i)
				mIslandCoordinator->mSplitter.SplitIsland(i, *mIslandCoordinator, &mContactConstraintSolver, mConstraintSolver, &mBodyManager);
		}


		if (mSettings.solver.enable)
		{

			/// Requirements 
			///
			/// availabe parameters
			///	- BodyID/actual body
			/// - SolverBody & SolverIndex
			/// 
			/// - ContactConstraint
			/// - Constraint (Non-Contact)
			/// >>>>
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



#define SOLVER_CONSTRAINT_VIA_ISLAND 1



#if CONTACT_USE_SOLVERBODY
			Linear1DRow* constraint_solver_rows = mConstraintSolver->GetLinearRowPtr();
			uint32 constraint_solver_row_count = mConstraintSolver->LinearRowCount(); 
			SolverBody* solver_bodies = mConstraintSolver->GetBodiesPtr();

			if (mSettings.solver.warmstart)
			{
				/// perform warm starts
				ConstraintSolver::WarmStart(constraint_solver_rows, 0, constraint_solver_row_count, solver_bodies);
				//fix this bad nested if branches
				if (mSettings.solver.enableContact)
					ContactConstraintSolver::WarmStart(mContactConstraintSolver.ContactConstraintsPtr(), mContactConstraintSolver.NumContactConstraints(), solver_bodies);
			}

#if SOLVER_CONSTRAINT_VIA_ISLAND
			//////solve islands multicore 
			{
				VX_PROFILE_SCOPE("Solving Velocity Constraints");
				const uint32 island_count = mIslandCoordinator->IslandCount();

#define TEST_SOLVER_CONSTRAINT_MT 1
#if TEST_SOLVER_CONSTRAINT_MT
				//for (int i = 0; i < mSettings.solver.velocityIterations; ++i)
				//{
				//	///later use sorted island to solver bigger island first
				//	for (uint32 island = 0; island < island_count; ++island)
				//	{
				//		IslandCoordinator::IslandRange<uint32> constraint_island_indices_range = mIslandCoordinator->IslandNonContactConstraintRowIndicesRange(island);
				//		IslandCoordinator::IslandRange<uint32> contact_constraint_island_indices_range = mIslandCoordinator->ContactConstraintIndicesIslandRange(island);
				//		mTaskCoordinator->ConstructTask([constraint_range = constraint_island_indices_range, constact_constraint_range = contact_constraint_island_indices_range, 
				//		solver_bodies, constraint_solver = mConstraintSolver, contact_constraint_solver = &mContactConstraintSolver, enable_contact = mSettings.solver.enableContact]() {
				//			///solve non constraint first
				//			{
				//				if (constraint_range.begin != nullptr)
				//				{
				//					const uint32 count = constraint_range.end - constraint_range.begin;
				//					VX_ASSERT(count >= 0);
				//					constraint_solver->SolverVelocityLinear1DRowsIndices(constraint_range.begin, count);
				//				}
				//			}


				//			if (enable_contact && constact_constraint_range.begin != nullptr)
				//			{
				//				const uint32 count = constact_constraint_range.end - constact_constraint_range.begin;
				//				VX_ASSERT(count >= 0);
				//				contact_constraint_solver->SolveVelocityConstraint(constact_constraint_range.begin, count, solver_bodies);
				//			}
				//			},0);

				//	}
				//	mTaskCoordinator->WaitForTasks();

				//}

				std::atomic<uint32> next_island = { 0 };

				struct SolvingIslandContext
				{
					ConstraintSolver& constraintSolver;
					ContactConstraintSolver& contactConstraintSolver;
					IslandCoordinator* islandCoordinator;
					const uint32 velocityIterations;
					const bool enableContact;
				};
				SolvingIslandContext solving_island_ctx{ 
					*mConstraintSolver,
					mContactConstraintSolver,
					mIslandCoordinator,
					mSettings.solver.velocityIterations,
					mSettings.solver.enableContact };

				/// create task as the count; thread workers
				for(uint32 i = 0; i < mTaskCoordinator->MaxConcurrency(); ++i)
				{
					mTaskCoordinator->ConstructTask([&solving_island_ctx, &next_island, island_count = mIslandCoordinator->IslandCount()]()
						{

#define VX_SPLIT_ISLAND 1
							const uint32 velocity_iteration_count = solving_island_ctx.velocityIterations;


#if VX_SPLIT_ISLAND
							bool normal_island_complete = false;
							bool large_island_complete = false;
							///NEW+
							for (;;)
							{
								bool worked = false;

								if (large_island_complete && normal_island_complete)
									break;


								auto* solver_bodies = solving_island_ctx.constraintSolver.GetBodiesPtr();

								///try splitting island 
								IslandCoordinator::IslandRange<uint32> island_contact_range(nullptr, nullptr), island_noncontact_range(nullptr, nullptr);
								uint32 solving_split_island_idx;
								uint32 debug_bin = uint32(-1);
								IslandCoordinator::Splitter::EStatus status = solving_island_ctx.islandCoordinator->mSplitter.NextConstactConstraintBatchRange(solving_split_island_idx, island_count, island_contact_range, island_noncontact_range, debug_bin);

								switch (status)
								{
								case vx::IslandCoordinator::Splitter::EStatus::Complete:
									large_island_complete = true;
									break;
								case vx::IslandCoordinator::Splitter::EStatus::WaitingForBatches:
									break;
								case vx::IslandCoordinator::Splitter::EStatus::RetrievedBatch:
								{
									worked = true;
									//solve batch
									if (island_noncontact_range.Valid())
										solving_island_ctx.constraintSolver.SolverVelocityLinear1DRowsIndices(island_noncontact_range.begin, island_noncontact_range.Size());


									if (solving_island_ctx.enableContact && island_contact_range.Valid())
										solving_island_ctx.contactConstraintSolver.SolveVelocityConstraint(island_contact_range.begin, island_contact_range.Size(), solver_bodies);


									/// couple of issues 
									/// 1. range of value, null return 2 as size instead of 1
									/// 2. submitting/marking batch but bin as already processed

									///mark batch as processed 
									uint32 processed_count = island_contact_range.Size();// +island_noncontact_range.Size();
									solving_island_ctx.islandCoordinator->mSplitter.MarkConstactConstraintBatchRangeComplete(solving_split_island_idx, processed_count, velocity_iteration_count, debug_bin);

									continue;
								}
								break;
								default:
									break;
								}




								///solve island as small island
								/// find the first not large island to solve
								/// 
								uint32 normal_island_idx = next_island.load(std::memory_order_relaxed);
								if (normal_island_idx >= island_count)
								{
									normal_island_complete = true;
									continue;
								}

								if (!solving_island_ctx.islandCoordinator->mSplitter.IsIslandLarge(normal_island_idx))
								{
									/// race condition
									if (next_island.compare_exchange_strong(normal_island_idx, normal_island_idx + 1))
									{
										worked = true;
										IslandCoordinator::IslandRange<uint32> constraint_island_indices_range = solving_island_ctx.islandCoordinator->IslandNonContactConstraintRowIndicesRange(normal_island_idx);
										IslandCoordinator::IslandRange<uint32> contact_constraint_island_indices_range = solving_island_ctx.islandCoordinator->ContactConstraintIndicesIslandRange(normal_island_idx);

										for (int i = 0; i < velocity_iteration_count; ++i)
										{
											if (constraint_island_indices_range.Valid())
												solving_island_ctx.constraintSolver.SolverVelocityLinear1DRowsIndices(constraint_island_indices_range.begin, constraint_island_indices_range.Size());


											if (solving_island_ctx.enableContact && contact_constraint_island_indices_range.Valid())
												solving_island_ctx.contactConstraintSolver.SolveVelocityConstraint(contact_constraint_island_indices_range.begin, contact_constraint_island_indices_range.Size(), solver_bodies);

										}
									}
								}
								else
								{
									///increment for next iteration
									/// only increment is some else as not 
									next_island.compare_exchange_strong(normal_island_idx, normal_island_idx + 1);
								}


								//if (large_island_complete && normal_island_complete)
								//	break;

								if (!worked)
								{

								}
							}

#else
							
							for (;;)
							{
								const uint32 island_idx = next_island.fetch_add(1, std::memory_order_relaxed);

								if (island_idx >= island_count)
									break;

								IslandCoordinator::IslandRange<uint32> constraint_island_indices_range = solving_island_ctx.islandCoordinator->IslandNonContactConstraintRowIndicesRange(island_idx);
								IslandCoordinator::IslandRange<uint32> contact_constraint_island_indices_range = solving_island_ctx.islandCoordinator->ContactConstraintIndicesIslandRange(island_idx);

								auto* solver_bodies = solving_island_ctx.constraintSolver.GetBodiesPtr();

								for (int i = 0; i < velocity_iteration_count; ++i)
								{
									if (constraint_island_indices_range.Valid())
										solving_island_ctx.constraintSolver.SolverVelocityLinear1DRowsIndices(constraint_island_indices_range.begin, constraint_island_indices_range.Size());


									if (solving_island_ctx.enableContact && contact_constraint_island_indices_range.Valid())
										solving_island_ctx.contactConstraintSolver.SolveVelocityConstraint(contact_constraint_island_indices_range.begin, contact_constraint_island_indices_range.Size(), solver_bodies);

								}

								//if (next_island.load(std::memory_order_relaxed) >= island_count)
								//	break;
							}
#endif // VX_SPLIT_ISLAND


						}, 0);
				}
				
				mTaskCoordinator->WaitForTasks();

				mIslandCoordinator->mSplitter.ReleaseMemAllocation(mScratchAllocator, mBodyManager.GetNumActiveBodies(), mIslandCoordinator->IslandCount());
#else
				//for (int i = 0; i < mSettings.solver.velocityIterations; ++i)
				//{
				//	///later use sorted island to solver bigger island first
				//	for (uint32 island = 0; island < island_count; ++island)
				//	{
				//		///solve non constraint first
				//		{
				//			IslandCoordinator::IslandRange<uint32> constraint_island_indices_range = mIslandCoordinator->GetIslandNonContactConstraintRowIndices(island);
				//			if (constraint_island_indices_range.begin != nullptr)
				//			{
				//				const uint32 count = constraint_island_indices_range.end - constraint_island_indices_range.begin;
				//				VX_ASSERT(count >= 0);
				//				mConstraintSolver->SolverVelocityLinear1DRowsIndices(constraint_island_indices_range.begin, count, solver_bodies);
				//			}
				//		}


				//		IslandCoordinator::IslandRange<uint32> contact_constraint_island_indices_range = mIslandCoordinator->GetIslandContactConstraintIndices(island);
				//		if (mSettings.solver.enableContact && contact_constraint_island_indices_range.begin != nullptr)
				//		{
				//			const uint32 count = contact_constraint_island_indices_range.end - contact_constraint_island_indices_range.begin;
				//			VX_ASSERT(count >= 0);
				//			mContactConstraintSolver.SolveVelocityConstraint(contact_constraint_island_indices_range.begin, count, solver_bodies);
				//		}
				//	}
				//}

				///later use sorted island to solver bigger island first
				for (uint32 island = 0; island < island_count; ++island)
				{
					for (int i = 0; i < mSettings.solver.velocityIterations; ++i)
					{
						///solve non constraint first
						{
							IslandCoordinator::IslandRange<uint32> constraint_island_indices_range = mIslandCoordinator->IslandNonContactConstraintRowIndicesRange(island);
							if (constraint_island_indices_range.Valid())
								mConstraintSolver->SolverVelocityLinear1DRowsIndices(constraint_island_indices_range.begin, constraint_island_indices_range.Size());
						}


						IslandCoordinator::IslandRange<uint32> contact_constraint_island_indices_range = mIslandCoordinator->ContactConstraintIndicesIslandRange(island);
						if (mSettings.solver.enableContact && contact_constraint_island_indices_range.Valid())
							mContactConstraintSolver.SolveVelocityConstraint(contact_constraint_island_indices_range.begin, contact_constraint_island_indices_range.Size(), solver_bodies);
					}
				}


#endif // TEST_SOLVER_CONSTRAINT_MT

#else
			for (int i = 0; i < mSettings.solver.velocityIterations; ++i)
			{
				ConstraintSolver::SolverVelocityLinear1DRows(constraint_solver_rows, 0, constraint_solver_row_count, solver_bodies);
				//fix this bad nested if branches
				if (mSettings.solver.enableContact)
					mContactConstraintSolver.SolveVelocityConstraint(solver_bodies);
			}

#endif // SOLVER_CONSTRAINT_VIA_ISLAND

			} /// Solving velocity constraints



			//commit solver state to constraint
			if (mSettings.solver.warmstart)
				mConstraintSolver->CommitStateConstraint();

			//write back bodies 
			ConstraintSolver::WriteBackBodies(solver_bodies, mConstraintSolver->GetBodiesCount(), mBodyManager);

#else
			mContactConstraintSolver.SolveVelocityConstraint(mSettings.solver);
			
			mConstraintSolver->SolverAll(mContext, mSettings.solver.velocityIterations);

			///test constraint solving isolating, central solver with solver bodi4es
			//{
			//	VX_PROFILE_SCOPE("Hack Solve Joint Constraints");

			//	for(int i = 0; i < mSettings.solver.velocityIterations; ++i)
			//	{
			//		for (auto& c : mConstraintCoordinator->GetConstraints())
			//		{
			//			DistanceConstraint* _c = static_cast<DistanceConstraint*>(c);
			//			_c->QuickSolve(dt);
			//		}
			//	}
			//}
#endif // CONTACT_USE_SOLVERBODY


			//commit contact constraint state to constraint
			ContactConstraintSolver::WriteBackImplusesManifoldCache(
				mContactConstraintSolver.ContactConstraintsPtr(), mContactConstraintSolver.NumContactConstraints());
		}

#if USE_ISLAND_COORD
		UpdateBodiesIslandActivationState(dt);
#else
		mBodyManager.UpdateBodiesActiveState(dt, mSettings.sleeping); //prevent extra time update
#endif // USE_ISLAND_COORD




		{
			VX_PROFILE_SCOPE("Integrate bodies velocities and clear accumulated force");

			uint32 num_active_bodies = GetNumActiveBodies();
			BodyID* active_bodies = GetActiveBodies();

			for (int i = 0; i < num_active_bodies; ++i)
			{
				Body& body = mBodyManager.GetBody(active_bodies[i]);
				body.IntegrateVelocity(dt);

				//simulation debugger
				mBodyManager.UpdateBodyVelocitySimStat(body);
			}
		}

		if (mSettings.solver.enable)
		{
			VX_PROFILE_SCOPE("Resolve Position Correction");
			float baumgarte = mSettings.solver.baumgarte;
			Constraint** solve_constraint_position = mConstraintSolver->GetConstraintResolvePositionQueuePtr();
			uint32 num_position_constraint = mConstraintSolver->ConstraintResolvePositionQueueCount();
			for (int i = 0; i < mSettings.solver.positionIterations; ++i)
			{
				ConstraintSolver::SolveConstraintsPosition(solve_constraint_position, num_position_constraint, dt, baumgarte);

#if CONTACT_USE_SOLVERBODY
				//fix this bad nested if branches
				if (mSettings.solver.enableContact)
				{
					mContactConstraintSolver.SolvePositionCorrections(
						mConstraintSolver->GetBodiesPtr(), mBodyManager,
						mSettings.solver.baumgarte, mSettings.solver.positionCorrectionSlop,
						mSettings.solver.positionCorrectionGlobalLimits[0], mSettings.solver.positionCorrectionGlobalLimits[1],
						mSettings.solver.positionCorrectionBodyLimitScale);
				}
#endif // CONTACT_USE_SOLVERBODY
			}

#if !CONTACT_USE_SOLVERBODY
			mContactConstraintSolver.SolvePositionConstraint(mSettings.solver);
#endif // !CONTACT_USE_SOLVERBODY

		}

		mConstraintSolver->ReleaseAllocation(mScratchAllocator);

		mContactConstraintSolver.FinaliseStepManifoldCache(mBodyManager);

		PhysicsWorld::mFrameIdx++;
	}


	template<EShapeType Type>
	inline void PhysicsWorld::OnDrawBody(const Body& body, Renderer* draw_renderer, const RenderSettings& settings, const Colour& c)
	{
		///most shapes are 1:2 physics size : rendering size on all axes
		/// with capsule as exception (1:1:1) : (2:1:2)

		
		Mat44 M = Mat44::RotationTranslation(body.Orientation(), body.Position());
		
		if constexpr (Type == EShapeType::Sphere)
		{
			ERenderInstanceFlags flags = settings.sphereInstanceFlags;
			Vec3 scale = body.GetShape()->HalfExtents() * 2.0f;
			M = M.MultiplyAffine(Mat44::Scale(scale));

			draw_renderer->SubmitSpherePrimitive(
				{nullptr, M, mSettings.drawSettings.drawBodiesAsSolid ,
				true, c,  (int(flags & ERenderInstanceFlags::UseTexture) == 0)}, flags);
		}
		else if constexpr (Type == EShapeType::Box)
		{
			ERenderInstanceFlags flags = settings.boxInstanceFlags;
			Vec3 scale = body.GetShape()->HalfExtents() * 2.0f;
			M = M.MultiplyAffine(Mat44::Scale(scale));

			draw_renderer->SubmitCubePrimitive(
				{ nullptr,
				M,
				mSettings.drawSettings.drawBodiesAsSolid , true,
				c, (int(flags & ERenderInstanceFlags::UseTexture) == 0) }, flags);

			//draw_renderer->DrawText3D("Mass", body.Position() + scale, settings.textScale, vx::Colour::sWhite, settings.textAlignment);
		}
		else if constexpr (Type == EShapeType::Capsule)
		{
			///most shapes are 1:2 physics size : rendering size on all axes
			/// with capsule as exception (1:1:1) : (2:1:2)
			
			ERenderInstanceFlags flags = settings.capsuleInstanceFlags;
			Vec3 scale = body.GetShape()->HalfExtents() * Vec3(2.0f, 1.0f, 2.0f);
			M = M.MultiplyAffine(Mat44::Scale(scale));

			draw_renderer->SubmitCapsulePrimitive(
				{ nullptr,M, 
				mSettings.drawSettings.drawBodiesAsSolid , true,
				c,  (int(flags & ERenderInstanceFlags::UseTexture) == 0)}, flags);

			//draw_renderer->DrawText3D("Mass", body.Position() + scale, settings.textScale, vx::Colour::sWhite, settings.textAlignment);
		}
		else if constexpr (Type == EShapeType::Plane)
		{
			ERenderInstanceFlags flags = settings.planeInstanceFlags;
			Vec3 scale = body.GetShape()->HalfExtents() * 2.0f;
			M = M.MultiplyAffine(Mat44::Scale(scale));

			draw_renderer->SubmitQuadXZPrimitive(
				{ nullptr,
				M,
				mSettings.drawSettings.drawBodiesAsSolid , false/*true*/,
				c,  (int(flags & ERenderInstanceFlags::UseTexture) == 0) }, flags);

			//draw_renderer->DrawText3D("Mass", body.Position() + scale, settings.textScale, vx::Colour::sWhite, settings.textAlignment);
		}



	}
	void PhysicsWorld::OnDrawBodies(Renderer* draw_renderer, const RenderSettings& settings)
	{
		VX_PROFILE_FUNCTION();
		VX_ASSERT(draw_renderer, "Draw Renderer is null");


		const bool draw_bodies_with_tex = false;

		using Draw_Func = void(PhysicsWorld::*)(const Body&, Renderer*, const RenderSettings&, const Colour&);
		static const Draw_Func draw_table[4] =
		{
			&PhysicsWorld::OnDrawBody<EShapeType::Sphere>,
			&PhysicsWorld::OnDrawBody<EShapeType::Box>,
			&PhysicsWorld::OnDrawBody<EShapeType::Capsule>,
			&PhysicsWorld::OnDrawBody<EShapeType::Plane>
		};


		/// move later ShapeType
		static const Colour shape_col_type[4] =
		{
			 Colour::sCyan, //sphere
			 Colour::sYellow, //box
			 Colour(0.4f, 1.0f, 0.4f), // capsule light green
			 Colour(0.5f), //plane
		};


		const DrawSettings draw_settings = mSettings.drawSettings;
		//return;
		for (auto it = GetBodies().begin(); it != GetBodies().end(); ++it)
		{
			if (!it->GetID().IsValid())
				continue;

			/// support to colour bodies by 
			/// 1. State: Dynamic(Awake/Sleeping), Static
			/// 2. Collision: Colliding(Dyn-Dyn/Dyn-Static) , Not Colliding
			/// 3. Phases(or final stage): No detect(not in broad), in narrow(pair but fail), Colliding(Dyn-Dyn/Dyn-Static) , Not Colliding
			/// 4. Advance later when support island

			Colour c = Colour::sMagenta;

			if (draw_settings.bodyColourMode == vx::EBodyColourMode::Instances)
				c = Colour::RandomColour(it->GetID().ID());
			else if (draw_settings.bodyColourMode == vx::EBodyColourMode::MotionType)
				c = it->IsDynamic() ? draw_settings.dynamicColour : draw_settings.staticColour;
			else if (draw_settings.bodyColourMode == vx::EBodyColourMode::MotionState)
			{
				c = it->IsDynamic() ?
					(it->IsSleeping() ? draw_settings.sleepingColour : draw_settings.dynamicColour) :
					draw_settings.staticColour;
			}
			else if (draw_settings.bodyColourMode == vx::EBodyColourMode::ShapeType)
				c = shape_col_type[(int)it->GetShape()->Type()];
			else if (draw_settings.bodyColourMode == vx::EBodyColourMode::Phase)
				c = GetBodySimphaseDebugColour(*it);
			else if (draw_settings.bodyColourMode == vx::EBodyColourMode::IslandIdx)
			{
				uint32 island_idx = it->GetIslandIndex();
				if (it->IsStatic())
					c = Colour(0.5f);
				else if (island_idx == Body::kInvalidIslandIdx)
					c = draw_settings.sleepingColour;
				else
					c = Colour::RandomColour(island_idx);
			}
			else if (draw_settings.bodyColourMode == vx::EBodyColourMode::IslandConstraintGroup)
			{
				uint32 island_idx = it->GetIslandIndex();
				if (it->IsStatic())
					c = Colour(0.5f);
				else if (island_idx == Body::kInvalidIslandIdx)
					c = draw_settings.sleepingColour;
				else
				{
					uint32 island_constraint_grp = it->mIslandConstraintGroupMask;


					Vec3 accumulated_colour = Vec3(0.0f);
					uint32 count = 0;
					for (uint32 i = 0; i < 16; ++i)
					{
						//if (Bit32(island_constraint_grp) & Bit32(i))
						if (island_constraint_grp & Bit32(i))
						{
							accumulated_colour += Colour::RandomColour(i);
							count++;
							//break;
						}
					}

					if (count > 0)
						accumulated_colour /= count;

					c = vx::Colour(accumulated_colour);
				}
			}

			c.SetAlpha(draw_settings.bodiesDrawColourAlpha);
			//int shape_enum_idx = (int)(it->GetShape()->Type());
			///// a quick hack to get sphere shape to render with tex 
			///// XOR, since Sphere is 0, so normalise
			//bool has_tex = draw_bodies_with_tex ^ (shape_enum_idx == 0);
			(this->*draw_table[(int)(it->GetShape()->Type())])(*it, draw_renderer, settings, c);



			if (draw_settings.drawBodiesMassText)
			{
				//StackString<8> test = ToStackString<>("%.2f", 2.12345f);
				float inv_mass = it->GetInverseMass();
				StackString<32> mass_text;
				mass_text << "Mass: " << ToStackString("%.2f", (inv_mass) ? (1.0f / inv_mass) : 0.0f).Data() << "kg";// << ToStackString<>("%.2f", 2.12345f).Data();
				//std::string mass_text = "Mass: " + std::to_string((inv_mass) ? (1.0f / inv_mass) : 0.0f) + "kg";
				//mass_text = std::to_string(0.0f);
				Vec3 he = it->GetShape()->HalfExtents();
				he = Vec3::Zero();
				(!settings.useTestDynamicScale) ? draw_renderer->DrawText3D(mass_text.Data(), it->Position() + he, settings.textScale, vx::Colour::sOrange, settings.textAlignment)
					: draw_renderer->DrawText3D_DynScale(mass_text.Data(), it->Position() + he, settings.textScale, vx::Colour::sOrange, settings.textAlignment);
			}
		}
	}

	void PhysicsWorld::OnDebugDraw(DebugGizmosRenderer* debug_renderer)
	{
		VX_PROFILE_FUNCTION();
		VX_ASSERT(debug_renderer, "Debug Renderer is null");
#if VX_DEBUG_DRAW
		mHackDebugRenderer = debug_renderer;
#endif // VX_DEBUG_DRAW



		if (mSettings.drawSettings.nonContactConstraintDrawSettings.drawConstraints)
			mConstraintCoordinator.DebugGizmos(debug_renderer, mSettings.drawSettings.nonContactConstraintDrawSettings);

		//debug_renderer->DrawLine(mExperimentRay.origin, mExperimentRay.End(), Colour::sGreen);

		DrawSettings draw_settings = mSettings.drawSettings;
		if(mBroadphase)
			mBroadphase->DebugDraw(debug_renderer, draw_settings);

		mContactConstraintSolver.DebugDraw(debug_renderer, mConstraintSolver, &mBodyManager, draw_settings);

		if(mSettings.drawSettings.drawDebugInertia)
			QuickDebugDrawInertia(debug_renderer);
		//////DEBUG BOX BOX SAT 
		if(draw_settings.drawAABBContactManifoldInFrame)
		{
			for (const auto& box_box_sat : sBoxVsBoxSATDebugInstances)
			{
				//sample location 
				Vec3 c = vx::Vec3::LoadFloat3Raw(box_box_sat.manifoldCenter);
				//debug_renderer->DrawWireSphereDiscs(c, 0.1, vx::Colour::sMagenta);


				//Draw best axis 
				Vec3 best_axis = Vec3::LoadFloat3Raw(box_box_sat.best_axis);
				debug_renderer->DrawLine(c - best_axis * 2, c + best_axis * 2, vx::Colour::sGreen);

				vx::Vec2 plane_half_extent = box_box_sat.planeSize;
				//vx::Vec3 size_scaled_t0 = vx::Vec3::LoadFloat3Raw(box_box_sat.planeTangent) * plane_half_extent[0] * 2;
				//vx::Vec3 size_scaled_t1 = vx::Vec3::LoadFloat3Raw(box_box_sat.planeBiTangent) * plane_half_extent[1] * 2;


				//// DRAW PLANE
				if(draw_settings.drawAABBContactManifoldInFrameWcPlane)
				{
					///alright let create a proper tangetnt bitange to the normal
					Vec3 contactN = vx::Vec3::LoadFloat3Raw(box_box_sat.contactNormal);
					Vec3 tangentN = contactN.NormalisedPerpendicular();
					Vec3 bitangentN = contactN.Cross(tangentN);
					//ensure orthogonality
					tangentN = bitangentN.Cross(contactN);
					vx::Vec3 size_scaled_t0 = tangentN * plane_half_extent[0] * 2;
					vx::Vec3 size_scaled_t1 = bitangentN * plane_half_extent[1] * 2;

					//time to draw contact plane
					vx::Vec3 vert0 = c + size_scaled_t0 + size_scaled_t1;
					vx::Vec3 vert1 = c + size_scaled_t0 - size_scaled_t1;
					vx::Vec3 vert2 = c - size_scaled_t0 - size_scaled_t1;
					vx::Vec3 vert3 = c - size_scaled_t0 + size_scaled_t1;

					//draw triangles
					/// plane colour changes base on face contact / edge to edge
					Colour plane_col =
						(box_box_sat.best_axis_type == 2) ? Colour::sBlue :
						(box_box_sat.best_axis_type == 3) ? Colour::sDeepTeal : Colour::sCyan;
					debug_renderer->DrawSolidWireTriangle(vert0, vert2, vert1, plane_col);
					debug_renderer->DrawSolidWireTriangle(vert0, vert3, vert2, plane_col);
				}

				//draw ref'ing body 
				BoxShape* ref = box_box_sat.ref;
				if (ref)
				{
					VX_ASSERT_WARN(false, "Trying to use debug info that as incorrect");
					debug_renderer->DrawAABB(ref->ComputeWorldBounds(Mat44::Identity(), vx::Vec3::One()), vx::Colour::sOrange);
				}


				//draw contact normal 
				debug_renderer->DrawArrowCone(c, c + vx::Vec3::LoadFloat3Raw(box_box_sat.contactNormal) * 2, 0.05, 0.1, 0.05, 3, vx::Colour::sYellow);


				///draw all generated points
				for (int i = 0; i < box_box_sat.generatedPt; ++i)
					debug_renderer->DrawWireSphereDiscs(vx::Vec3::LoadFloat3Raw(box_box_sat.pointsGenerated[i]), 0.1f, vx::Colour::sRed);
				////draw contact point on a
				//debug_renderer->DrawWireDisc


				if (box_box_sat.isEdgeDetection)
				{
					vx::Vec3 dir;
					{
						vx::Vec3 eA0 = vx::Vec3::LoadFloat3Raw(box_box_sat.edgeA0);
						vx::Vec3 eA1 = vx::Vec3::LoadFloat3Raw(box_box_sat.edgeA1);

						dir = (eA1 - eA0).Normalised();
						debug_renderer->DrawArrowCone(eA0, eA0 + dir * 2.0f, 0.05, 0.1, 0.05, 3, vx::Colour::sTurquoise);
					}



					vx::Vec3 eB0 = vx::Vec3::LoadFloat3Raw(box_box_sat.edgeB0);
					vx::Vec3 eB1 = vx::Vec3::LoadFloat3Raw(box_box_sat.edgeB1);

					dir = (eB1 - eB0).Normalised();
					debug_renderer->DrawArrowCone(eB0, eB0 + dir * 2.0f, 0.05, 0.1, 0.05, 3, vx::Colour::sDeepTeal);


					debug_renderer->DrawWireSphereDiscs(vx::Vec3::LoadFloat3Raw(box_box_sat.edgeCenter0), 0.1, Colour::sYellow);
					debug_renderer->DrawWireSphereDiscs(vx::Vec3::LoadFloat3Raw(box_box_sat.edgeCenter1), 0.1, Colour::sYellow);
					
				}

			}
		}



		/// Draw bodies properties
		if (draw_settings.drawAABB || draw_settings.drawOBB ||
			draw_settings.drawBodiesPrincipalAxes ||
			draw_settings.drawShapeOrientedBoundCorners || draw_settings.drawBodiesVelocities)
		{
			for (const auto& body : GetBodies())
			{
				if (!body.IsIDValid())
					continue;

				Colour c = ((mBodyManager.GetBodySimStats(body).phase & EBodySimphaseFlags::IsColliding) == EBodySimphaseFlags::IsColliding) ?
						draw_settings.contactWireColour : draw_settings.shapeColliderWireColour;
				if (draw_settings.drawAABB)
				{
					AABB bounds = body.GetAABBWorld();
					debug_renderer->DrawAABB(bounds.mMin, bounds.mMax, c);
				}
				if (draw_settings.drawOBB)
				{
					OBB obb = OBB(body.GetShape()->LocalBounds(), body.Orientation());
					debug_renderer->DrawBox(obb.ComputeCorners(body.Position()), c);
				}


				if (draw_settings.drawBodiesPrincipalAxes)
				{

					Vec3 start = body.Position();


					Vec3 half_extent = body.GetShape()->HalfExtents();
					if (body.GetShape()->Type() == EShapeType::Plane) half_extent.SetY(0.0f);

					Vec3 x_axis = body.Orientation().Rotate(Vec3(1.0f, 0.0f, 0.0f));
					debug_renderer->DrawLine(start,
						(start + x_axis * half_extent.X()),
						Colour(1.0f, 0.0f, 0.0f));

					Vec3 y_axis = body.Orientation().Rotate(Vec3(0.0f, 1.0f, 0.0f));
					debug_renderer->DrawLine(start,
						(start + y_axis * half_extent.Y()),
						Colour(0.0f, 1.0f, 0.0f));

					Vec3 z_axis = body.Orientation().Rotate(Vec3(0.0f, 0.0f, 1.0f));
					debug_renderer->DrawLine(start,
						(start + z_axis * half_extent.Z()),
						Colour(0.0f, 0.0f, 1.0f));
				}

				if (draw_settings.drawBodiesVelocities)
				{
					const Vec3 center = body.Position();

					const Vec3 lin_vel = body.GetLinearVelocity();
					const Vec3 ang_vel = body.GetAngularVelocity();

					debug_renderer->DrawArrowCone(center, center + lin_vel, 0.2f, 0.5f, 0.2f, 4, draw_settings.bodyLinearVelocityCol);
					debug_renderer->DrawArrowCone(center, center + ang_vel, 0.2f, 0.5f, 0.2f, 4, draw_settings.bodyAngularVelocityCol);

				}


				if (draw_settings.drawShapeOrientedBoundCorners)
				{
					Colour col = draw_settings.drawShapeCornersColour;
					Quat q = body.Orientation();

					OBB obb = OBB(body.GetShape()->LocalBounds(), q);

					Vec3 axis_x = q.RotateAxisX();
					Vec3 axis_y = q.RotateAxisY();
					Vec3 axis_z = q.RotateAxisZ();
					Vec3 center = body.Position();

					auto& corners = obb.ComputeCorners(center);
					///debug_renderer->DrawBox(corners, Colour::sMagenta);

					for (const auto& c : corners)
					{
						debug_renderer->DrawLine( 
							(c - Vec3(draw_settings.drawContactPointSize, 0.0f, 0.0f)),
							(c + Vec3(draw_settings.drawContactPointSize, 0.0f, 0.0f)),
							col);
						debug_renderer->DrawLine(
							(c - Vec3(0.0f, draw_settings.drawContactPointSize, 0.0f)),
							(c + Vec3(0.0f, draw_settings.drawContactPointSize, 0.0f)),
							col);

					}
				}

			}
		}


		

	}

	void PhysicsWorld::QuickDebugDrawInertia(DebugGizmosRenderer* debug_renderer)
	{
		VX_PROFILE_FUNCTION();
		for(const auto& body : mBodyManager.GetBodies())
		{
			if (body.GetInverseMass() <= 0.0f)
				continue;

			float mass = 1.0f / body.GetInverseMass();
			Vec3 com = body.Position();
			Quat q = body.Orientation();

			Vec3  rt = q * Vec3::Right();
			Vec3  up = q * Vec3::Up();
			Vec3  fwd = q * Vec3::Forward();

			Vec3 I_local = body.GetLocalInvInertiaDiagonal().Reciprocal();

			float axis_len = 1.0f;
			//principal axes
			debug_renderer->DrawLine(com, com + rt * axis_len, Colour::sRed);
			debug_renderer->DrawLine(com, com + up * axis_len, Colour::sGreen);
			debug_renderer->DrawLine(com, com + fwd * axis_len, Colour::sBlue);


			///equivalent inertia box
			float w = VxSqrt(VxMax(0.0f, 6.0f * (I_local.Y() + I_local.Z() - I_local.X()) / mass));
			float h = VxSqrt(VxMax(0.0f, 6.0f * (I_local.X() + I_local.Z() - I_local.Y()) / mass));
			float d = VxSqrt(VxMax(0.0f, 6.0f * (I_local.X() + I_local.Y() - I_local.Z()) / mass));

			float h_w = w * 0.5f;
			float h_h = h * 0.5f;
			float h_d = d * 0.5f;

			Vec3 diag = I_local * (12.0f / mass);
			Vec3 _size = Vec3(VxSqrt(0.5f * (-diag[0] + diag[1] + diag[2])),
				VxSqrt(0.5f * (diag[0] - diag[1] + diag[2])),
				VxSqrt(0.5f * (diag[0] + diag[1] - diag[2])));

			OBB obb = OBB(AABB(-0.5f * _size, 0.5f * _size), q);
			//debug_renderer->DrawBox(aabb.GetOBB(Mat44::RotationTranslation(q, com)), Colour::sYellow);
			debug_renderer->DrawBox(obb.ComputeCorners(com), mSettings.drawSettings.drawDebugInertiaColour);


			////momentum
			//	body.GetPointVelocityRelCOM
			Vec3 ang_vel = body.GetAngularVelocity();
			Vec3 local_omega(ang_vel.Dot(rt), ang_vel.Dot(up), ang_vel.Dot(fwd));
			Vec3 local_L(local_omega.X() * I_local.X(),
				local_omega.Y() * I_local.Y(),
				local_omega.Z() * I_local.Z());

			Vec3 world_L = (rt * local_L.X()) + (up * local_L.Y()) + (fwd * local_L.Z());
			float momentum_scale = 0.5f;
			debug_renderer->DrawLine(com, com + world_L * momentum_scale, Colour::sMagenta);
		}
	}

	const CollisionResolutionStat PhysicsWorld::GetNarrowphaseStats() const
	{
		return mNarrowphaseQuery.Stats();
	}



	void PhysicsWorld::UpdateBodiesIslandActivationState(float dt)
	{
		/// citeria to put bodies to sleep 
		/// for island to sleep all its bodies need to meet this citeria 
		/// 
		
		/// i.e if a body in island is a wake all is awake 

		///or maybe 2 loops 

		
		/// 1. check island state
		/// 2. update bodies in islands
		///
		if(mSettings.sleeping.enable)
		{
			//auto& islands = mIslandCoordinator->islands;

			std::vector<uint32> islands_to_sleep;
			std::vector<uint32> islands_awake;

			for (uint32 island = 0; island < mIslandCoordinator->IslandCount(); ++island)
			{
				IslandCoordinator::IslandRange<BodyID> body_island = mIslandCoordinator->IslandBodyIDsRange(island);

				bool put_to_sleep = true;

				for (const BodyID* body_id = body_island.begin; body_id < body_island.end; ++body_id)
				{
					Body& body = mBodyManager.GetBody(*body_id);
					//if (mSettings.sleeping.enable)

					body.UpdateSleepState(dt, mSettings.sleeping);

					if (body.IsAwake())
					{
						put_to_sleep = false;
						break;
					}
				}
				if (put_to_sleep)
					islands_to_sleep.push_back(island);
				else
					islands_awake.push_back(island);
			}

			//// haxk for bodies in the awake list force wake up 
			/// as some bodies are trying to sleep 
			for (auto& island_idx : islands_awake)
			{
				IslandCoordinator::IslandRange<BodyID> body_island = mIslandCoordinator->IslandBodyIDsRange(island_idx);
				for (const BodyID* body_id = body_island.begin; body_id < body_island.end; ++body_id)
				{
					Body& body = mBodyManager.GetBody(*body_id);
					body.WakeUp(false);
				}
			}

		}


		/// re-add bodies to active list
		mBodyManager.ResetActivateBodies();
		mBodyManager.AddBodiesToActivate(false);
	}

	Colour PhysicsWorld::GetBodySimphaseDebugColour(const Body& body) const
	{
		auto flags = mBodyManager.GetBodySimStats(body).phase;

		const bool is_colliding = Contains(flags, EBodySimphaseFlags::IsColliding);
		const bool touching_static = Contains(flags, EBodySimphaseFlags::IsTouchingStatic);
		const bool in_narrow = Contains(flags, EBodySimphaseFlags::InNarrowphase);
		const bool in_broad = Contains(flags, EBodySimphaseFlags::InBroadphase);

		const DrawSettings draw_settings = mSettings.drawSettings;
		if (is_colliding)
		{
			return Colour(1.0f, 0.5f, 0.0f);


			/// ignore dyn-sta collision
			/// later deep penetration solver 
			//if (touching_static)
			//{
			//	return body.IsDynamic() ?
			//		draw_settings.dynamicStaticCollisionColour :
			//		draw_settings.staticColour;
			//}
			//return body.IsDynamic() ?
			//	draw_settings.dynamicColour :
			//	draw_settings.staticColour;
		}


		if (in_narrow) return draw_settings.narrowPhaseColour;
		if (in_broad) return draw_settings.broadPhaseColour;
		return draw_settings.broadPhaseColour;// draw_settings.neuralPhaseColour;

	}

}