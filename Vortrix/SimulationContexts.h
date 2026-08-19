#pragma once

#include "Vortrix/PhysicsWorldSettings.h"

namespace vx {
	//struct CollisionContext
	//{
	//	const CollisionSettings& settings;
	//	DebugGizmosRenderer* debugRenderer = nullptr;
	//	bool drawContactTBNs = false;
	//	uint32 physicsFrameIdx = 0;

	//	uint32 broadphasePairCount = 0;

	//	//might become island builder/coordinator
	//	class ConstraintSolver* constraintSolver = nullptr;
	//	class IslandCoordinator* islandCoordinator = nullptr;
	//	class BodyManager* bodyManager = nullptr;
	//	//class PhysicsWorld* physicsWorld = nullptr;
	//};


	struct SimStep
	{
		/// kProcessBodyPairBatch
		/// broadphase Body pair batch count per thread processing 
		/// in narrowphase
		static constexpr uint32 kProcessBodyPairBatch = 32;
		static constexpr uint32 kAcclerationGravityTaskBatch = 64;
		static constexpr uint32 kVelocityIntergrationTaskBatch = 64;

		PhysicsStepContext* mPhysicsStepContext = nullptr;

		/// fix for debugging
		//struct BroadphaseBuffer
		//{
		//	BroadphasePair* data = nullptr;
		//	uint32 count = 0;
		//	uint32 maxPairs = 0;
		//}mBroadphaseBuffer;

		struct BroadphasePair* broadphasePair = nullptr;
		uint32 broadphasePairCount = 0;

		std::atomic<uint32> nextProcessPairIdx = 0;

		///island 
		std::atomic<uint32> solveVelocityNextIslandIdx = { 0 };


		///probably hold pointer to active constraint (non contact)
		//Constraint** constraints
	};

}