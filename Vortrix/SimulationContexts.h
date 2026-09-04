#pragma once

#include "Vortrix/PhysicsWorldSettings.h"

namespace vx {

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

		std::atomic<uint32> solvePositionNextIslandSortedIdx = { 0 };




		struct SimulationStepProfilingData
		{
			std::atomic<uint32> writeStepTotalJacobianSolved{ 0 };
			uint32 total_jacobian_solved[12]; //<-- later max thread or hardware concurrency
		};

		SimulationStepProfilingData mSimStepProfiling;


		///probably hold pointer to active constraint (non contact)
		//Constraint** constraints
	};



	struct VelocitySolveProfile
	{
		uint32 total_jacobian_solved[12]{ 0 }; //<-- later max thread or hardware concurrency
		/// write on main thread, zero data races
		uint32 sampleCount = 0;
		uint32 contributionSampleCount[12] = { 0 };

		double totalLoadBalanceEff = 0;

		void ResetAccumulation()
		{
			for (uint32 i = 0; i < 12; ++i)
			{
				total_jacobian_solved[i] = 0;
				contributionSampleCount[i] = 0;
			}

			sampleCount = 0;
			totalLoadBalanceEff = 0.0;
		}
	};



}