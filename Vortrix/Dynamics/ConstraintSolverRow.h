#pragma once
#include "SolverBodyIndex.h"
#include "Vortrix/Maths/Vec3.h"
#include "Vortrix/Maths/Float3.h"

#include "Body/Body.h"

namespace vx {

	struct ConstraintRowInfo
	{
		/// lower 24 bits 0x00ffffff is the constraint the row partipate in represent
		/// the 1 bit after 24 bit (25 bits) is flag for is the constraint needs/wants position correction
		/// 
		/// 7 bits up is details of the jacobian linear row group in solving consrtraint 
		/// a constraint could have 1 - 6 linear rows, depending on the num of degree of freedom removed 
		/// first lower 3 of 7 bits is the index of row in local group (bit shift 25)
		/// next 4 bits id the total row in local group (bit shift 28)
		static constexpr uint32 kConstraintIndexMask = 0x00ffffff;
		static constexpr uint32 kPositionCorrectionMask = 0x01000000;//0x01;
		static constexpr uint32 kRowIndexMask = 0x0e000000;
		static constexpr uint32 kRowCountMask = 0xf0000000;

		static constexpr uint32 kPositionCorrectionBitShift = 24;
		static constexpr uint32 kRowIndexBitShift = 25;
		static constexpr uint32 kRowCountBitShift = 28;

		ConstraintRowInfo(uint32 constraint_idx,
			uint8 row_idx, uint8 row_count,
			bool position_correction) : data(
				(uint32(row_count) << kRowCountBitShift) |
				(uint32(row_idx) << kRowIndexBitShift) |
				(uint32(position_correction) << kPositionCorrectionBitShift) |
				constraint_idx) {
		}
		ConstraintRowInfo() = default;

		uint32 ConstraintIndex() const { return data & kConstraintIndexMask; }
		bool NeedPositionCorrection() const { return (data & kPositionCorrectionMask) != 0; }

		///mask out lower, it belongs to solve position flag 
		uint8 RowLocalIndex() const { return uint8((data & kRowIndexMask) >> kRowIndexBitShift); }
		uint8 RowCount() const { return uint8((data & kRowCountMask) >> kRowCountBitShift); }

	private:
		uint32 data;
	};
	static_assert(std::is_trivial_v<ConstraintRowInfo>);

	/// i think a quick and dirty solution after set up most of the data are read only
	/// then i would grouop the datas together and surpiseingly this struct is read only over multiple required iteration; 
	/// because of the SolverBody index; only lambda is update inbetween velocity iterations
	struct alignas(16) Linear1DRow
	{
		SolverBodyIndex bodyAidx;	/// later change to SolverBody only caches required data 
		SolverBodyIndex bodyBidx;	/// like position, velocities before write back, and constraint stores actual BodyID

		float effMass;
		float gamma;

		Float3 axis;
		float bias;

		/// rAXn = rA.Cross(nor);
		Float3 rAXn;
		float lambda;
		
		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);
		Float3 invIrAXn;
		float minLambda;

		/// rAXn = rA.Cross(nor);
		Float3 rBXn;	/// rAXn = rA.Cross(nor);
		float maxLambda;
		
		///invIrBXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn
		Float3 invIrBXn;

		ConstraintRowInfo info;
	};
	static_assert(std::is_trivial_v<Linear1DRow>, "Linear1DRow Must be a trivial type!");



	struct Rigid1DConstraint
	{
		Vec3 axis;

		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);
		Float3 invIrAXn;	

		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn
		Float3 invIrBXn;


		float effMass;

		VX_INLINE void SolvePosition(Body& body0, Body& body1, float C, float baumgarte)
		{
			float lambda = -effMass * baumgarte * C;

			if (body0.IsDynamic())
			{
				Vec3 x = lambda * body0.InverseMass() * axis;
				body0.ApplyLinearDisplacement(-x);
				body0.ApplyAngularDisplacement(-lambda * Vec3::LoadFloat3Raw(invIrAXn));
			}
			if (body1.IsDynamic())
			{
				Vec3 x = lambda * body1.InverseMass() * axis;
				body1.ApplyLinearDisplacement(x);
				body1.ApplyAngularDisplacement(lambda * Vec3::LoadFloat3Raw(invIrBXn));
			}
		}
	};



} //namespace vx