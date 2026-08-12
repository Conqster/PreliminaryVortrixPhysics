#pragma once

#include "Vortrix/Core/Core.h"
#include "Vortrix/Core/Logger.h"

namespace vx {

	class Body;


	enum class ECombineMode : uint8
	{
		Multiply, 
		SquareRoot,
		Average,
		Minimum, 
		Maximum,


		/// more predefined mode
		/// function callback 
		/// float(*)(const Body&, const Body&); see "CombinedCoefficient::Func" & "CombinedCoefficient::RegisterInternals()" for implementation
		/// Add to registery CombinedCoeffiecient::"RegisterFrictionCombine/RegisterRestitutionCombine"
		/// RegisterFrictionCombine<CombineMode::Multiply>([](const Body& b0, const Body& b1) {return b0.FrictionCoeff() * b1.FrictionCoeff(); });
		Count
	};

	constexpr const char* CoefficientCombineModeNames = "Multiply\0""SquareRoot\0""Average\0""Minimum\0""Maximum\0""\0";





	/// Quick fix to this make the FrictionCoeff and Restitution 
	/// be a switch case; for defau\lr internal supported 
	/// 
	/// and the custom code use use the table to look up fucntion callback 
	/// internal does not need callbacks 
	class CombinedCoefficient
	{
	public:
		//template<CombineMode Mode>
		static float GetFriction(ECombineMode Mode, const Body& body0, const Body& body1);
		//template<CombineMode Mode>
		static float GetRestitution(ECombineMode Mode, const Body& body0, const Body& body1);

#pragma region 
	//	CombinedCoefficient() = default
	//	{
	//		for (int i = 0; i < int(CombineMode::Count); ++i)
	//		{
	//			mFrictionCombineFuncs[i] = &InvalidCombineCoeffFn;
	//			mRestitutionCombineFuncs[i] = &InvalidCombineCoeffFn;
	//		}
	//		RegisterInternals();
	//	}

	//	static VX_INLINE CombinedCoefficient& Get()
	//	{
	//		static CombinedCoefficient inst;
	//		return inst;
	//	}

	//	using Func = float(*)(const Body&, const Body&);

	//	template<CombineMode Mode>
	//	VX_INLINE void RegisterFrictionCombine(Func func)
	//	{
	//		mFrictionCombineFuncs[int(Mode)] = func;
	//	}

	//	template<CombineMode Mode>
	//	VX_INLINE void RegisterRestitutionCombine(Func func)
	//	{
	//		mRestitutionCombineFuncs[int(Mode)] = func;
	//	}

	//private:


	//	void RegisterInternals();

	//	Func mFrictionCombineFuncs[int(CombineMode::Count)];
	//	Func mRestitutionCombineFuncs[int(CombineMode::Count)];

	//	static float InvalidCombineCoeffFn(const Body& b0, const Body& b1) { /*VX_LOG_DEBUG("No specilisation for mode yet"); */ VX_WARN("No specilaisation for this mode yet"); return 0.4f; };


	//	struct Reg
	//	{
	//		Reg()
	//		{
	//			VX_LOG_DEBUG("init Combined coefficent");
	//		}
	//	};
	//	static inline Reg __;
#pragma endregion
	};
}