#pragma once

#include "Core.h"


#if VX_PROFILING
#define VX_ENABLE_PROFILING
#endif // VX_PROFILING


#ifdef VX_ENABLE_PROFILING
#include <chrono>
#include "Logger.h"
#include "EProfileMode.h"
#include <unordered_map>
#endif // VPHX_ENABLE_PROFILING
				

namespace vx
{
	namespace Profiler
	{
#ifdef VX_ENABLE_PROFILING

		struct ProfileData
		{
			uint64 samples = 0;
			float lastMs = 0.0f;
			float avgMs = 0.0f;
			float maxMs = 0.0f;
		};

		class ProfilerCollector
		{
		public:
			static ProfilerCollector& Instance()
			{
				static ProfilerCollector inst;
				return inst;
			}


			void AddSample(const std::string_view& name, float ms)
			{
				ProfileData& data = mProfiles[name];
				data.lastMs = ms;
				data.avgMs = data.avgMs * 0.9 + ms * 0.1;
				data.samples++;
				if (ms > data.maxMs)
					data.maxMs = ms;
			}

			const std::unordered_map<std::string_view, ProfileData>& GetProfiles() const { return mProfiles; }

		private:
			std::unordered_map<std::string_view, ProfileData> mProfiles;
		};



		class TimeTaken
		{
		public:
			explicit TimeTaken(const char* name,
				float* duration,
				bool log,
				EProfileMode mode = EProfileMode::Deterministic) :
				mStart({}),
				mName(name),
				mDurationMS(duration),
				bLog(log),
				mSampleRate(sDefaultGlobalSampleRate),
				bActive(false),
				mMode(mode)
			{
				Query();
			}

			//TimeTaken(const char* name,
			//	float* duration,
			//	bool log,
			//	EProfileMode mode = EProfileMode::Deterministic) :
			//	mStart({}),
			//	mName(name),
			//	mDurationMS(duration),
			//	bLog(log),
			//	mSampleRate(sDefaultGlobalSampleRate),
			//	bActive(false),
			//	mMode(mode)
			//{
			//	Query();
			//}
			explicit TimeTaken(const char* name,
				float* duration, EProfileMode mode = EProfileMode::Deterministic) :
				mStart({}),
				mName(name),
				mDurationMS(duration),
				bLog(true),
				mSampleRate(sDefaultGlobalSampleRate),
				bActive(false),
				mMode(mode)
			{
				Query();
			}

			explicit TimeTaken(const char* name,
				uint32_t sample_rate, EProfileMode mode = EProfileMode::Deterministic) :
				mStart({}),
				mName(name),
				mDurationMS(nullptr),
				bLog(true),
				mSampleRate(sample_rate),
				bActive(false),
				mMode(mode)
			{
				Query();
			}


			explicit TimeTaken(const char* name, EProfileMode mode = EProfileMode::Deterministic) :
				mStart({}),
				mName(name),
				mDurationMS(nullptr),
				bLog(true),
				mSampleRate(sDefaultGlobalSampleRate),
				bActive(false),
				mMode(mode)
			{
				Query();
			}

			//void Query(const char* name, float* duration, bool b_log, 
			//	EProfileMode mode, uint32_t sample_rate) 
			void Query()
			{
				//Only profile if mode matches filter
				if (!Contains(sActiveFilters, mMode))
				{
					bActive = false;
					return;
				}

				switch (mMode)
				{
				case EProfileMode::Deterministic:
					//increment the counter for this scope
					bActive = (++sScopeCounter[mName] % mSampleRate == 0);
					break;

				case EProfileMode::VariableRate:
					auto now = Clock::now();
					float elapsed = std::chrono::duration<float>(now - sLastSampleTime[mName]).count();
					if (elapsed > sSampleIntervalSeconds)
					{
						sLastSampleTime[mName] = now;
						bActive = true;
					}
					break;
				}

				if (bActive)
					mStart = Clock::now();

			}

			~TimeTaken()
			{
				if (!bActive)
					return;

				auto end = Clock::now();
				float ms = std::chrono::duration<float, std::milli>(end - mStart).count();

				if (mDurationMS)
					*mDurationMS = ms;
				if (sAllowConsoleLog && bLog)
					VX_LOG_INFO(mName, " - time: ", ms, "ms.");

				ProfilerCollector::Instance().AddSample(mName, ms);
			}

			//set global rate
			static void SetDefaultGlobalSampleRate(const uint32_t value) { sDefaultGlobalSampleRate = VxMax(value, uint(1)); }
			static void SetSampleIntervalSeconds(const float value) { sSampleIntervalSeconds = value; }
			static void SetProfileFilter(EProfileMode filter) { sActiveFilters = filter; }
			static void SetAllowConsoleLog(bool value) { sAllowConsoleLog = value; }

		private:
			using Clock = std::chrono::steady_clock;
			//using ClockDuration = std::chrono::duration;
			std::chrono::time_point<Clock> mStart;
			const char* mName;
			float* mDurationMS = nullptr;
			bool bLog = true;
			uint32_t mSampleRate = 1;
			bool bActive = false;
			EProfileMode mMode = EProfileMode::Deterministic;


			//static counter Per scope counter
			inline static std::unordered_map<std::string_view, uint32_t> sScopeCounter;
			inline static std::unordered_map<std::string_view, Clock::time_point> sLastSampleTime;
			inline static uint32_t sDefaultGlobalSampleRate = 30; //every 30 physics step
			inline static float sSampleIntervalSeconds = 0.5f;
			inline static EProfileMode sActiveFilters = EProfileMode::All;
			inline static bool sAllowConsoleLog = true;
		}; //TimeTaken struct


		//#define CORE_FUNCTION_NAME __FUNCTION__
		//argument scope name, 2nd arg memory address (float) to write scope duration
#define VX_SELECT_2(arg1, arg2, TARGET_MACRO, ...) TARGET_MACRO

#define VX_PROFILER_FUNCTION_NO_PARAM(name, mode) \
	Profiler::TimeTaken scope_time(name, mode)

#define VX_PROFILER_FUNCTION_WITH_PARAM(name, mode, ...) \
	Profiler::TimeTaken scope_time(name, __VA_ARGS__, mode)



#define VX_PROFILE_SCOPE(name, ...) \
	VX_SELECT_2(name, VX_PROFILER_FUNCTION_NO_PARAM, VX_PROFILER_FUNCTION_WITH_PARAM) \
	(name, Profiler::EProfileMode::Deterministic, __VA_ARGS__) 


#define VX_PROFILE_FUNCTION(...) \
	EXPAND_MACRO(VX_SELECT_2(__FUNCTION__, VX_PROFILER_FUNCTION_NO_PARAM, VX_PROFILER_FUNCTION_WITH_PARAM) \
	(__FUNCTION__, Profiler::EProfileMode::Deterministic, __VA_ARGS__) )



#define VX_VARIABLE_PROFILE_SCOPE(name, ...) \
	VX_SELECT_2(name, VX_PROFILER_FUNCTION_NO_PARAM, VX_PROFILER_FUNCTION_WITH_PARAM) \
		(name, Profiler::EProfileMode::VariableRate, __VA_ARGS__)

#define VX_VARIABLE_PROFILE_FUNCTION(...) \
	VX_SELECT_2(__FUNCTION__, VX_PROFILER_FUNCTION_NO_PARAM, VX_PROFILER_FUNCTION_WITH_PARAM) \
		(__FUNCTION__, Profiler::EProfileMode::VariableRate, __VA_ARGS__)


#define SET_VX_PROFILER_GLOBAL_SAMPLE_RATE(rate) Profiler::TimeTaken::SetDefaultGlobalSampleRate(rate)
#define SET_VX_PROFILER_SAMPLE_INTERVAL_SECONDS(rate) Profiler::TimeTaken::SetSampleIntervalSeconds(rate)
#define SET_VX_PROFILER_FILTER(filter) Profiler::TimeTaken::SetProfileFilter(filter)
#define SET_VX_PROFILER_ALLOW_CONSOLE_LOG(value) Profiler::TimeTaken::SetAllowConsoleLog(value)

#define VX_PROFILER_DETERMINISTIC Profiler::EProfileMode::Deterministic
#define VX_PROFILER_VARIABLERATE Profiler::EProfileMode::VariableRate
#else
#define VX_PROFILE_SCOPE(...) ((void)0)
#define VX_PROFILE_FUNCTION(...) ((void)0)
#define PROFILE_FUNCTION_SAMPLE(...) ((void)0)
#define SET_VX_PROFILER_GLOBAL_SAMPLE_RATE(rate) ((void)0)
#define SET_VX_PROFILER_SAMPLE_INTERVAL_SECONDS(rate) ((void)0)
#define SET_VX_PROFILER_FILTER(filter) ((void)0)
#define SET_VX_PROFILER_ALLOW_CONSOLE_LOG(value) ((void)0)

#define VX_PROFILER_DETERMINISTIC 0
#define VX_PROFILER_VARIABLERATE 0
#endif // VX_ENABLE_PROFILING
	} //Profiler namespace
} //vx namespace