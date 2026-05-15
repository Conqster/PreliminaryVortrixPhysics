#pragma once


#include <fstream>
#include <sstream>
#include <iostream>

#include "ELogBuffer.h"

#include "Core.h"

#include <ctime>
#include <time.h>

	namespace vx
	{
		enum class ELogLevel : uint8_t
		{
			Debug,
			Info,
			Warning,
			Error,
		};

		constexpr const char* kDefaultLogpath = "Logs/Vortrix.log";

		constexpr const char* kAnsiRedColour = "\033[1;31m";
		constexpr const char* kAnsiGreenColour = "\033[1;32m";
		constexpr const char* kAnsiYellowColour = "\033[1;33m";
		constexpr const char* kAnsiBlueColour = "\033[1;34m";
		constexpr const char* kAnsiCyanColour = "\033[1;36m";
		constexpr const char* kAnsiResetColour = "\033[0m";


	class Logger
	{
	public:
		static Logger& Instance()
		{
			static Logger inst;
			return inst;
		}

		template<typename... Args>
		inline void Log(ELogLevel lvl, Args&&... args); //bring back inline

		/**
		* Helpers
		* logs debug-level message for internal development insight.
		* VX_DEBUG("Intergrating particle ", i, ": pos= ", pos, " vel=", vel);*/
		template<typename... Args>
		//void LogDebug(Args&&... args) { Log(ELogLevel::Debug, std::forward<Args>(args)...); }
		void LogDebug(Args&&... args) { Log(ELogLevel::Debug, (args)...); }
		/// logs an info-level message for programming event.
		/// Usage: VX_INFO("Physics world initialised: ", body_count, " bodies.");
		template<typename... Args>
		void LogInfo(Args&&... args) { Log(ELogLevel::Info, (args)...); }
		/// logs an warning-level message for soft recoverable issue.
		/// Usage: VX_WARN("Particle mass was zero, clamped to 1.0f");
		template<typename... Args>
		void LogWarn(Args&&... args) { Log(ELogLevel::Warning, (args)...); }
		/// logs an error-level message for serious issue or breakage.
		//VX_ERROR("Null particle body pointer in collision detection!");
		template<typename... Args>
		void LogError(Args&&... args) { Log(ELogLevel::Error, (args)...); }


		ELogLevel GetMinLevel() const { return mMinLevel; }
		ELogBuffer GetLogTragetBuffer() const { return mTargetBuffer; }
		bool GetLogTimestamp() const { return bLogTimestamp; }

		void SetLevel(const ELogLevel& lvl) { mMinLevel = lvl; }
		void SetLogTragetBuffer(ELogBuffer target_buff) { mTargetBuffer = target_buff; }
		inline void SetFileLogging(const std::string& filename, bool append);
		void SetLogTimestamp(bool value) { bLogTimestamp = value; }

	private:
		Logger();
		~Logger();

		[[nodiscard]] static constexpr const char* LevelToChar(ELogLevel lvl) noexcept;
		[[nodiscard]] static constexpr const char* LevelAnsiColour(ELogLevel lvl) noexcept;

		void WriteLine(ELogLevel lvl, const char* data, size_t len);
		void TryOpenFile();

		ELogLevel mMinLevel = ELogLevel::Debug;
		std::string mFilePath{};
		std::ofstream mFile{};
		ELogBuffer mTargetBuffer = ELogBuffer::Console | ELogBuffer::File;
		bool bAppendToFile = false;
		bool bLogTimestamp = true;
	};
}//namespace vx
	
#include "Logger.inl"



#if defined(VX_DEBUG) || defined(VX_DEV)
	/// logs debug-level message for internal development insight.
	/// Usage: VX_DEBUG("Intergrating particle ", i, ": pos= ", pos, " vel=", vel);
	#define VX_LOG_DEBUG(...) vx::Logger::Instance().LogDebug(__VA_ARGS__)
#else
	#define VX_LOG_DEBUG(...) ((void)0)
#endif // defined(VX_DEBUG) || defined(VX_DEV)

#if defined(VX_DIST)
	#define VX_LOG_INFO(...) ((void)0)	
	#define VX_LOG_WARN(...) ((void)0)	
	#define VX_LOG_ERROR(...) ((void)0)	
#else
	/// logs an info-level message for programming event.
	/// Usage: VX_INFO("Physics world initialised: ", body_count, " bodies.");
	#define VX_LOG_INFO(...) vx::Logger::Instance().LogInfo(__VA_ARGS__)

	/// logs an warning-level message for soft recoverable issue.
	/// Usage: VX_WARN("Particle mass was zero, clamped to 1.0f");
	#define VX_LOG_WARN(...) vx::Logger::Instance().LogWarn(__VA_ARGS__)

	/// logs an error-level message for serious issue or breakage.
	//VX_ERROR("Null particle body pointer in collision detection!");
	#define VX_LOG_ERROR(...) vx::Logger::Instance().LogError(__VA_ARGS__)
#endif // defined(VX_DIST)


#define VX_LOG_DEBUG_LEVEL vx::ELogLevel::Debug
#define VX_LOG_INFO_LEVEL vx::ELogLevel::Info
#define VX_LOG_WARNING_LEVEL vx::ELogLevel::Warning
#define VX_LOG_ERROR_LEVEL vx::ELogLevel::Error
#define VX_LOG_SET_LEVEL(lvl) Logger::Instance().SetLevel(lvl)
