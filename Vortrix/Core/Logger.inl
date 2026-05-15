#include "Logger.h"
#include "StackString.h"

namespace vx
{

	template<typename... Args>
	void Logger::Log(ELogLevel lvl, Args&&... args)
	{
		if (lvl < mMinLevel || mTargetBuffer == ELogBuffer::None)
			return;

		StackString<512> buff;
		((buff << std::forward<Args>(args)), ...);

		if (!buff.Empty())
			WriteLine(lvl, buff.Data(), buff.Length());
	}


	void Logger::SetFileLogging(const std::string& filename, bool append = false)
	{
		mFilePath = filename + ".log";
		bAppendToFile = append;
		TryOpenFile();
	}

	constexpr const char* Logger::LevelToChar(ELogLevel lvl) noexcept
	{
		switch (lvl)
		{
		case ELogLevel::Debug:	return "DEBUG";
		case ELogLevel::Info:	return "INFO";
		case ELogLevel::Warning:return "WARNING";
		case ELogLevel::Error:	return "ERROR";
		default:				return "?    ";
		}
	}

	constexpr const char* Logger::LevelAnsiColour(ELogLevel lvl) noexcept
	{
		switch (lvl)
		{
		case ELogLevel::Debug:	return kAnsiCyanColour;
		case ELogLevel::Info:	return kAnsiGreenColour;
		case ELogLevel::Warning:return kAnsiYellowColour;
		case ELogLevel::Error:	return kAnsiRedColour;
		default:				return kAnsiResetColour;
		}
	}
} // VPHX namespace