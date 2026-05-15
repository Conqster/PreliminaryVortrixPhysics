#include "Logger.h"
#include "StackString.h"

#include <filesystem>

namespace vx
{
	Logger::Logger()
	{
		std::cout << "-------------------------------------------------------------\n";
		std::cout << "--------------------" << "Vortrix Physics Log" << "----------------------\n";
		std::cout << "-------------------------------------------------------------\n";
	}
	Logger::~Logger()
	{
		if (mFile.is_open())
			mFile.close();
	}


	void Logger::WriteLine(ELogLevel lvl, const char* data, size_t len)
	{
		StackString<512> output;
		output.Append('[');

		if (bLogTimestamp)
		{
			std::time_t t = std::time(nullptr);
			std::tm time_info;
			localtime_s(&time_info, &t);

			char time_buff[16];
			int count = std::strftime(time_buff, sizeof(time_buff), "%H:%M:%S", &time_info);

			output.Append(time_buff, count);
			output.Append("] [", 4);
		}

		const char* lvl_prefix = LevelToChar(lvl);

		output.Append(lvl_prefix, strlen(lvl_prefix));
		output.Append("]: ", 4);
		output.Append(data, len);
		output.Append("\n", 2);

#if VX_DEBUG
		if (Contains(mTargetBuffer, ELogBuffer::Console))
		{
			const char* lvl_ansi_col = LevelAnsiColour(lvl);
			std::cout.write(lvl_ansi_col, strlen(lvl_ansi_col));
			std::cout.write(output.Data(), output.Length());
			auto reset_ansi_col = kAnsiResetColour;
			std::cout.write(reset_ansi_col, strlen(reset_ansi_col));
		}
#endif // VX_DEBUG
			//std::cout << std::string(LevelAnsiColour(lvl)) << output << kAnsiResetColour;

		if (Contains(mTargetBuffer, ELogBuffer::File))
		{
			if (!mFile.is_open())
				TryOpenFile();

			//mFile << output;
			mFile.write(output.Data(), output.Length());
		}
	}

	void Logger::TryOpenFile()
	{
		if (mFilePath.empty())
			mFilePath = kDefaultLogpath;

		auto mode = std::ios::out | (bAppendToFile ? std::ios::app : std::ios::trunc);

		//check for directory 
		std::filesystem::path _path = mFilePath;
		std::filesystem::create_directories(_path.parent_path());

		mFile.open(_path, mode);
		if (!mFile.is_open())
			std::cerr << "Failed to open file: " << mFilePath << "\n";

		if (!mFile.good())
			std::cerr << "File stream is not good \n";

		mFile << "-------------------------------------------------------------\n";
		mFile << "--------------------" << "Vortrix Physics Log" << "----------------------\n";
		mFile << "-------------------------------------------------------------\n";
	}


} // VPHX namespace