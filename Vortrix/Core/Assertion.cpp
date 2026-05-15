#include "Assertion.h"
#include "Logger.h"



#ifdef VX_CORE_ENABLE_ASSERTS
namespace vx {

	VxAssertFailedFunction VxAssertionFailedFunc = internals::DefaultAssertHandler;

	namespace internals {

		static void DefaultAssertHandler(const char* expr, const char* message, const unsigned int lvl, const char* file, unsigned int line, const char* func)
		{
			std::ostringstream oss;
			oss << "\n[Assertion Failed] (" << expr << ") in " << func;

			if (message && *message)
				oss << "\nMessage: " << message;

			oss << "\nFile: " << StripProjectPath(file) << " (Line: " << line << ").";


			if (lvl == 1)
				VX_WARN(oss.str());
			else if (lvl == 2)
				VX_ERROR(oss.str());
		}
	}
}
#endif // VX_CORE_ENABLE_ASSERTS