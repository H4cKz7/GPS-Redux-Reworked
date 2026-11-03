#include "Logger.h"

#include <string>
#include <windows.h>

namespace util
{
	// Full path of SA.GPS.LOG.txt, located in the same folder as this ASI.
	static const std::string &LogPath()
	{
		static const std::string path = []() {
			HMODULE mod = nullptr;
			if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
										GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
									reinterpret_cast<LPCSTR>(&LogPath), &mod))
				return std::string("SA.GPS.LOG.txt");

			char buf[MAX_PATH] = {};
			const DWORD n = GetModuleFileNameA(mod, buf, MAX_PATH);
			if (n == 0 || n >= MAX_PATH)
				return std::string("SA.GPS.LOG.txt");

			std::string p(buf, n);
			const size_t slash = p.find_last_of("\\/");
			return (slash == std::string::npos ? std::string() : p.substr(0, slash + 1)) + "SA.GPS.LOG.txt";
		}();
		return path;
	}

	void Logger::Log(const char *val)
	{
		if (!enabled)
			return;

		if (logLines < 2048)
		{
			time_t timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

			strftime(stime, 128, "%c", localtime(&timenow));
			logfile << (std::string(stime) + " | " + val + "\n").c_str();
			logfile.flush();
			logLines++;
		}
		else
		{
			logfile.close();
			logfile.open(LogPath(), std::ios::out);
			logLines = 0;
			Log(val);
		}
	}

	Logger::Logger(const bool enabled)
	{
		this->enabled = enabled;
		if (enabled)
		{
			logfile.open(LogPath(), std::ios::out);
		}
	}
} // namespace util