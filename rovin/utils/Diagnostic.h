#pragma once

#include <cassert>
#include <string>

namespace rovin
{
	namespace utils
	{
		static void Log(const std::string& message, const bool& shutdown)
		{
			if (shutdown)
			{
				assert(false);
			}
		}

		static bool Log(const bool& statement, const std::string& message, const bool& shutdown)
		{
			if (statement)
			{
				Log(message, shutdown);
			}
			return statement;
		}
	}
}