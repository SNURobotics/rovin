#pragma once

#include <cassert>
#include <string>

namespace rovin
{
	namespace utils
	{
		static bool Log(const bool statement, const std::string message = (""), const bool shutdown = (false))
		{
			if (statement)
			{
				Log(message, shutdown);
			}
			return statement;
		}

		static void Log(const std::string message, const bool shutdown = (false))
		{
			if (shutdown)
			{
				assert(0, message);
			}
		}
	}
}