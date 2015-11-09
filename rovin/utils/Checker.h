#pragma once

#include <string>

namespace rovin
{
	namespace utils
	{
		
		static bool checkName(std::string name)
		{
			static const int _NUM_OF_BANNED_CHARACTERS = 9;
			static const char _BANNED_CHRACTERS[_NUM_OF_BANNED_CHARACTERS] = { '\\', '/', ':', '*', '?', '\"', '<', '>', '|' };
			int i;
			for (std::string::iterator pos = name.begin(); pos != name.end(); pos++)
			{
				for (i = 0; i < _NUM_OF_BANNED_CHARACTERS; i++)
				{
					if (*pos == _BANNED_CHRACTERS[i]) return false;
				}
			}
			return true;
		}
	}
}