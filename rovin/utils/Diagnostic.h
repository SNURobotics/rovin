#pragma once

#include <cassert>
#include <string>

#include <ctime>

#define PERFORM_TEST(loop,iteration)\
	do{\
	unsigned begin  = clock();\
	for(int i = 0 ; i < iteration ; i++){loop}\
	cout<<clock()-begin<<"	ms"<<endl;\
	}\
	while(false);\

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