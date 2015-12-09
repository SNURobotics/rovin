#pragma once
//#include <rovin/Math/Constant.h>
#include <string>
#include <fstream>
#include <iostream>
#include <Eigen/Dense>

using namespace std;
namespace rovin
{
	namespace utils
	{
		template <typename Derived>
		void writeText(const Eigen::MatrixBase<Derived>& mat, const string& fileName)
		{
			ofstream outText(fileName);
			for (int row = 0; row < mat.rows(); row++)
			{
				for (int col = 0; col < mat.cols(); col++)
					outText << mat(row, col) << '\t';
				outText << '\n';
			}
			outText.close();
		}
	}
}