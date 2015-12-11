#pragma once
//#include <rovin/Math/Constant.h>
#include <string>
#include <fstream>
#include <iostream>
#include <list>
#include <sstream>
#include <vector>
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

		Eigen::MatrixXd readFromFile(const string& fileName)
		{
			Eigen::MatrixXd	returnMat;
			ifstream inputFile(fileName);
			string line;
			vector<stringstream>	lineList;
			double	r;

			while (getline(inputFile, line))
			{
				if (line.empty())
					continue;
				lineList.push_back(stringstream(line));
			}

			while (lineList[0] >> r)
			{
				returnMat.conservativeResize(1, returnMat.cols() + 1);
				returnMat(0, returnMat.cols() - 1) = r;
			}
			returnMat.conservativeResize(lineList.size(), Eigen::NoChange);

			for (unsigned int i = 1; i < lineList.size(); i++)
				for (int j = 0; j < returnMat.cols() && lineList[i] >> r; j++)
					returnMat(i, j) = r;
			
			return returnMat;
		}
	}
}