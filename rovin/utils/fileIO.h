#pragma once
//#include <rovin/Math/Constant.h>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <cstdio>
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

		static Eigen::MatrixXd readText(const string& fileName)
		{
		/*	ifstream file;
			
			file.open(fileName);

			int n_row, n_col = 0;

			if (file.is_open())
			{
				while (!file.eof())
				{
					file.getline()
				}
			}*/
			FILE *in;
			fopen_s(&in, fileName.c_str(), "r");
			
			int lineM;
			int n, m;
			n = m = 0;
			std::vector<std::vector<double>> elem;
			char c;
			double temp;

			m = -1;
			while (!feof(in))
			{
				elem.push_back(std::vector<double>());
				c = NULL;
				lineM = 0;
				for (;;)
				{
					fscanf_s(in, "%lf", &temp);
					elem[n].push_back(temp);
					lineM++;
					while (1)
					{
						c = NULL;
						fscanf_s(in, "%c", &c, 1);
						if (c == NULL) break;
						if (c == ' ') continue;
						else break;
					}
					if (c == NULL) break;
					if (c == '\n') break;
					fseek(in, -1, SEEK_CUR);
					
				}
				if (m == -1) m = lineM;
				else
				{
					if (m != lineM)
					{
						if (c == NULL) break;
						else
						{
							assert(m == lineM);
						}
					}
				}
				n++;
			}

			Eigen::MatrixXd data(n, m);
			for (int i = 0; i < n; i++)
			{
				for (int j = 0; j < m; j++)
				{
					data(i, j) = elem[i][j];
				}
			}
			return data;
		}
	}
}