/**
*	\file	LinearAlgegra.h
*	\date	2015.11.13
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Linear algegra 관련 함수
*/

#pragma once

#include "Constant.h"

namespace rovin
{
	namespace Math
	{
		static MatrixX pInv(const MatrixX& mat)
		{
			Eigen::JacobiSVD<Math::MatrixX> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Real tolerance = RealEps;
			VectorX singular_values = svd.singularValues();
			MatrixX S(mat.rows(), mat.cols());
			S.setZero();
			for (int i = 0; i < singular_values.size(); i++)
			{
				if (singular_values(i) > tolerance)
				{
					S(i, i) = 1.0 / singular_values(i);
				}
				else
				{
					S(i, i) = 0;
				}
			}
			return svd.matrixV() * S.transpose() * svd.matrixU().transpose();
		}
	}
}