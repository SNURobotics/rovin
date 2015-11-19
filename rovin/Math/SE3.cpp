#include "SE3.h"
#include "Numeric.h"

#include <Eigen/Dense>

namespace rovin
{
	namespace Math
	{
		SE3& SE3::operator = (const SE3& operand)
		{
			double* T = &((*this)(0));
			const double* O = &(operand(0));

			(*T) = (*O);
			(*(T + 1)) = (*(O + 1));
			(*(T + 2)) = (*(O + 2));
			(*(T + 3)) = (*(O + 3));

			(*(T + 4)) = (*(O + 4));
			(*(T + 5)) = (*(O + 5));
			(*(T + 6)) = (*(O + 6));
			(*(T + 7)) = (*(O + 7));

			(*(T + 8)) = (*(O + 8));
			(*(T + 9)) = (*(O + 9));
			(*(T + 10)) = (*(O + 10));
			(*(T + 11)) = (*(O + 11));

			return (*this);
		}

		SE3& SE3::operator = (const SE3List& operand)
		{
			Real tmp1, tmp2, tmp3;
			Real* T = &((*this)(0));
			const Real* O;

			*this = *operand.Temp[operand.l];
			for (unsigned int i = operand.l + 1; i < operand.r; i++)
			{
				O = &(operand.Temp[i]->operator()(0));

				(*(T + 3)) += (*(T + 0)) * (*(O + 3)) + (*(T + 1)) * (*(O + 7)) + (*(T + 2)) * (*(O + 11));
				(*(T + 7)) += (*(T + 4)) * (*(O + 3)) + (*(T + 5)) * (*(O + 7)) + (*(T + 6)) * (*(O + 11));
				(*(T + 11)) += (*(T + 8)) * (*(O + 3)) + (*(T + 9)) * (*(O + 7)) + (*(T + 10)) * (*(O + 11));

				tmp1 = (*(T + 0)) * (*(O + 0)) + (*(T + 1)) * (*(O + 4)) + (*(T + 2)) * (*(O + 8));
				tmp2 = (*(T + 0)) * (*(O + 1)) + (*(T + 1)) * (*(O + 5)) + (*(T + 2)) * (*(O + 9));
				tmp3 = (*(T + 0)) * (*(O + 2)) + (*(T + 1)) * (*(O + 6)) + (*(T + 2)) * (*(O + 10));
				(*(T + 0)) = tmp1;
				(*(T + 1)) = tmp2;
				(*(T + 2)) = tmp3;

				tmp1 = (*(T + 4)) * (*(O + 0)) + (*(T + 5)) * (*(O + 4)) + (*(T + 6)) * (*(O + 8));
				tmp2 = (*(T + 4)) * (*(O + 1)) + (*(T + 5)) * (*(O + 5)) + (*(T + 6)) * (*(O + 9));
				tmp3 = (*(T + 4)) * (*(O + 2)) + (*(T + 5)) * (*(O + 6)) + (*(T + 6)) * (*(O + 10));
				(*(T + 4)) = tmp1;
				(*(T + 5)) = tmp2;
				(*(T + 6)) = tmp3;

				tmp1 = (*(T + 8)) * (*(O + 0)) + (*(T + 9)) * (*(O + 4)) + (*(T + 10)) * (*(O + 8));
				tmp2 = (*(T + 8)) * (*(O + 1)) + (*(T + 9)) * (*(O + 5)) + (*(T + 10)) * (*(O + 9));
				tmp3 = (*(T + 8)) * (*(O + 2)) + (*(T + 9)) * (*(O + 6)) + (*(T + 10)) * (*(O + 10));
				(*(T + 8)) = tmp1;
				(*(T + 9)) = tmp2;
				(*(T + 10)) = tmp3;
			}
			return (*this);
		}

		SE3& SE3::operator *= (const SE3& operand)
		{
			Real tmp1, tmp2, tmp3;

			Real* T = &((*this)(0));
			const Real* O = &(operand(0));

			(*(T + 3)) += (*(T + 0)) * (*(O + 3)) + (*(T + 1)) * (*(O + 7)) + (*(T + 2)) * (*(O + 11));
			(*(T + 7)) += (*(T + 4)) * (*(O + 3)) + (*(T + 5)) * (*(O + 7)) + (*(T + 6)) * (*(O + 11));
			(*(T + 11)) += (*(T + 8)) * (*(O + 3)) + (*(T + 9)) * (*(O + 7)) + (*(T + 10)) * (*(O + 11));

			tmp1 = (*(T + 0)) * (*(O + 0)) + (*(T + 1)) * (*(O + 4)) + (*(T + 2)) * (*(O + 8));
			tmp2 = (*(T + 0)) * (*(O + 1)) + (*(T + 1)) * (*(O + 5)) + (*(T + 2)) * (*(O + 9));
			tmp3 = (*(T + 0)) * (*(O + 2)) + (*(T + 1)) * (*(O + 6)) + (*(T + 2)) * (*(O + 10));
			(*(T + 0)) = tmp1;
			(*(T + 1)) = tmp2;
			(*(T + 2)) = tmp3;

			tmp1 = (*(T + 4)) * (*(O + 0)) + (*(T + 5)) * (*(O + 4)) + (*(T + 6)) * (*(O + 8));
			tmp2 = (*(T + 4)) * (*(O + 1)) + (*(T + 5)) * (*(O + 5)) + (*(T + 6)) * (*(O + 9));
			tmp3 = (*(T + 4)) * (*(O + 2)) + (*(T + 5)) * (*(O + 6)) + (*(T + 6)) * (*(O + 10));
			(*(T + 4)) = tmp1;
			(*(T + 5)) = tmp2;
			(*(T + 6)) = tmp3;

			tmp1 = (*(T + 8)) * (*(O + 0)) + (*(T + 9)) * (*(O + 4)) + (*(T + 10)) * (*(O + 8));
			tmp2 = (*(T + 8)) * (*(O + 1)) + (*(T + 9)) * (*(O + 5)) + (*(T + 10)) * (*(O + 9));
			tmp3 = (*(T + 8)) * (*(O + 2)) + (*(T + 9)) * (*(O + 6)) + (*(T + 10)) * (*(O + 10));
			(*(T + 8)) = tmp1;
			(*(T + 9)) = tmp2;
			(*(T + 10)) = tmp3;

			return *this;
		}

		SE3& SE3::operator *= (const SE3List& operand)
		{
			for (unsigned int i = operand.l; i < operand.r; i++)
			{
				(*this) *= *operand.Temp[i];
			}
			return (*this);
		}

		SE3 SE3::operator * (const SE3& operand) const
		{
			const double* T = &((*this)(0));
			const double* O = &((operand)(0));

			return SE3((*(T + 0)) * (*(O + 0)) + (*(T + 1)) * (*(O + 4)) + (*(T + 2)) * (*(O + 8)),
				(*(T + 0)) * (*(O + 1)) + (*(T + 1)) * (*(O + 5)) + (*(T + 2)) * (*(O + 9)),
				(*(T + 0)) * (*(O + 2)) + (*(T + 1)) * (*(O + 6)) + (*(T + 2)) * (*(O + 10)),
				(*(T + 0)) * (*(O + 3)) + (*(T + 1)) * (*(O + 7)) + (*(T + 2)) * (*(O + 11)) + (*(T + 3)),

				(*(T + 4)) * (*(O + 0)) + (*(T + 5)) * (*(O + 4)) + (*(T + 6)) * (*(O + 8)),
				(*(T + 4)) * (*(O + 1)) + (*(T + 5)) * (*(O + 5)) + (*(T + 6)) * (*(O + 9)),
				(*(T + 4)) * (*(O + 2)) + (*(T + 5)) * (*(O + 6)) + (*(T + 6)) * (*(O + 10)),
				(*(T + 4)) * (*(O + 3)) + (*(T + 5)) * (*(O + 7)) + (*(T + 6)) * (*(O + 11)) + (*(T + 7)),

				(*(T + 8)) * (*(O + 0)) + (*(T + 9)) * (*(O + 4)) + (*(T + 10)) * (*(O + 8)),
				(*(T + 8)) * (*(O + 1)) + (*(T + 9)) * (*(O + 5)) + (*(T + 10)) * (*(O + 9)),
				(*(T + 8)) * (*(O + 2)) + (*(T + 9)) * (*(O + 6)) + (*(T + 10)) * (*(O + 10)),
				(*(T + 8)) * (*(O + 3)) + (*(T + 9)) * (*(O + 7)) + (*(T + 10)) * (*(O + 11)) + (*(T + 11)));
		}

		//SE3List SE3::operator * (const SE3& operand) const
		//{
		//	return SE3List(this, &operand);
		//}

		SE3List& SE3::operator * (SE3List& operand) const
		{
			operand.push_left(this);
			return operand;
		}

		void SE3List::push_left(const SE3* item)
		{
			Temp[--l] = item;
		}

		void SE3List::push_right(const SE3* item)
		{
			Temp[r++] = item;
		}

		SE3List::operator SE3() const
		{
			SE3 T(*Temp[l]);
			for (unsigned int i = l + 1; i < r; i++)
			{
				T *= *Temp[i];
			}
			return T;
		}

		SE3List::operator Matrix4() const
		{
			return SE3(*this);
		}

		SE3List& SE3List::operator * (const SE3& operand)
		{
			push_right(&operand);
			return *this;
		}
	}
}