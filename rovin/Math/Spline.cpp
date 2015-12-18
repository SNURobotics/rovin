#include "Spline.h"
//#include "Common.cpp"
#include "Optimization.h"
using namespace std;
namespace rovin
{
	namespace Math
	{
		SO3CubicSplineInterpolation::SO3CubicSplineInterpolation(const Math::VectorX & x, const std::vector<Math::SO3> fs, const Math::Vector3 & df1, const Math::Vector3 & dfn)
		{
			_x = x;
			_fs = fs;
			_df1 = df1;
			_dfn = dfn;
			solveBoundaryCondition();
			_isBoundaryConditionSolved = true;
		}
		Math::SO3 SO3CubicSplineInterpolation::operator()(const Math::Real & x)
		{
			if (!_isBoundaryConditionSolved)
			{
				solveBoundaryCondition();
				forwardPass();
				_isBoundaryConditionSolved = true;
			}

			SO3 R;
			int xIdx = findIdx(_x, x);
			int j;
			Real tau;
			if (xIdx == _x.size() - 1)
			{
				j = xIdx;
				tau = 1.0;
				return _fs[_K - 1];
			}
			else
			{
				j = xIdx + 1;
				tau = (x - _x[j - 1]) / (_x[j] - _x[j - 1]);
				return _fs[j - 1] * SO3::Exp(_a[j - 1] * tau*tau*tau + _b[j - 1] * tau*tau + _c[j - 1] * tau);
			}
		}
		void SO3CubicSplineInterpolation::setX(const Math::VectorX & x)
		{
			_x = x;
		}
		void SO3CubicSplineInterpolation::setElements(const std::vector<Math::SO3>& fs)
		{
			_fs = fs;
		}
		void SO3CubicSplineInterpolation::setBoundaryConditions(const Math::Vector3 & df1, const Math::Vector3 & dfn)
		{
			_df1 = df1;
			_dfn = dfn;
		}

		Math::Vector3 SO3CubicSplineInterpolation::getBodyVelocity(const Math::Real & x)
		{
			if (!_isBoundaryConditionSolved)
			{
				solveBoundaryCondition();
				forwardPass();
				_isBoundaryConditionSolved = true;
			}
			Vector3 w;
			Vector3 r;
			Vector3 dotr;
			Matrix3 A;
			int xIdx = findIdx(_x, x);
			int j;
			Real tau;
			Real dtau_dt;
			Real r_norm;
			if (xIdx == _x.size() - 1)
			{
				j = xIdx;
				tau = 1.0;
			}
			else
			{
				j = xIdx + 1;
				tau = (x - _x[j - 1]) / (_x[j] - _x[j - 1]);
			}
			dtau_dt = 1.0 / (_x[j] - _x[j - 1]);
			r = _a[j - 1] * tau*tau*tau + _b[j - 1] * tau*tau + _c[j - 1] * tau;
			dotr = (_a[j - 1] * 3.0 *tau*tau + _b[j - 1] * 2.0 * tau + _c[j - 1])*dtau_dt;
			r_norm = r.norm();
			if (RealBigger(r_norm, 0))
			{
				A.setIdentity();
				A -= (1 - cos(r_norm)) / r_norm / r_norm*Bracket(r);
				A += (r_norm - sin(r_norm)) / r_norm / r_norm / r_norm*Bracket(r)*Bracket(r);
				w = A*dotr;
			}
			else
			{
				w = dotr;
			}
			
			return w;
		}
		Math::Vector3 SO3CubicSplineInterpolation::getBodyAcceleration(const Math::Real & x)
		{
			if (!_isBoundaryConditionSolved)
			{
				solveBoundaryCondition();
				forwardPass();
				_isBoundaryConditionSolved = true;
			}
			Vector3 dotw;
			so3 r;
			Vector3 dotr;
			Vector3 ddotr;
			int xIdx = findIdx(_x, x);
			int j;
			Real tau;
			Real dtau_dt;
			if (xIdx == _x.size() - 1)
			{
				j = xIdx;
				tau = 1.0;
			}
			else
			{
				j = xIdx + 1;
				tau = (x - _x[j - 1]) / (_x[j] - _x[j - 1]);
			}
			dtau_dt = 1.0 / (_x[j] - _x[j - 1]);
			r = _a[j - 1] * tau*tau*tau + _b[j - 1] * tau*tau + _c[j - 1] * tau;
			dotr = (_a[j - 1] * 3.0 *tau*tau + _b[j - 1] * 2.0 * tau + _c[j - 1])*dtau_dt;
			ddotr = (_a[j - 1] * 6.0 *tau + _b[j - 1] * 2.0)*dtau_dt*dtau_dt;
			Real r_norm = r.norm();
			Real r_norm_sq = r_norm*r_norm;
			if (RealBigger(r_norm, 0.0))
				dotw = ddotr - r.dot(dotr) / r_norm_sq / r_norm_sq*(2 * cos(r_norm) + r_norm*sin(r_norm) - 2)*r.cross(dotr) - (1 - cos(r_norm)) / r_norm_sq*r.cross(ddotr) + r.dot(dotr) / r_norm_sq / r_norm_sq / r_norm*(3 * sin(r_norm) - r_norm*cos(r_norm) - 2 * r_norm)*r.cross(r.cross(dotr)) + (r_norm - sin(r_norm)) / r_norm_sq / r_norm*(dotr.cross(r.cross(dotr)) + r.cross(r.cross(ddotr)));
			else
				dotw = ddotr;
			return dotw;
		}
		void SO3CubicSplineInterpolation::forwardPass()
		{
			_K = _x.size();
			_a.resize(_K);
			_b.resize(_K);
			_c.resize(_K);
			vector<so3> r(_K);
			for (int i = 0; i < _K - 1; i++)
				r[i] = SO3::Log((_fs[i]).inverse() * _fs[i + 1]);

			// cf. I.G.KANG AND F.C.PARK, CUBIC SPLINE ALGORITHMS FOR ORIENTATION INTERPOLATION
			// Initialization, assume initial angular acc is zero 
			_c[0] = _df1;							// initial vel 
			if (_dfn.norm() > 0.99e99)
				_b[0] = Vector3::Zero();
			else
				_b[0] = 0.5*_ddf1;						// initial acc * 0.5
			_a[0] = r[0] - _b[0] - _c[0];
			Vector3 s;
			Vector3 t;
			Vector3 u;
			Real s_norm;
			Real s_sq_norm;
			Real s_tr_norm;
			for (int i = 1; i < _K - 1; i++)
			{
				s = r[i];
				t = 3.0 * _a[i - 1] + 2.0 * _b[i - 1] + _c[i - 1];
				u = 6.0 * _a[i - 1] + 2.0 * _b[i - 1];
				s_norm = s.norm();
				s_sq_norm = s_norm*s_norm;
				s_tr_norm = s_sq_norm*s_norm;
				if (s_norm > 0.0000001)
				{
					_c[i] = t - (1.0 - cos(s_norm)) / s_sq_norm*s.cross(t) + (s_norm - sin(s_norm)) / s_tr_norm*s.cross(s.cross(t));
					_b[i] = 0.5*(u - s.dot(t) / s_sq_norm / s_sq_norm*(2.0 * cos(s_norm) + s_norm*sin(s_norm) - 2.0) * s.cross(t) - (1.0 - cos(s_norm)) / s_sq_norm * s.cross(u) + s.dot(t) / s_sq_norm / s_tr_norm*(3.0 * sin(s_norm) - s_norm*cos(s_norm) - 2.0 * s_norm)* s.cross(s.cross(t)) + (s_norm - sin(s_norm)) / s_tr_norm*(t.cross(s.cross(t)) + s.cross(s.cross(u))));
					_a[i] = s - _b[i] - _c[i];
				}
				else
				{
					_c[i] = t;
					_b[i] = 0.5*u;
					_a[i] = s;
				}
			}
		}
		void SO3CubicSplineInterpolation::solveBoundaryCondition()
		{
			
			if (_dfn.norm() > 0.99e99)
			{
				// natural spline
			}
			else
			{
				FunctionPtr boundaryFunc = FunctionPtr(new BoundaryConditionFunction);
				static_pointer_cast<BoundaryConditionFunction>(boundaryFunc)->_x = _x;
				static_pointer_cast<BoundaryConditionFunction>(boundaryFunc)->_fs = _fs;
				static_pointer_cast<BoundaryConditionFunction>(boundaryFunc)->_df1 = _df1;
				static_pointer_cast<BoundaryConditionFunction>(boundaryFunc)->_dfn = _dfn;
				NewtonRapshon newton;
				newton._func = boundaryFunc;
				Vector3 x0 = Vector3::Zero();
				_ddf1 = newton.solve(x0);
			}
			

		}

		Math::VectorX SO3CubicSplineInterpolation::BoundaryConditionFunction::func(const Math::VectorX & x) const
		{
			int									_K;				// number of control points
			std::vector<Math::Vector3>			_a;				// 3rd order term
			std::vector<Math::Vector3>			_b;				// 2nd order term
			std::vector<Math::Vector3>			_c;				// 1st order term

			_K = _x.size();
			_a.resize(_K);
			_b.resize(_K);
			_c.resize(_K);
			vector<so3> r(_K - 1);
			for (int i = 0; i < _K - 1; i++)
			{
				r[i] = SO3::Log((_fs[i]).inverse() * _fs[i + 1]);
				cout << r[i].transpose() << endl;
			}
				

			// cf. I.G.KANG AND F.C.PARK, CUBIC SPLINE ALGORITHMS FOR ORIENTATION INTERPOLATION
			// Initialization, assume initial angular acc is zero 
			_c[0] = _df1;							// initial vel 
			_b[0] = 0.5*x;						// initial acc * 0.5
			_a[0] = r[0] - _b[0] - _c[0];
			Vector3 s;
			Vector3 t;
			Vector3 u;
			Real s_norm;
			Real s_sq_norm;
			Real s_tr_norm;
			for (int i = 1; i < _K - 1; i++)
			{
				s = r[i];
				t = 3.0 * _a[i - 1] + 2.0 * _b[i - 1] + _c[i - 1];
				u = 6.0 * _a[i - 1] + 2.0 * _b[i - 1];
				s_norm = s.norm();
				s_sq_norm = s_norm*s_norm;
				s_tr_norm = s_sq_norm*s_norm;
				if (s_norm > 0.0000001)
				{
					_c[i] = t - (1.0 - cos(s_norm)) / s_sq_norm*s.cross(t) + (s_norm - sin(s_norm)) / s_tr_norm*s.cross(s.cross(t));
					_b[i] = 0.5*(u - s.dot(t) / s_sq_norm / s_sq_norm*(2.0 * cos(s_norm) + s_norm*sin(s_norm) - 2.0) * s.cross(t) - (1.0 - cos(s_norm)) / s_sq_norm * s.cross(u) + s.dot(t) / s_sq_norm / s_tr_norm*(3.0 * sin(s_norm) - s_norm*cos(s_norm) - 2.0 * s_norm)* s.cross(s.cross(t)) + (s_norm - sin(s_norm)) / s_tr_norm*(t.cross(s.cross(t)) + s.cross(s.cross(u))));
					_a[i] = s - _b[i] - _c[i];
					cout << "a: " << _a[i].transpose() << endl;
					cout << "b: " << _b[i].transpose() << endl;
					cout << "c: " << _c[i].transpose() << endl;
					cout << s_norm << endl;
					cout << 3.0*sin(s_norm) - s_norm*cos(s_norm) - 2 * s_norm << endl;
					cout << (3.0*sin(s_norm) - s_norm*cos(s_norm) - 2 * s_norm)/s_tr_norm/s_sq_norm << endl;
					cout << 1.0 / s_sq_norm / s_sq_norm*(2.0 * cos(s_norm) + s_norm*sin(s_norm) - 2.0) << endl;
					cout << (1.0 - cos(s_norm)) / s_sq_norm << endl;
					cout << (s_norm - sin(s_norm)) / s_tr_norm << endl;
				}
				else
				{
					_c[i] = t;
					_b[i] = 0.5*u;
					_a[i] = s - _b[i] - _c[i];
				}
			}

			Vector3 w;
			Vector3 rr;
			Vector3 dotr;
			Matrix3 A;
			int j = _x.size() - 1;
			Real tau = 1.0;
			Real dtau_dt = 1.0 / (_x[_K - 1] - _x[_K - 2]);
			Real r_norm;

			rr = _a[j - 1] * tau*tau*tau + _b[j - 1] * tau*tau + _c[j - 1] * tau;
			dotr = (_a[j - 1] * 3.0 *tau*tau + _b[j - 1] * 2.0 * tau + _c[j - 1])*dtau_dt;
			r_norm = rr.norm();
			A.setIdentity();
			A -= (1 - cos(r_norm)) / r_norm / r_norm*Bracket(rr);
			A += (r_norm - sin(r_norm)) / r_norm / r_norm / r_norm*Bracket(rr)*Bracket(rr);
			w = A*rr;
			return w - _dfn;
		}
	}
}
