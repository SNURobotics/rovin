#include "GivenPathOptimization.h"
#include <rovin/utils/fileIO.h>
#include <rovin/Model/MotorJoint.h>
#include <rovin/Math/Optimization.h>
#include <ctime>
#include <cmath>
#include <memory>
namespace rovin
{
	namespace TrajectoryOptimization
	{
		GivenPathOptimization::GivenPathOptimization()
		{
			_pathN = 0;
			_pathTruncated = false;
			_thBoundGenerated = false;
		}


		GivenPathOptimization::~GivenPathOptimization()
		{
		}

		void GivenPathOptimization::setSOCRobotModel(const Model::socAssemblyPtr & socAssem, const SE3& Tbase, const SE3& TlastLinkToEndeffector)
		{
			_socAssem = socAssem;
			_defaultState = socAssem->makeState();
			_nDOF = _defaultState->getDOF(State::TARGET_JOINT::ACTIVEJOINT);
			_Tbase = Tbase;
			_TlastLinkToEndeffector = TlastLinkToEndeffector;
		}

		void GivenPathOptimization::setConstraint(bool velConstraintExist, bool accConstraintExist, bool torqueConstraintExist)
		{
			_velConstraintExist = velConstraintExist;
			_accConstraintExist = accConstraintExist;
			_torqueConstraintExist = torqueConstraintExist;
		}

		void GivenPathOptimization::loadToolPath(const std::string& fileName)
		{
			MatrixX file = utils::readText(fileName);
			_posTrj = file.block(0, 0, file.rows(), 3) * 0.001;		// convert the unit to m
			_zAxisTrj = file.block(0, 3, file.rows(), 3);
		}

		void GivenPathOptimization::setNumberofTimeStep(const int nStep)
		{
			_nStep = nStep;
		}

		void GivenPathOptimization::setParameters(const Real feedRate, const Real chordError, const Real robotTimeStep, const Real An, const Real Jn, const Real At, const Real Jt)
		{
			_feedRate = feedRate;
			_chordError = chordError;
			_robotTimeStep = robotTimeStep;
			_An = An;
			_Jn = Jn;
			_At = At;
			_Jt = Jt;
			_criticalCurvature = min (8.0 * _chordError / (4.0 * _chordError*_chordError + _feedRate*_feedRate * _robotTimeStep * _robotTimeStep), An / _feedRate / _feedRate);
			_criticalCurvature = min(_criticalCurvature, sqrt(_Jn / _feedRate / _feedRate / _feedRate));
		}

		void GivenPathOptimization::truncatePath(const Real curvTol)
		{
			if (_pathTruncated == false)
			{
				// set ds = 1
				MatrixX dPos = 0.5*(_posTrj.bottomRows(_posTrj.rows() - 2) - _posTrj.topRows(_posTrj.rows() - 2));
				MatrixX ddPos = _posTrj.bottomRows(_posTrj.rows() - 2) + _posTrj.topRows(_posTrj.rows() - 2) - 2.0*_posTrj.block(1, 0, _posTrj.rows() - 2, 3);
				_speed = (dPos.cwiseAbs2()*Vector3::Ones()).cwiseSqrt(); // norm of dPos/ds
				_curvature = ((dPos.cwiseAbs2()*Vector3::Ones()).cwiseProduct(ddPos.cwiseAbs2()*Vector3::Ones()) - (dPos.cwiseProduct(ddPos)*Vector3::Ones()).cwiseAbs2()).cwiseSqrt().cwiseQuotient(_speed.cwiseAbs2().cwiseProduct(_speed));
				

				_startPathIdx.resize(0);
				_endPathIdx.resize(0);
				_startPathIdx.push_back(0);
				//cout << "curvature: " << endl;
				//cout << _curvature.segment(0, 20).transpose() << endl;
				//cout << "speed: " << endl;
				//cout << _speed.segment(0, 20).transpose() << endl;
				//cout << "dpos" << endl;
				//cout << dPos.block(0, 0, 20, 3) << endl;
				// erase the first and last data
				MatrixX posTrj = _posTrj.block(1, 0, _posTrj.rows() - 2, 3);
				_posTrj = posTrj;
				posTrj.resize(0, 0);
				MatrixX zAxisTrj = _zAxisTrj.block(1, 0, _zAxisTrj.rows() - 2, 3);
				_zAxisTrj = zAxisTrj;
				zAxisTrj.resize(0, 0);
				bool isAddedBefore = false;
				for (int i = 0; i < _curvature.size(); i++)
				{
					if (!RealLessEqual(_curvature(i), curvTol) && isAddedBefore == false)
					{
						_endPathIdx.push_back(i);
						_pathN += 1;
						isAddedBefore = true;
					}
					if (RealLessEqual(_curvature(i), curvTol) && isAddedBefore == true)
					{
						_startPathIdx.push_back(i);
						isAddedBefore = false;
					}
				}
				_endPathIdx.push_back(_curvature.size() - 1);
				_pathN += 1;
				_pathTruncated = true;
			}
			//cout << _curvature.segment(0,10) << endl;

		}

		void GivenPathOptimization::setThetaGridNumber(const int thN)
		{
			_thN = thN;
			_thNdouble = 2 * thN - 1;
			_thSpan.resize(_thN);
			_thSpanDouble.resize(_thNdouble);
			for (int i = 0; i < _thN; i++)
			{
				_thSpan(i) = (Real)i / (Real)(_thN - 1) * PI_DOUBLE;
				_thSpanDouble(i) = _thSpan(i) - PI_DOUBLE;
				_thSpanDouble(i + _thN - 1) = _thSpan(i);
			}
			_thSpanDouble(_thSpanDouble.size() - 1) = _thSpan(_thSpan.size() - 1);
			//cout << "thspan = " << _thSpanDouble << endl;
		}

		void GivenPathOptimization::setPathNum(const int pathNum, const VectorX& qInit)
		{
			_thBoundGenerated = false;
			generateSdotMax(_startPathIdx[pathNum], _endPathIdx[pathNum]);
			_sf = (Real)(_endPathIdx[pathNum] - _startPathIdx[pathNum]);
			_startIdx = _startPathIdx[pathNum];
			_endIdx = _endPathIdx[pathNum];

			solveInverseKinematics(_startPathIdx[pathNum], _endPathIdx[pathNum], qInit);
			
			// cubic spline interpolation
			VectorX sSet(_endIdx - _startIdx + 1);
			vector<VectorX> posTrjSet(sSet.size());
			vector<VectorX> zAxisTrjSet(sSet.size());
			for (int i = 0; i < sSet.size(); i++)
			{
				sSet(i) = i;
				posTrjSet[i] = _posTrj.row(_startIdx + i).transpose();
				zAxisTrjSet[i] = _zAxisTrj.row(_startIdx + i).transpose();
			}
			VectorX Vinit = posTrjSet[1] - posTrjSet[0];
			VectorX Vend = posTrjSet[posTrjSet.size() - 1] - posTrjSet[posTrjSet.size() - 2];
			VectorX Winit = zAxisTrjSet[1] - zAxisTrjSet[0];
			VectorX Wend = zAxisTrjSet[zAxisTrjSet.size() - 1] - zAxisTrjSet[zAxisTrjSet.size() - 2];
			_posCubic = CubicSplineInterpolation::CubicSplineInterpolation(sSet, posTrjSet, Vinit, Vend);
			_zAxisCubic = CubicSplineInterpolation::CubicSplineInterpolation(sSet, zAxisTrjSet, Winit, Wend);

		}
		Math::VectorX GivenPathOptimization::cubicFunction::func(const Math::VectorX & x) const
		{
			VectorX a(1);
			a(0) = x(0)*x(0)*x(0) + _vn*x(0)*x(0) - _vn*_vn*x(0) - _vn*_vn*_vn - _Jt*_si*_si;
			return a;
		}

		Math::VectorX GivenPathOptimization::quadFunction::func(const Math::VectorX & x) const
		{
			VectorX a(1);
			a(0) = _Jt*x(0)*x(0) + _At*_At*x(0) - _Jt*_vn*_vn + _At*_At*_vn - 2 * _At*_Jt*_si;
			return a;
		}

		void GivenPathOptimization::generateRealSwrtTime()
		{
			// generate spline for s from real length
			vector<VectorX> sSet(_sN);
			VectorX tempsSet(1);
			VectorX realS(_sN);
			VectorX ds0(1), dsf(1);

			for (int i = 0; i < _sN; i++)
			{
				tempsSet(0) = (Real) i;
				sSet[i] = tempsSet;
				if (i > 0)
					realS(i) = realS(i - 1) + (_posTrj.row(_startIdx + i) - _posTrj.row(_startIdx + i - 1)).norm();
				else
					realS(i) = 0;

				//cout << realS(i) << " " << i << endl;
			}
			ds0(0) = 1.0 / (realS(1) - realS(0));
			dsf(0) = 1.0 / (realS(realS.size() - 1) - realS(realS.size() - 2));
			_sCubic.setX(realS);
			_sCubic.setElements(sSet);
			_sCubic.setBoundaryConditions(ds0, dsf);

			//cout << "----------------" << endl;
			//int nnn = 200;
			//Real sss;
			//for (int i = 0; i < nnn; i++)
			//{
			//	sss = realS(realS.size() - 1) * (Real)i / (Real)(nnn - 1);
			//	cout << sss << " " << _sCubic(sss) << endl;
			//}


			// find critical points
			int n = 10;
			int start;
			int end;
			vector<int> localMaxIdx(0);
			for (int i = 1; i < _sN - 1; i++)
			{
				start = max(0, i - n);
				end = min(_sN - 1, i + n);
				if (_curvature(_startIdx + i) == _curvature.segment(start, end - start + 1).maxCoeff() && _curvature(_startIdx + i) > 1.1*_curvature.segment(_startIdx + 1, _sN - 2).mean())
					localMaxIdx.push_back(i);
			}

			vector<Real> si(localMaxIdx.size() + 1);
			for (unsigned int j = 0; j < localMaxIdx.size() + 1; j++)
			{
				si[j] = 0;
				if (j == 0)
					start = 0;
				else
					start = localMaxIdx[j - 1];
				if (j == localMaxIdx.size())
					end = _endIdx - _startIdx;
				else
					end = localMaxIdx[j];
				
				si[j] = realS(end) - realS(start);
			}
			vector<Real> si_cumul(si.size() + 1);
			si_cumul[0] = 0;
			for (unsigned int i = 1; i < si.size() + 1; i++)
				si_cumul[i] = si_cumul[i - 1] + si[i - 1];
			// backward scanning
			FunctionPtr _cubicFunc = FunctionPtr(new cubicFunction());
			static_pointer_cast<cubicFunction>(_cubicFunc)->_Jt = _Jt;
			NewtonRapshon newton1;
			newton1._func = _cubicFunc;

			FunctionPtr _quadFunc = FunctionPtr(new quadFunction());
			static_pointer_cast<quadFunction>(_quadFunc)->_Jt = _Jt;
			static_pointer_cast<quadFunction>(_quadFunc)->_At = _At;
			NewtonRapshon newton2;
			newton2._func = _quadFunc;

			vector<Real> vb(localMaxIdx.size() + 2);
			VectorX vInit(1);
			vb[0] = 0;
			vb[vb.size() - 1] = 0;
			Real vtemp;
			Real k;
			for (unsigned int i = 0; i < localMaxIdx.size(); i++)
			{
				static_pointer_cast<cubicFunction>(_cubicFunc)->_si = si[si.size() - 1 - i];
				static_pointer_cast<cubicFunction>(_cubicFunc)->_vn = vb[vb.size() - 1 - i];
				vInit(0) = vb[vb.size() - 1 - i];
				vtemp = newton1.solve(vInit)(0);
				if (vtemp > _At*_At / _Jt + vb[vb.size() - 1 - i])
				{
					static_pointer_cast<quadFunction>(_quadFunc)->_si = si[si.size() - 1 - i];
					static_pointer_cast<quadFunction>(_quadFunc)->_vn = vb[vb.size() - 1 - i];
					vtemp = newton2.solve(vInit)(0);
				}
				k = _curvature(_startIdx + localMaxIdx[localMaxIdx.size() - 1 - i]);
				cout << vtemp << endl;
				vtemp = min(vtemp, 0.5/_robotTimeStep*sqrt(2 * _chordError / k - _chordError*_chordError));
				cout << vtemp << endl;
				vtemp = min(vtemp, sqrt(_An / k));
				cout << vtemp << endl;
				vtemp = min(vtemp, pow(_Jn / k / k, 1.0 / 3.0));
				cout << vtemp << endl;
				vb[vb.size() - 2 - i] = min(vtemp, _feedRate);
				cout << vtemp << endl;
			}
			
			// forward scanning
			vector<Real> v(localMaxIdx.size() + 2);
			v[0] = 0;
			v[v.size() - 1] = 0;
			//static_pointer_cast<cubicFunction>(_cubicFunc)->_Jt = -_Jt;
			//static_pointer_cast<quadFunction>(_quadFunc)->_Jt = -_Jt;
			//static_pointer_cast<quadFunction>(_quadFunc)->_At = -_At;
			for (unsigned int i = 0; i < localMaxIdx.size(); i++)
			{
				static_pointer_cast<cubicFunction>(_cubicFunc)->_si = si[i];
				static_pointer_cast<cubicFunction>(_cubicFunc)->_vn = v[i];
				//while (1)
				//{
				//	srand(time(NULL));
				//	vInit(0) = 100.0*rand();
				//	vtemp = newton1.solve(vInit)(0);
				//	if (vtemp > 0)
				//		break;
				//}
				vInit(0) = v[i];
				vtemp = newton1.solve(vInit)(0);
				if (vtemp > _At*_At / _Jt + v[i])
				{
					static_pointer_cast<quadFunction>(_quadFunc)->_si = si[i];
					static_pointer_cast<quadFunction>(_quadFunc)->_vn = v[i];
					vtemp = newton2.solve(vInit)(0);
				}
				v[i + 1] = min(vtemp, vb[i + 1]);
			}

			// S-shape profile
			Real vs;
			Real ve;
			Real s_cri_s;
			Real s_cri_l;
			vector<Real> Ttemp;
			Real tend = 0;
			VectorX time;
			vector<VectorX> s;
			VectorX s1(1);
			pair<vector<Real>, vector<Real>> result;
			time.resize(1);
			s.resize(1);
			time[0] = 0;
			s1(0) = 0;
			s[0] = s1;
			for (unsigned int i = 0; i < localMaxIdx.size() + 1; i++)
			{
				vs = v[i];
				ve = v[i + 1];
				if (vs < ve)
					s_cri_s = s_a(vs, ve);
				else
					s_cri_s = s_d(vs, ve);
				if (RealEqual(si[i], s_cri_s))
					Ttemp = findDurationForShortBlock2(vs, ve, _feedRate);
				else
				{
					s_cri_l = s_a(vs, _feedRate) + s_d(_feedRate, ve);
					if (si[i] > s_cri_l)
						Ttemp = findDurationForLongBlock(vs, ve, si[i]);
					else
						Ttemp = findDurationForMediumBlock(vs, ve, si[i]);
				}

				result = getArcLengthFromTimeDuration(Ttemp, vs);
				time.conservativeResize(time.rows() + result.first.size());
				for (unsigned int j = 0; j < result.first.size(); j++)
				{
					time(time.size() - 1 - j) = tend + result.first[result.first.size() - 1 - j];
					s1(0) = si_cumul[i] + result.second[j];
					s.push_back(s1);
				}
				tend += result.first[result.first.size() - 1];
			}
			_realSCubic.setX(time);
			_realSCubic.setElements(s);
			s1(0) = 0;
			_realSCubic.setBoundaryConditions(s1, s1);
			_tfForThetaOnly = time(time.size() - 1);
			
			//cout << "time : " << " ";
			//for (int i = 0; i < time.size(); i++)
			//	cout << time(i) << " ";
			//cout << endl;
			//for (int i = 0; i < time.size(); i++)
			//	cout << time(i) << " " << s[i] << endl;
			//cout << "sf = " << si_cumul[si_cumul.size() - 1] << endl;

			//int nn = 100;
			//Real tf = time(time.size() - 1);
			//Real ti;
			//
			//for (int i = 0; i < nn; i++)
			//{
			//	ti = (Real)i / (Real)(nn - 1) * tf;
			//	cout << ti << " " << _realSCubic(ti) << endl;
			//}
				

		}

		vector<Real> GivenPathOptimization::findDurationForShortBlock(const Math::Real vs, const Math::Real ve, const Math::Real vmax)
		{
			vector<Real> T(3);
			for (unsigned int i = 0; i < T.size(); i++)
				T[i] = 0;
			T[0] = _At / _Jt;
			T[2] = T[0];
			if (RealBigger(abs(ve - vs), _At*_At / _Jt))
				T[1] = (vmax - min(vs, ve)) / _At - _At / _Jt;
			
			return T;
		}

		std::vector<Math::Real> GivenPathOptimization::findDurationForShortBlock2(const Math::Real vs, const Math::Real ve, const Math::Real vmax)
		{
			vector<Real> T(7);
			for (unsigned int i = 0; i < T.size(); i++)
				T[i] = 0;

			if (vs < ve)
			{
				T[0] = _At / _Jt;
				T[2] = T[0];
				if (RealBigger((ve - vs), _At*_At / _Jt))
					T[1] = (vmax - vs) / _At - _At / _Jt;
			}
			else
			{
				T[4] = _At / _Jt;
				T[6] = T[4];
				if (RealBigger((vs - ve), _At*_At / _Jt))
					T[5] = (vmax - ve) / _At - _At / _Jt;
			}

			return T;
		}

		vector<Real> GivenPathOptimization::findDurationForLongBlock(const Math::Real vs, const Math::Real ve, const Math::Real si)
		{
			vector<Real> T(7);

			vector<Real> Tfront = findDurationForShortBlock(vs, _feedRate, _feedRate);
			vector<Real> Tback = findDurationForShortBlock(_feedRate, ve, _feedRate);

			for (int i = 0; i < 3; i++)
			{
				T[i] = Tfront[i];
				T[4 + i] = Tback[i];
			}

			Real sa = s_a(vs, _feedRate);
			Real sd = s_d(_feedRate, ve);
			T[3] = (si - sa - sd) / _feedRate;

			return T;
		}

		vector<Real> GivenPathOptimization::findDurationForMediumBlock(const Math::Real vs, const Math::Real ve, const Math::Real si, const Real eps)
		{
			vector<Real> T(7);
			T[3] = 0;
			
			Real v0 = max(vs, ve);
			Real v1 = _feedRate;
			Real v = 0.5*(v0 + v1);
			Real ds;
			while (1)
			{
				ds = s_a(vs, v) + s_d(v, ve) - si;

				if (RealBigger(ds, eps))
				{
					v1 = v;
					v = 0.5*(v0 + v);
				}
				else if (RealLess(ds, -eps))
				{
					v0 = v;
					v = 0.5*(v + v1);
				}
				else
					break;
			}
			vector<Real> Tfront = findDurationForShortBlock(vs, v, v);
			vector<Real> Tback = findDurationForShortBlock(v, ve, v);
			for (int i = 0; i < 3; i++)
			{
				T[i] = Tfront[i];
				T[4 + i] = Tback[i];
			}
			return T;
		}

		std::pair<std::vector<Math::Real>, std::vector<Math::Real>> GivenPathOptimization::getArcLengthFromTimeDuration(const std::vector<Math::Real> T, const Real vs)
		{
			vector<Real> s(7);
			Real v;
			s[0] = vs* T[0] + _Jt*T[0] * T[0] * T[0] / 6.0;
			v = vs + 0.5*_Jt*T[0] * T[0];
			s[1] = s[0] + v * T[1] + 0.5*_At*T[1] * T[1];
			v += _At*T[1];
			s[2] = s[1] + v*T[2] + 0.5*_At*T[2] * T[2] - _Jt*T[2] * T[2] * T[2] / 6.0;
			v += _At*T[2] - 0.5*_Jt*T[2] * T[2];
			s[3] = s[2] + v*T[3];
			s[4] = s[3] + v*T[4] - _Jt*T[4] * T[4] * T[4] / 6.0;
			v -= 0.5*_Jt*T[4] * T[4];
			s[5] = s[4] + v*T[5] - 0.5*_At*T[5] * T[5];
			v -= _At*T[5];
			s[6] = s[5] + v*T[6] - 0.5*_At*T[6] * T[6] + _Jt*T[6] * T[6] * T[6] / 6.0;

			pair<vector<Real>, vector<Real>> result;
			result.first.resize(0);
			result.second.resize(0);
			Real t = 0;
			for (unsigned int i = 0; i < s.size(); i++)
			{
				t += T[i];
				if (T[i] > 0)
				{
					result.first.push_back(t);
					result.second.push_back(s[i]);
				}
			}

			return result;
		}

		Math::Real GivenPathOptimization::s_a(const Math::Real vs, const Math::Real ve)
		{
			if (ve - vs > _At*_At / _Jt)
			{
				return 0.5*(vs + ve)*(_At / _Jt + (ve - vs) / _At);
			}
			else
				return (vs + ve) * sqrt((ve - vs) / _Jt);
		}

		Math::Real GivenPathOptimization::s_d(const Math::Real vs, const Math::Real ve)
		{
			return s_a(ve, vs);
		}

		void GivenPathOptimization::solveInverseKinematics(const int startIdx, const int endIdx, const VectorX& qInit)
		{
			_sN = endIdx - startIdx + 1;
			_sSpan.resize(_sN);
			for (int i = 0; i < _sN; i++)
				_sSpan(i) = i;
			////////////////////// store only upper 4 values for continuity of the solution
			_invKinSol.resize(4);
			for (int i = 0; i < 4; i++)
			{
				_invKinSol[i].resize(6);
				for (int j = 0; j < 6; j++)
					_invKinSol[i][j] = MatrixX::Zero(_thN, _sN);
			}
			SO3 R0;
			if (qInit.size() == 0)
			{
				Vector3 x0;
				Vector3 z0 = _zAxisTrj.row(startIdx).transpose();
				z0.normalize();
				if (z0.segment(0, 2).squaredNorm() > 0)
				{
					x0 << z0(1), -z0(0), 0;
				}
				else
					x0 << 1, 0, 0;

				x0.normalize();
				Vector3 y0 = z0.cross(x0);
				Matrix3 Rtemp;
				Rtemp.col(0) = x0; Rtemp.col(1) = y0; Rtemp.col(2) = z0;
				R0 = SO3::Projection(Rtemp);
			}
			else
			{
				_defaultState->setJointq(State::TARGET_JOINT::ACTIVEJOINT, qInit);
				R0 = _Tbase.getRotation()*(Kinematics::calculateEndeffectorFrame(*_socAssem, *_defaultState)*_TlastLinkToEndeffector).getRotation();
			}
			SE3 T0(R0, _posTrj.row(startIdx).transpose());
			//cout << "T0 = " << endl;
			//cout << T0 << endl;
			SE3 Ti;
			Vector3 w_hat;
			Real ang;
			Vector3 z_bf;
			Vector3 z_cur;
			SE3 Tj;
			vector<VectorX> tempInvKinSol;
			//cout << "p = " << endl;
			//cout << _posTrj.row(startIdx).transpose() << endl;
			_Ri.resize(0);
			for (int i = 0; i < _sN; i++)
			{
				Ti.setPosition(_posTrj.row(startIdx + i));
				if (i == 0)
				{ 
					Ti.setRotation(T0.getRotation());
				}
				else
				{
					z_bf = _zAxisTrj.row(startIdx + i - 1).transpose();
					z_cur = _zAxisTrj.row(startIdx + i).transpose();
					w_hat = z_bf.cross(z_cur);
					if (w_hat.norm() < RealEps)
						w_hat.setZero();
					else
						w_hat.normalize();
					ang = std::acos((z_bf.transpose()*z_cur)(0) / z_bf.norm() / z_cur.norm());
					Ti.setRotation(SO3::Exp(w_hat * ang) * Ti.getRotation());
				}
				_Ri.push_back(Ti.getRotation());
				for (int j = 0; j < _thN; j++)
				{
					Tj = Ti;
					Tj.setRotation(Ti.getRotation()*SO3::RotZ(_thSpan(j)));
					//cout << "real kine" << endl;
					//cout << Tj << endl;
					// inverse kinematics
					tempInvKinSol = Kinematics::solveInverseKinematicsOnlyForEfort(*_socAssem, _Tbase.inverse()*Tj*_TlastLinkToEndeffector.inverse());
					//cout << "theta = " << _thSpan(j) << endl;
					//cout << tempInvKinSol[0].transpose() << endl;
					//cout << _Tbase*Tj << endl;
					//cout << Tj*_TlastLinkToEndeffector.inverse() << endl;
					////////////////////// store only upper 4 values for continuity of the solution
					for (int k = 0; k < 4; k++)
					{
						for (int l = 0; l < 6; l++)
							_invKinSol[k][l](j, i) = tempInvKinSol[k](l);
					}
				}
			}

			// matrix spline
			_RCubic.resize(3);
			// cubic spline interpolation
			VectorX sSet(_endIdx - _startIdx + 1);
			Matrix3 dR1, dRn;
			for (int i = 0; i < sSet.size(); i++)
			{
				sSet(i) = i;
			}
			
			dR1 = _Ri[0].matrix() * Bracket(SO3::Log(_Ri[0].inverse()*_Ri[1]));
			dRn = _Ri[_Ri.size() - 2].matrix() * Bracket(SO3::Log(_Ri[_Ri.size() - 2].inverse()*_Ri[_Ri.size() - 1]));
			vector<VectorX> _axisCubic(_Ri.size());
			for (int j = 0; j < 3; j++)
			{
				for (unsigned int i = 0; i < _Ri.size(); i++)
				{
					_axisCubic[i] = _Ri[i].matrix().col(j);
				}
				_RCubic[j].setX(sSet);
				_RCubic[j].setElements(_axisCubic);
				_RCubic[j].setBoundaryConditions(dR1.col(j), dRn.col(j));
			}

			///////////////////////////////////////
			//VectorX error(_endIdx - _startIdx + 1);
			//Real si;
			//SO3 R;
			//Matrix3 Rtemp;
			//for (int i = 0; i < error.size(); i++)
			//{
			//	si = i;
			//	for (int j = 0; j < 3; j++)
			//		Rtemp.col(j) = _RCubic[j](si);
			//	error(i) = (SO3::Projection(Rtemp).getZ() - _zAxisTrj.row(startIdx + i).transpose()).norm();
			//}
			////////////////////////////////////////
			//cout << "error = " << endl;
			//cout << error << endl;
		}

		VectorX GivenPathOptimization::transferJointValueTest(VectorX qiTrj, const Real etha)
		{
			// transfer parts of qi by 2*pi to make qi continuous
			int size = qiTrj.size();
			VectorX qtemp = qiTrj;
			VectorX dqtemp;
			VectorX qtemp2;
			VectorX sinq(0);
			VectorX cosq(0);
			VectorX dsinq(0);
			VectorX dcosq(0);
			int iter = 0;
			int k = 0;
			// etha: tolerance for checking discontinuity
			vector<int> flipIdx;

			// flip forward for once
			flipIdx.resize(0);
			dqtemp = qtemp.bottomRows(size - 1) - qtemp.topRows(size - 1);
			for (int i = 0; i < dqtemp.size(); i++)
			{
				if (abs(dqtemp(i)) > etha)
					flipIdx.push_back(i);
			}

			if (flipIdx.size() == 0)
				return qtemp;
			else
			{
				for (unsigned int k = 0; k < flipIdx.size() - 1; k++)
				{
					if (qtemp(flipIdx[k]) < qtemp(flipIdx[k] + 1))
						qtemp.segment(flipIdx[k] + 1, size - flipIdx[k] - 1) -= PI_DOUBLE * VectorX::Ones(size - flipIdx[k] - 1);
					else
						qtemp.segment(flipIdx[k] + 1, size - flipIdx[k] - 1) += PI_DOUBLE * VectorX::Ones(size - flipIdx[k] - 1);
				}
			}


			if (qtemp.maxCoeff() > PI_DOUBLE)
				qtemp -= PI_DOUBLE * VectorX::Ones(size);
			if (qtemp.minCoeff() < -PI_DOUBLE)
				qtemp += PI_DOUBLE * VectorX::Ones(size);


			////////////////////////////////////////////////////////////////////////////////////
			bool isConnected = false;
			bool isVibrating = false;
			while (1)
			{
				iter += 1;
				if (iter > 5)
				{
					if (isVibrating == false)
						qtemp2 = qtemp;
					isVibrating = true;
					
					if (iter > 6)
						break;
				}
				flipIdx.resize(0);
				dqtemp = qtemp.bottomRows(size - 1) - qtemp.topRows(size - 1);
				for (int i = 0; i < dqtemp.size(); i++)
				{
					if (abs(dqtemp(i)) > etha)
						flipIdx.push_back(i);
				}

				if (flipIdx.size() == 0)
					return qtemp;
				else
				{
					k = 0;
					for (unsigned int j = 0; j < flipIdx.size() - 1; j++)
					{
						if (flipIdx[j + 1] == flipIdx[j] + 1)
						{
							isConnected = true;
							k = j + 1;
						}
						else
							break;
					}

					if (qtemp(flipIdx[k]) < qtemp(flipIdx[k] + 1))
						qtemp.segment(flipIdx[k] + 1, size - flipIdx[k] - 1) -= PI_DOUBLE * VectorX::Ones(size - flipIdx[k] - 1);
					else
						qtemp.segment(flipIdx[k] + 1, size - flipIdx[k] - 1) += PI_DOUBLE * VectorX::Ones(size - flipIdx[k] - 1);

					if (isConnected == true)
					{
						sinq.resize(qtemp.size());
						cosq.resize(qtemp.size());
						dsinq.resize(qtemp.size());
						dcosq.resize(qtemp.size());
						bool isInterpolated = false;

						for (int i = 0; i < qtemp.size(); i++)
						{
							sinq(i) = sin(qtemp(i));
							cosq(i) = cos(qtemp(i));
						}
						dsinq = (sinq.bottomRows(sinq.size() - 1) - sinq.topRows(sinq.size() - 1)).cwiseQuotient(qtemp.bottomRows(sinq.size() - 1) - qtemp.topRows(sinq.size() - 1));
						dcosq = (cosq.bottomRows(cosq.size() - 1) - cosq.topRows(cosq.size() - 1)).cwiseQuotient(qtemp.bottomRows(cosq.size() - 1) - qtemp.topRows(cosq.size() - 1));

						if (dsinq.cwiseAbs().maxCoeff() > 1.0 || dcosq.cwiseAbs().maxCoeff() > 1.0)
							isInterpolated = true;
						if (isInterpolated == true)
						{
							cout << "a" << endl;
							// this is the case when the intermediate points b/w discontinuity are generated due to interpolation 
							// solve this problem by linearly connecting  points back and forth
							for (int i = flipIdx[k - 1]; i < flipIdx[k] + 1; i++)
								qtemp(i) = qtemp(flipIdx[k - 1]) + (qtemp(flipIdx[k] + 1) - qtemp(flipIdx[k - 1])) * ((Real)i - flipIdx[k - 1]) / (Real)(flipIdx[k] + 1 - flipIdx[k - 1]);

							isConnected = false;
						}
						else
						{
							//if (qtemp(flipIdx[0]) < qtemp(flipIdx[0] + 1))
							//	qtemp.segment(flipIdx[0] + 1, size - flipIdx[0] - 1) -= PI_DOUBLE * VectorX::Ones(size - flipIdx[0] - 1);
							//else
							//	qtemp.segment(flipIdx[0] + 1, size - flipIdx[0] - 1) += PI_DOUBLE * VectorX::Ones(size - flipIdx[0] - 1);
							isConnected = false;
						}
					}
				}
				

				if (qtemp.maxCoeff() > PI_DOUBLE)
					qtemp -= PI_DOUBLE * VectorX::Ones(size);
				if (qtemp.minCoeff() < -PI_DOUBLE)
					qtemp += PI_DOUBLE * VectorX::Ones(size);

				//cout << qtemp.transpose() << endl;
			}

			if (isVibrating)
			{
				VectorX ddqtemp = qtemp.bottomRows(qtemp.size() - 2) + qtemp.topRows(qtemp.size() - 2) - 2.0*qtemp.segment(1, qtemp.size() - 2);
				VectorX ddqtemp2 = qtemp2.bottomRows(qtemp2.size() - 2) + qtemp2.topRows(qtemp2.size() - 2) - 2.0*qtemp2.segment(1, qtemp2.size() - 2);
				Real absnorm = ddqtemp.cwiseAbs().transpose()*VectorX::Ones(ddqtemp.size());
				Real absnorm2 = ddqtemp2.cwiseAbs().transpose()*VectorX::Ones(ddqtemp2.size());
				if ( absnorm > absnorm2 )
					return qtemp2;
				else
					return qtemp;
			}
			else
				return qtemp;
		}

		Math::VectorX GivenPathOptimization::flipJointValue(const Math::VectorX & qTrj, Real q_bf)
		{
			VectorX qtemp;
			qtemp = qTrj;
			if (q_bf > qTrj(0))
				qtemp += PI_DOUBLE * VectorX::Ones(qTrj.size());
			else
				qtemp -= PI_DOUBLE * VectorX::Ones(qTrj.size());
			return qtemp;
		}

		Math::VectorX GivenPathOptimization::transferJointValue(const Math::VectorX& qiTrj)
		{
			int len = qiTrj.size();
			VectorX dqiTrj = qiTrj.bottomRows(len - 1) - qiTrj.topRows(len - 1);
			vector<vector<int>> flipIdx(0);
			Real mean_dq = dqiTrj.cwiseAbs().mean();
			vector<int> flipSubIdx(0);
			VectorX qTransferred = qiTrj;
			int r;
			int c;
			int iter = 0;
			VectorX qtemp(0);
			int s;
			while (1)
			{
				flipIdx.resize(0);
				flipSubIdx.resize(0);
				iter += 1;
				for (int i = 0; i < len - 1; i++)
				{
					if (abs(dqiTrj(i)) > 10.0 * mean_dq)
					{
						if (flipIdx.size() == 0)
						{
							r = 0;
							c = 0;
							flipSubIdx.push_back(i);
							flipIdx.push_back(flipSubIdx);
						}
						else
						{
							if (flipIdx[c][r] == i - 1)
							{
								r += 1;
								flipIdx[c].push_back(i);
							}
							else
							{
								r = 0;
								c += 1;
								flipSubIdx[0] = i;
								flipIdx.push_back(flipSubIdx);
							}
						}
					}
				}

				if (flipIdx.size() == 0)
					break;
				if (iter > 10)
				{
					cout << "q6 discontinuity..." << endl;
					//cout << flipIdx.size() << endl;
					//cout << qTransferred.transpose() << endl;
					break;
				}
				
				for (unsigned int i = 0; i < flipIdx.size(); i++)
				{
					s = flipIdx[i].size();
					if (s == 1)
					{
						qtemp = flipJointValue(qTransferred.bottomRows(len - flipIdx[i][0] - 1), qTransferred(flipIdx[i][0]));
						qTransferred.bottomRows(len - flipIdx[i][0] - 1) = qtemp;
					}
					else
					{
						qtemp = flipJointValue(qTransferred.bottomRows(len - flipIdx[i][s - 1] - 1), qTransferred(flipIdx[i][0]));
						qTransferred.bottomRows(len - flipIdx[i][s - 1] - 1) = qtemp;
						for (int j = 0; j < s + 1; j++)
						{
							qTransferred(flipIdx[i][0] + j) = qTransferred(flipIdx[i][0]) + (qTransferred(flipIdx[i][s - 1] + 1) - qTransferred(flipIdx[i][0])) * ((Real)j / (Real)s);
							//cout << qTransferred(flipIdx[i][0] + j) << "   changed"<< endl;
						}
					}
				}
			}
			return qTransferred;
		}

		VectorX GivenPathOptimization::transferJointValueOld(VectorX qiTrj, const Real etha)
		{
			// transfer parts of qi by 2*pi to make qi continuous
			int size = qiTrj.size();
			VectorX qtemp = qiTrj;
			VectorX dqtemp;
			int iter = 0;
			int k = 0;
			// etha: tolerance for checking discontinuity
			vector<int> flipIdx;

			/////////////////////////////////////////////////////////////
			bool isConnected = false;
			while (1)
			{
				iter += 1;
				if (iter > 10)
				{
					break;
				}
				flipIdx.resize(0);
				dqtemp = qtemp.bottomRows(size - 1) - qtemp.topRows(size - 1);
				for (int i = 0; i < dqtemp.size(); i++)
				{
					if (abs(dqtemp(i)) > etha)
						flipIdx.push_back(i);
				}

				if (flipIdx.size() == 0)
					return qtemp;
				else
				{
					k = 0;
					for (unsigned int j = 0; j < flipIdx.size() - 1; j++)
					{
						if (flipIdx[j + 1] == flipIdx[j] + 1)
						{
							isConnected = true;
							k = j + 1;
						}
						else
							break;
					}

					if (qtemp(flipIdx[k]) < qtemp(flipIdx[k] + 1))
						qtemp.segment(flipIdx[k] + 1, size - flipIdx[k] - 1) -= PI_DOUBLE * VectorX::Ones(size - flipIdx[k] - 1);
					else
						qtemp.segment(flipIdx[k] + 1, size - flipIdx[k] - 1) += PI_DOUBLE * VectorX::Ones(size - flipIdx[k] - 1);

					if (isConnected == true)
					{

						// this is the case when the intermediate points b/w discontinuity are generated due to interpolation 
						// solve this problem by linearly connecting  points back and forth
						for (int i = flipIdx[k - 1]; i < flipIdx[k] + 1; i++)
							qtemp(i) = qtemp(flipIdx[k - 1]) + (qtemp(flipIdx[k] + 1) - qtemp(flipIdx[k - 1])) * ((Real)i - flipIdx[k - 1]) / (Real)(flipIdx[k] + 1 - flipIdx[k - 1]);

						isConnected = false;
					}
				}


				if (qtemp.maxCoeff() > PI_DOUBLE)
					qtemp -= PI_DOUBLE * VectorX::Ones(size);
				if (qtemp.minCoeff() < -PI_DOUBLE)
					qtemp += PI_DOUBLE * VectorX::Ones(size);

				//cout << qtemp.transpose() << endl;
			}

			return qiTrj;
		}


		Math::Real GivenPathOptimization::getManipulability(const Math::Matrix6X& J)
		{
			Matrix6 JJt = J*J.transpose();

			Eigen::JacobiSVD<Math::Matrix6> svd(JJt, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Vector6 singular_values = svd.singularValues();


			return singular_values.minCoeff() / singular_values.maxCoeff();
		}

		void GivenPathOptimization::findFeasibleJointSpace(const int invIdx, const Real manipTol, const Real invSolTol)
		{
			_invIdx = invIdx;
			_feasibleSet.resize(2 * _thN - 1, _sN);		// theta range from -2*pi to 2*pi
			_feasibleSet.setZero();
			_feasibleScore = 0;
			// check manipulability
			_feasibleScore += 1;
			Matrix6X J;
			VectorX q(_nDOF);
			MatrixX finiteDiffq4(_thN, _sN);
			MatrixX manipTest(_thN, _sN);
			Real manip;
			for (int i = 0; i < _thN; i++)
			{
				for (int j = 0; j < _sN; j++)
				{
					for (int k = 0; k < q.size(); k++)
						q(k) = _invKinSol[invIdx][k](i, j);
					_defaultState->setJointq(State::TARGET_JOINT::ACTIVEJOINT, q);
					J = Kinematics::computeJacobian(*_socAssem, *_defaultState);
					manip = getManipulability(SE3::InvAd(Kinematics::calculateEndeffectorFrame(*_socAssem, *_defaultState)*_TlastLinkToEndeffector)*J);
					manipTest(i, j) = manip;
					if (manip > manipTol)
					{
						_feasibleSet(i, j) += 1;
						if (i > 0)
							_feasibleSet(i + _thN - 1, j) += 1;
					}
				}
			}
			
			//cout << manipTest.block(0, 0, 10, 10) << endl;
			// flip 6-th joint
			for (int i = 0; i < _sN; i++)
			{
				//cout << "before transfer: " << endl;
				//cout << _invKinSol[invIdx][5].col(i).transpose() << endl;
				_invKinSol[invIdx][5].col(i) = transferJointValue(_invKinSol[invIdx][5].col(i));
				//cout << "after transfer: " << endl;
				//cout << _invKinSol[invIdx][5].col(i).transpose() << endl;
			}

			// check joint limit
			_feasibleScore += _nDOF;
			MatrixX jointLimitTest(_thN, _sN);
			jointLimitTest.setZero();
			for (int i = 0; i < _thN; i++)
			{
				for (int j = 0; j < _sN; j++)
				{
					for (int k = 0; k < _nDOF; k++)
					{
						if (_invKinSol[invIdx][k](i, j) < _socAssem->getJointPtrByMateIndex(k)->getLimitPosUpper()(0) && _invKinSol[invIdx][k](i, j) > _socAssem->getJointPtrByMateIndex(k)->getLimitPosLower()(0))
						{
							_feasibleSet(i, j) += 1;
							jointLimitTest(i, j) += 1;
							if (i > 0)
								_feasibleSet(i + _thN - 1, j) += 1;
						}
					}
				}
			}

			// check continuity of wrist joint by finite difference (q4)
			_feasibleScore += 1;
			MatrixX finiteDiffTest(_thN, _sN);
			finiteDiffTest.setZero();
			finiteDiffq4.setZero();
			finiteDiffq4.block(0, 0, _thN, _sN - 1) += (_invKinSol[invIdx][3].block(0, 1, _thN, _sN - 1) - _invKinSol[invIdx][3].block(0, 0, _thN, _sN - 1)).cwiseAbs();
			finiteDiffq4.block(0, 0, _thN - 1, _sN) += (_invKinSol[invIdx][3].block(1, 0, _thN - 1, _sN) - _invKinSol[invIdx][3].block(0, 0, _thN - 1, _sN)).cwiseAbs();
			finiteDiffq4.row(_thN - 1) = finiteDiffq4.row(0);

			for (int i = 0; i < _thN; i++)
			{
				for (int j = 0; j < _sN; j++)
				{
					if (finiteDiffq4(i, j) < invSolTol)
					{
						_feasibleSet(i, j) += 1;
						finiteDiffTest(i, j) += 1;
						if (i > 0)
							_feasibleSet(i + _thN - 1, j) += 1;
					}
				}
			}
			
			
			
			//cout << "manip : " << endl;
			//cout << manipTest.block(0, 0, 10, 10) << endl;
			//cout << "jointLimit : " << endl;
			//cout << jointLimitTest.block(0, 0, 10, 10) << endl;
			//cout << "finiteDiff : " << endl;
			//cout << finiteDiffTest.block(0, 0, 10, 10) << endl;
			//cout << "diffVal : " << endl;
			//cout << finiteDiffq4.block(0, 0, 10, 10) << endl;
			
			//cout << "total : " << endl;
			//cout << _feasibleSet.block(0, 0, 19, 10) << endl;
		}

		void GivenPathOptimization::generateSdotMax(const int startIdx, const int endIdx)
		{
			int sN = endIdx - startIdx + 1;
			_sdotMax.resize(sN);
			for (int i = 0; i < sN; i++)
				_sdotMax(i) = _feedRate / _speed(startIdx + i);
			VectorX sdotMaxTemp(sN);
			for (int i = 0; i < sN; i++)
				sdotMaxTemp(i) = 2.0 / _robotTimeStep*sqrt(2.0 * _chordError / _curvature(startIdx + i) - _chordError*_chordError ) / _speed(startIdx + i);
			
			//cout << _curvature(startIdx) << endl;
			//cout << _speed(startIdx) << endl;
			//cout << "from feedrate" << endl;
			//cout << _sdotMax << endl;
			//cout << "from curvature" << endl;
			//cout << sdotMaxTemp << endl;
			for (int i = 0; i < sN; i++)
			{
				if (_sdotMax(i) > sdotMaxTemp(i))
					_sdotMax(i) = sdotMaxTemp(i);
			}
			//cout << "sdotmax: " << endl;
			//cout << _sdotMax << endl;
		}

		void GivenPathOptimization::findContinuousFeasibleSearchSpace()
		{
			// find feasible theta bounds which satisfies manipulability, joint limit, inverse kinematics solution continuity, (collision avoidance)

			// get continuous set which can go to initial s0 to final sf
			int thN = _feasibleSet.rows();
			int sN = _feasibleSet.cols();
			Eigen::Matrix<int, -1, -1> contiSet(thN, sN);		// save the set number
			contiSet.setZero();
			Eigen::Matrix<int, -1, -1> contiSetTemp(thN, sN);
			contiSetTemp.setZero();
			int setNumber = 0;				// setNumber starts from 1
			int setNumberTemp1;
			int setNumberTemp2;
			vector<int> connectedSetNumber(0);
			VectorU check(2);

			_contiFeasibleRegion.resize(0);

			for (int i = 0; i < thN; i++)
			{
				if (_feasibleSet(i, 0) == _feasibleScore)
				{
					if (setNumber == 0)
						setNumber += 1;
					else if (_feasibleSet(i - 1, 0) != _feasibleScore)
						setNumber += 1;
					contiSet(i, 0) = setNumber;
				}
			}

			for (int j = 1; j < sN; j++)
			{
				if (_feasibleSet(0, j) == _feasibleScore)
				{
					if (contiSet(0, j - 1) != 0)
						contiSet(0, j) = contiSet(0, j - 1);
					else
					{
						setNumber += 1;
						contiSet(0, j) = setNumber;
					}
				}

				for (int i = 1; i < thN; i++)
				{
					if (_feasibleSet(i, j) == _feasibleScore)
					{
						if (contiSet(i - 1, j) != 0 && contiSet(i, j - 1) != 0)
						{
							if (contiSet(i - 1, j) == contiSet(i, j - 1))
								contiSet(i, j) = contiSet(i - 1, j);
							else
							{
								setNumberTemp1 = contiSet(i - 1, j);
								setNumberTemp2 = contiSet(i, j - 1);
								// should be improved later
								for (int k = 0; k < thN; k++)
								{
									for (int l = 0; l < j + 1; l++)
									{
										if (contiSet(k, l) == setNumberTemp2)
											contiSet(k, l) = setNumberTemp1;
									}
								}
								contiSet(i, j) = setNumberTemp1;
							}
						}
						else if (contiSet(i - 1, j) == 0 && contiSet(i, j - 1) != 0)
						{
							contiSet(i, j) = contiSet(i, j - 1);
						}
						else if (contiSet(i - 1, j) != 0 && contiSet(i, j - 1) == 0)
						{
							contiSet(i, j) = contiSet(i - 1, j);
						}
						else
						{
							setNumber += 1;
							contiSet(i, j) = setNumber;
						}
					}
				}
			}
			//cout << contiSet.block(0,0,19,10) << endl;

			_domain.resize(thN, sN);
			int size_bf;
			int size_cur;

			// find connected set from s0 to sf
			for (int k = 1; k < setNumber + 1; k++) 
			{
				check.setZero();
				for (int i = 0; i < thN; i++)
				{
					if (contiSet(i, 0) == k)
						check(0) += 1;
					if (contiSet(i, sN - 1) == k)
						check(1) += 1;
				}
				if (check(0) > 0 && check(1) > 0)
					connectedSetNumber.push_back(k);
			}
			int val;
			vector<vector<int>> temp;
			vector<int> region;
			int sSumBefore;
			int sSumNext;
			
			// erase unreachable region from the condition sdot >= 0
			for (unsigned int k = 0; k < connectedSetNumber.size(); k++)
			{
				val = connectedSetNumber[k];
				contiSetTemp = contiSet;
				for (int i = 0; i < thN; i++)
				{
					for (int j = 0; j < sN; j++)
					{
						if (contiSetTemp(i, j) != val)
							contiSetTemp(i, j) = 0;
					}
				}
				// erase unreachable region by forward pass (s is monotone increasing)
				temp.resize(0);
				for (int j = 1; j < sN; j++)
				{
					region.resize(0);
					for (int i = 0; i < thN; i++)
					{
						if (contiSetTemp(i, j) == val)
						{
							if (region.size() == 0)
							{
								region.push_back(i);
							}
							else
							{
								if (contiSetTemp(i - 1, j) == val)
								{
									region.push_back(i);
								}
								else
								{
									temp.push_back(region);
									region.resize(0);
									region.push_back(i);
								}
							}

						}
						
					}
					sSumBefore = 0;
					for (unsigned int i = 0; i < temp.size(); i++)
					{
						for (unsigned int l = 0; l < temp[i].size(); l++)
						{
							sSumBefore += (contiSetTemp(temp[i][l], j - 1));
						}
					}
					if (sSumBefore == 0)
					{
						for (unsigned int i = 0; i < temp.size(); i++)
						{
							for (unsigned int l = 0; l < temp[i].size(); l++)
								contiSetTemp(temp[i][l], j) = 0;
						}
					}

				}


				// erase unreachable region by backward pass (s is monotone decreasing)
				temp.resize(0);
				for (int j = 1; j < sN; j++)
				{
					region.resize(0);
					for (int i = 0; i < thN; i++)
					{
						if (contiSetTemp(i, sN - 1 - j) == val)
						{
							if (region.size() == 0)
							{
								region.push_back(i);
							}
							else
							{
								if (contiSetTemp(i - 1, sN - 1 - j) == val)
								{
									region.push_back(i);
								}
								else
								{
									temp.push_back(region);
									region.resize(0);
									region.push_back(i);
								}
							}

						}
					}
					sSumNext = 0;
					for (unsigned int i = 0; i < temp.size(); i++)
					{
						for (unsigned int l = 0; l < temp[i].size(); l++)
						{
							sSumNext += (contiSetTemp(temp[i][l], sN - j));
						}
					}
					if (sSumNext == 0)
					{
						for (unsigned int i = 0; i < temp.size(); i++)
						{
							for (unsigned int l = 0; l < temp[i].size(); l++)
								contiSetTemp(temp[i][l], sN - 1 - j) = 0;
						}
					}

				}
				_contiFeasibleRegion.push_back(contiSetTemp);
				// save biggest connected region
				if (k == 0)
				{
					_domain = contiSetTemp;
					size_cur = 0;
					for (int i = 0; i < thN; i++)
					{
						for (int j = 0; j < sN; j++)
						{
							if (contiSetTemp(i, j) > 0)
								size_cur += 1;
						}
					}
				}
				else
				{
					size_bf = size_cur;
					size_cur = 0;
					for (int i = 0; i < thN; i++)
					{
						for (int j = 0; j < sN; j++)
						{
							if (contiSetTemp(i, j) > 0)
								size_cur += 1;
						}
					}
					if (size_cur > size_bf)
						_domain = contiSetTemp;
				}
			}
		}

		void GivenPathOptimization::setThetaBound(Eigen::Matrix<int, -1, -1>& feasibleRegion)
		{
			int val = feasibleRegion.col(0).maxCoeff();
			int sN = feasibleRegion.cols();
			int thN = feasibleRegion.rows();
			_thMin.resize(sN);
			_thMax.resize(sN);
			bool minChecked;
			bool maxChecked;
			for (int j = 0; j < sN; j++)
			{
				minChecked = false;
				maxChecked = false;
				for (int i = 0; i < thN; i++)
				{
					if (feasibleRegion(i, j) == val)
					{
						if (minChecked == false)
						{
							_thMin(j) = i;
							minChecked = true;
						}
					}
					else if (minChecked == true && maxChecked == false)
					{
						_thMax(j) = i - 1;
						maxChecked = true;
					}
					if (maxChecked == true)
						break;
				}
			}
			//cout << "min = " << endl;
			//cout << _thMin << endl;
			//cout << "max = " << endl;
			//cout << _thMax << endl;
			_thMin *= PI_DOUBLE / (Real)((thN + 1) / 2 - 1);
			_thMax *= PI_DOUBLE / (Real)((thN + 1) / 2 - 1);
			_thMin -= PI_DOUBLE*VectorX::Ones(sN);
			_thMax -= PI_DOUBLE*VectorX::Ones(sN);
			_thBoundGenerated = true;
		}

		void GivenPathOptimization::setThetaBound()
		{
			setThetaBound(_domain);
		}

		void GivenPathOptimization::setBoundaryConditionForSdot(const VectorX sdot0, const VectorX sdotf, const VectorX sddot0, const VectorX sddotf)
		{
			_sdot0 = sdot0;
			_sdotf = sdotf;
			_sddot0 = sddot0;
			_sddotf = sddotf;
		}

		void GivenPathOptimization::setBoundaryConditionForTh(const VectorX th0, const VectorX thf, const VectorX thdot0, const VectorX thdotf, const VectorX thddot0, const VectorX thddotf)
		{
			if (_thBoundGenerated && th0.size() > 0)
			{
				if (th0(0) > _thMin(0) && th0(0) < _thMax(0))
					_th0 = th0;
				else
				{
					cout << "Given theta0 is not feasible, and moved to satisfy bounds." << endl;
					_th0.resize(1);
					_th0(0) = 0.5*(_thMin(0) + _thMax(0));
				}
			}
			else
				_th0 = th0;
			_thf = thf;
			_thdot0 = thdot0;
			_thdotf = thdotf;
			_thddot0 = thddot0;
			_thddotf = thddotf;
		}

		vector<VectorX> GivenPathOptimization::sortInvKinSol(vector<VectorX> qCur, const vector<VectorX> qBf)
		{
			return std::vector<Math::VectorX>();
		}

		BSplineGivenPathOptimization::BSplineGivenPathOptimization()
		{
			
		}

		BSplineGivenPathOptimization::~BSplineGivenPathOptimization()
		{
		}

		void BSplineGivenPathOptimization::setSplineCondition(const unsigned int orderTh, const unsigned int nMiddleCPTh, const unsigned int orderS, const unsigned int nMiddleCPS, bool isKnotUniform, int par)
		{
			_orderTh = orderTh;
			_orderSdot = orderS;
			_nMiddleCPTh = nMiddleCPTh;
			_nMiddleCPSdot = nMiddleCPS;
			_nInitCPsdot = 0;
			_nFinalCPsdot = 0;

			if (_sdot0.size() > 0)
			{
				_nInitCPsdot += 1;
				if (_sddot0.size() > 0)
					_nInitCPsdot += 1;
			}

			if (_sdotf.size() > 0)
			{
				_nFinalCPsdot += 1;
				if (_sddotf.size() > 0)
					_nFinalCPsdot += 1;
			}

			if (par < 0)
				par = 10;
			if (par % 2 == 1)
				par += 1;

			// set knot vector for sdot
			int nK_sdot = _nInitCPsdot + _nFinalCPsdot + _nMiddleCPSdot + _orderSdot;

			_knotSdot.resize(nK_sdot);
			for (int i = 0; i < _orderSdot; i++)
				_knotSdot(i) = 0;
			for (int i = 0; i < _orderSdot; i++)
				_knotSdot(nK_sdot - 1 - i) = _sf;
			
			for (int i = 0; i < nK_sdot - 2 * _orderSdot; i++)
				_knotSdot(_orderSdot + i) = (Real)(i + 1) / (Real)(nK_sdot - 2 * _orderSdot + 1) * _sf;

			// set boundary control points
			_boundaryCPsdot.resize(4);
			_boundaryCPsdot[0] = _sdot0;
			_boundaryCPsdot[3] = _sdotf;
			if (_sddot0.size() > 0)
				_boundaryCPsdot[1] = (_knotSdot(_orderSdot) - _knotSdot(_orderSdot - 1)) / (Real)(_orderSdot - 1) * _sddot0 + _sdot0;
			else
				_boundaryCPsdot[1] = _sddot0;
			if (_sddotf.size() > 0)
				_boundaryCPsdot[2] = -(_knotSdot(nK_sdot - _orderSdot) - _knotSdot(nK_sdot - _orderSdot - 1)) / (Real)(_orderSdot - 1) * _sddotf + _sdotf;
			else
				_boundaryCPsdot[2] = _sddotf;
			

			// set knot vector for theta
			_nInitCPTh = 0;
			_nFinalCPTh = 0;
			if (_th0.size() > 0)
			{
				_nInitCPTh += 1;
				if (_thdot0.size() > 0)
				{
					_nInitCPTh += 1;
					if (_thddot0.size() > 0)
						_nInitCPTh += 1;
				}
			}
			if (_thf.size() > 0)
			{
				_nFinalCPTh += 1;
				if (_thdotf.size() > 0)
				{
					_nFinalCPTh += 1;
					if (_thddotf.size() > 0)
						_nFinalCPTh += 1;
				}
			}
			int nK_th = _nInitCPTh + _nFinalCPTh + _nMiddleCPTh + _orderTh;

			_knotTh.resize(nK_th);
			for (int i = 0; i < _orderTh; i++)
				_knotTh(i) = 0;
			for (int i = 0; i < _orderTh; i++)
				_knotTh(nK_th - i - 1) = _sf;
			for (int i = 0; i < nK_th - 2 * _orderTh; i++)
				_knotTh(_orderTh + i) = (Real)(i + 1) / (Real)(nK_th - 2 * _orderTh + 1) * _sf;

			// Boundary control points for initial and final val, vel, acc
			_boundaryCPth.resize(6);
			_boundaryCPth[0] = _th0;
			_boundaryCPth[5] = _thf;
			if (_thdot0.size() > 0)
				_boundaryCPth[1] = (_knotTh(_orderTh) - _knotTh(_orderTh - 1)) / (Real)(_orderTh - 1) * _thdot0 + _th0;
			else
				_boundaryCPth[1] = _thdot0;
			if (_thdotf.size() > 0)
				_boundaryCPth[4] = -(_knotTh(nK_th - _orderTh) - _knotTh(nK_th - _orderTh - 1)) / (Real)(_orderTh - 1) * _thdotf + _thf;
			else
				_boundaryCPth[4] = _thdotf;
			if (_thddot0.size() > 0)
				_boundaryCPth[2] = (_knotTh(_orderTh + 1) - _knotTh(_orderTh - 1))*((_knotTh(_orderTh) - _knotTh(_orderTh - 1)) / (Real)(_orderTh - 1) / (Real)(_orderTh - 2)*_thddot0 + _boundaryCPth[1] * (1.0 / (_knotTh(_orderTh + 1) - _knotTh(_orderTh - 1)) + 1.0 / (_knotTh(_orderTh) - _knotTh(_orderTh - 1))) - _th0 / (_knotTh(_orderTh) - _knotTh(_orderTh - 1)));
			else
				_boundaryCPth[2] = _thddot0;
			if (_thddotf.size() > 0)
				_boundaryCPth[3] = (_knotTh(nK_th - _orderTh) - _knotTh(nK_th - _orderTh - 2)) * ((_knotTh(nK_th - _orderTh) - _knotTh(nK_th - _orderTh - 1)) / (Real)(_orderTh - 1) / (Real)(_orderTh - 2)*_thddotf + _boundaryCPth[4] * (1.0 / (_knotTh(nK_th - _orderTh) - _knotTh(nK_th - _orderTh - 2)) + 1.0 / (_knotTh(nK_th - _orderTh) - _knotTh(nK_th - _orderTh - 1))) - _thf / (_knotTh(nK_th - _orderTh) - _knotTh(nK_th - _orderTh - 1)));
			else
				_boundaryCPth[3] = _thddotf;

		}

		void BSplineGivenPathOptimization::setSplineConditionForThetaOnly(const unsigned int orderTh, const unsigned int nMiddleCPTh)
		{
			_orderTh = orderTh;
			_nMiddleCPTh = nMiddleCPTh;
			
			// set knot vector for theta
			_nInitCPTh = 0;
			_nFinalCPTh = 0;
			if (_th0.size() > 0)
			{
				_nInitCPTh += 1;
				if (_thdot0.size() > 0)
				{
					_nInitCPTh += 1;
					if (_thddot0.size() > 0)
						_nInitCPTh += 1;
				}
			}
			if (_thf.size() > 0)
			{
				_nFinalCPTh += 1;
				if (_thdotf.size() > 0)
				{
					_nFinalCPTh += 1;
					if (_thddotf.size() > 0)
						_nFinalCPTh += 1;
				}
			}
			int nK_th = _nInitCPTh + _nFinalCPTh + _nMiddleCPTh + _orderTh;

			_knotTh.resize(nK_th);
			for (int i = 0; i < _orderTh; i++)
				_knotTh(i) = 0;
			for (int i = 0; i < _orderTh; i++)
				_knotTh(nK_th - i - 1) = _tfForThetaOnly;
			for (int i = 0; i < nK_th - 2 * _orderTh; i++)
				_knotTh(_orderTh + i) = (Real)(i + 1) / (Real)(nK_th - 2 * _orderTh + 1) * _tfForThetaOnly;

			// Boundary control points for initial and final val, vel, acc
			_boundaryCPth.resize(6);
			_boundaryCPth[0] = _th0;
			_boundaryCPth[5] = _thf;
			if (_thdot0.size() > 0)
				_boundaryCPth[1] = (_knotTh(_orderTh) - _knotTh(_orderTh - 1)) / (Real)(_orderTh - 1) * _thdot0 + _th0;
			else
				_boundaryCPth[1] = _thdot0;
			if (_thdotf.size() > 0)
				_boundaryCPth[4] = -(_knotTh(nK_th - _orderTh) - _knotTh(nK_th - _orderTh - 1)) / (Real)(_orderTh - 1) * _thdotf + _thf;
			else
				_boundaryCPth[4] = _thdotf;
			if (_thddot0.size() > 0)
				_boundaryCPth[2] = (_knotTh(_orderTh + 1) - _knotTh(_orderTh - 1))*((_knotTh(_orderTh) - _knotTh(_orderTh - 1)) / (Real)(_orderTh - 1) / (Real)(_orderTh - 2)*_thddot0 + _boundaryCPth[1] * (1.0 / (_knotTh(_orderTh + 1) - _knotTh(_orderTh - 1)) + 1.0 / (_knotTh(_orderTh) - _knotTh(_orderTh - 1))) - _th0 / (_knotTh(_orderTh) - _knotTh(_orderTh - 1)));
			else
				_boundaryCPth[2] = _thddot0;
			if (_thddotf.size() > 0)
				_boundaryCPth[3] = (_knotTh(nK_th - _orderTh) - _knotTh(nK_th - _orderTh - 2)) * ((_knotTh(nK_th - _orderTh) - _knotTh(nK_th - _orderTh - 1)) / (Real)(_orderTh - 1) / (Real)(_orderTh - 2)*_thddotf + _boundaryCPth[4] * (1.0 / (_knotTh(nK_th - _orderTh) - _knotTh(nK_th - _orderTh - 2)) + 1.0 / (_knotTh(nK_th - _orderTh) - _knotTh(nK_th - _orderTh - 1))) - _thf / (_knotTh(nK_th - _orderTh) - _knotTh(nK_th - _orderTh - 1)));
			else
				_boundaryCPth[3] = _thddotf;
		}

		void BSplineGivenPathOptimization::setLinearInequalityConstraint()
		{
			// generate Aineq and bineq from sdot bound and theta bound
			_Aineq.resize(3 * _sN + _nMiddleCPSdot, _nMiddleCPSdot + _nMiddleCPTh);
			_Aineq.setZero();
			_bineq.resize(3 * _sN + _nMiddleCPSdot);
			_bineq.setZero();
			// sdot should be bigger than zero
			for (int i = 0; i < _nMiddleCPSdot; i++)
			{
				_Aineq(i, i) = -1.0;
				_bineq(i) = 0.0;
			}

			// set inequality for sdot maximum
			MatrixX tempCPsdot(1, _nInitCPsdot + _nMiddleCPSdot + _nFinalCPsdot);
			tempCPsdot.setZero();
			
			for (int j = 0; j < _nMiddleCPSdot; j++)
			{
				tempCPsdot(0, _nInitCPsdot + j) = 1.0;
				BSpline<-1, -1, 1> tempSplineSdot(_knotSdot, tempCPsdot);
				tempCPsdot(0, _nInitCPsdot + j) = 0.0;

				for (int i = 0; i < _sN; i++)
				{
					_Aineq(_nMiddleCPSdot + i, j) = tempSplineSdot(_sSpan(i))(0);
					_bineq(_nMiddleCPSdot + i) = -_sdotMax(i);
				}
			}
			
			for (int j = 0; j < _nInitCPsdot; j++)
			{
				tempCPsdot(0, j) = 1.0;
				BSpline<-1, -1, 1> tempSplineSdot(_knotSdot, tempCPsdot);
				tempCPsdot(0, j) = 0.0;
				for (int i = 0; i < _sN; i++)
				{
					_bineq(_nMiddleCPSdot + i) += tempSplineSdot(_sSpan(i))(0)*_boundaryCPsdot[j](0);
				}
			}
			for (int j = 0; j < _nFinalCPsdot; j++)
			{
				tempCPsdot(0, _nInitCPsdot + _nMiddleCPSdot + j) = 1.0;
				BSpline<-1, -1, 1> tempSplineSdot(_knotSdot, tempCPsdot);
				tempCPsdot(0, _nInitCPsdot + _nMiddleCPSdot + j) = 0.0;
				for (int i = 0; i < _sN; i++)
				{
					_bineq(_nMiddleCPSdot + i) += tempSplineSdot(_sSpan(i))(0)*_boundaryCPsdot[3 - j](0);
				}
			}

			// set inequality for theta min and max
			MatrixX tempCPth(1, _nInitCPTh + _nMiddleCPTh + _nFinalCPTh);
			tempCPth.setZero();

			for (int j = 0; j < _nMiddleCPTh; j++)
			{
				tempCPth(0, _nInitCPTh + j) = 1.0;
				BSpline<-1, -1, 1> tempSplineTh(_knotTh, tempCPth);
				tempCPth(0, _nInitCPTh + j) = 0.0;

				for (int i = 0; i < _sN; i++)
				{
					_Aineq(_nMiddleCPSdot + _sN + i, _nMiddleCPSdot + j) = tempSplineTh(_sSpan(i))(0);
					_bineq(_nMiddleCPSdot + _sN + i) = -_thMax(i);
					_Aineq(_nMiddleCPSdot + 2*_sN + i, _nMiddleCPSdot + j) = -tempSplineTh(_sSpan(i))(0);
					_bineq(_nMiddleCPSdot + 2*_sN + i) = _thMin(i);
				}
			}

			for (int j = 0; j < _nInitCPTh; j++)
			{
				tempCPth(0, j) = 1.0;
				BSpline<-1, -1, 1> tempSplineTh(_knotTh, tempCPth);
				tempCPth(0, j) = 0.0;
				for (int i = 0; i < _sN; i++)
				{
					_bineq(_nMiddleCPSdot + _sN + i) += tempSplineTh(_sSpan(i))(0)*_boundaryCPth[j](0);
					_bineq(_nMiddleCPSdot + 2*_sN + i) -= tempSplineTh(_sSpan(i))(0)*_boundaryCPth[j](0);
				}
			}
			for (int j = 0; j < _nFinalCPTh; j++)
			{
				tempCPth(0, _nInitCPTh + _nMiddleCPTh + j) = 1.0;
				BSpline<-1, -1, 1> tempSplineTh(_knotTh, tempCPth);
				tempCPth(0, _nInitCPTh + _nMiddleCPTh + j) = 0.0;
				for (int i = 0; i < _sN; i++)
				{
					_bineq(_nMiddleCPSdot + _sN + i) += tempSplineTh(_sSpan(i))(0)*_boundaryCPth[5 - j](0);
					_bineq(_nMiddleCPSdot + 2*_sN + i) -= tempSplineTh(_sSpan(i))(0)*_boundaryCPth[5 - j](0);
				}
			}


		}

		void BSplineGivenPathOptimization::setLinearInequalityConstraintForThetaOnly()
		{
			// generate Aineq and bineq from sdot bound and theta bound
			_Aineq.resize(2 * _sN, _nMiddleCPTh);
			_Aineq.setZero();
			_bineq.resize(2 * _sN);
			_bineq.setZero();

			// set inequality for theta min and max
			MatrixX tempCPth(1, _nInitCPTh + _nMiddleCPTh + _nFinalCPTh);
			tempCPth.setZero();

			for (int j = 0; j < _nMiddleCPTh; j++)
			{
				tempCPth(0, _nInitCPTh + j) = 1.0;
				BSpline<-1, -1, 1> tempSplineTh(_knotTh, tempCPth);
				tempCPth(0, _nInitCPTh + j) = 0.0;

				for (int i = 0; i < _sN; i++)
				{
					_Aineq(i, j) = tempSplineTh(_sSpan(i))(0);
					_bineq(i) = -_thMax(i);
					_Aineq(_sN + i, j) = -tempSplineTh(_sSpan(i))(0);
					_bineq(_sN + i) = _thMin(i);
				}
			}

			for (int j = 0; j < _nInitCPTh; j++)
			{
				tempCPth(0, j) = 1.0;
				BSpline<-1, -1, 1> tempSplineTh(_knotTh, tempCPth);
				tempCPth(0, j) = 0.0;
				for (int i = 0; i < _sN; i++)
				{
					_bineq(i) += tempSplineTh(_sSpan(i))(0)*_boundaryCPth[j](0);
					_bineq(_sN + i) -= tempSplineTh(_sSpan(i))(0)*_boundaryCPth[j](0);
				}
			}
			for (int j = 0; j < _nFinalCPTh; j++)
			{
				tempCPth(0, _nInitCPTh + _nMiddleCPTh + j) = 1.0;
				BSpline<-1, -1, 1> tempSplineTh(_knotTh, tempCPth);
				tempCPth(0, _nInitCPTh + _nMiddleCPTh + j) = 0.0;
				for (int i = 0; i < _sN; i++)
				{
					_bineq(i) += tempSplineTh(_sSpan(i))(0)*_boundaryCPth[5 - j](0);
					_bineq(_sN + i) -= tempSplineTh(_sSpan(i))(0)*_boundaryCPth[5 - j](0);
				}
			}
		}

		BSplineGivenPathOptimization::sharedVar::sharedVar(const Model::socAssemblyPtr socAssem, const int nStep, const Real sf, const VectorX& sSetInvKin, const Math::VectorX& thSetInvKin, std::vector<Math::MatrixX> invKinSolData,
			const int nInitCPsdot, const int nMiddleCPsdot, const int nFinalCPsdot, const int nInitCPTh, const int nMiddleCPTh, const int nFinalCPTh, 
			const VectorX& knotSdot, const VectorX& knotTh, 
			const vector<VectorX>& boundaryCPsdot, const vector<VectorX>& boundaryCPth)
		{
			_isInitiated = false;
			_isIDUpdated = false;
			_isGaussianQuadratureTimeInitialized = false;
			_isGaussianQuadratureSdotInitialized = false;
			_nStep = nStep;
			_sf = sf;
			_timeSpanForObj.resize(_nStep);
			_timeSpanWeight.resize(_nStep);
			_socAssem = socAssem;
			// save inverse kinematics solution data (change range of theta from 0~2*pi to -2*pi~2*pi)
			int sN = invKinSolData[0].cols();
			int thN = invKinSolData[0].rows();
			_invKinData.resize(sN);
			_nDOF = invKinSolData.size();
			for (int i = 0; i < sN; i++)
			{
				_invKinData[i].resize(2 * thN - 1);
				for (int j = 0; j < thN; j++)
				{
					_invKinData[i][j].resize(invKinSolData.size());
					_invKinData[i][j + thN - 1].resize(invKinSolData.size());
					for (int k = 0; k < _nDOF; k++)
					{
						_invKinData[i][j](k) = invKinSolData[k](j, i);
						_invKinData[i][j + thN - 1](k) = invKinSolData[k](j, i);
					}
				}
				for (int k = 0; k < _nDOF; k++)
					_invKinData[i][2 * thN - 2](k) = invKinSolData[k](0, i);
			}
			_sSetInvKin = sSetInvKin;
			_thSetInvKin = thSetInvKin;
			// set bilinear interpolation for q(s,th(s))
			_bilinterp.setX(_sSetInvKin);
			_bilinterp.setY(_thSetInvKin);
			_bilinterp.setElements(_invKinData);


			// set knot vector
			_knotSdot = knotSdot;
			_knotTh = knotTh;
			// set boundary controlpoints
			_nInitCPsdot = nInitCPsdot;
			_nMiddleCPSdot = nMiddleCPsdot;
			_nFinalCPsdot = nFinalCPsdot;
			_nInitCPTh = nInitCPTh;
			_nMiddleCPTh = nMiddleCPTh;
			_nFinalCPTh = nFinalCPTh;
			_boundaryCPsdot = boundaryCPsdot;
			_boundaryCPth = boundaryCPth;


			_sdotCP.resize(_nInitCPsdot + _nMiddleCPSdot + _nFinalCPsdot);
			for (int i = 0; i < _nInitCPsdot; i++)
				_sdotCP(i) = _boundaryCPsdot[i](0);
			for (int i = 0; i < _nFinalCPsdot; i++)
				_sdotCP(_sdotCP.size() - 1 - i) = _boundaryCPsdot[3 - i](0);
			_thCP.resize(_nInitCPTh + _nMiddleCPTh + _nFinalCPTh);
			for (int i = 0; i < _nInitCPTh; i++)
				_thCP(i) = _boundaryCPth[i](0);
			for (int i = 0; i < _nFinalCPTh; i++)
				_thCP(_thCP.size() - 1 - i) = _boundaryCPth[5 - i](0);

			// set state
			_stateTrj.resize(_nStep);
			for (int i = 0; i < _nStep; i++)
				_stateTrj[i] = socAssem->makeState();
			_optimizeThetaOnly = false;
		}

		BSplineGivenPathOptimization::sharedVar::sharedVar(const Model::socAssemblyPtr socAssem, const int nStep, const Real tf, const CubicSplineInterpolation& sCubic, const CubicSplineInterpolation& realSCubic, const Math::VectorX & sSetInvKin, const Math::VectorX & thSetInvKin, std::vector<Math::MatrixX> invKinSolData, const int nInitCPTh, const int nMiddleCPTh, const int nFinalCPTh, const Math::VectorX & knotTh, const std::vector<Math::VectorX>& boundaryCPth)
		{
			_isInitiated = false;
			_isIDUpdated = false;
			_isGaussianQuadratureTimeInitialized = false;
			_isGaussianQuadratureSdotInitialized = false;
			_socAssem = socAssem;
			_nStep = nStep;
			_tf = tf;
			_sCubic = sCubic;
			_realSCubic = realSCubic;
			_timeSpanForObj.resize(_nStep);
			_timeSpanWeight.resize(_nStep);
			
			// save time span and s(t)
			setTimeSpan();
			_sSpanwrtTimeSpan.resize(_nStep);

			for (int i = 0; i < _nStep; i++)
				_sSpanwrtTimeSpan(i) = _sCubic(_realSCubic(_timeSpanForObj(i))(0))(0);

			// save inverse kinematics solution data (change range of theta from 0~2*pi to -2*pi~2*pi)
			int sN = invKinSolData[0].cols();
			int thN = invKinSolData[0].rows();
			_invKinData.resize(sN);
			_nDOF = invKinSolData.size();
			for (int i = 0; i < sN; i++)
			{
				_invKinData[i].resize(2 * thN - 1);
				for (int j = 0; j < thN; j++)
				{
					_invKinData[i][j].resize(invKinSolData.size());
					_invKinData[i][j + thN - 1].resize(invKinSolData.size());
					for (int k = 0; k < _nDOF; k++)
					{
						_invKinData[i][j](k) = invKinSolData[k](j, i);
						_invKinData[i][j + thN - 1](k) = invKinSolData[k](j, i);
					}
				}
				for (int k = 0; k < _nDOF; k++)
					_invKinData[i][2 * thN - 2](k) = invKinSolData[k](0, i);
			}
			_sSetInvKin = sSetInvKin;
			_thSetInvKin = thSetInvKin;
			// set bilinear interpolation for q(s,th(s))
			_bilinterp.setX(_sSetInvKin);
			_bilinterp.setY(_thSetInvKin);
			_bilinterp.setElements(_invKinData);


			// set knot vector
			_knotTh = knotTh;
			// set boundary controlpoints
			_nInitCPTh = nInitCPTh;
			_nMiddleCPTh = nMiddleCPTh;
			_nFinalCPTh = nFinalCPTh;
			_boundaryCPth = boundaryCPth;

			_thCP.resize(_nInitCPTh + _nMiddleCPTh + _nFinalCPTh);
			for (int i = 0; i < _nInitCPTh; i++)
				_thCP(i) = _boundaryCPth[i](0);
			for (int i = 0; i < _nFinalCPTh; i++)
				_thCP(_thCP.size() - 1 - i) = _boundaryCPth[5 - i](0);

			// set state
			_stateTrj.resize(_nStep);
			for (int i = 0; i < _nStep; i++)
				_stateTrj[i] = socAssem->makeState();
			_optimizeThetaOnly = true;
		}

		void BSplineGivenPathOptimization::sharedVar::getSFromSdot()
		{
			//cout << _knotSdot.transpose() << endl;

			// 1/sdot integration
			VectorX sspan(101);
			for (int i = 0; i < sspan.size(); i++)
			{
				sspan(i) = 0 + (Real)i * (_sf - 0) / (Real)(sspan.size() - 1);
			}
			sspan(sspan.size() - 1) = _sf - 1e-10;
			VectorX sdot_s(101);

			sdot_s = _sdotSpline(sspan);
			//cout << sdot_s(sdot_s.size() - 1) << endl;
			// generate cumulative density
			VectorX dsdot_ds = (sdot_s.bottomRows(sdot_s.size() - 1) - sdot_s.topRows(sdot_s.size() - 1)) / (sspan(2) - sspan(1));
			//cout << "dsdot_ds: " << endl;
			//cout << dsdot_ds << endl;
			
			VectorX cumul(dsdot_ds.size() + 1);
			Real uniformity = 0.02*(dsdot_ds.cwiseAbs().transpose()*VectorX::Ones(dsdot_ds.size()) / (Real)dsdot_ds.size())(0);

			cumul.setZero();
			for (int i = 0; i < cumul.size() - 1; i++)
			{
				cumul(i + 1) = cumul(i) + abs(dsdot_ds(i)) + uniformity;
			}
			//cout << cumul.maxCoeff() << endl;
			cumul /= cumul.maxCoeff();

			Real etha = 0.002;		// should be less than 0.01
			Real minCoeff = abs(_sdotSpline._controlPoints.row(0).maxCoeff());
			Real temp;
			for (int i = 0; i < _sdotSpline._controlPoints.cols(); i++)
			{
				temp = _sdotSpline._controlPoints.col(i)(0);
				if (temp > 0)
				{
					if (minCoeff > temp)
						minCoeff = temp;
				}
			}

			if (minCoeff < 0.1)
				minCoeff = 0.1;
			int num_s = (int) round(1.0 / (minCoeff * etha));

			//cout << "cumul:" << endl;
			//cout << cumul << endl;

			LinearInterpolation linterp;
			linterp.setX(cumul);
			vector<VectorX> sspan_v(sspan.size());
			VectorX a(1);
			for (int i = 0; i < sspan.size(); i++)
			{
				a(0) = sspan(i);
				sspan_v[i] = a;
			}
			linterp.setElements(sspan_v);

			//cout << linterp(0.1) << endl;

			_sSpanwrtTimeSpan.resize(num_s);
			for (int i = 0; i < _sSpanwrtTimeSpan.size(); i++)
				_sSpanwrtTimeSpan(i) = linterp((Real)i / (Real)(num_s - 1))(0);

		/*	for (int i = 0; i < _sSpanwrtTimeSpan.size() / 20; i++)
				cout << _sSpanwrtTimeSpan.segment(20*i, 20) << endl;*/

			VectorX sdotSet = _sdotSpline(_sSpanwrtTimeSpan);

			//cout << sdotSet.segment(0, 20) << endl;


			_timeSpanForS.resize(_sSpanwrtTimeSpan.size());
			_timeSpanForS.setZero();
			for (int i = 0; i < _sSpanwrtTimeSpan.size() - 1; i++)
			{
				if (sdotSet(i) > 0)
					_timeSpanForS(i + 1) = _timeSpanForS(i) + 1.0 / sdotSet(i) * (_sSpanwrtTimeSpan(i + 1) - _sSpanwrtTimeSpan(i));
			}
			_tf = _timeSpanForS(_timeSpanForS.size() - 1);
			//cout << "tiem" << endl;
			//cout << _timeSpanForS.segment(0, 20) << endl;
		}

		void BSplineGivenPathOptimization::sharedVar::getSFromSdotUsingGaussianQuadrature()
		{
			if (!_isGaussianQuadratureSdotInitialized)
			{
				_gaussianQuadratureSdot = GaussianQuadrature(30, 0, _sf);
				_isGaussianQuadratureSdotInitialized = true;
			}
			else
				_gaussianQuadratureSdot.setTimeInterval(0, _sf);
			_sSpanwrtTimeSpan = _gaussianQuadratureSdot.getQueryPoints();
			VectorX w = _gaussianQuadratureSdot.getWeights();
			int nStep = _sSpanwrtTimeSpan.size();
			_timeSpanForS.resize(nStep);
			_timeSpanForS.setZero();
			_sSpanwrtTimeSpan(nStep - 1) = _sf - 1e-10;
			Real sdoti;
			for (int i = 1; i < nStep; i++)
			{
				sdoti = _sdotSpline(_sSpanwrtTimeSpan(i))(0, 0);
				if (sdoti > 0)
					_timeSpanForS(i) = _timeSpanForS(i - 1) + (w(i) / sdoti);
			}
				

			_tf = _timeSpanForS(nStep - 1);
			//cout << "timespan:" << endl;
			//cout << _timeSpanForS << endl;
			//cout << "sspan:" << endl;
			//cout << _sSpanwrtTimeSpan << endl;
		}
		void BSplineGivenPathOptimization::sharedVar::compareControlPoint(const Math::VectorX & controlPoint)
		{
			// generate sdot(s) and th(s) when new control point enters
			if (_isInitiated == false || !RealEqual(_currentControlPoint, controlPoint))
			{
				_isInitiated = true;
				_currentControlPoint = controlPoint;
				_isIDUpdated = false;

				for (int i = 0; i < _nMiddleCPSdot; i++)
					_sdotCP(_nInitCPsdot + i) = _currentControlPoint(i);
				for (int i = 0; i < _nMiddleCPTh; i++)
					_thCP(_nInitCPTh + i) = _currentControlPoint(_nMiddleCPSdot + i);

				_sdotSpline = Math::BSpline<-1, -1, 1>(_knotSdot, _sdotCP);
				_thetaSpline = Math::BSpline<-1, -1, 1>(_knotTh, _thCP);

				// get s(t) from sdot(s) and calculate tf
				getSFromSdot();
				//getSFromSdotUsingGaussianQuadrature();
				
				
				// set time interval from calculated _tf
				setTimeSpan();
				//setTimeSpanUsingGaussianQuadrature();

				// set linear interpolation for s(t)
				Real si;
				LinearInterpolation linterp;
				linterp.setX(_timeSpanForS);
				vector<VectorX> sSpan(_sSpanwrtTimeSpan.size());
				for (int i = 0; i < _sSpanwrtTimeSpan.size(); i++)
				{
					sSpan[i].resize(1);
					sSpan[i](0) = _sSpanwrtTimeSpan(i);
				}

				linterp.setElements(sSpan);

				Real thi;
				
				//cout << "s = " << endl;
				//cout << _bilinterp._x << endl;
				//cout << "th = " << endl;
				//cout << _bilinterp._y << endl;

				_jointVal = MatrixX(_stateTrj[0]->getDOF(State::TARGET_JOINT::ASSEMJOINT), _stateTrj.size());
				_jointVel = MatrixX(_stateTrj[0]->getDOF(State::TARGET_JOINT::ASSEMJOINT), _stateTrj.size());
				_jointAcc = MatrixX(_stateTrj[0]->getDOF(State::TARGET_JOINT::ASSEMJOINT), _stateTrj.size());
				
				VectorX qtemp6(_nStep);
				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					si = linterp(_timeSpanForObj(i))(0);
					thi = _thetaSpline(si)(0);
					_jointVal.col(i) = _bilinterp(si, thi);
					qtemp6(i) = _jointVal(5, i);
				}
				qtemp6 = transferJointValue(qtemp6);



				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					_jointVal(5, i) = qtemp6(i);
					_stateTrj[i]->setJointq(State::TARGET_JOINT::ACTIVEJOINT, _jointVal.col(i));
				}
				Real dt_inv;

				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					dt_inv = 1.0 / _timeSpanWeight(i);
					if (i == 0)
					{
						_jointVel.col(i) = (_jointVal.col(i + 1) - _jointVal.col(i))*dt_inv;
						_jointAcc.col(i) = (_jointVal.col(i + 2) + _jointVal.col(i) - 2.0*_jointVal.col(i + 1)) *dt_inv*dt_inv;
					}
					else if (i == _stateTrj.size() - 1)
					{
						_jointVel.col(i) = (_jointVal.col(i) - _jointVal.col(i - 1))*dt_inv;
						_jointAcc.col(i) = (_jointVal.col(i) + _jointVal.col(i - 2) - 2.0*_jointVal.col(i - 1)) *dt_inv*dt_inv;
					}
					else
					{
						_jointVel.col(i) = (_jointVal.col(i + 1) - _jointVal.col(i - 1))*0.5*dt_inv;
						_jointAcc.col(i) = (_jointVal.col(i + 1) + _jointVal.col(i - 1) - 2.0*_jointVal.col(i)) *dt_inv*dt_inv;
					}
					_stateTrj[i]->setJointqdot(State::TARGET_JOINT::ACTIVEJOINT, _jointVel.col(i));
					_stateTrj[i]->setJointqddot(State::TARGET_JOINT::ACTIVEJOINT, _jointAcc.col(i));
					Dynamics::solveInverseDynamics(*_socAssem, *_stateTrj[i]);
				}

				_tau = MatrixX(_stateTrj[0]->getDOF(State::TARGET_JOINT::ASSEMJOINT), _stateTrj.size());
				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					_tau.col(i) = _stateTrj[i]->getJointTorque(State::TARGET_JOINT::ASSEMJOINT);
				}
				_isIDUpdated = true;
			}

		}

		void BSplineGivenPathOptimization::sharedVar::setTimeSpan()
		{
			Real dt = _tf / (Real)(_nStep - 1);
			for (int i = 0; i < _nStep; i++)
			{
				_timeSpanWeight(i) = dt;
				_timeSpanForObj(i) = (Real)i * dt;
			}
				
		}


		void BSplineGivenPathOptimization::sharedVar::setTimeSpanUsingGaussianQuadrature()
		{
			if (!_isGaussianQuadratureTimeInitialized)
			{
				_gaussianQuadratureTime = GaussianQuadrature(_nStep, 0, _tf);
			}
			else
				_gaussianQuadratureTime.setTimeInterval(0, _tf);

			_timeSpanForObj = _gaussianQuadratureTime.getQueryPoints();
			_timeSpanWeight = _gaussianQuadratureTime.getWeights();

		}

		const Math::MatrixX & BSplineGivenPathOptimization::sharedVar::getTau(const Math::VectorX & controlPoint)
		{
			if (_optimizeThetaOnly)
				compareControlPointThetaOnly(controlPoint);
			else
				compareControlPoint(controlPoint);
			return _tau;
		}
		const Math::MatrixX & BSplineGivenPathOptimization::sharedVar::getJointVal(const Math::VectorX & controlPoint)
		{
			if (_optimizeThetaOnly)
				compareControlPointThetaOnly(controlPoint);
			else
				compareControlPoint(controlPoint);
			return _jointVal;
		}
		const Math::MatrixX & BSplineGivenPathOptimization::sharedVar::getJointVel(const Math::VectorX & controlPoint)
		{
			if (_optimizeThetaOnly)
				compareControlPointThetaOnly(controlPoint);
			else
				compareControlPoint(controlPoint);
			return _jointVel;
		}
		const Math::MatrixX & BSplineGivenPathOptimization::sharedVar::getJointAcc(const Math::VectorX & controlPoint)
		{
			if (_optimizeThetaOnly)
				compareControlPointThetaOnly(controlPoint);
			else
				compareControlPoint(controlPoint);
			return _jointAcc;
		}

		void BSplineGivenPathOptimization::sharedVar::compareControlPointThetaOnly(const Math::VectorX & controlPoint)
		{
			// generate th(t) when new control point enters
			if (_isInitiated == false || !RealEqual(_currentControlPoint, controlPoint))
			{
				_isInitiated = true;
				_currentControlPoint = controlPoint;
				_isIDUpdated = false;

				for (int i = 0; i < _nMiddleCPTh; i++)
					_thCP(_nInitCPTh + i) = _currentControlPoint(i);

				_thetaSpline = Math::BSpline<-1, -1, 1>(_knotTh, _thCP);

				Real thi;
				_jointVal = MatrixX(_stateTrj[0]->getDOF(State::TARGET_JOINT::ASSEMJOINT), _stateTrj.size());
				_jointVel = MatrixX(_stateTrj[0]->getDOF(State::TARGET_JOINT::ASSEMJOINT), _stateTrj.size());
				_jointAcc = MatrixX(_stateTrj[0]->getDOF(State::TARGET_JOINT::ASSEMJOINT), _stateTrj.size());

				VectorX qtemp6(_nStep);
				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					thi = _thetaSpline(_timeSpanForObj(i))(0);
					_jointVal.col(i) = _bilinterp(_sSpanwrtTimeSpan(i), thi);
					qtemp6(i) = _jointVal(5, i);
				}
				qtemp6 = transferJointValue(qtemp6);



				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					_jointVal(5, i) = qtemp6(i);
					_stateTrj[i]->setJointq(State::TARGET_JOINT::ACTIVEJOINT, _jointVal.col(i));
				}
				Real dt_inv;

				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					dt_inv = 1.0 / _timeSpanWeight(i);
					if (i == 0)
					{
						_jointVel.col(i) = (_jointVal.col(i + 1) - _jointVal.col(i))*dt_inv;
						_jointAcc.col(i) = (_jointVal.col(i + 2) + _jointVal.col(i) - 2.0*_jointVal.col(i + 1)) *dt_inv*dt_inv;
					}
					else if (i == _stateTrj.size() - 1)
					{
						_jointVel.col(i) = (_jointVal.col(i) - _jointVal.col(i - 1))*dt_inv;
						_jointAcc.col(i) = (_jointVal.col(i) + _jointVal.col(i - 2) - 2.0*_jointVal.col(i - 1)) *dt_inv*dt_inv;
					}
					else
					{
						_jointVel.col(i) = (_jointVal.col(i + 1) - _jointVal.col(i - 1))*0.5*dt_inv;
						_jointAcc.col(i) = (_jointVal.col(i + 1) + _jointVal.col(i - 1) - 2.0*_jointVal.col(i)) *dt_inv*dt_inv;
					}
					_stateTrj[i]->setJointqdot(State::TARGET_JOINT::ACTIVEJOINT, _jointVel.col(i));
					_stateTrj[i]->setJointqddot(State::TARGET_JOINT::ACTIVEJOINT, _jointAcc.col(i));
					Dynamics::solveInverseDynamics(*_socAssem, *_stateTrj[i]);
				}

				_tau = MatrixX(_stateTrj[0]->getDOF(State::TARGET_JOINT::ASSEMJOINT), _stateTrj.size());
				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					_tau.col(i) = _stateTrj[i]->getJointTorque(State::TARGET_JOINT::ASSEMJOINT);
				}
				_isIDUpdated = true;
			}

		}

		Math::VectorX BSplineGivenPathOptimization::effortFunction::func(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedVar->getTau(x);

			VectorX val(1);
			val(0) = 0;
			for (int i = 0; i < tauTrj.cols(); i++)
				val(0) += tauTrj.col(i).squaredNorm()*_sharedVar->_timeSpanWeight(i);
			return val;
		}

		Math::VectorX BSplineGivenPathOptimization::energyLossFunction::func(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedVar->getTau(x);
			shared_ptr<MotorJoint> tempJointPtr;
			Real voltage;
			Real current;
			VectorX val(1);
			val(0) = 0;
			for (int i = 0; i < tauTrj.cols(); i++)
			{
				for (unsigned int j = 0, dofIdx = 0; j < _sharedVar->_socAssem->getMateList().size(); j++)
				{
					tempJointPtr = static_pointer_cast<MotorJoint>(_sharedVar->_socAssem->getJointPtrByMateIndex(j));
					for (unsigned int k = 0; k < tempJointPtr->getDOF(); k++, dofIdx++)
					{
						current = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * tauTrj(dofIdx, i)
							+ tempJointPtr->getRotorInertia() * tempJointPtr->getGearRatio() / tempJointPtr->getMotorConstant() * _sharedVar->_stateTrj[i]->getJointStateByMateIndex(j).getqddot()(k);
						voltage = current * tempJointPtr->getResistance() + tempJointPtr->getBackEMFConstant() * tempJointPtr->getGearRatio() * _sharedVar->_stateTrj[i]->getJointStateByMateIndex(j).getqdot()(k);
						val(0) += _sharedVar->_timeSpanWeight(i) * max(current * voltage, 0.0);

					}
				}
			}
			return val;
		}

		void BSplineGivenPathOptimization::nonLinearInequalityConstraint::loadConstraint(const socAssemblyPtr& socAssem, bool velConstraintExist, bool torqueConstraintExist, bool accConstraintExist)
		{
			_defaultState = socAssem->makeState();
			int nDOF = _defaultState->getDOF(State::TARGET_JOINT::ASSEMJOINT);
			if (torqueConstraintExist)
			{
				_tauMax.resize(nDOF);
				_tauMin.resize(nDOF);
				int dof = 0;
				for (unsigned int i = 0; i < socAssem->getMateList().size(); i++)
				{
					_tauMax.block(dof, 0, socAssem->getJointPtrByMateIndex(i)->getDOF(), 1) = socAssem->getJointPtrByMateIndex(i)->getLimitInputUpper();
					_tauMin.block(dof, 0, socAssem->getJointPtrByMateIndex(i)->getDOF(), 1) = socAssem->getJointPtrByMateIndex(i)->getLimitInputLower();
					dof += socAssem->getJointPtrByMateIndex(i)->getDOF();
				}
			}

			_nConstraint = 0;
			if (velConstraintExist)
				_nConstraint += 2 * nDOF;
			if (torqueConstraintExist)
				_nConstraint += 2 * _tauMax.size();
			if (accConstraintExist)
				_nConstraint += 2 * nDOF;

			_velConstraintExist = velConstraintExist;
			_torqueConstraintExist = torqueConstraintExist;
			_accConstraintExist = accConstraintExist;

			_qdotMax.resize(nDOF);
			_qdotMin.resize(nDOF);
			_qddotMax.resize(nDOF);
			_qddotMin.resize(nDOF);
			unsigned int jointID;
			for (unsigned int i = 0, dof = 0; i < socAssem->getMateList().size(); i++)
			{
				jointID = _defaultState->getJointID(State::TARGET_JOINT::ASSEMJOINT, i);
				for (unsigned int j = 0; j < socAssem->getJointPtrByMateIndex(jointID)->getDOF(); j++, dof++)
				{
					if (velConstraintExist)
					{
						_qdotMax(dof) = socAssem->getJointPtrByMateIndex(jointID)->getLimitVelUpper()(j);
						_qdotMin(dof) = socAssem->getJointPtrByMateIndex(jointID)->getLimitVelLower()(j);
					}
					if (accConstraintExist)
					{
						_qddotMax(dof) = socAssem->getJointPtrByMateIndex(jointID)->getLimitAccUpper()(j);
						_qddotMin(dof) = socAssem->getJointPtrByMateIndex(jointID)->getLimitAccLower()(j);
					}
				}
			}
		}

		Math::VectorX BSplineGivenPathOptimization::inequalityConstraint::func(const Math::VectorX & x) const
		{
			VectorX linearIneq = (*_linearIneqConstraint)(x);
			VectorX nonlinearIneq = (*_nonLinearIneqConstraint)(x);
			VectorX val(linearIneq.size() + nonlinearIneq.size());
			for (int i = 0; i < linearIneq.size(); i++)
				val(i) = linearIneq(i);
			for (int i = 0; i < nonlinearIneq.size(); i++)
				val(linearIneq.size() + i) = nonlinearIneq(i);

			//	cout << val << endl;

			return val;
		}

		Math::MatrixX BSplineGivenPathOptimization::inequalityConstraint::Jacobian(const Math::VectorX & x) const
		{
			MatrixX linearIneq = (*_linearIneqConstraint).Jacobian(x);
			MatrixX nonlinearIneq = (*_nonLinearIneqConstraint).Jacobian(x);
			MatrixX val(linearIneq.rows() + nonlinearIneq.rows(), x.size());
			val.block(0, 0, linearIneq.rows(), x.size()) = linearIneq;
			val.block(linearIneq.rows(), 0, nonlinearIneq.rows(), x.size()) = nonlinearIneq;
			return val;
		}

		vector<Math::MatrixX> BSplineGivenPathOptimization::inequalityConstraint::Hessian(const Math::VectorX & x) const
		{
			vector<MatrixX> val = (*_linearIneqConstraint).Hessian(x);
			vector<MatrixX> nonlinearIneq = (*_nonLinearIneqConstraint).Hessian(x);
			val.insert(val.end(), nonlinearIneq.begin(), nonlinearIneq.end());
			return val;
		}

		VectorX BSplineGivenPathOptimization::nonLinearInequalityConstraint::func(const Math::VectorX & x) const
		{
			int nConstraint = 0;
			bool isTrajUpdated = false;
			VectorX val(_nConstraint);

			if (_torqueConstraintExist)
			{
				const MatrixX& tauTrj = _sharedVar->getTau(x);
				isTrajUpdated = true;
				for (int i = 0; i < tauTrj.rows(); i++)
				{
					val(nConstraint + i) = tauTrj.row(i).maxCoeff() - _tauMax(i);
					val(nConstraint + i + tauTrj.rows()) = _tauMin(i) - tauTrj.row(i).minCoeff();
				}
				nConstraint += 2 * tauTrj.rows();
			}

			if (_velConstraintExist)
			{
				const MatrixX& jointVelTrj = _sharedVar->getJointVel(x);
				for (int i = 0; i < jointVelTrj.rows(); i++)
				{
					val(nConstraint + i) = jointVelTrj.row(i).maxCoeff() - _qdotMax(i);
					val(nConstraint + jointVelTrj.rows() + i) = _qdotMin(i) - jointVelTrj.row(i).minCoeff();
				}
				nConstraint += 2 * jointVelTrj.rows();
			}

			if (_accConstraintExist)
			{
				const MatrixX& jointAccTrj = _sharedVar->getJointAcc(x);
				for (int i = 0; i < jointAccTrj.rows(); i++)
				{
					val(nConstraint + i) = jointAccTrj.row(i).maxCoeff() - _qddotMax(i);
					val(nConstraint + jointAccTrj.rows() + i) = _qddotMin(i) - jointAccTrj.row(i).minCoeff();
				}
				nConstraint += 2 * jointAccTrj.rows();
			}

			return val;
		}

		void BSplineGivenPathOptimization::nonLinearInequalityConstraint::showConstraint(const Math::VectorX & x, Type type) const
		{
			VectorX val = func(x);
			int nStartTorque = 0;
			int nStartVel = 0;
			int nStartAcc = 0;
			if (_torqueConstraintExist)
			{
				nStartVel += _tauMax.size() * 2;
				nStartAcc += _tauMax.size() * 2;
			}
			if (_velConstraintExist)
				nStartAcc += _qdotMin.size() * 2;
			if (type == Type::TORQUE && _torqueConstraintExist)
			{
				cout << "torque constraint values: " << endl;
				for (int i = 0; i < _tauMax.size() * 2; i++)
					cout << val(nStartTorque + i) << endl;
			}
			if (type == Type::VEL && _velConstraintExist)
			{
				cout << "velocity constraint values: " << endl;
				for (int i = 0; i < _qdotMin.size() * 2; i++)
					cout << val(nStartVel + i) << endl;
			}
			if (type == Type::ACC && _accConstraintExist)
			{
				cout << "acceleration constraint values: " << endl;
				for (int i = 0; i < _qddotMin.size() * 2; i++)
					cout << val(nStartAcc + i) << endl;
			}
		}
		
		void BSplineGivenPathOptimization::getRobotTrajectory(sharedVar& sharedvar, const VectorX& controlPoint)
		{
			if (sharedvar._optimizeThetaOnly)
				sharedvar.compareControlPointThetaOnly(controlPoint);
			else
				sharedvar.compareControlPoint(controlPoint);

			int nTimeStep = (int) floor(sharedvar._tf / _robotTimeStep) + 1;

			VectorX tSpan(nTimeStep);
			for (int i = 0; i < nTimeStep - 1; i++)
				tSpan(i) = (Real)i * _robotTimeStep;
			tSpan(nTimeStep - 1) = sharedvar._tf;
			LinearInterpolation linterp;
			if (!sharedvar._optimizeThetaOnly)
			{
				linterp.setX(sharedvar._timeSpanForS);
				vector<VectorX> sSpan(sharedvar._sSpanwrtTimeSpan.size());
				for (int i = 0; i < sharedvar._sSpanwrtTimeSpan.size(); i++)
				{
					sSpan[i].resize(1);
					sSpan[i](0) = sharedvar._sSpanwrtTimeSpan(i);
				}

				linterp.setElements(sSpan);
			}
			
			Real si;
			Real thi;
			VectorX qtemp6(nTimeStep);
			_robotJointValTrj.resize(_nDOF, nTimeStep);
			Vector3 pos;
			Matrix3 R;
			SE3 T;
			
			for (int i = 0; i < nTimeStep; i++)
			{
				if (!sharedvar._optimizeThetaOnly)
				{
					si = linterp(tSpan(i))(0);
					thi = sharedvar._thetaSpline(si)(0);
				}
				else
				{
					si = _sCubic(_realSCubic(tSpan(i))(0))(0);
					thi = sharedvar._thetaSpline(tSpan(i))(0);
				}
				
				pos = _posCubic(si);
				for (int j = 0; j < 3; j++)
					R.col(j) = _RCubic[j](si);
				T.setPosition(pos);
				T.setRotation(SO3::Projection(R)*SO3::RotZ(thi));
				//cout << si << " " << thi << endl;
				_robotJointValTrj.col(i) = Kinematics::solveInverseKinematicsOnlyForEfort(*_socAssem, _Tbase.inverse()*T*_TlastLinkToEndeffector.inverse())[_invIdx];
				qtemp6(i) = _robotJointValTrj(5, i);
			}
			cout << "final robot path: " << endl;
			qtemp6 = transferJointValue(qtemp6);

			for (int i = 0; i < _robotJointValTrj.cols(); i++)			
				_robotJointValTrj(5, i) = qtemp6(i);
			

		}

		VectorX BSplineGivenPathOptimization::run(const ObjectiveFunctionType& objectiveType)
		{
			NonlinearOptimization nonlinearSolver;
			double c = clock();
			///////////////////////////////////// INITIAL GUESS ///////////////////////////////////////

			VectorX sdotCP0(_nMiddleCPSdot);
			srand((unsigned int) time(NULL));
			sdotCP0.setRandom();
			sdotCP0.cwiseAbs();
			sdotCP0 *= 10.0;
			VectorX thCP0(_nMiddleCPTh);
			thCP0.setOnes();
			thCP0 *= 0.9076;
			
			VectorX x(_nMiddleCPSdot + _nMiddleCPTh);
			x << sdotCP0, thCP0;

			
			//////////////////////////////////// EQUALITY CONSTRAINT //////////////////////////////////
			_eqFunc = FunctionPtr(new EmptyFunction());

			////////////////////////////////// INEQUALITY CONSTRAINT //////////////////////////////////
			setLinearInequalityConstraint();

			_ineqFunc = FunctionPtr(new inequalityConstraint());

			std::shared_ptr<LinearFunction> _linearIneqFunc = std::shared_ptr<LinearFunction>(new LinearFunction());

			_linearIneqFunc->A = _Aineq;
			_linearIneqFunc->b = _bineq;


			std::static_pointer_cast<inequalityConstraint> (_ineqFunc)->_linearIneqConstraint = _linearIneqFunc;

			std::shared_ptr<nonLinearInequalityConstraint> _nonLinearIneqFunc = std::shared_ptr<nonLinearInequalityConstraint>(new nonLinearInequalityConstraint());
			std::static_pointer_cast<nonLinearInequalityConstraint> (_nonLinearIneqFunc)->loadConstraint(_socAssem, _velConstraintExist, _torqueConstraintExist, _accConstraintExist);
			std::static_pointer_cast<inequalityConstraint> (_ineqFunc)->_nonLinearIneqConstraint = _nonLinearIneqFunc;

			////////////////////////////////// SHARED VAR /////////////////////////////////////////////
			std::shared_ptr<sharedVar> _SharedVar = std::shared_ptr<sharedVar>(new sharedVar(_socAssem, _nStep, _sf, _sSpan, _thSpanDouble, _invKinSol[0],
				_nInitCPsdot, _nMiddleCPSdot, _nFinalCPsdot, _nInitCPTh, _nMiddleCPTh, _nFinalCPTh, _knotSdot, _knotTh,
				_boundaryCPsdot, _boundaryCPth));
			std::static_pointer_cast<nonLinearInequalityConstraint>(_nonLinearIneqFunc)->_sharedVar = _SharedVar;
			///////////////////////////////////////////////////////////////////////////////////////////


			//////////////////////////////////////////////////////////////////////////////
			cout << _SharedVar->getJointVal(x).block(0, _nStep - 6, 6, 6) << endl;

			/*for (int i = 0; i < _nStep / 10; i++)
				cout << _SharedVar->getJointVal(x).block(0, 10 * i, 6, 10) << endl;*/
			cout << _SharedVar->getJointVal(x).row(5) << endl;




			////////////////////////////////// OBJECTIVE FUNCTION /////////////////////////////////////
			
			if (objectiveType == ObjectiveFunctionType::Effort)
			{
				_objectiveFunc = FunctionPtr(new effortFunction());
				static_pointer_cast<effortFunction>(_objectiveFunc)->_sharedVar = _SharedVar;
			}
			else if (objectiveType == ObjectiveFunctionType::EnergyLoss)
			{
				_objectiveFunc = FunctionPtr(new energyLossFunction());
				static_pointer_cast<energyLossFunction>(_objectiveFunc)->_sharedVar = _SharedVar;
			}




			///////////////////////////////////////////////////////////////////////////////////////////
			cout << "obj0: " << (*_objectiveFunc)(x) << endl;
			cout << "maxIneq: " << (*_ineqFunc)(x).maxCoeff() << endl;


			nonlinearSolver._NLoptSubAlgo = NonlinearOptimization::NLoptAlgorithm::Other;
			nonlinearSolver._objectiveFunc = _objectiveFunc;
			nonlinearSolver._eqFunc = _eqFunc;
			nonlinearSolver._ineqFunc = _ineqFunc;
			_resultFlag = false;
			_solX = nonlinearSolver.solve(x);
			_fval = (*_objectiveFunc)(_solX)(0);
			if (_eqFunc != NULL) _eqConstraintVal = (*_eqFunc)(_solX);
			if (_ineqFunc != NULL) _ineqConstraintVal = (*_ineqFunc)(_solX);
			

			getRobotTrajectory(*_SharedVar, _solX);
			cout << _robotJointValTrj.row(5) << endl;
			
			Real maxIneq = (*_ineqFunc)(_solX).maxCoeff();

			if (RealLess(maxIneq, 1e-4))
			{
				cout << "solved" << endl;
				_resultFlag = true;
			}
			
			
			//cout << "x : " << endl << _solX << endl;	
			cout << "obj : " << endl << (*_objectiveFunc)(_solX) << endl;
			cout << "tf : " << _SharedVar->_tf << endl;
			//cout << "eq : " << endl << (*_eqFunc)(_solX) << endl;
			//cout << "ineq : " << endl << (*_ineqFunc)(_solX) << endl;
			cout << "maxInEq : " << maxIneq << endl;
				

			_computationTime = clock() - c;
			_jointVal = _SharedVar->getJointVal(_solX);
			_jointVel = _SharedVar->getJointVel(_solX);
			_jointAcc = _SharedVar->getJointAcc(_solX);
			_jointTorque = _SharedVar->getTau(_solX);

			

			return _solX;
			///////////////////////////////////////////////////////////////////////////////////////////
		}
		Math::VectorX BSplineGivenPathOptimization::runThetaOnly(const ObjectiveFunctionType & objectiveType)
		{
			NonlinearOptimization nonlinearSolver;
			double c = clock();
			///////////////////////////////////// INITIAL GUESS ///////////////////////////////////////
			srand((unsigned int)time(NULL));
			VectorX x(_nMiddleCPTh);
			x.setRandom();
			//x *= 0.9076;

			//////////////////////////////////// EQUALITY CONSTRAINT //////////////////////////////////
			_eqFunc = FunctionPtr(new EmptyFunction());

			////////////////////////////////// INEQUALITY CONSTRAINT //////////////////////////////////
			setLinearInequalityConstraintForThetaOnly();

			_ineqFunc = FunctionPtr(new inequalityConstraint());

			std::shared_ptr<LinearFunction> _linearIneqFunc = std::shared_ptr<LinearFunction>(new LinearFunction());

			_linearIneqFunc->A = _Aineq;
			_linearIneqFunc->b = _bineq;


			std::static_pointer_cast<inequalityConstraint> (_ineqFunc)->_linearIneqConstraint = _linearIneqFunc;


			std::shared_ptr<nonLinearInequalityConstraint> _nonLinearIneqFunc = std::shared_ptr<nonLinearInequalityConstraint>(new nonLinearInequalityConstraint());
			std::static_pointer_cast<nonLinearInequalityConstraint> (_nonLinearIneqFunc)->loadConstraint(_socAssem, _velConstraintExist, _torqueConstraintExist, _accConstraintExist);
			std::static_pointer_cast<inequalityConstraint> (_ineqFunc)->_nonLinearIneqConstraint = _nonLinearIneqFunc;

			////////////////////////////////// SHARED VAR /////////////////////////////////////////////
			std::shared_ptr<sharedVar> _SharedVar = std::shared_ptr<sharedVar>(new sharedVar(_socAssem, _nStep, _tfForThetaOnly,
				_sCubic, _realSCubic, _sSpan, _thSpanDouble, _invKinSol[0],
				_nInitCPTh, _nMiddleCPTh, _nFinalCPTh, _knotTh,
				_boundaryCPth));
			std::static_pointer_cast<nonLinearInequalityConstraint>(_nonLinearIneqFunc)->_sharedVar = _SharedVar;
			///////////////////////////////////////////////////////////////////////////////////////////


			//////////////////////////////////////////////////////////////////////////////
			//cout << _SharedVar->getJointVal(x).block(0, _nStep - 6, 6, 6) << endl;

			/*for (int i = 0; i < _nStep / 10; i++)
			cout << _SharedVar->getJointVal(x).block(0, 10 * i, 6, 10) << endl;*/
			//cout << _SharedVar->getJointVal(x).row(5) << endl;




			////////////////////////////////// OBJECTIVE FUNCTION /////////////////////////////////////

			if (objectiveType == ObjectiveFunctionType::Effort)
			{
				_objectiveFunc = FunctionPtr(new effortFunction());
				static_pointer_cast<effortFunction>(_objectiveFunc)->_sharedVar = _SharedVar;
			}
			else if (objectiveType == ObjectiveFunctionType::EnergyLoss)
			{
				_objectiveFunc = FunctionPtr(new energyLossFunction());
				static_pointer_cast<energyLossFunction>(_objectiveFunc)->_sharedVar = _SharedVar;
			}

			///////////////////////////////////////////////////////////////////////////////////////////
			cout << "x0: " << x << endl;
			cout << "obj0: " << (*_objectiveFunc)(x) << endl;
			cout << "maxIneq: " << (*_ineqFunc)(x).maxCoeff() << endl;


			nonlinearSolver._NLoptSubAlgo = NonlinearOptimization::NLoptAlgorithm::Other;
			nonlinearSolver._objectiveFunc = _objectiveFunc;
			nonlinearSolver._eqFunc = _eqFunc;
			nonlinearSolver._ineqFunc = _ineqFunc;
			_resultFlag = false;
			_solX = nonlinearSolver.solve(x);
			_fval = (*_objectiveFunc)(_solX)(0);
			if (_eqFunc != NULL) _eqConstraintVal = (*_eqFunc)(_solX);
			if (_ineqFunc != NULL) _ineqConstraintVal = (*_ineqFunc)(_solX);


			getRobotTrajectory(*_SharedVar, _solX);
			//cout << _robotJointValTrj.row(5) << endl;

			Real maxIneq = (*_ineqFunc)(_solX).maxCoeff();

			if (RealLess(maxIneq, 1e-4))
			{
				cout << "solved" << endl;
				_resultFlag = true;
			}


			cout << "x : " << endl << _solX << endl;	
			cout << "obj : " << endl << (*_objectiveFunc)(_solX) << endl;
			cout << "tf : " << _SharedVar->_tf << endl;
			//cout << "eq : " << endl << (*_eqFunc)(_solX) << endl;
			//cout << "ineq : " << endl << (*_ineqFunc)(_solX) << endl;
			cout << "maxInEq : " << maxIneq << endl;

			
			_computationTime = clock() - c;
			_jointVal = _SharedVar->getJointVal(_solX);
			_jointVel = _SharedVar->getJointVel(_solX);
			_jointAcc = _SharedVar->getJointAcc(_solX);
			_jointTorque = _SharedVar->getTau(_solX);

			//cout << _linearIneqFunc->func(_solX) << endl;

			_nonLinearIneqFunc->showConstraint(_solX, nonLinearInequalityConstraint::Type::TORQUE);
			_nonLinearIneqFunc->showConstraint(_solX, nonLinearInequalityConstraint::Type::VEL);
			_nonLinearIneqFunc->showConstraint(_solX, nonLinearInequalityConstraint::Type::ACC);

			utils::writeText(_jointVal, "val.txt");
			utils::writeText(_jointVel, "vel.txt");
			utils::writeText(_jointAcc, "acc.txt");
			
			//////////////////////////////////////////////////////
			VectorX thTrj(_nStep);
			for (int i = 0; i < _nStep; i++)
				thTrj(i) = _SharedVar->_thetaSpline(_SharedVar->_timeSpanForObj(i))(0);
			utils::writeText(thTrj, "th.txt");

			int nValStep = _robotJointValTrj.cols();
			Real dt_inv = 1.0 / (_tfForThetaOnly / (Real)(nValStep - 1));
			
			MatrixX robotVel = (_robotJointValTrj.block(0, 2, _nDOF, nValStep - 2) - _robotJointValTrj.block(0, 0, _nDOF, nValStep - 2)) * dt_inv;
			MatrixX robotAcc = (_robotJointValTrj.block(0, 2, _nDOF, nValStep - 2) + _robotJointValTrj.block(0, 0, _nDOF, nValStep - 2) - 2.0*_robotJointValTrj.block(0, 1, _nDOF, nValStep - 2)) * dt_inv * dt_inv;
			utils::writeText(_robotJointValTrj, "rval.txt");
			utils::writeText(robotVel, "rvel.txt");
			utils::writeText(robotAcc, "racc.txt");
			//////////////////////////////////////////////////////

			cout << "vel max" << endl;
			cout << _nonLinearIneqFunc->_qdotMax << endl;
			cout << "acc max" << endl;
			cout << _nonLinearIneqFunc->_qddotMax << endl;
			return _solX;
			///////////////////////////////////////////////////////////////////////////////////////////
		}
	}
}


