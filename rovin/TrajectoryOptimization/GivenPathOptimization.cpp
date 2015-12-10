#include "GivenPathOptimization.h"
#include <rovin/utils/fileIO.h>
#include <cmath>
namespace rovin
{
	namespace TrajectoryOptimization
	{
		GivenPathOptimization::GivenPathOptimization()
		{
			_pathN = 0;
		}


		GivenPathOptimization::~GivenPathOptimization()
		{
		}

		void GivenPathOptimization::setSOCRobotModel(const Model::socAssemblyPtr & socAssem)
		{
			_socAssem = socAssem;
			_defaultState = socAssem->makeState();
			_nDOF = _defaultState->getDOF(State::TARGET_JOINT::ACTIVEJOINT);
		}

		void GivenPathOptimization::loadToolPath(const std::string& fileName)
		{
			MatrixX file = utils::readText(fileName);
			_posTrj = file.block(0, 0, file.rows(), 3) * 0.001;		// convert the unit to m
			_zAxisTrj = file.block(0, 3, file.rows(), 3);
		}

		void GivenPathOptimization::setFinalTimeAndTimeSpan(const Math::Real tf, const int nStep)
		{
			_tf = tf;
			_nStep = nStep;
			_timeSpan.resize(nStep);
			_timeSpanWeight.resize(nStep);
			Real _dt = tf / (Real)(nStep - 1.0);
			for (int i = 0; i < nStep; i++)
			{
				_timeSpan(i) = (Real)i * _dt;
				_timeSpanWeight(i) = _dt;
			}
			_timeSpan(nStep - 1) = tf - 1e-10;
		}

		void GivenPathOptimization::setFinalTimeAndTimeSpanUsingGaussianQuadrature(const Math::Real tf, const int nStep)
		{
			_tf = tf;
			if (_nStep != nStep || _gaussianQuadratureInitialized == false)
			{
				_gaussianQuadrature = GaussianQuadrature(nStep, 0.0, _tf);
			}
			else
			{
				_gaussianQuadrature.setTimeInterval(0.0, _tf);
			}
			_nStep = nStep;
			_gaussianQuadratureInitialized = true;
			_timeSpan = _gaussianQuadrature.getQueryPoints();
			_timeSpanWeight = _gaussianQuadrature.getWeights();
		}

		void GivenPathOptimization::setFeedRate(const Real feedRate)
		{
			_feedRate = feedRate;
		}

		void GivenPathOptimization::truncatePath(const Real curvTol)
		{
			MatrixX dPos = 0.5*(_posTrj.bottomRows(_posTrj.rows() - 2) - _posTrj.topRows(_posTrj.rows() - 2));
			MatrixX ddPos = _posTrj.bottomRows(_posTrj.rows() - 2) + _posTrj.topRows(_posTrj.rows() - 2) - 2.0*_posTrj.block(1, 0, _posTrj.rows() - 2, 3);
			VectorX vnorm = (dPos.cwiseAbs2()*Vector3::Ones()).cwiseSqrt();
			_curvature = ((dPos.cwiseAbs2()*Vector3::Ones()).cwiseProduct(ddPos.cwiseAbs2()*Vector3::Ones()) - (dPos.cwiseProduct(ddPos)*Vector3::Ones()).cwiseAbs2()).cwiseSqrt().cwiseQuotient(vnorm.cwiseAbs2().cwiseProduct(vnorm));
			_startPathIdx.resize(0);
			_endPathIdx.resize(0);
			_startPathIdx.push_back(0);

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
		}

		//void GivenPathOptimization::setThetaGridNumber(const int thN)
		//{
		//	_thN = thN;
		//	_thSpan.resize(_thN);
		//	for (int i = 0; i < _thN; i++)
		//	{
		//		_thSpan(i) = (Real)i * PI_DOUBLE;
		//	}
		//}

		void GivenPathOptimization::solveInverseKinematics(const int pathIdx, const VectorX& qInit)
		{
			_sN = _endPathIdx[pathIdx] - _startPathIdx[pathIdx] + 1;
			SO3 R0;
			if (qInit.size() == 0)
			{
				Vector3 x0;
				Vector3 z0 = _zAxisTrj.row(_startPathIdx[pathIdx]).transpose();
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
				R0 = Kinematics::calculateEndeffectorFrame(*_socAssem, *_defaultState).getRotation();
			}
			SE3 T0(R0, _posTrj.row(_startPathIdx[pathIdx]).transpose());
			SE3 Ti;
			Vector3 w_hat;
			Real ang;
			Vector3 z_bf;
			Vector3 z_cur;
			SE3 Tj;
			for (int i = 0; i < _sN; i++)
			{
				Ti.setPosition(_posTrj.row(_startPathIdx[pathIdx] + i));
				if (i == 0)
				{ 
					Ti.setRotation(T0.getRotation());
				}
				else
				{
					z_bf = _zAxisTrj.row(_startPathIdx[pathIdx] + i - 1).transpose();
					z_cur = _zAxisTrj.row(_startPathIdx[pathIdx] + i).transpose();
					w_hat = z_bf.cross(z_cur);
					if (w_hat.norm() < RealEps)
						w_hat.setZero();
					else
						w_hat.normalize();
					ang = std::acos((z_bf.transpose()*z_cur)(0) / z_bf.norm() / z_cur.norm());
					Ti.setRotation(SO3::Exp(w_hat * ang) * Ti.getRotation());
				}
				for (int j = 0; j < _thN; j++)
				{
					Tj = Ti;
					Tj.setRotation(Ti.getRotation()*SO3::RotZ(_thSpan(i)));
					// inverse kinematics
				}
			}



		}

		void GivenPathOptimization::findFeasibleJointSpace()
		{
		}

		void GivenPathOptimization::generateMaxSpeed()
		{
		}

		BSplineGivenPathOptimization::BSplineGivenPathOptimization()
		{
		}

		BSplineGivenPathOptimization::~BSplineGivenPathOptimization()
		{
		}

		void BSplineGivenPathOptimization::setSplineCondition(const unsigned int order, const unsigned int nMiddleCP)
		{
		}

	}
}


