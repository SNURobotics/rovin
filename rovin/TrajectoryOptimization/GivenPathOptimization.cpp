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

		void GivenPathOptimization::setParameters(const Real feedRate, const Real chordError, const Real robotTimeStep)
		{
			_feedRate = feedRate;
			_chordError = chordError;
			_robotTimeStep = robotTimeStep;
			_criticalCurvature = 8.0 * _chordError / (4.0 * _chordError*_chordError + _feedRate*_feedRate * _robotTimeStep * _robotTimeStep);
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
			solveInverseKinematics(_startPathIdx[pathNum], _endPathIdx[pathNum], qInit);
			_sf = (Real)(_endPathIdx[pathNum] - _startPathIdx[pathNum]);
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
		}

		VectorX GivenPathOptimization::transferJointValueTemp(VectorX qiTrj, const Real etha)
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

		VectorX GivenPathOptimization::transferJointValue(VectorX qiTrj, const Real etha)
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
			compareControlPoint(controlPoint);
			return _tau;
		}
		const Math::MatrixX & BSplineGivenPathOptimization::sharedVar::getJointVal(const Math::VectorX & controlPoint)
		{
			compareControlPoint(controlPoint);
			return _jointVal;
		}
		const Math::MatrixX & BSplineGivenPathOptimization::sharedVar::getJointVel(const Math::VectorX & controlPoint)
		{
			compareControlPoint(controlPoint);
			return _jointVel;
		}
		const Math::MatrixX & BSplineGivenPathOptimization::sharedVar::getJointAcc(const Math::VectorX & controlPoint)
		{
			compareControlPoint(controlPoint);
			return _jointAcc;
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
			cout << "maxInEq: " << (*_ineqFunc)(x).maxCoeff() << endl;


			nonlinearSolver._NLoptSubAlgo = NonlinearOptimization::NLoptAlgorithm::Other;
			nonlinearSolver._objectiveFunc = _objectiveFunc;
			nonlinearSolver._eqFunc = _eqFunc;
			nonlinearSolver._ineqFunc = _ineqFunc;
			_resultFlag = false;
			_solX = nonlinearSolver.solve(x);
			if (_solX.size() != 0)
			{
				cout << "solved" << endl;
				_resultFlag = true;
				_fval = (*_objectiveFunc)(_solX)(0);
				if (_eqFunc != NULL) _eqConstraintVal = (*_eqFunc)(_solX);
				if (_ineqFunc != NULL) _ineqConstraintVal = (*_ineqFunc)(_solX);
			}
			
			cout << "x : " << endl << _solX << endl;	
			cout << "obj : " << endl << (*_objectiveFunc)(_solX) << endl;
			cout << "tf : " << _SharedVar->_tf << endl;
			//cout << "eq : " << endl << (*_eqFunc)(_solX) << endl;
			//cout << "ineq : " << endl << (*_ineqFunc)(_solX) << endl;
			cout << "maxInEq : " << (*_ineqFunc)(_solX).maxCoeff() << endl;
			_computationTime = clock() - c;
			_jointVal = _SharedVar->getJointVal(_solX);
			_jointVel = _SharedVar->getJointVel(_solX);
			_jointAcc = _SharedVar->getJointAcc(_solX);
			_jointTorque = _SharedVar->getTau(_solX);
			return _solX;
			///////////////////////////////////////////////////////////////////////////////////////////
		}

	}
}


