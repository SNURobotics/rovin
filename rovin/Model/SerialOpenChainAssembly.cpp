#include "SerialOpenChainAssembly.h"

#include <memory>

#include <rovin/utils/Diagnostic.h>
#include "ScrewJoint.h"

using namespace rovin::Math;

namespace rovin
{
	namespace Model
	{
		StatePtr SerialOpenChainAssembly::makeState() const
		{
			StatePtr state = Assembly::makeState();

			unsigned int i;
			for (i = 0; i < _Tree.size(); i++)
			{
				state->addActiveJoint(_Mate[_Tree[i].first]._joint->getName());
			}

			return state;
		}

		void SerialOpenChainAssembly::completeAssembling(const std::string& baseLinkName)
		{
			Assembly::completeAssembling(baseLinkName);

			_socLink.resize(_linkPtr.size(), SerialOpenChainLink());
			_socMate.resize(_Mate.size(), SerialOpenChainMate());

			unsigned int i;
			for (i = 0; i < _Tree.size(); i++)
			{
				if (_Tree[i].second == JointDirection::REGULAR && _Mate[_Tree[i].first]._joint->getJointType() == Joint::SCREWJOINT && _Depth[_Mate[_Tree[i].first].getChildLinkIdx()] == i + 1);
				else break;
			}
			utils::Log(i != _Tree.size() || _baseLink != _Mate[_Tree[0].first].getParentLinkIdx(), "Serial Open Chain의 조건을 갖춘 Assembly가 아닙니다.", true);

			_endeffectorLink = _Mate[_Tree[_Tree.size() - 1].first].getChildLinkIdx();

			SE3 T;
			_socLink[_baseLink]._M = SE3();
			_socLink[_baseLink]._G = _linkPtr[_baseLink]->getInertia();
			for (i = 0; i < _Tree.size(); i++)
			{
				unsigned int linkIdx = _Mate[_Tree[i].first].getChildLinkIdx();

				T *= _Mate[_Tree[i].first]._Tmj;
				_socMate[_Tree[i].first]._axes = SE3::Ad(T) * (std::static_pointer_cast<ScrewJoint> (_Mate[_Tree[i].first]._joint))->getAxes();

				T *= _Mate[_Tree[i].first]._Tja;
				_socLink[linkIdx]._M = T;
				_socLink[linkIdx]._G = _linkPtr[linkIdx]->getInertia();
				_socLink[linkIdx]._G.changeFrame(T);
			}
		}

		void SerialOpenChainAssembly::updateJointKinematics(const unsigned int mateIdx, State::JointState& jointState, const unsigned int options) const
		{
			if (jointState.getJointReferenceFrame() != JointReferenceFrame::SPATIAL)
			{
				jointState.setInfoUpToDate(State::JointState::ALL_INFO, false);
				jointState.setJointReferenceFrame(JointReferenceFrame::SPATIAL);
			}

			const Math::VectorX &q = jointState.getq();
			const Math::VectorX &qdot = jointState.getqdot();
			int dof = _socMate[mateIdx]._axes.cols();

			if (((options & JOINT_TRANSFORM) | (options & JOINT_JACOBIAN) | (options & JOINT_JACOBIANDOT)) && !jointState.getInfoUpToDate(State::JointState::TRANSFORM))
			{
				jointState._T[0] = SE3::Exp(_socMate[mateIdx]._axes.col(0), q[0]);
				for (int i = 1; i < dof; i++)
				{
					jointState._T[i] = jointState._T[i - 1] * SE3::Exp(_socMate[mateIdx]._axes.col(i), q[i]);
				}
				jointState.setInfoUpToDate(State::JointState::TRANSFORM);
			}

			if (((options & JOINT_JACOBIAN) | (options & JOINT_JACOBIANDOT)) && !jointState.getInfoUpToDate(State::JointState::JACOBIAN))
			{
				jointState._J.col(0) = _socMate[mateIdx]._axes.col(0);
				for (int i = 1; i < dof; i++)
				{
					jointState._J.col(i) = SE3::Ad(jointState._T[i - 1]) * _socMate[mateIdx]._axes.col(i);
				}
				jointState.setInfoUpToDate(State::JointState::JACOBIAN);
			}

			if ((options & JOINT_JACOBIANDOT) && !jointState.getInfoUpToDate(State::JointState::JACOBIAN_DOT))
			{
				jointState._JDot.setZero();
				for (int i = 1; i < dof; i++)
				{
					for (int j = 0; j < i; j++)
					{
						jointState._JDot.col(i) += SE3::ad(jointState._J.col(j)) * jointState._J.col(i) * qdot(j);
					}
				}
				jointState.setInfoUpToDate(State::JointState::JACOBIAN_DOT);
			}
		}

		const Math::SE3& SerialOpenChainAssembly::getTransform(const unsigned int mateIdx, State::JointState& jointState) const
		{
			updateJointKinematics(mateIdx, jointState, JOINT_TRANSFORM);

			return jointState._T[_socMate[mateIdx]._axes.cols() - 1];
		}

		const Math::Matrix6X& SerialOpenChainAssembly::getJacobian(const unsigned int mateIdx, State::JointState& jointState) const
		{
			updateJointKinematics(mateIdx, jointState, JOINT_JACOBIAN);

			return jointState._J;
		}

		const Math::Matrix6X& SerialOpenChainAssembly::getJacobianDot(const unsigned int mateIdx, State::JointState& jointState) const
		{
			updateJointKinematics(mateIdx, jointState, JOINT_JACOBIANDOT);

			return jointState._JDot;
		}
	}
}