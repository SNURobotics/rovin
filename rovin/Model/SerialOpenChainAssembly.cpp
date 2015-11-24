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
				if (_Tree[i].second != JointDirection::REGULAR || _Mate[_Tree[i].first]._joint->getJointType() != Joint::SCREWJOINT)
					break;
			}
			utils::Log(i != _Tree.size(), "Serial Open Chain의 조건을 갖춘 Assembly가 아닙니다.", true);

			_endeffectorLink = _Mate[_Tree[_Tree.size() - 1].first].getChildLinkIdx();

			SE3 T;
			_socLink[_baseLink]._M = SE3();
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

		const Math::SE3& SerialOpenChainAssembly::getTransform(const unsigned int mateIdx, State::JointState& jointState) const
		{
			if (jointState.getJointReferenceFrame() != JointReferenceFrame::SPATIAL)
			{
				jointState.needUpdate(true, true, true);
				jointState.setJointReferenceFrame(JointReferenceFrame::SPATIAL);
			}

			const Math::VectorX &q = jointState.getq();
			int dof = _socMate[mateIdx]._axes.cols();

			if (!jointState.isUpdated(true, false, false))
			{
				jointState._T[0] = SE3::Exp(_socMate[mateIdx]._axes.col(0), q[0]);
				for (int i = 1; i < dof; i++)
				{
					jointState._T[i] = jointState._T[i - 1] * SE3::Exp(_socMate[mateIdx]._axes.col(i), q[i]);
				}
				jointState.TUpdated();
			}

			return jointState._T[dof - 1];
		}

		const Math::Matrix6X& SerialOpenChainAssembly::getJacobian(const unsigned int mateIdx, State::JointState& jointState) const
		{
			if (jointState.getJointReferenceFrame() != JointReferenceFrame::SPATIAL)
			{
				jointState.needUpdate(true, true, true);
				jointState.setJointReferenceFrame(JointReferenceFrame::SPATIAL);
			}

			const Math::VectorX &q = jointState.getq();
			int dof = _socMate[mateIdx]._axes.cols();

			if (!jointState.isUpdated(true, false, false))
			{
				jointState._T[0] = SE3::Exp(_socMate[mateIdx]._axes.col(0), q[0]);
				for (int i = 1; i < dof; i++)
				{
					jointState._T[i] = jointState._T[i - 1] * SE3::Exp(_socMate[mateIdx]._axes.col(i), q[i]);
				}
				jointState.TUpdated();
			}

			if (!jointState.isUpdated(false, true, false))
			{
				jointState._J.col(0) = _socMate[mateIdx]._axes.col(0);
				for (int i = 1; i < dof; i++)
				{
					jointState._J.col(0) = SE3::Ad(jointState._T[i - 1]) * _socMate[mateIdx]._axes.col(i);
				}
				jointState.JUpdated();
			}

			return jointState._J;
		}

		const Math::Matrix6X& SerialOpenChainAssembly::getJacobianDot(const unsigned int mateIdx, State::JointState& jointState) const
		{
			if (jointState.getJointReferenceFrame() != JointReferenceFrame::SPATIAL)
			{
				jointState.needUpdate(true, true, true);
				jointState.setJointReferenceFrame(JointReferenceFrame::SPATIAL);
			}

			const Math::VectorX &q = jointState.getq();
			int dof = _socMate[mateIdx]._axes.cols();

			if (!jointState.isUpdated(true, false, false))
			{
				jointState._T[0] = SE3::Exp(_socMate[mateIdx]._axes.col(0), q[0]);
				for (int i = 1; i < dof; i++)
				{
					jointState._T[i] = jointState._T[i - 1] * SE3::Exp(_socMate[mateIdx]._axes.col(i), q[i]);
				}
				jointState.TUpdated();
			}

			if (!jointState.isUpdated(false, true, false))
			{
				jointState._J.col(0) = _socMate[mateIdx]._axes.col(0);
				for (int i = 1; i < dof; i++)
				{
					jointState._J.col(0) = SE3::Ad(jointState._T[i - 1]) * _socMate[mateIdx]._axes.col(i);
				}
				jointState.JUpdated();
			}

			if (!jointState.isUpdated(false, false, true))
			{
				// TODO
			}
			
			return jointState._JDot;
		}
	}
}