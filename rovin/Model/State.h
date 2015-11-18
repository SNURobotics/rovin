/**
*	\file	State.h
*	\date	2015.11.17
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	State의 정보를 저장하는 클래스 정의
*/

#pragma once

#include <map>
#include <vector>
#include <rovin/Math/LieGroup.h>

namespace rovin
{
	namespace Model
	{
		class Assembly;

		class State
		{
			friend Assembly;

		public:
			class LinkState
			{
			public:
				LinkState();

				Math::SE3 _T;
				Math::se3 _V;
			};

			class JointState
			{
			public:
				JointState(const unsigned int& dof);

				unsigned int _dof;

				Math::VectorX _q;
				Math::VectorX _qdot;
				Math::VectorX _qddot;

				Math::VectorX _tau;

				Math::dse3 _constraintF;

				std::vector< Math::SE3 > _T;
				Math::se3 _v;

			};

			unsigned int getTotalJointDof() const;
			unsigned int getActiveJointDof() const;

			LinkState& getLinkState(const unsigned int linkIndex);
			LinkState& getLinkState(const std::string& linkName);

			JointState& getJointState(const unsigned int jointIndex);
			JointState& getJointStateByMateIndex(const unsigned int mateIndex);
			JointState& getJointState(const std::string& jointName);

			unsigned int getLinkIndex(const std::string& linkName) const;
			unsigned int getJointIndex(const std::string& jointName) const;

			void addActiveJoint(const std::string& jointName);
			void addActiveJoint(const std::vector< std::string >& jointNameList);

			void eraseActiveJoint(const std::string& jointName);

			std::vector< std::string > getJointList() const;
			std::vector< std::string > getActiveJointList() const;
			std::vector< std::string > getPassiveJointList() const;

		private:
			State(const std::vector< std::string >& linkNameList, const std::vector< std::pair< std::string, unsigned int >>& jointNameList);

		private:
			unsigned int _totalJointDof, _activeJointDof;

			std::vector< LinkState > _linkState;
			std::vector< JointState > _jointState;

			std::vector< std::string > _linkName;
			std::vector< std::string > _jointName;

			std::map< std::string, unsigned int > _linkIndexMap;
			std::map< std::string, unsigned int > _jointIndexMap;

			std::vector< bool > _isActiveJoint;
			std::vector< unsigned int > _activeJointList;
			std::vector< unsigned int > _passiveJointList;

			std::vector< unsigned int > _assemIndex;
			std::vector< unsigned int > _stateIndex;
		};
	}
}