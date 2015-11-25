/**
*	\file	State.h
*	\date	2015.11.17
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	State의 정보를 저장하는 클래스 정의
*/

#pragma once

#include <map>
#include <memory>
#include <vector>
#include <rovin/Math/LieGroup.h>

namespace rovin
{
	namespace Model
	{
		class State;
		class Assembly;

		typedef std::shared_ptr< State > StatePtr;

		class State
		{
			friend class Assembly;

		public:
			class LinkState
			{
			public:
				LinkState();

				Math::SE3 _T;
				Math::se3 _V;
				Math::se3 _VDot;
			};

			class JointState
			{
				friend State;

			public:
				JointState(const unsigned int& dof);

				unsigned int _dof;

				Math::VectorX _tau;
				Math::dse3 _constraintF;

				std::vector< Math::SE3 > _T;
				Math::Matrix6X _J;
				Math::Matrix6X _JDot;
				
				Math::SE3 _accumulatedT;
				Math::Matrix6X _accumulatedJ;
				Math::Matrix6X _accumulatedJDot;

				const Math::VectorX& getq() const { return _q; }
				const Math::VectorX& getqdot() const { return _qdot; }
				const Math::VectorX& getqddot() const { return _qddot; }

				const int getJointReferenceFrame() const { return _JointReferenceFrame; }
				void setJointReferenceFrame(const int JointReferenceFrame) { _JointReferenceFrame = JointReferenceFrame; }
				
				bool isUpdated(bool transform, bool jacobian, bool jacobiandot) const { return (!transform | _TUpdated) & (!jacobian | _JUpdated) & (!jacobiandot | _JDotUpdated); }
				void needUpdate(bool transform, bool jacobian, bool jacobiandot) { _TUpdated &= !transform; _JUpdated &= !jacobian; _JDotUpdated &= !jacobiandot; }
				
				void TUpdated() { _TUpdated = true; }
				void JUpdated() { _JUpdated = true; }
				void JDotUpdated() { _JDotUpdated = true; }

			private:
				void setq(const Math::VectorX& q) { _q = q; needUpdate(true, true, true); }
				void setqdot(const Math::VectorX& qdot) { _qdot = qdot; needUpdate(false, false, true); }
				void setqddot(const Math::VectorX& qddot) { _qddot = qddot; needUpdate(false, false, false); }

				void addq(const Math::VectorX& q) { _q += q; needUpdate(true, true, true); }
				void addqdot(const Math::VectorX& qdot) { _qdot += qdot; needUpdate(false, false, true); }
				void addqddot(const Math::VectorX& qddot) { _qddot += qddot; needUpdate(false, false, false); }

			private:
				Math::VectorX _q;
				Math::VectorX _qdot;
				Math::VectorX _qddot;

				bool _TUpdated;
				bool _JUpdated;
				bool _JDotUpdated;

				int _JointReferenceFrame;
			};

			enum RETURN_STATE
			{
				ASSEMJOINT,
				STATEJOINT,
				ACTIVEJOINT,
				PASSIVEJOINT
			};

			unsigned int getTotalJointDof() const;
			unsigned int getActiveJointDof() const;

			LinkState& getLinkState(const unsigned int linkIndex);
			LinkState& getLinkState(const std::string& linkName);

			JointState& getJointState(const unsigned int jointIndex);
			JointState& getJointStateByMateIndex(const unsigned int mateIndex);
			JointState& getJointState(const std::string& jointName);

			const LinkState& getLinkState(const unsigned int linkIndex) const;
			const LinkState& getLinkState(const std::string& linkName) const;

			const JointState& getJointState(const unsigned int jointIndex) const;
			const JointState& getJointStateByMateIndex(const unsigned int mateIndex) const;
			const JointState& getJointState(const std::string& jointName) const;

			unsigned int getLinkIndex(const std::string& linkName) const;
			unsigned int getJointIndex(const std::string& jointName) const;
			unsigned int getJointIndexByMateIndex(const unsigned int& mateIdx) const;

			void addActiveJoint(const std::string& jointName);
			void addActiveJoint(const std::vector< std::string >& jointNameList);

			void eraseActiveJoint(const std::string& jointName);

			std::vector< std::string > getJointList() const;
			std::vector< std::string > getActiveJointList() const;
			std::vector< std::string > getPassiveJointList() const;

			void setJointq(const unsigned int jointIdx, const Math::VectorX& q) { setJointq(_jointState[jointIdx], q); }
			void setJointq(const std::string& jointName, const Math::VectorX& q) { setJointq(getJointState(jointName), q); }
			void setJointq(JointState& jointState, const Math::VectorX& q) { jointState.setq(q); _accumulatedT = _accumulatedJ = _accumulatedJDot = false; needUpdate(true, true, true); }

			const Math::VectorX& getJointq(const unsigned int jointIdx) const { return getJointq(_jointState[jointIdx]); }
			const Math::VectorX& getJointq(const std::string& jointName) const { return getJointq(getJointState(jointName)); }
			const Math::VectorX& getJointq(const JointState& jointState) const { return jointState.getq(); }

			void setJointqdot(const unsigned int jointIdx, const Math::VectorX& qdot) { setJointqdot(_jointState[jointIdx], qdot); }
			void setJointqdot(const std::string& jointName, const Math::VectorX& qdot) { setJointqdot(getJointState(jointName), qdot); }
			void setJointqdot(JointState& jointState, const Math::VectorX& qdot) { jointState.setqdot(qdot); _accumulatedJDot = false; needUpdate(false, true, true); }

			const Math::VectorX& getJointqdot(const unsigned int jointIdx) const { return getJointqdot(_jointState[jointIdx]); }
			const Math::VectorX& getJointqdot(const std::string& jointName) const { return getJointqdot(getJointState(jointName)); }
			const Math::VectorX& getJointqdot(const JointState& jointState) const { return jointState.getqdot(); }

			void setJointqddot(const unsigned int jointIdx, const Math::VectorX& qddot) { setJointqddot(_jointState[jointIdx], qddot); }
			void setJointqddot(const std::string& jointName, const Math::VectorX& qddot) { setJointqddot(getJointState(jointName), qddot); }
			void setJointqddot(JointState& jointState, const Math::VectorX& qddot) { jointState.setqddot(qddot); needUpdate(false, false, true); }

			const Math::VectorX& getJointqddot(const unsigned int jointIdx) const { return getJointq(_jointState[jointIdx]); }
			const Math::VectorX& getJointqddot(const std::string& jointName) const { return getJointq(getJointState(jointName)); }
			const Math::VectorX& getJointqddot(const JointState& jointState) const { return jointState.getq(); }

			void setActiveJointq(const Math::VectorX& q);
			void setPassiveJointq(const Math::VectorX& q);

			void addActiveJointq(const Math::VectorX& q);
			void addPassiveJointq(const Math::VectorX& q);

			void setActiveJointqdot(const Math::VectorX& qdot);
			void setPassiveJointqdot(const Math::VectorX& qdot);

			void addActiveJointqdot(const Math::VectorX& qdot);
			void addPassiveJointqdot(const Math::VectorX& qdot);

			void setActiveJointqddot(const Math::VectorX& qddot);
			void setPassiveJointqddot(const Math::VectorX& qddot);

			void addActiveJointqddot(const Math::VectorX& qddot);
			void addPassiveJointqddot(const Math::VectorX& qddot);

			unsigned int returnDof(const RETURN_STATE& return_state) const;
			void writeReturnMatrix(Math::MatrixX& returnMatrix, const Math::MatrixX& value, const unsigned int startRow, const unsigned int jointIndex, const RETURN_STATE& return_state) const;
			void writeReturnVector(Math::VectorX& returnVector, const Math::VectorX& value, const unsigned int jointIndex, const RETURN_STATE& return_state) const;

			const int getJointReferenceFrame() const { return _JointReferenceFrame; }
			void setJointReferenceFrame(const int JointReferenceFrame) { _JointReferenceFrame = JointReferenceFrame; }

			bool isUpdated(bool transform, bool velocity, bool acceleration) const { return (!transform | _TUpdated) & (!velocity | _VUpdated) & (!acceleration | _VDotUpdated); }
			void needUpdate(bool transform, bool velocity, bool acceleration) { _TUpdated &= !transform; _VUpdated &= !velocity; _VDotUpdated &= !acceleration; }

			void TUpdated() { _TUpdated = true; }
			void VUpdated() { _VUpdated = true; }
			void VDotUpdated() { _VDotUpdated = true; }

		private:
			State(const std::vector< std::string >& linkNameList, const std::vector< std::pair< std::string, unsigned int >>& jointNameList);

		private:
			unsigned int _totalJointDof, _activeJointDof;

			std::vector< LinkState, Eigen::aligned_allocator< LinkState >> _linkState;
			std::vector< JointState, Eigen::aligned_allocator< JointState >> _jointState;

			std::vector< std::string > _linkName;
			std::vector< std::string > _jointName;

			std::map< std::string, unsigned int > _linkIndexMap;
			std::map< std::string, unsigned int > _jointIndexMap;

			std::vector< bool > _isActiveJoint;
			std::vector< unsigned int > _activeJointList;
			std::vector< unsigned int > _passiveJointList;

			std::vector< unsigned int > _assemIndex;
			std::vector< unsigned int > _stateIndex;

			bool _TUpdated;
			bool _VUpdated;
			bool _VDotUpdated;

			int _JointReferenceFrame;

		public:
			bool _accumulatedT;
			bool _accumulatedJ;
			bool _accumulatedJDot;
		};
	}
}