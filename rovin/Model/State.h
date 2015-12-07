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
				Math::Real getq(unsigned int idx) const { return _q[idx]; }
				const Math::VectorX& getqdot() const { return _qdot; }
				Math::Real getqdot(unsigned int idx) const { return _qdot[idx]; }
				const Math::VectorX& getqddot() const { return _qddot; }
				Math::Real getqddot(unsigned int idx) const { return _qddot[idx]; }

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

			enum TARGET_JOINT
			{
				//	active joint + passive joint, sorted in assembled order
				ASSEMJOINT,
				//	active joint + passive joint, sorted in state order ??
				STATEJOINT,
				//	only active joint
				ACTIVEJOINT,
				//	only passive joint
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

			void setActiveJointTorque(const Math::VectorX& torque);

			void addActiveJointqddot(const Math::VectorX& qddot);
			void addPassiveJointqddot(const Math::VectorX& qddot);

			Math::VectorX getJointTorque(const TARGET_JOINT& target) const;
			Math::VectorX getJointq(const TARGET_JOINT& target) const;
			Math::VectorX getJointqdot(const TARGET_JOINT& target) const;
			Math::VectorX getJointqddot(const TARGET_JOINT& target) const;

			unsigned int getDOF(const TARGET_JOINT& return_state) const;
			void writeReturnMatrix(Math::MatrixX& returnMatrix, const Math::MatrixX& value, const unsigned int startRow, const unsigned int jointIndex, const TARGET_JOINT& return_state) const;
			void writeReturnVector(Math::VectorX& returnVector, const Math::VectorX& value, const unsigned int jointIndex, const TARGET_JOINT& return_state) const;

			const int getJointReferenceFrame() const { return _JointReferenceFrame; }
			void setJointReferenceFrame(const int JointReferenceFrame) { _JointReferenceFrame = JointReferenceFrame; }

			bool isUpdated(bool transform, bool velocity, bool acceleration) const { return (!transform | _TUpdated) & (!velocity | _VUpdated) & (!acceleration | _VDotUpdated); }
			void needUpdate(bool transform, bool velocity, bool acceleration) { _TUpdated &= !transform; _VUpdated &= !velocity; _VDotUpdated &= !acceleration; }

			void TUpdated() { _TUpdated = true; }
			void VUpdated() { _VUpdated = true; }
			void VDotUpdated() { _VDotUpdated = true; }

			unsigned int getAssemIndex(const unsigned int jointID)
			{
				return _assemIndex[jointID];
			}

		private:
			State(const std::vector< std::string >& linkNameList, const std::vector< std::pair< std::string, unsigned int >>& jointNameList);

		private:
			unsigned int _totalJointDof, _activeJointDof;

			//	'LinkState's are stored in assembled order.
			std::vector< LinkState, Eigen::aligned_allocator< LinkState >> _linkState;
			//	'JointState's are stored in assembled order.
			std::vector< JointState, Eigen::aligned_allocator< JointState >> _jointState;

			//	Name of links are stored in assembled order.
			std::vector< std::string > _linkName;
			//	Name of joints are stored in assembled order.
			std::vector< std::string > _jointName;

			//	Map from name of link to index in '_linkName' or '_linkState'.
			std::map< std::string, unsigned int > _linkIndexMap;
			//	Map from name of joint to index in '_jointName' or '_jointState'.
			std::map< std::string, unsigned int > _jointIndexMap;

			//	Notify whether i-th joint (assembled order) is active or not.
			std::vector< bool > _isActiveJoint;
			//	Index (in assembled order) of active joints.
			std::vector< unsigned int > _activeJointList;
			//	Index (in assembled order) of passive joints.
			std::vector< unsigned int > _passiveJointList;

			//	DOF index of i-th (assembled order) joint in whole DOF.
			std::vector< unsigned int > _assemIndex;
			//	DOF index of i-th (assembled order) joint in active / passive (each) DOF
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