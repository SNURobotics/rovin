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
		class Assembly;
		class State;
		typedef std::shared_ptr< State > StatePtr;
		enum JointReferenceFrame { UNDEFINED = -1, JOINTFRAME = 0, SPATIAL = 1, EE = 2 };

		class State
		{
			friend class Assembly;

		public:
			class LinkState;
			class JointState;
			const enum STATE_INFO;
			const enum TARGET_JOINT
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

			////////////////////////////////////////////
			//	Utility (External use)
			unsigned int getDOF(const TARGET_JOINT& return_state) const;
			void addActiveJoint(const std::string& jointName);
			void addActiveJoint(const std::vector< std::string >& jointNameList);
			void eraseActiveJoint(const std::string& jointName);
			void writeReturnMatrix(Math::MatrixX& returnMatrix, const Math::MatrixX& value, const unsigned int startRow, const unsigned int jointIndex, const TARGET_JOINT& return_state) const;
			void writeReturnVector(Math::VectorX& returnVector, const Math::VectorX& value, const unsigned int jointIndex, const TARGET_JOINT& return_state) const;
			
			//	Example of use:
			//	getInfoUpToDate(JOINTS_T_FROM_BASE | LINKS_POS);
			bool	getInfoUpToDate(int infoIdx);
			//	Example of use:
			//	setInfoUpToDate(JOINTS_T_FROM_BASE | JOINTS_JACOBIAN | JOINTS_JACOBIAN_DOT, false);
			void	setInfoUpToDate(int infoIdx, bool upToDate = true);

			////////////////////////////////////////////
			//	Get information related to Link.
			unsigned int		getLinkIndex(const std::string& linkName) const;
			LinkState&			getLinkState(const unsigned int linkIndex);
			LinkState&			getLinkState(const std::string& linkName);
			const LinkState&	getLinkState(const unsigned int linkIndex) const;
			const LinkState&	getLinkState(const std::string& linkName) const;

			////////////////////////////////////////////
			//	Get values from joints.
			std::vector< std::string >getJointList(const TARGET_JOINT& target) const;
			const int				getJointReferenceFrame() const { return _JointReferenceFrame; }
			unsigned int			getJointID(const TARGET_JOINT& target, const unsigned int idx) const;
			unsigned int			getJointIndex(const std::string& jointName) const;
			unsigned int			getJointIndexByMateIndex(const unsigned int& mateIdx) const { return mateIdx; }
			JointState&				getJointState(const unsigned int jointIndex);
			JointState&				getJointState(const std::string& jointName);
			JointState&				getJointState(const TARGET_JOINT& target, const unsigned int idx);
			const JointState&		getJointState(const unsigned int jointIndex) const;
			const JointState&		getJointState(const std::string& jointName) const;
			const JointState&		getJointState(const TARGET_JOINT& target, const unsigned int idx) const;
			JointState&				getJointStateByMateIndex(const unsigned int mateIndex);
			const JointState&		getJointStateByMateIndex(const unsigned int mateIndex) const;
			const Math::VectorX&	getJointq(const JointState& jointState) const;
			const Math::VectorX&	getJointq(const unsigned int jointIdx) const;
			const Math::VectorX&	getJointq(const std::string& jointName) const;
			Math::VectorX			getJointq(const TARGET_JOINT& target) const;
			const Math::VectorX&	getJointq(const TARGET_JOINT& target, unsigned int jointIdx) const;
			Math::VectorX			getJointq(const TARGET_JOINT& target, const Math::VectorU& jointIndices) const;
			const Math::VectorX&	getJointqdot(const unsigned int jointIdx) const;
			const Math::VectorX&	getJointqdot(const std::string& jointName) const;
			const Math::VectorX&	getJointqdot(const JointState& jointState) const;
			Math::VectorX			getJointqdot(const TARGET_JOINT& target) const;
			const Math::VectorX&	getJointqdot(const TARGET_JOINT& target, unsigned int jointIdx) const;
			Math::VectorX			getJointqdot(const TARGET_JOINT& target, const Math::VectorU& jointIndices) const;
			const Math::VectorX&	getJointqddot(const JointState& jointState) const;
			const Math::VectorX&	getJointqddot(const unsigned int jointIdx) const;
			const Math::VectorX&	getJointqddot(const std::string& jointName) const;
			Math::VectorX			getJointqddot(const TARGET_JOINT& target) const;
			const Math::VectorX&	getJointqddot(const TARGET_JOINT& target, unsigned int jointIdx) const;
			Math::VectorX			getJointqddot(const TARGET_JOINT& target, const Math::VectorU& jointIndices) const;
			Math::VectorX			getJointTorque(const TARGET_JOINT& target) const;

			////////////////////////////////////////////
			//	Set values to joints.
			void setJointReferenceFrame(const JointReferenceFrame JointReferenceFrame) { _JointReferenceFrame = JointReferenceFrame; }
			void setJointq(JointState& jointState, const Math::VectorX& q);
			void setJointq(const unsigned int jointIdx, const Math::VectorX& q);
			void setJointq(const std::string& jointName, const Math::VectorX& q);
			void setJointq(const TARGET_JOINT& target, const Math::VectorX& q);
			void setJointq(const TARGET_JOINT& target, unsigned int jointIdx, const Math::VectorX& q);
			void setJointq(const TARGET_JOINT& target, const Math::VectorU& jointIndices, const Math::VectorX& q);
			void setJointqdot(JointState& jointState, const Math::VectorX& qdot);
			void setJointqdot(const unsigned int jointIdx, const Math::VectorX& qdot);
			void setJointqdot(const std::string& jointName, const Math::VectorX& qdot);
			void setJointqdot(const TARGET_JOINT& target, const Math::VectorX& qdot);
			void setJointqdot(const TARGET_JOINT& target, unsigned int jointIdx, const Math::VectorX& qdot);
			void setJointqdot(const TARGET_JOINT& target, const Math::VectorU& jointIndices, const Math::VectorX& qdot);
			void setJointqddot(JointState& jointState, const Math::VectorX& qddot);
			void setJointqddot(const unsigned int jointIdx, const Math::VectorX& qddot);
			void setJointqddot(const std::string& jointName, const Math::VectorX& qddot);
			void setJointqddot(const TARGET_JOINT& target, const Math::VectorX& qddot);
			void setJointqddot(const TARGET_JOINT& target, unsigned int jointIdx, const Math::VectorX& qddot);
			void setJointqddot(const TARGET_JOINT& target, const Math::VectorU& jointIndices, const Math::VectorX& qddot);

			////////////////////////////////////////////
			//	Add values to joints.
			void addJointq(const TARGET_JOINT& target, const Math::VectorX& q);
			void addJointqdot(const TARGET_JOINT& target, const Math::VectorX& qdot);
			void addJointqddot(const TARGET_JOINT& target, const Math::VectorX& qddot);
			void addJointTorque(const TARGET_JOINT& target, const Math::VectorX& tau);
		
		public:
			//	Public member variables.
			const enum STATE_INFO
			{
				LINKS_POS = 1 << 0,		//	Position(SE3) of all links.
				LINKS_VEL = 1 << 1,		//	Velocity(se3) of all links.
				LINKS_ACC = 1 << 2,		//	Accelaration(se3) of all links.
				JOINTS_T_FROM_BASE = 1 << 3,	//	Product of transform from base to each joint.
				JOINTS_JACOBIAN = 1 << 4,		//	Jacobian(theta)
				JOINTS_JACOBIAN_DOT = 1 << 5,	//	d Jacobian(theta) / dt

				ALL_LINKS = LINKS_POS | LINKS_VEL | LINKS_ACC,
				ALL_JOINTS = JOINTS_T_FROM_BASE | JOINTS_JACOBIAN | JOINTS_JACOBIAN_DOT,
				ALL_INFO = LINKS_POS | LINKS_VEL | LINKS_ACC | JOINTS_T_FROM_BASE | JOINTS_JACOBIAN | JOINTS_JACOBIAN_DOT,
			};

			unsigned int getAssemIndex(const unsigned int jointID)
			{
				return _assemIndex[jointID];
			}

		private:
			State(const std::vector< std::string >& linkNameList, const std::vector< std::pair< std::string, unsigned int >>& jointNameList);

			JointReferenceFrame _JointReferenceFrame;

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

			//	i-th bit of '_stateInfoUpToDate' contains up-to-date state of i-th 'STATE_INFO'.
			int	_stateInfoUpToDate;

		public:
			//////////////////////////////////////////////////////////////
			//						JOINT_STATE							//
			//////////////////////////////////////////////////////////////
			class JointState
			{
				friend State;
			public:
				const enum JOINT_INFO
				{
					TRANSFORM = 1 << 0,
					JACOBIAN = 1 << 1,
					JACOBIAN_DOT = 1 << 2,
					ALL_INFO = TRANSFORM | JACOBIAN | JACOBIAN_DOT,
				};

				JointState(const unsigned int& dof);
				unsigned int _dof;

				Math::VectorX _tau;
				Math::dse3 _constraintF;

				//	exp([S_i] * theta_i).
				std::vector< Math::SE3 > _T;
				//	Jacobian Matrix at zero position (only depends on state of current joint).
				Math::Matrix6X _J;
				//	Time derivative of Jacobian Matrix.
				Math::Matrix6X _JDot;

				//	Products of exp([S_i] * theta_i) from base.
				Math::SE3 _accumulatedT;
				//	Jacobian at current state. (Adjoint of _J with ).
				Math::Matrix6X _accumulatedJ;
				//	Time derivative of Jacobian at current state.
				Math::Matrix6X _accumulatedJDot;

				const Math::VectorX& getq() const { return _q; }
				Math::Real getq(unsigned int idx) const { return _q[idx]; }
				const Math::VectorX& getqdot() const { return _qdot; }
				Math::Real getqdot(unsigned int idx) const { return _qdot[idx]; }
				const Math::VectorX& getqddot() const { return _qddot; }
				Math::Real getqddot(unsigned int idx) const { return _qddot[idx]; }

				const unsigned int getDOF() const { return _dof; }

				const JointReferenceFrame getJointReferenceFrame() const { return _JointReferenceFrame; }
				void setJointReferenceFrame(const JointReferenceFrame JointReferenceFrame) { _JointReferenceFrame = JointReferenceFrame; }

				//	Example of use:
				//	getInfoUpToDate(TRANSFORM | JACOBIAN);
				bool	getInfoUpToDate(int infoIdx);
				//	Example of use:
				//	setInfoUpToDate(TRANSFORM | JACOBIAN | JACOBIAN_DOT, false);
				void	setInfoUpToDate(int infoIdx, bool upToDate = true);

			private:
				//	Warning: Do access in 'State'. //
				void setq(const Math::VectorX& q) { assert(_q.size() == q.size());  _q = q; setInfoUpToDate(ALL_INFO,false); }
				void setqdot(const Math::VectorX& qdot) { _qdot = qdot; setInfoUpToDate(JACOBIAN_DOT); }
				void setqddot(const Math::VectorX& qddot) { _qddot = qddot;}
				//	Warning: Do access in 'State'. //
				void addq(const Math::VectorX& q) { _q += q; setInfoUpToDate(ALL_INFO, false);}
				void addqdot(const Math::VectorX& qdot) { _qdot += qdot; setInfoUpToDate(JACOBIAN_DOT);}
				void addqddot(const Math::VectorX& qddot) { _qddot += qddot;}

			private:
				Math::VectorX _q;
				Math::VectorX _qdot;
				Math::VectorX _qddot;
				JointReferenceFrame _JointReferenceFrame;
				int _jointInfoUpToDate = 0;
			};

			//////////////////////////////////////////////////////////////
			//						LINK_STATE							//
			//////////////////////////////////////////////////////////////

			class LinkState
			{
			public:
				LinkState()
				{
					_V.setZero();
				}

				Math::SE3 _T;
				Math::se3 _V;
				Math::se3 _VDot;
			};

		};
	}
}