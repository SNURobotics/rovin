/**
*	\file	Assembly.h
*	\date	2015.11.07
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Assembly의 정보를 저장하는 클래스 정의
*/

#pragma once

#include <string>
#include <vector>
#include <map>
#include <memory>

#include "Link.h"
#include "Joint.h"
#include "State.h"

namespace rovin
{
	class Kinematics;
	class Dynamics;

	namespace Model
	{
		class Joint;
		class Link;
		class Assembly;

		typedef std::shared_ptr< Assembly > AssemblyPtr;

		enum JointDirection			{ REGULAR, REVERSE };
		

		/**
		*	\class Assembly
		*	\brief Link와 joint를 연결시켜주고 모델에 대한 정보를 저장하는 클래스입니다.
		*/
		class Assembly
		{
			friend class Kinematics;
			friend class Dynamics;

		public:
			class Mate
			{
			public:
				JointPtr _joint;
				unsigned int _mountLinkIdx;
				unsigned int _actionLinkIdx;

				Math::SE3 _Tmj;
				Math::SE3 _Tja;

				Mate(const Model::JointPtr& joint, const unsigned int mountLinkIdx, const unsigned int actionLinkIdx,
					const Math::SE3& Tmj, const Math::SE3& Tja);

				unsigned int getParentLinkIdx(const JointDirection& jointDirection = JointDirection::REGULAR) const;
				unsigned int getChildLinkIdx(const JointDirection& jointDirection = JointDirection::REGULAR) const;
			};

			enum JOINT_KINEMATICS_OPTION
			{
				JOINT_TRANSFORM = 1 << 0,
				JOINT_JACOBIAN = 1 << 1,
				JOINT_JACOBIANDOT = 1 << 2
			};

			Assembly(const std::string& assemblyName) : 
				_assemblyName((utils::checkName(assemblyName) ? (assemblyName) : (assert(0 && "Assmebly의 이름으로 사용할 수 없는 이름이 들어왔습니다."), ""))), _complete(false), _linkPtr(), _Mate(), 
				_linkIndexMap(), _markerIndexMap(), _mateIndexMap(), _baseLink(-1), _Tree(), _ClosedLoopConstraint() {}
			Assembly(const Assembly& target);

			~Assembly();

			StatePtr makeState() const;

			Assembly& operator = (const Assembly& target);
			Assembly operator + (const Assembly& target);
			Assembly& operator += (const Assembly& target);

			std::string getName() const;
			void setName(const std::string assemblyName);

			const std::vector< LinkPtr >& getLinkList() const;
			const std::vector< Mate >& getMateList() const;

			unsigned int getLinkIndex(const std::string& linkName) const;
			unsigned int getLinkIndexByMarkerName(const std::string& markerName) const;
			unsigned int getMateIndexByJointName(const std::string& jointName) const;

			LinkPtr Assembly::getLinkPtr(const std::string& linkName) const;
			JointPtr Assembly::getJointPtr(const std::string& jointName) const;
			LinkPtr Assembly::getLinkPtr(const unsigned int linkIndex) const;
			JointPtr Assembly::getJointPtrByMateIndex(const unsigned int mateIndex) const;

			void addLink(const Model::LinkPtr& linkPtr);
			void addMate(const Model::JointPtr& jointPtr, const Model::LinkPtr& mountLinkPtr, const Model::LinkPtr& actionLinkPtr,
				const Math::SE3& Tmj, const Math::SE3& Tja);
			void addMate(const Model::JointPtr& jointPtr, const std::string& mountLinkMarkerName, const Model::LinkPtr& actionLinkPtr,
				const Math::SE3& Tmj, const Math::SE3& Tja);
			void addMate(const Model::JointPtr& jointPtr, const Model::LinkPtr& mountLinkPtr, const std::string& actionLinkMarkerName,
				const Math::SE3& Tmj, const Math::SE3& Tja);
			void addMate(const Model::JointPtr& jointPtr, const std::string& mountLinkMarkerName, const std::string& actionLinkMarkerName,
				const Math::SE3& Tmj, const Math::SE3& Tja);

			void deleteLink(const std::string& linkName);
			void deleteMateByJointName(const std::string& jointName);

			void changeName(const std::string& oldName, const std::string& newName);
			void changeNameAll(const std::string& prefix);
			AssemblyPtr copy(const std::string& prefiex = ("")) const;

			bool isCompleted() const;
			void setAssemblyMode();
			void completeAssembling(const std::string& baseLinkName);

			void updateJointKinematics(const unsigned int mateIdx, State::JointState& jointState, const unsigned int options) const;

			Math::SE3 getTransform(const unsigned int mateIdx, State::JointState& jointState, const JointDirection& jointDirection) const;
			Math::Matrix6X getJacobian(const unsigned int mateIdx, State::JointState& jointState, const JointDirection& jointDirection) const;
			Math::Matrix6X getJacobianDot(const unsigned int mateIdx, State::JointState& jointState, const JointDirection& jointDirection) const;

			static std::pair< unsigned int, JointDirection > reverseDirection(const std::pair< unsigned int, JointDirection >& target);

		public:
			std::vector< std::pair< unsigned int, Model::JointDirection >> _Tree;

		protected:
			std::string _assemblyName;

			bool _complete;

			std::vector< Model::LinkPtr >	_linkPtr;
			std::vector< Mate > _Mate;

			std::map< std::string, unsigned int > _linkIndexMap;
			std::map< std::string, unsigned int > _markerIndexMap;
			std::map< std::string, unsigned int > _mateIndexMap;

			unsigned int _baseLink;
			std::vector< std::pair< unsigned int, Model::JointDirection >> _Parent;
			std::vector< unsigned int > _Depth;
			std::vector< std::vector< std::pair< unsigned int, Model::JointDirection >>> _ClosedLoopConstraint;
		};
	}
}