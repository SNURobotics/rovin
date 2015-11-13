#include "System.h"

#include <queue>

using namespace std;

namespace rovin
{
	namespace Dynamics
	{
		System::System(const std::shared_ptr< Model::Assembly >& model, const string& baselink)
		{
			_model = model;
			_model->LOCK();
			_baselink = baselink;

			list< string > linklist = _model->getLinkNameList();
			list< string > jointlist = _model->getJointNameList();

			_num_link = linklist.size();
			_num_joint = jointlist.size();

			_link.resize(_num_link);
			_joint.resize(_num_joint);

			_linkptr.resize(_num_link);
			_jointptr.resize(_num_joint);

			_connectionlist.resize(_num_link);

			_tree.resize(_num_link);
			_trace.resize(_num_link);

			unsigned int i;
			list< string >::iterator iter;
			for (i = 0, iter = linklist.begin(); i < _num_link; i++, iter++)
			{
				_link[i] = *iter;
				_linkptr[i] = _model->findLink(*iter);
				_linkmap.insert(pair< string, int >(*iter, i));
			}
			for (i = 0, iter = jointlist.begin(); i < _num_joint; i++, iter++)
			{
				_joint[i] = *iter;
				_jointptr[i] = _model->findJoint(*iter);
				_jointmap.insert(pair< string, int >(*iter, i));
			}

			const list< Model::Assembly::Connection > connection = _model->getConnection();
			for (list< Model::Assembly::Connection >::const_iterator coniter = connection.begin(); coniter != connection.end(); coniter++)
			{
				unsigned int s = getLinkNum(coniter->_mountLink_pointer->getName());
				unsigned int e = getLinkNum(coniter->_actionLink_pointer->getName());
				unsigned int j = getJointNum(coniter->_joint_pointer->getName());
				_connectionlist[s].push_back(_CONN(s, coniter->_mountLink_T, j, coniter->_actionLink_T, e, false));
				_connectionlist[e].push_back(_CONN(e, coniter->_actionLink_T.inverse(), j, coniter->_mountLink_T.inverse(), s, true));
			}

			_BFS.clear();
			queue< pair< unsigned int, list< _CONN >>> que;
			std::vector< bool > lcheck, jcheck;
			lcheck.resize(_num_link);
			jcheck.resize(_num_joint);
			que.push(pair< unsigned int, list< _CONN >>(_baseLinkIndex = getLinkNum(_baselink), list< _CONN >()));
			lcheck[_baseLinkIndex] = true;
			while (!que.empty())
			{
				pair< unsigned int, list< _CONN >> item = que.front();
				que.pop();

				for (list< _CONN >::iterator iter = _connectionlist[item.first].begin(); iter != _connectionlist[item.first].end(); iter++)
				{
					if (jcheck[iter->_joint])
					{
						continue;
					}
					jcheck[iter->_joint] = true;

					if (lcheck[iter->_elink])
					{
						list< _CONN >::iterator n1;
						list< _CONN >::iterator n2;

						list< _CONN > closedloop1;
						list< _CONN > closedloop2;

						list< _CONN > closedloop;

						for (n1 = _trace[iter->_slink].begin(), n2 = _trace[iter->_elink].begin(); n1 != _trace[iter->_slink].end() && n2 != _trace[iter->_elink].end(); n1++, n2++)						
						{
							if (n1->_elink == n2->_elink) continue;
							break;
						}
						for (; n1 != _trace[iter->_slink].end(); n1++)
						{
							closedloop1.push_back(*n1);
						}
						for (; n2 != _trace[iter->_elink].end(); n2++)
						{
							closedloop2.push_back(*n2);
						}
						closedloop = closedloop1;
						closedloop.push_back(*iter);
						list< _CONN >::iterator lip = closedloop2.end();
						while (lip != closedloop2.begin())
						{
							lip--;
							closedloop.push_back((*lip).flip());
						}

						_closedloop.push_back(closedloop);
					}
					else
					{
						_tree[iter->_slink].push_back(*iter);
						lcheck[iter->_elink] = true;
						_BFS.push_back(*iter);

						list< _CONN > nexttrace = item.second;
						nexttrace.push_back(*iter);
						que.push(pair< unsigned int, list< _CONN >>(iter->_elink, nexttrace));

						_trace[iter->_elink] = nexttrace;
					}
				}
			}
		}
	}
}