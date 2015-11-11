#include "System.h"

#include <queue>

using namespace std;

namespace rovin
{
	namespace Dynamics
	{
		System::System(const Model::Assembly& model, const string& baselink)
		{
			_model = &model;
			_baselink = baselink;

			list< string > linklist = _model->getLinkNameList();
			list< string > jointlist = _model->getJointNameList();

			_num_link = linklist.size();
			_num_joint = jointlist.size();

			_link = new string[_num_link];
			_joint = new string[_num_joint];

			_linkptr = new shared_ptr< Model::Link >[_num_link];
			_jointptr = new shared_ptr< Model::Joint >[_num_joint];

			_connectionlist = new list< _CONN >[_num_link];

			_tree = new list< _CONN >[_num_link];
			_trace = new list< _CONN >[_num_link];

			int i;
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
				_connectionlist[s].push_back(_CONN(s, coniter->_mountLink_T, j, coniter->_actionLink_T, e, true));
				_connectionlist[e].push_back(_CONN(e, coniter->_actionLink_T.inverse(), j, coniter->_mountLink_T.inverse(), s, false));
			}

			queue<pair< unsigned int, list< _CONN >>> que;
			bool *check = new bool[_num_link];
			que.push(pair< unsigned int, list< _CONN >>(_root = getLinkNum(_baselink), list< _CONN >()));
			check[_root] = true;
			while (!que.empty())
			{
				pair< unsigned int, list< _CONN >> item = que.front();
				que.pop();

				unsigned int lastj;
				if (item.second.size() == 0) lastj = _num_joint + 1;
				else
				{
					list< _CONN >::iterator tmp = item.second.end();
					tmp--;
					lastj = tmp->_joint;
				}
				for (list< _CONN >::iterator iter = _connectionlist[item.first].begin(); iter != _connectionlist[item.first].end(); iter++)
				{
					if (lastj == iter->_joint) continue;

					if (check[iter->_elink])
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
						check[iter->_elink] = true;

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