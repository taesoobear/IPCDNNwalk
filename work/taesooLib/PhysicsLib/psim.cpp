/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * The University of Tokyo
 */
/**
 * psim.cpp
 * Create: Katsu Yamane, 04.02.23
 */


#include "psim.h"
#include <assert.h>



int pSim::ConstraintForces(fVec& cf)
{
	int i;
	for(i=0; i<n_joint; i++)
	{
		fVec& f_final = joint_info[i].pjoints[0]->f_final;
		int idx = 6*joint_info[i].pjoints[0]->joint->i_joint;
		cf(idx) = f_final(0);
		cf(idx+1) = f_final(1);
		cf(idx+2) = f_final(2);
		cf(idx+3) = f_final(3);
		cf(idx+4) = f_final(4);
		cf(idx+5) = f_final(5);
	}
	return 0;
}

int pSim::TotalCost()
{
	return subchains->total_cost();
}

int pSubChain::total_cost()
{
	if(!this) return 0;
	// cost is proportional to square of n_outer_joints
	int ret = 0;
	if(n_links > 1) ret = n_outer_joints*n_outer_joints;
	ret += children[0]->total_cost();
	if(children[0] != children[1])
		ret += children[1]->total_cost();
	return ret;
}

int pSim::ScheduleDepth()
{
	return subchains->schedule_depth();
}

int pSubChain::schedule_depth()
{
	if(!this || n_links == 1) return 0;
	int ret0 = 0, ret1 = 0;
	if(children[0]) ret0 = children[0]->schedule_depth();
	if(children[1]) ret1 = children[1]->schedule_depth();
	return (ret0 > ret1) ? (ret0+1) : (ret1+1);
}

int pSim::NumLeaves()
{
	return subchains->num_leaves();
}

int pSubChain::num_leaves()
{
	if(!this || n_links == 1) return 0;
	if(children[0] && children[1] &&
	   children[0]->n_links == 1 && children[1]->n_links == 1)
	{
		return 1;
	}
	int ret = children[0]->num_leaves();
	if(children[0] != children[1])
		ret += children[1]->num_leaves();
	return ret;
}

/**
 * Clear
 */
void pSim::Clear()
{
	if(joint_info) 
	{
		for(int i=0; i<n_joint; i++)
		{
			delete joint_info[i].pjoints[0];
			delete joint_info[i].pjoints[1];
			if(joint_info[i].plink) delete joint_info[i].plink;
		}
		delete[] joint_info;
	}
	if(subchains) delete subchains;
	joint_info = 0;
	subchains = 0;
	Chain::Clear();
}

int pSim::clear_data()
{
	if(joint_info)
	{
		for(int i=0; i<n_joint; i++)
		{
			delete joint_info[i].pjoints[0];
			delete joint_info[i].pjoints[1];
			if(joint_info[i].plink) delete joint_info[i].plink;
		}
		delete[] joint_info;
	}
	if(subchains) delete subchains;
	joint_info = 0;
	subchains = 0;
	return Chain::clear_data();
}

int pSim::clear_contact()
{
	int n_contacts = contact_vjoints.size();
	if(n_contacts == 0) return 0;
	int org_n_joint = n_joint;
	JointInfo* jinfo_save = joint_info;
	for(joint_list::iterator j=contact_vjoints.begin(); j!=contact_vjoints.end(); j++)
	{
		RemoveJoint(*j);
		delete *j;
	}
	// clear and init Chain's data
	Chain::clear_data();
#ifdef SEGA
	Chain::init();
#else
	Chain::init(0);
#endif
	joint_info = new JointInfo [n_joint];
	for(int i=0; i<org_n_joint; i++)
	{
		if(contact_vjoint_index(jinfo_save[i].pjoints[0]->joint) < 0)
		{
			Joint* j = jinfo_save[i].pjoints[0]->joint;
			joint_info[j->i_joint].pjoints[0] = jinfo_save[i].pjoints[0];
			joint_info[j->i_joint].pjoints[1] = jinfo_save[i].pjoints[1];
			joint_info[j->i_joint].plink = jinfo_save[i].plink;
		}
		else
		{
			delete jinfo_save[i].pjoints[0];
			delete jinfo_save[i].pjoints[1];
			if(jinfo_save[i].plink) delete jinfo_save[i].plink;
		}
	}
	if(jinfo_save) delete[] jinfo_save;
	contact_vjoints.clear();
	contact_relvels.clear();
	fric_coefs.clear();
	all_vjoints.clear();
	all_Jv.clear();
	all_Jr.clear();
	all_jdot_v.clear();
	all_jdot_r.clear();
	return 0;
}

/**
 * Init
 */
int pSim::init_contact()
{
	int n_contacts = contact_vjoints.size();
//	if(n_contacts == 0) return 0;
	int org_n_joint = n_joint;
	// clear and init Chain's data
	Chain::clear_data();
#ifdef SEGA
	Chain::init();
#else
	Chain::init(0);
#endif
	// create pjoint and plink for contact joints
	JointInfo* jinfo_save = joint_info;
	joint_info = new JointInfo [n_joint];
	for(int i=0; i<org_n_joint; i++)
	{
		int new_index = jinfo_save[i].pjoints[0]->joint->i_joint;
		joint_info[new_index].pjoints[0] = jinfo_save[i].pjoints[0];
		joint_info[new_index].pjoints[1] = jinfo_save[i].pjoints[1];
		joint_info[new_index].plink = jinfo_save[i].plink;
	}
	// contact joints
	for(int i=0; i<n_contacts; i++)
	{
		Joint* j = contact_vjoints[i];
		assert(j->real);  // must be virtual joint
		pJoint* pj0 = new pJoint(j, j->real);
		pJoint* pj1 = new pJoint(j, j->parent);
		pj0->plink = joint_info[j->real->i_joint].plink;
		if(j->parent)
			pj1->plink = joint_info[j->parent->i_joint].plink;
		pj1->pair = pj0;
		joint_info[j->i_joint].pjoints[0] = pj0;
		joint_info[j->i_joint].pjoints[1] = pj1;
		joint_info[j->i_joint].plink = 0;
	}
	if(jinfo_save) delete[] jinfo_save;
	subchains->init();
	return 0;
}

#ifdef SEGA
int pSim::init()
#else
int pSim::init(SceneGraph* sg)
#endif
{
#ifdef SEGA
	int ret = Chain::init();
#else
	int ret = Chain::init(sg);
#endif
	if(ret) return ret;
	myinit();
	return 0;
}

int pSim::myinit()
{
	if(joint_info) delete[] joint_info;
	joint_info = 0;
	if(n_joint == 0) return 0;
	joint_info = new JointInfo[n_joint];
	setup_pjoint(root);
	setup_pjoint_virtual(root);
	calc_consts();
	return 0;
}

void pSim::setup_pjoint(Joint* j)
{
	if(!j) return;
	if(j->real) return;  // process virtual links later
	if(j->i_joint >= 0)
	{
		pJoint* pj0 = new pJoint(j, j);
		pJoint* pj1 = new pJoint(j, j->parent);
		pLink* pl = new pLink(j);
		pj0->plink = pl;
		if(j->parent)
			pj1->plink = joint_info[j->parent->i_joint].plink;
		pj0->pair = pj1;
		pj1->pair = pj0;
		joint_info[j->i_joint].pjoints[0] = pj0;
		joint_info[j->i_joint].pjoints[1] = pj1;
		joint_info[j->i_joint].plink = pl;
	}
	setup_pjoint(j->brother);
	setup_pjoint(j->child);
}

void pSim::setup_pjoint_virtual(Joint* j)
{
	if(!j) return;
	if(j->real)
	{
		pJoint* pj0 = new pJoint(j, j->real);
		pJoint* pj1 = new pJoint(j, j->parent);
//		pJoint* pj0 = new pJoint(j, j->parent);
//		pJoint* pj1 = new pJoint(j, j->real);
		pj0->plink = joint_info[j->real->i_joint].plink;
		if(j->parent)
			pj1->plink = joint_info[j->parent->i_joint].plink;
		pj1->pair = pj0;
		joint_info[j->i_joint].pjoints[0] = pj0;
		joint_info[j->i_joint].pjoints[1] = pj1;
		joint_info[j->i_joint].plink = 0;
	}
	setup_pjoint_virtual(j->brother);
	setup_pjoint_virtual(j->child);
}

void pSim::calc_consts()
{
	int i;
	for(i=0; i<n_joint; i++)
	{
		if(joint_info[i].plink) joint_info[i].plink->calc_inertia();
	}
}

void pLink::calc_inertia()
{
	if(joint->n_root_dof == 0) return;
//	if(!joint->parent) return;  // space
	static fMat33 m11, m12, m22;  // m21 equals to m12
	double sx = joint->loc_com(0), sy = joint->loc_com(1), sz = joint->loc_com(2);
	fMat33 scross(0, -sz, sy,
				  sz, 0, -sx,
				  -sy, sx, 0);
	m11.identity();
	m11 *= joint->mass;
	m12.mul(joint->mass, scross);
	m22.mul(scross, scross);
	m22 *= -joint->mass;
	m22 += joint->inertia;
	if(joint->j_type == JROTATE)
	{
		static fVec3 n2J;
		static fMat33 n2Jmat;
		n2J.mul(joint->gear_ratio*joint->gear_ratio*joint->rotor_inertia, joint->axis);
		n2Jmat.mul(n2J, n2J);
		m22 += n2Jmat;
	}
	int i, j;
	for(i=0; i<3; i++)
	{
		for(j=0; j<3; j++)
		{
			M(i, j) = m11(i, j);
			M(i, j+3) = -m12(i, j);
			M(i+3, j) = m12(i, j);
			M(i+3, j+3) = m22(i, j);
		}
	}
	Minv.inv_posv(M);
//	Minv.inv_porfs(M);
}

/*
 * Dump
 */
void pSim::DumpSchedule(ostream& ost)
{
	subchains->dump(ost);
}

void pSubChain::dump(ostream& ost)
{
	if(!this) return;
	int i;
	ost << "-- pSubChain ->" << endl;
	if(last_pjoints[0]) ost << "\tlast: " << last_pjoints[0]->joint->name << endl;
	else ost << "\tsingle link" << endl;
	if(parent) ost << "\tparent: " << parent->last_pjoints[0]->joint->name << endl;
	ost << "\touter: " << n_outer_joints << endl;
	for(i=0; i<n_outer_joints; i++)
		outer_joints[i]->dump(ost);
	ost << "\tlinks: " << n_links << endl;
	for(i=0; i<n_links; i++)
		links[i]->dump(ost);
	ost << "<-" << endl;
	children[0]->dump(ost);
	if(children[1] != children[0]) children[1]->dump(ost);
}

void pJoint::dump(ostream& ost)
{
	ost << "\t\t" << joint->name << "->" << link_side->name << endl;
}

void pLink::dump(ostream& ost)
{
	ost << "\t\t" << joint->name << endl;
}
