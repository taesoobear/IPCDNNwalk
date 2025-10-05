/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * The University of Tokyo
 */
/*!
 * @file   psim.h
 * @author Katsu Yamane
 * @date   02/23/2004
 * @brief  Forward dynamics computation based on Assembly-Disassembly Algorithm.
 */

#ifndef __PSIM_H__
#define __PSIM_H__

#include <chain.h>
#include <fMatrix3.h>
#include <list>
#include <vector>

class pLink;
class pSim;

struct JointInfo;
typedef std::list<class pSubChain*> subchain_list;
typedef std::vector<Joint*> joint_list;
typedef std::vector<class pJoint*> p_joint_list;

//! Number of vertices of the friction cone approximation.
//#define N_FRIC_CONE_DIV 8
#define N_FRIC_CONE_DIV 4  // by taesoo

//#define VERBOSE

// Some definitions for test and logging.
// Divide-and-Conquer Algorithm (Featherstone 1999) test 
//#define USE_DCA
// use normal Lemke method
//#define USE_NORMAL_LEMKE
// measure CPU time for collision simulation
// check timing for parallel processing
//#define TIMING_CHECK


/*!
 * enum for defining the joint axis
 * IMPORTANT NOTICE:
 *  The axis of 1DOF joints should be parallel to
 *  x, y, or z axis of the parent joint.
 *  Other axis is not supported. 
 */
enum PSIM_AXIS
{
	PSIM_AXIS_NULL = -1,
	PSIM_AXIS_X, 
	PSIM_AXIS_Y,
	PSIM_AXIS_Z,
	PSIM_AXIS_RX,
	PSIM_AXIS_RY,
	PSIM_AXIS_RZ,
};


/*!
 * @class pJoint
 * @brief Class for representing "handle"; two pJoint instances are attached to both sides of each joint.
 */
class pJoint
{
	friend class pSim;
	friend class pLink;
	friend class pSubChain;
public:
	pJoint(Joint* _joint, Joint* _link_side) {
		joint = _joint;
		link_side = _link_side;
		if(!link_side || joint == link_side) parent_side = false;
		else parent_side = true;
		J.resize(6, 6);
		Jdot.resize(6);
		J.identity();
		Jdot.zero();
		f_final.resize(6);
		f_final.zero();
		acc_final.resize(6);
		acc_final.zero();
		dvel.resize(6);
		dvel.zero();
		colf_final.resize(6);
		colf_final.zero();
		vel_final.resize(6);
		vel_final.zero();
		plink = 0;
		subchain = 0;
	}
	
	~pJoint() {
	}

	pJoint* Pair() {
		return pair;
	}
	Joint* GetJoint() {
		return joint;
	}
	int ParentSide() {
		return parent_side;
	}

private:
	void calc_jacobian();
	void calc_jdot();
	void dump(ostream& ost);

	Joint* joint;
	Joint* link_side;  //!< null in parent side of space joint
	pLink* plink;
	pJoint* pair;
	int parent_side;  //!< link is in parent side?
	pSubChain* subchain;  //!< the subchain which only contains the link associated with this pjoint

	fMat J;
	fVec Jdot;
	fVec f_final;    // 6
	fVec acc_final;  // 6

	void calc_dvel();
	fVec dvel;
	fVec colf_final;
	fVec vel_final;
};

/*!
 * @class pLink
 * @brief Class for representing a single link in a schedule tree.
 */
class pLink
{
	friend class pSim;
	friend class pJoint;
	friend class pSubChain;
public:
	pLink(Joint* _joint) {
		joint = _joint;
		M.resize(6, 6);
		Minv.resize(6, 6);
		c.resize(6);
		acc.resize(6);
		M.zero();
		Minv.zero();
		c.zero();
		acc.zero();
		subchain = 0;
	}
	
	~pLink() {
	}

private:
	void calc_inertia();
	void calc_acc(const fVec3& g0);
	void dump(ostream& ost);
	
	Joint* joint;
	pSubChain* subchain;

	fMat M;     // 6x6
	fMat Minv;  // 6x6
	fVec c;     // 6
	fVec acc;   // 6
};

/*!
 * @class pSubChain
 * @brief Node for schedule tree; represents a partial chain.
 */
class pSubChain
{
	friend class pSim;
	friend class pJoint;
	friend class pLink;
public:
	pSubChain(pSim* _sim, pSubChain* _p, pJoint* _p0, pJoint* _p1) {
		sim = _sim;
		parent = _p;
		children[0] = 0;
		children[1] = 0;
		last_pjoints[0] = _p0;
		last_pjoints[1] = _p1;
		last_joint = last_pjoints[0]->joint;
		last_index[0] = last_index[1] = -1;
		axis = PSIM_AXIS_NULL;
		if(last_joint->j_type == JROTATE)
		{
			if(last_joint->axis(0) > 0.95) axis = PSIM_AXIS_RX;
			else if(last_joint->axis(1) > 0.95) axis = PSIM_AXIS_RY;
			else if(last_joint->axis(2) > 0.95) axis = PSIM_AXIS_RZ;
		}
		else if(last_joint->j_type == JSLIDE)
		{
			if(last_joint->axis(0) > 0.95) axis = PSIM_AXIS_X;
			else if(last_joint->axis(1) > 0.95) axis = PSIM_AXIS_Y;
			else if(last_joint->axis(2) > 0.95) axis = PSIM_AXIS_Z;
		}
		//outer_joints = 0;
		outer_joints.resize(0);
		outer_joints_origin = 0;
		outer_joints_index = 0;
		n_outer_joints = 0;
		links = 0;
		n_links = 0;
		Lambda = 0;
		acc_temp = 0;
		vel_temp = 0;
		n_dof = last_joint->n_dof;
		n_const = 6 - n_dof;
		joint_index = 0;
		const_index = 0;
		if(n_dof > 0) joint_index = new int [n_dof];
		if(n_const > 0) const_index = new int [n_const];
		int count, i;
		if(last_joint->t_given)
		{
			switch(last_joint->j_type)
			{
			case JROTATE:
			case JSLIDE:
				count = 0;
				for(i=0; i<6; i++)
				{
					if(i == (int)axis) joint_index[0] = i;
					else
					{
						const_index[count] = i;
						count++;
					}
				}
				break;
			case JSPHERE:
				const_index[0] = 0;
				const_index[1] = 1;
				const_index[2] = 2;
				joint_index[0] = 3;
				joint_index[1] = 4;
				joint_index[2] = 5;
				break;
			case JFREE:
				for(i=0; i<6; i++) joint_index[i] = i;
				break;
			case JFIXED:
				for(i=0; i<6; i++) const_index[i] = i;
				break;
			default:
				break;
			}
		}
		else
		{
			for(i=0; i<6; i++) const_index[i] = i;
		}
	}
	pSubChain(pSim* _sim, pSubChain* _p, pLink* _pl) {
		sim = _sim;
		parent = _p;
		children[0] = 0;
		children[1] = 0;
		last_pjoints[0] = 0;
		last_pjoints[1] = 0;
		last_joint = 0;
		last_index[0] = last_index[1] = -1;
		axis = PSIM_AXIS_NULL;
		outer_joints .resize(0);//= 0;
		outer_joints_origin = 0;
		outer_joints_index = 0;
		n_outer_joints = 0;
		links = new pLink* [1];
		links[0] = _pl;
		n_links = 1;
		Lambda = 0;
		acc_temp = 0;
		vel_temp = 0;
		n_dof = 6;
		n_const = 0;
		joint_index = 0;
		const_index = 0;
	}
	
	~pSubChain() {
		if(outer_joints.size())
		{
			int i;
			for(i=0; i<n_outer_joints; i++)
				delete[] Lambda[i];
			delete[] Lambda;
			delete[] acc_temp;
			delete[] vel_temp;
			//delete[] outer_joints;  
			delete[] outer_joints_origin;
			delete[] outer_joints_index;
		}
		if(joint_index) delete[] joint_index;
		if(const_index) delete[] const_index;
		if(links) delete[] links;
		if(children[0]) delete children[0];
		if(children[1] && children[1] != children[0]) delete children[1];
	}

private:
	int get_outer_index(pJoint* pj) {
		int i;
		for(i=0; i<n_outer_joints; i++)
			if(pj == outer_joints[i]) return i;
		return -1;
	}

	void init();

	int total_cost();
	int num_leaves();
	int schedule_depth();

	pSim* sim;
	pSubChain* parent;
	pSubChain* children[2];
	pJoint* last_pjoints[2];  // [0] is child side, [1] is parent side
	Joint* last_joint;
	PSIM_AXIS axis;
	int last_index[2];  // index of last_pjoints in outer_joints of children

	std::vector<pJoint*> outer_joints;
	int* outer_joints_origin;
	int* outer_joints_index;
	int n_outer_joints;
	pLink** links;
	int n_links;

	int n_dof;
	int n_const;
	int* const_index;
	int* joint_index;

	void calc_inertia();
	void calc_inertia_leaf();
	void calc_inertia_body();

	void calc_acc();
	void calc_acc_leaf();
	void calc_acc_body();

	void disassembly();
	void disassembly_body();
	void disassembly_leaf();
	
	fMat P;          // 6x6
#ifndef USE_DCA
	fMat Gamma;      // n_const x n_const
	fMat Gamma_inv;  // n_const x n_const
#endif
	fMat** Lambda;   // matrix of (n_outer_joints x n_outer_joints) matrices with size 6x6

	fVec da6;
	fMat W, IW;
	fVec* acc_temp;  // vector of (n_outer_joints) vectors with size 6x1
	fVec tau;        // n_dof x 1; joint torque
	fVec f_temp;     // n_const x 1
	fVec acc_final;  // n_dof x 1
#ifdef USE_DCA
	fMat Vhat;
	fMat SVS;
#endif

	void calc_dvel();
	void calc_dvel_leaf();
	void calc_dvel_body();
	void col_disassembly();
	void col_disassembly_body();
	fVec* vel_temp;
	fVec colf_temp;
	
	void dump(ostream& ost);

	// compute contact force based on LCP
	// only at the root of the schedule
	int calc_contact_force(double timestep);

	void clear_f_final();

};

struct JointInfo
{
	JointInfo() {
		pjoints[0] = pjoints[1] = 0;
		plink = 0;
	}
	~JointInfo() {
	}
	
	pJoint* pjoints[2];
	pLink* plink;
};

/*!
 * @class pSim
 * @brief Main class for forward dynamics computation.
 */
class pSim
	: virtual public Chain
{
	friend class pJoint;
	friend class pLink;
	friend class pSubChain;
public:
	//! Default constructor.
	/*!
	 * Default constructor.
	 * @param _rank  Rank of the process on which the instance is generated (only for parallel processing).
	 */
	pSim(): Chain() {
		joint_info = 0;
		subchains = 0;
		for(int i=0; i<N_FRIC_CONE_DIV; i++)
		{
			double ang = 2.0*i*PI/N_FRIC_CONE_DIV;
			cone_dir[i](0) = cos(ang);
			cone_dir[i](1) = sin(ang);
			cone_dir[i](2) = 0.0;
		}
	}
	~pSim() {
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
	}

	virtual void Clear();

	//! Compute joint accelerations.
	int Update();

	//! Compute joint accelerations and contact forces.
	/*!
	 * Compute joint accelerations and contact forces.  Contact forces
	 * are computed such that the relative velocity becomes zero after
	 * timestep [s].
	 * @param[in] timestep  Timestep of the integration.
	 * @param[in] col_info  Pointer to the ColInfo object containing the contact information.
	 */
	int Update(double timestep, std::vector<class SDContactPair*>& sdContactPairs);

	//! Creates default schedule, which is optimized for serial computation.
	int Schedule();

	//! Creates a schedule that assembles the chain in the specified order of joints.
	int Schedule(Joint** joints);

	//! Automatic scheduling for @c max_procs processors.
	int AutoSchedule(int max_procs);


	//! Dump the schedule information to @c ost.
	void DumpSchedule(ostream& ost);

	//! Approximate indicators of total computational cost.
	int TotalCost();
	int NumLeaves();
	int ScheduleDepth();

	//! Extract the constraint forces.
	int ConstraintForces(fVec& cf);

	void GetPJoint(Joint* _joint, pJoint* _pjoints[2]) {
		_pjoints[0] = joint_info[_joint->i_joint].pjoints[0];
		_pjoints[1] = joint_info[_joint->i_joint].pjoints[1];
	}

#ifdef SEGA
	virtual int init();
#else
	virtual int init(SceneGraph* sg);
#endif
	virtual int clear_contact();
	int init_contact();
protected:
	int myinit();
	virtual int clear_data();

private:
	pSubChain* default_schedule(pSubChain* p, Joint* j);
	void default_schedule_virtual(Joint* j);
	void setup_pjoint(Joint* j);
	void setup_pjoint_virtual(Joint* j);
	void calc_consts();

	void update_position();
	void update_velocity();
	void disassembly();

	// collision
	void update_collision();
	void calc_dvel();
	void col_disassembly();

	// contact force computation based on LCP
	joint_list contact_vjoints;
	std::vector<fVec3> contact_relvels;
	std::vector<double> fric_coefs;

	joint_list all_vjoints;
	std::vector<fMat> all_Jv;
	std::vector<fMat> all_Jr;
	std::vector<fVec3> all_jdot_v;
	std::vector<fVec3> all_jdot_r;

	fVec3 cone_dir[N_FRIC_CONE_DIV];

	int contact_vjoint_index(Joint* _jnt) {
		int count = 0;
		for(joint_list::iterator j=contact_vjoints.begin(); j!=contact_vjoints.end(); count++, j++)
		{
			if(_jnt == *j) return count;
		}
		return -1;
	}

	int build_subchain_tree(int _n_joints, Joint** joints, subchain_list& buf);
	void build_subchain_tree(Joint* cur_joint, subchain_list& buf);
	int in_subchain(pSubChain* sc, pLink* pl);

	JointInfo* joint_info;
	pSubChain* subchains;


};

#endif

