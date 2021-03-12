
#include "stdafx.h"
#include "BoneKinematics.h"
#include "MotionLoader.h"

BoneVelocityForwardKinematics::BoneVelocityForwardKinematics(const MotionLoader* loader):_loader(loader){
	int nb=loader->numBone();
	array_loc_lin_vel.setSize(nb);
	array_loc_ang_vel.setSize(nb);
	array_rel_lin_vel.setSize(nb);
	array_rel_ang_vel.setSize(nb);
}

void BoneVelocityForwardKinematics::setBodyVel(int ibone, vector3 const& lin_vel, vector3 const& ang_vel)
{
	array_loc_lin_vel(ibone)=lin_vel;
	array_loc_ang_vel(ibone)=ang_vel;
}
void BoneVelocityForwardKinematics::getJointVel(int ibone, vector3 & lin_vel, vector3 & ang_vel)
{
	lin_vel=array_rel_lin_vel(ibone);
	ang_vel=array_rel_ang_vel(ibone);
}

void BoneVelocityForwardKinematics::computeDS(BoneForwardKinematics const& fk)
{
	int nb=_loader->numBone();
	for(int i=1; i<nb; i++){
		const auto& _local=fk.local(i);
		auto* parent=_loader->bone(i).parent();
		auto& rel_lin_vel=array_rel_lin_vel(i);
		auto& rel_ang_vel=array_rel_ang_vel(i);
		auto& loc_lin_vel=array_loc_lin_vel(i);
		auto& loc_ang_vel=array_loc_ang_vel(i);
		if(i!=1)
		{
			auto& parent_loc_lin_vel=array_loc_lin_vel(parent->treeIndex());
			auto& parent_loc_ang_vel=array_loc_ang_vel(parent->treeIndex());
			quater t_rel_att;
			t_rel_att.inverse(_local.rotation);
			vector3 v1, v2;
			// compute loc_lin_vel
			v1.cross(parent_loc_ang_vel, _local.translation);
			v2.rotate(t_rel_att, parent_loc_lin_vel);
			loc_lin_vel.rotate(t_rel_att, v1);
			loc_lin_vel += v2;
			loc_lin_vel += rel_lin_vel;
			// compute loc_ang_vel
			loc_ang_vel.rotate(t_rel_att, parent_loc_ang_vel);
			loc_ang_vel += rel_ang_vel;
		}
		else
		{
			loc_ang_vel=rel_ang_vel;
			loc_lin_vel=rel_lin_vel;
		}
	}
}


void BoneVelocityForwardKinematics::computeDQfromDS(BoneForwardKinematics const& fk)
{
	int nb=_loader->numBone();
	for(int i=1; i<nb; i++){
		const auto& _local=fk.local(i);
		auto* parent=_loader->bone(i).parent();
		auto& rel_lin_vel=array_rel_lin_vel(i);
		auto& rel_ang_vel=array_rel_ang_vel(i);
		auto& loc_lin_vel=array_loc_lin_vel(i);
		auto& loc_ang_vel=array_loc_ang_vel(i);
		if(i!=1)
		{
			const auto& parent_loc_lin_vel=array_loc_lin_vel(parent->treeIndex());
			const auto& parent_loc_ang_vel=array_loc_ang_vel(parent->treeIndex());

			quater t_rel_att;
			t_rel_att.inverse(_local.rotation);
			vector3 v1, v2;
			// compute loc_lin_vel
			v1.cross(parent_loc_ang_vel, _local.translation);
			v2.rotate(t_rel_att, parent_loc_lin_vel);

			//loc_lin_vel= A+v2+rel_lin_vel
			//-> rel_lin_vel=-A-v2+loc_linvel
			rel_lin_vel.rotate(t_rel_att, v1);
			rel_lin_vel *=-1; // -A
			rel_lin_vel -= v2;
			rel_lin_vel += loc_lin_vel;

			// loc_ang_vel= B+rel_ang_vel
			// -> rel_ang_vel= -B+loc_ang_vel
			rel_ang_vel.rotate(t_rel_att, parent_loc_ang_vel);
			rel_ang_vel *=-1; // -B
			rel_ang_vel += loc_ang_vel ;
			//printf("%s %s\n", loc_ang_vel.output().ptr(), rel_ang_vel.output().ptr());
		}
		else
		{
			rel_ang_vel=loc_ang_vel;
			rel_lin_vel=loc_lin_vel;
		}
	}
}
