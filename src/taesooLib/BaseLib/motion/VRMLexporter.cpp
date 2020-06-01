#include "stdafx.h"
#include "Motion.h"
#include "MotionLoader.h"
#include "VRMLexporter.h"
#include "../BaseLib/utility/operatorString.h"

static TString space(int level)
{
	static TString temp;
	temp.empty();
	temp.add("\n");
	for(int i=0; i<level; i++)
		temp.add("    ");
	return temp;
}


static TString nameId(Bone& bone)
{
	static int curSite=0;
	TString out(bone.NameId);

	if(TString("Site")==bone.NameId)
	{
		out.add("%d", curSite++);
	}
	return out;
}

void packShape(Bone& bone,FILE* file, int level, MotionLoader* pLoader)
{
	TString shape;
	shape.add("Shape {\nappearance Appearance {\nmaterial Material {\ndiffuseColor 0 0.3529 1\nambientIntensity 0.2523\n");
	shape.add( "specularColor 0.7829 1.075 1.24\nshininess 0.544\ntransparency 0.2\nemissiveColor 0 0.3529 1}}\n");
	shape.add( "geometry DEF %s-FACES IndexedFaceSet {\n", nameId(bone).ptr());
	shape.add( "  ccw TRUE\n  solid FALSE\n  coord DEF %s-COORD Coordinate {\n  point [", nameId(bone).ptr());

	vector3 offset;
	bone.getOffset(offset);

	m_real width=0.02;
	vector3 v1(0,0,0);
	vector3 v3(offset);
	vector3 v2;
	v2=(v1+v3)*0.5;

	shape.add( " %f %f %f,\n", v1.x, v1.y, v1.z);
	shape.add( " %f %f %f,\n",v2.x+width, v2.y, v2.z+width);
	shape.add( " %f %f %f,\n",v2.x+width,v2.y, v2.z-width);
	shape.add( " %f %f %f,\n",v2.x-width,v2.y, v2.z+width);
	shape.add( " %f %f %f,\n",v2.x-width,v2.y, v2.z-width);
	shape.add( " %f %f %f]\n", v3.x,v3.y, v3.z);
	shape.add( "}\n");
	shape.add( "  coordIndex [0, 1, 2, -1,0,3,1,-1,0, 4, 3, -1,0, 2, 4,-1,1,3,5,-1,2, 1, 5,-1,3,4,5,-1,4,2,5,-1]\n");
	shape.add( "  }\n}\n");

	shape.op0(sz0::replace("\n", space(level)));
	fprintf(file, "%s", shape.ptr());
}

void packAnimation(Bone& bone, FILE* file, int level, Motion const& mot, int start, int end )
{
	TString anim;
	
	vectorn keys;
	keys.linspace(0, 1, end-start);
	
	if(level==1)
	{
		anim.add("DEF TIMER TimeSensor { loop TRUE cycleInterval %f },\n", mot.frameTime()*(end-start));

		// root joint
		anim.add("DEF %s-POS-INTERP PositionInterpolator {\n key [", bone.NameId);
		for(int i=start; i<end; i++)
			anim.add("%f,", keys[i-start]);

		anim.add("]\nkeyValue [");
		
		for(int i=start; i<end; i++)
			anim.add("%f %f %f, \n", mot.pose(i).m_aTranslations[0].x, mot.pose(i).m_aTranslations[0].y, mot.pose(i).m_aTranslations[0].z);

		anim.add("] },\n");
	}

	for(int i=0; i<mot.NumJoints(); i++)
	{
		if(&bone==&mot.skeleton().getBoneByRotJointIndex(i))
		{
			// joint i
			anim.add("DEF %s-ROT-INTERP OrientationInterpolator {\n key [", bone.NameId);
			for(int j=start; j<end; j++)
				anim.add("%f,", keys[j-start]);

			anim.add("]\nkeyValue [");

			for(int iframe=start; iframe<end; iframe++)
			{
				quater q;
				q=mot.pose(iframe).m_aRotations[i];
				vector3 v=q.rotationVector();
				m_real angle=v.length();
				v/=angle;

				anim.add("%f %f %f %f, \n", v.x, v.y, v.z, angle);
			}

			anim.add("] },\n");

			break;
		}
	}		

	anim.op0(sz0::replace("\n", space(level)));
	if (anim.length()>0)
		fprintf(file, "%s", anim.ptr());
}

void packTransform(Bone& bone, FILE* file, int level, Motion const& mot, int start, int end )
{
	TString transform;
	transform.add("DEF %s Transform {\n", nameId(bone).ptr());
	vector3 offset;
	bone.getOffset(offset);
	transform.add("  translation %f %f %f\n", offset.x, offset.y, offset.z);

	transform.add("  children [\n");
	transform.op0(sz0::replace("\n", space(level)));
	fprintf(file, "%s", transform.ptr());
	transform.empty();

	if(bone.m_pChildHead)
	{
		//if(level==0)
		packAnimation(bone, file, level+1, mot, start, end);

		for(Bone* node=(Bone*)bone.m_pChildHead; node!=NULL; node=(Bone*)(node->m_pSibling))
		{
			packShape(*node, file, level+1, &mot.skeleton());

			if(node->m_pChildHead)
				packTransform(*node, file, level+1, mot, start, end );
		}
	}

	transform.add("]\n");
	if(level==0)
	{
		for(int i=0; i<mot.NumJoints(); i++)
		{
			char* name=mot.skeleton().getBoneByRotJointIndex(i).NameId;
			if(i==0)
			{				
				transform.add("ROUTE TIMER.fraction_changed TO %s-POS-INTERP.set_fraction\n", name);
				transform.add("ROUTE %s-POS-INTERP.value_changed TO %s.set_translation\n", name,name);				
			}
			transform.add("ROUTE TIMER.fraction_changed TO %s-ROT-INTERP.set_fraction\n", name);
			transform.add("ROUTE %s-ROT-INTERP.value_changed TO %s.set_rotation\n", name,name);

		}
	}
	transform.add("}\n");
	transform.op0(sz0::replace("\n", space(level)));
	fprintf(file, "%s", transform.ptr());
}
void MotionUtil::exportVRML(Motion const& mot, const char* filename, int start, int end)
{
	FILE* file=fopen(filename, "wt");

	if(end>mot.numFrames())
		end=mot.numFrames();

	fprintf(file, "#VRML V2.0 utf8\n\n# Produced by 3D Studio MAX VRML97 exporter, Version 3.01, Revision 0\n# MAX File: t.max, Date: Fri Aug 25 16:57:48 2000\n");
	packTransform(mot.skeleton().getBoneByRotJointIndex(0), file, 0, mot, start, end);
	
	fclose(file);
}

#include "../math/operatorTemplate.hpp"
static double cylinderRadious=0.025;
void packShapeRobot(Bone& bone,FILE* file, int level, const MotionLoader* pLoader)
{
	vector3 offset;
	vector3N child_offsets;
	for(Bone* c=bone.child(); c; c=c->sibling())
	{
		c->getOffset(offset);
		child_offsets.pushBack(offset);
	}

	vector3 com(0,0,0);
	com=v::for_each(child_offsets, s0::avg<vector3>(vector3(0,0,0))).result();
	com/=2.0;

	m_real mass=0.01;	// actually just the sum of child-link-lengths.
	for(int i=0; i<child_offsets.size(); i++)
		mass+=child_offsets[i].length();

	TString shape;

	m_real radius=0.01; // for setting inertia
	if(child_offsets.size())
		radius=mass/child_offsets.size()/2.0;
	m_real inertia=1.0;
	
	inertia*=mass*2.0/5.0*SQR(radius);	// spherical inertia.

	shape.add("Segment { \n centerOfMass %f %f %f\n mass %f momentsOfInertia [%f 0 0 0 %f 0 0 0 %f]\n", com.x, com.y, com.z, mass, inertia, inertia, inertia);
	if(child_offsets.size())
	{
		shape.add("children [\n");

		for(int i=0; i<child_offsets.size();i++)
		{
			vector3 offset=child_offsets[i];
			quater q;
			vector3 axis;
			m_real angle;
			q.axisToAxis(vector3(0,1,0), offset.dir());
			q.toAxisAngle(axis, angle);
			shape.add("Transform { rotation %f %f %f %f translation %f %f %f\n",
					axis.x, axis.y, axis.z, angle, offset.x/2.0, offset.y/2.0, offset.z/2.0);

			shape.add( "children Shape { geometry Capsule { radius %f height %f }}}\n", cylinderRadious, offset.length());
		}

		shape.add("]\n}\n");
	}
	else
		shape.add("}\n");
	shape.op0(sz0::replace("\n", space(level)));
	fprintf(file, "%s", shape.ptr());
}

static void packTransformRobot(Bone& bone, FILE* file, int level, MotionLoader const& skel)
{
	TString transform;
	transform.add("DEF %s Joint {\n", nameId(bone).ptr());

	if(level==0)
	{
		transform.add("jointType \"free\"\n");
	}
	else
	{
		if (bone.getRotationalChannels().length()==0)
		{
			transform.add("jointType \"slide\"\n");
			transform.add("jointAxis \"XYZ\"\n");
		}
		else
		{
			transform.add("jointType \"rotate\"\n");
			transform.add("jointAxis \"ZXY\"\n");
		}
	}
	vector3 offset;
	bone.getOffset(offset);
	transform.add("  translation %f %f %f\n", offset.x, offset.y, offset.z);

	transform.add("  children [\n");
	transform.op0(sz0::replace("\n", space(level)));
	fprintf(file, "%s", transform.ptr());
	transform.empty();

	packShapeRobot(bone, file, level+1, &skel);

	if(bone.m_pChildHead)
	{
		//if(level==0)
		//packAnimation(bone, file, level+1, mot, start, end);
		

		for(Bone* node=(Bone*)bone.m_pChildHead; node!=NULL; node=(Bone*)(node->m_pSibling))
		{


			if(node->m_pChildHead || node->getRotationalChannels().length()>0 )
				packTransformRobot(*node, file, level+1, skel);
		}
	}

	transform.add("]\n");
	transform.add("}");
	transform.op0(sz0::replace("\n", space(level)));
	fprintf(file, "%s", transform.ptr());
}

void MotionUtil::exportVRMLforRobotSimulation(Motion const& mot, const char* filename, const char* robotname, double cylinder_radius)
{
	if(&mot.skeleton()==NULL)
	 Msg::error("exportVRMLforRobotSimulation: mot is not valid");
	exportVRMLforRobotSimulation(mot.skeleton(),  filename, robotname, cylinder_radius);

}
void MotionUtil::exportVRMLforRobotSimulation(MotionLoader const& skel, const char* filename, const char* robotname, double cylinder_radius)
{
	cylinderRadious=cylinder_radius;
	FILE* file=fopen(filename, "wt");
	if(!file) Msg::error("can't open %s for writing", filename);
	fprintf(file, "DEF SampleRobot Humanoid { \n name \"%s\"\n", robotname);
	fprintf(file, "humanoidBody [\n");
	packTransformRobot(skel.getBoneByRotJointIndex(0), file,0, skel);
	fprintf(file, "]\n}\n");
	fclose(file);
}
