#include "stdafx.h"
#include "mathclass.h"
#include "transf.h"


void transf::identity( )
{
	rotation.identity();
	translation.setValue(0,0,0);
}



transf
transf::inverse() const
{
    quater a = rotation.inverse();
	vector3 ti;
	ti.rotate(a, translation);
    return transf( a, -ti);
}

void transf::interpolate( m_real t, transf const& a, transf const& b )
{
	rotation.interpolate(t,a.rotation,b.rotation);
	translation.interpolate(t,a.translation,b.translation);
}

void transf::scale(double v)
{
	translation*=v;
	rotation.align(quater(1,0,0,0));
	rotation.scale(v);
}
#include "dualQuaternion.h"
void transf::slerp(double t, transf const& a, transf const& b)
{
	dualQuaternion da(a.translation, a.rotation);
	dualQuaternion db(b.translation, b.rotation);
	dualQuaternion c;
	c=dualQuaternion::sScLERP(t, da, db);
	rotation=c.getRotation();
	translation=c.getTranslationFromUnit();
}
/*
transf rotate_transf( m_real angle, vector const& axis )
{
    return transf( exp( angle * axis / 2.0 ), vector(0,0,0) );
}

transf translate_transf( vector const& axis )
{
    return transf( quater(1,0,0,0), axis );
}

transf translate_transf( m_real x, m_real y, m_real z )
{
    return transf( quater(1,0,0,0), vector(x,y,z) );
}
*/


//--------------------------------------------------------------------------//

vector3& operator*=( vector3& a, transf const& b )
{
	a.rotate(b.rotation);
	a+=b.translation;
	return a;
}

vector3 operator*( transf const& b , vector3 const& a)
{
	vector3 c;
	c.rotate(b.rotation, a);
	c+=b.translation;
	return c;
}

transf operator*( transf const& a, transf const& b )
{
	transf c;
	c.mult(a,b);
	return c;
}

void transf::operator=(matrix4 const& a)
{
	rotation.setRotation(a); 
	translation.translation(a);
}
void transf::operator=(transf const& a)
{
	rotation=a.rotation;
	translation=a.translation;
}

void transf::mult(transf const& a, transf const& b )
{
	rotation.mult(a.rotation, b.rotation);
	translation.rotate(a.rotation, b.translation);
	translation+=a.translation;
}
// this=a*this;
void transf::leftMult(const transf& a)
{
	translation.rotate(a.rotation);
	translation+=a.translation;
	rotation=a.rotation*rotation;
}

void transf::operator*=(const transf& b)
{
	transf a=*this;
	this->mult(a,b);
}


void transf::setCoordinate(vector3 const& orig, vector3 const& front) 
{
	origin()=orig;
	orientation().setAxisRotation(vector3(0,1,0) , vector3(0,0,1), front);
	orientation().align(quater(1,0,0,0));
}

transf transf::toLocal(transf const& global) const
{
	transf local;
	local.mult(inverse(), global);
	return local;
}

quater transf::toLocalRot(quater const& global) const
{
	quater local;
	local.mult(orientation().inverse(), global);
	return local;
}


transf transf::toGlobal(transf const& local) const
{
	transf global;
	global.mult(*this, local);
	return global;
}

quater transf::toGlobalRot(quater const& local) const
{
	return orientation()*local;
}

quater transf::toLocalDRot(quater const& drot) const
{
	// drot=ori2*ori1.inv;
	// localDrot= ori2 * mOri.inv * (ori1* mOri.inv ).inv
	//			= ori2 * mOri.inv * mOri *ori1.inv 
	//			= ori2* ori1.inv =drot

	return drot;
}

quater transf::toGlobalDRot(quater const& ori) const
{
	return ori;
}

vector3 transf::toLocalPos(vector3 const& pos) const
{
	vector3 lpos;
	lpos.rotate(orientation().inverse(), pos-origin());
	return lpos;
}

vector3 transf::toGlobalPos(vector3 const& lpos) const
{
	vector3 gpos;
	gpos.rotate(orientation(), lpos);
	return gpos+origin();
}

vector3 transf::toLocalDir(vector3 const& dir) const
{
	vector3 gdir;
	gdir.rotate( orientation().inverse(), dir);
	return gdir;
}

vector3 transf::toGlobalDir(vector3 const& dir) const
{
	vector3 ldir;
	ldir.rotate( orientation(), dir);
	return ldir;
}

quater dep_toLocalRot_old(transf const& f, quater const& ori)
{
	quater local;
	local.difference(f.orientation(), ori);
	return local;
}
quater dep_toGlobalRot_old(transf const& f, quater const& ori)
{
	return ori*f.orientation();
}

void transf::difference(transf const& q1, transf const& q2)
{
	mult(q2, q1.inverse());    
}

vector3 transf::encode2D() const
{
	vector3 start_transf;
	start_transf.x=translation.x;
	start_transf.z=translation.z;
	quater rot_y, offset;

	double len=rotation.length();
	if(len<0.95 || len>1.05)
	{
		printf("warning! normalizing\n");
		quater q;
		q.normalize(rotation);
		q.decompose(rot_y, offset);
	}
	else
		rotation.decompose(rot_y, offset);

	start_transf.y=rot_y.rotationAngle(vector3(0,1,0));

	return start_transf;
}

void transf::align2D(transf const& other)
{
	quater rot_y, offset;
	rotation.decompose(rot_y, offset);

	vector3 o=other.encode2D();
	translation.x=o.x;
	translation.z=o.z;
	rot_y.setRotation(vector3(0,1,0), o.y);

	rotation=rot_y*offset;
}

void transf::decode2D(vector3 const& in)
{
	translation.x=in.x;
	translation.y=0;
	translation.z=in.z;

	rotation.setRotation(vector3(0,1,0), in.y);
}
std::ostream &operator << (std::ostream &os, const transf &v)
{
	os << "[ r=" << v.rotation<<std::endl << ", t=" << v.translation << "]\n";
	return os;
}

#include "../motion/IK_sdls/Node.h"
void transf::integrateBodyVel(vector3 const& rel_ang_vel, vector3 const& rel_lin_vel, double dt)
{
	quater p_ep_dot; 
	vector3 p_lin_vel;
	p_lin_vel.rotate(rotation, rel_lin_vel*dt);
	IK_sdls::angvel2qdot(p_ep_dot, rotation, rel_ang_vel*dt);
	translation+=p_lin_vel;
	rotation+=p_ep_dot;
	rotation.normalize();
}
