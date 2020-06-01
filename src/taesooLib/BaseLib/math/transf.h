
#ifndef transf_H
#define transf_H

// rigid transformation. matrix4의 subset이다.
class transf
{
  public:
	quater rotation;		
	vector3 translation;		
	
	// unary operations
	void leftMult(const transf& a);	//!< this=a*this;
	void operator*=(const transf& a);	//!< this=this*a;
	friend vector3&      operator*=( vector3& v, transf const& f);	//!< v=f*v

    // binary operations
    void interpolate( m_real, transf const&, transf const& );
	void mult(transf const&, transf const&);
    friend transf       operator* ( transf const&, transf const& );
    friend vector3       operator* ( transf const& , vector3 const&);

    // constructors
    transf() {};
    transf( quater const& a, vector3 const& b ) { rotation=a; translation=b; }
    transf( vector3 const& b ) { rotation.identity(); translation=b; }
    transf( quater const& a ) { rotation=a; translation.zero();}
	transf(matrix4 const& a)	{ rotation.setRotation(a); translation.translation(a);}
    transf			inverse() const;

	void difference(transf const& f1, transf const& f2);			//!< f2*f1.inv
	void identity();
	void scale(double f); // simple component-wise scale
	// Do screw linear interpolation (the slerp for dual quaternions)
	void slerp(double t, transf const& a, transf const& b);

	void operator=(matrix4 const& a);

	vector3 encode2D() const;
	void decode2D(vector3 const& in);
	void align2D(transf const& other);

	// coordinate frame
	quater& orientation()				{ return rotation;}
	vector3& origin()					{ return translation;}
	const quater& orientation()	const	{ return rotation;}
	const vector3& origin()	const 		{ return translation;}

	void setCoordinate(vector3 const& origin, vector3 const& front);
	void integrateBodyVel(vector3 const& rel_ang_vel, vector3 const& rel_lin_vel, double dt);

	transf toLocal(transf const& global) const;
	transf toGlobal(transf const& local) const;

	quater toLocalRot(quater const& ori) const;
	quater toGlobalRot(quater const& ori) const;
	quater toLocalDRot(quater const& ori) const;
	quater toGlobalDRot(quater const& ori) const;

	vector3 toLocalPos(vector3 const& pos) const;
	vector3 toGlobalPos(vector3 const& pos) const;

	vector3 toLocalDir(vector3 const& dir) const;
	vector3 toGlobalDir(vector3 const& dir) const;

	friend std::ostream &operator << (std::ostream &os, const transf &s);
};

// deprecated. local을 정의하는 convention이 바뀌었음.
quater dep_toLocalRot_old(transf const& f, quater const& ori);
quater dep_toGlobalRot_old(transf const& f, quater const& ori);
#endif
