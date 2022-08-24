#include "stdafx.h"
#include "intersectionTest.h"
#include "../../BaseLib/math/Operator.h"
#include "../../BaseLib/math/Operator_NR.h"

Box2D::Box2D()
{
	min(0)=0.0;
	min(1)=0.0;
	max(0)=0.0;
	max(1)=0.0;
}
Box2D::Box2D(vector2 const& m, vector2 const& M)
	:min(m), max(M)
{
}

bool Box2D::contains(vector2 const& pt, double margin)	const	
{ 
	double left=min(0);
	double right=max(0);
	double top=min(1);
	double bottom=max(1);
	if(pt.x()>=left -margin && pt.y()>=top -margin && pt.x()<right +margin && pt.y()<bottom+margin) return true; 
	return false;
}

inline double __max_hyu(double a, double b, double c)
{
	if(a>b)
	{
		if(a>c)
			return a;
		return c;	
	}
	else
	{
		if(b>c)
			return b;
		return c;
	}
}
double Box2D::distance(vector2 const& p)
{
	Box2D& rect=*this;
	double dx = __max_hyu(rect.min.x() - p.x(), 0.0, p.x() - rect.max.x());
	double dy = __max_hyu(rect.min.y() - p.y(), 0.0, p.y() - rect.max.y());
	return sqrt(dx*dx + dy*dy);
}
double Box2D::negativeDistance(vector2 const& p)
{
	Box2D& rect=*this;
	double dx = __max_hyu(rect.min.x() - p.x(), 0.0, p.x() - rect.max.x());
	double dy = __max_hyu(rect.min.y() - p.y(), 0.0, p.y() - rect.max.y());
	if(dx==0.0)
	{
	}
	assert(false);
	return 0;
}

Plane::Plane ()
:normal(0,0,0),
d(0.0)
{

}

Plane::Plane (double nx, double ny, double nz, double _d)
	:normal(nx,ny,nz),d(_d)
{
}

Plane::Plane (const vector3& vNormal, m_real offset)
{
	normal = vNormal;
	d = -offset;
}

Plane::Plane (const vector3& vNormal, const vector3& vPoint)
{
	setPlane(vNormal, vPoint);
}

Plane::Plane (const vector3& vPoint0, const vector3& vPoint1,
	const vector3& vPoint2)
{
	setPlane(vPoint0, vPoint1, vPoint2);
}

m_real Plane::distance(const vector3& vPoint) const
{
	return normal%vPoint + d;
}

void Plane::setPlane(const vector3& vPoint0, const vector3& vPoint1,
	const vector3& vPoint2)
{
	vector3 kEdge1 = vPoint1 - vPoint0;
	vector3 kEdge2 = vPoint2 - vPoint0;
	normal.cross(kEdge1, kEdge2);
	normal.normalize();
	d = -normal%vPoint0;
}

void Plane::setPlane(const vector3& vNormal, const vector3& vPoint)
{
	normal = vNormal;
	normal .normalize();
	d = -vNormal%vPoint;
}

bool Sphere::isInside(std::vector<Plane>& vol) const
{
	if(vol.size())
	{
		bool bInside=true;
		for(int j=0; j<vol.size(); j++)
		{
			//                  dist= -1
			// --------------   dist=0
			//         |        dist=1
			//  inside |        dist=2
			if(vol[j].distance(center)<radius)
			{
				bInside=false; break;
			}
		}

		if(bInside)	return true;
	}
	return false;
}

std::pair<bool, m_real> Ray::intersects(const vector3& a,
    const vector3& b, const vector3& c, bool bCCW) const
{
	vector3 normal;
	normal.cross(b-a, c-a);
	normal.normalize();

	if(!bCCW) normal*=-1;

	return Ray::intersects(a,b,c,normal, true, false);
}


std::pair<bool, m_real> Ray::intersects(const vector3& a,
    const vector3& b, const vector3& c, const vector3& normal,
    bool positiveSide, bool negativeSide) const
{
	const Ray& ray=*this;
    //
    // Calculate intersection with plane.
    //
    m_real t;
    {
        m_real denom = normal%ray.direction();

        // Check intersect side
        if (denom > + std::numeric_limits<m_real>::epsilon())
        {
            if (!negativeSide)
                return std::pair<bool, m_real>(false, 0);
        }
        else if (denom < - std::numeric_limits<m_real>::epsilon())
        {
            if (!positiveSide)
                return std::pair<bool, m_real>(false, 0);
        }
        else
        {
            // Parallel or triangle area is close to zero when
            // the plane normal not normalised.
            return std::pair<bool, m_real>(false, 0);
        }

        t = normal%(a - ray.origin()) / denom;

        if (t < 0)
        {
            // Intersection is behind origin
            return std::pair<bool, m_real>(false, 0);
        }
    }

    //
    // Calculate the largest area projection plane in X, Y or Z.
    //
    int i0, i1;
    {
        m_real n0 = ABS(normal[0]);
        m_real n1 = ABS(normal[1]);
        m_real n2 = ABS(normal[2]);

        i0 = 1; i1 = 2;
        if (n1 > n2)
        {
            if (n1 > n0) i0 = 0;
        }
        else
        {
            if (n2 > n0) i1 = 0;
        }
    }

    //
    // Check the intersection point is inside the triangle.
    //
    {
        m_real u1 = b[i0] - a[i0];
        m_real v1 = b[i1] - a[i1];
        m_real u2 = c[i0] - a[i0];
        m_real v2 = c[i1] - a[i1];
        m_real u0 = t * ray.direction()[i0] + ray.origin()[i0] - a[i0];
        m_real v0 = t * ray.direction()[i1] + ray.origin()[i1] - a[i1];

        m_real alpha = u0 * v2 - u2 * v0;
        m_real beta  = u1 * v0 - u0 * v1;
        m_real area  = u1 * v2 - u2 * v1;

        // epsilon to avoid float precision error
        const m_real EPSILON = 1e-3f;

        m_real tolerance = - EPSILON * area;

        if (area > 0)
        {
            if (alpha < tolerance || beta < tolerance || alpha+beta > area-tolerance)
                return std::pair<bool, m_real>(false, 0);
        }
        else
        {
            if (alpha > tolerance || beta > tolerance || alpha+beta < area-tolerance)
                return std::pair<bool, m_real>(false, 0);
        }
    }

    return std::pair<bool, m_real>(true, t);
}


std::pair<bool, m_real> Ray::intersects(const Plane& plane) const
{
	Ray const& ray=*this;

	m_real denom = plane.normal%ray.direction();

	if (ABS(denom) < std::numeric_limits<m_real>::epsilon())
    {
        // Parallel
        return std::pair<bool, m_real>(false, 0);
    }
    else
    {
		m_real nom = plane.normal%ray.origin() + plane.d;
        m_real t = -(nom/denom);
        return std::pair<bool, m_real>(t >= 0, t);
    }
}

std::pair<bool, double> Ray::intersects(const std::vector<Plane>& planes) const
{
	const Ray& ray=*this;

    std::vector<Plane>::const_iterator planeit, planeitend;
    planeitend = planes.end();
    std::pair<bool, double> ret;
    ret.first = false;
    ret.second = -1e9;
	interval overlap(-1e9, 1e9);

    for (planeit = planes.begin(); planeit != planeitend; ++planeit)
    {
        const Plane& plane = *planeit;
        {
            // Test single plane
            std::pair<bool, double> planeRes =
                ray.intersects(plane);
            if (planeRes.first)
            {
				// is origin outside?
				if (plane.distance(ray.origin())>0)
					overlap&=interval(planeRes.second,1e9);
				else
					overlap&=interval(-1e9, planeRes.second);

            }
			else
				if (plane.distance(ray.origin())>0)
				{
					ret.first=false;
					ret.second=0.0;
					return ret;
				}

        }
    }

	//printf(">> %f %f:", overlap.start_pt(), overlap.end_pt());
	if (overlap.len()>0)
    {
        ret.first = true;
        ret.second = overlap.start_pt();
    }
	else
	{
		ret.first=false;
	}

    return ret;
}

std::pair<bool, m_real> Ray::intersects(const Sphere& sphere) const
{
	const Ray& ray=*this;
	const vector3& raydir = ray.direction();
    // Adjust ray origin relative to sphere center
    const vector3& rayorig = ray.origin() - sphere.center;
    m_real radius = sphere.radius;

#ifdef DISCARD_INSIDE
    // Check origin inside first
    if (rayorig%rayorig <= radius*radius )
    {
        return std::pair<bool, m_real>(true, 0);
    }
#endif

    // Mmm, quadratics
    // Build coeffs which can be used with std quadratic solver
    // ie t = (-b +/- sqrt(b*b + 4ac)) / 2a
    m_real a = raydir%raydir;
    m_real b = 2 * rayorig%raydir;
    m_real c = rayorig%rayorig - radius*radius;

    // Calc determinant
    m_real d = (b*b) - (4 * a * c);
    if (d < 0)
    {
        // No intersection
        return std::pair<bool, m_real>(false, 0);
    }
    else
    {
        // BTW, if d=0 there is one intersection, if d > 0 there are 2
        // But we only want the closest one, so that's ok, just use the
        // '-' version of the solver
        m_real t = ( -b - sqrt(d) ) / (2 * a);
        if (t < 0)
            t = ( -b + sqrt(d) ) / (2 * a);
        return std::pair<bool, m_real>(true, t);
    }
}

int Ray::pickBarycentric(const OBJloader::Mesh& mesh, vector3 & baryCoeffs, vector3 & pickPos)
{
	auto& ray=*this;
	double rayParam=DBL_MAX;
	int argMin=-1;
	vector3 normal;
	for(int i=0; i<mesh.numFace(); i++)
	{
		OBJloader::Face const& f=mesh.getFace(i);
		vector3 a=mesh.getVertex(f.vi(0));
		vector3 b=mesh.getVertex(f.vi(1));
		vector3 c=mesh.getVertex(f.vi(2));
		normal=mesh.calcFaceNormal(i);

		auto res=ray.intersects(a,b,c,normal,true,false);
		if(res.first) 
		{
			if(res.second<rayParam)
			{
				rayParam=res.second;
				argMin=i;
			}
		}
	}
	if (rayParam!=DBL_MAX)
	{
		pickPos=ray.getPoint(rayParam);

		OBJloader::Face const& f=mesh.getFace(argMin);
		// getBarycentric
		{
			auto& p=pickPos;
			vector3 v1=mesh.getVertex(f.vi(0));
			vector3 v2=mesh.getVertex(f.vi(1));
			vector3 v3=mesh.getVertex(f.vi(2));
			vector3 v3v1 = v3 - v1;
			vector3 v2v1 = v2 - v1;
			vector3 pv1 = p - v1;

			double v123Area = v2v1.cross( v3v1).length();
			double v12PArea = pv1.cross( v2v1).length();
			double v13PArea = pv1.cross( v3v1).length();

			double v3Coeff = v12PArea / v123Area;
			double v2Coeff = v13PArea / v123Area;
			double v1Coeff = 1.0 - v2Coeff - v3Coeff;

			baryCoeffs[0] = v1Coeff;
			baryCoeffs[1] = v2Coeff;
			baryCoeffs[2] = v3Coeff;
		}
		return argMin;
	}
	return -1;
}
namespace intersectionTest
{
void LineSegment::resetPosition(const vector3& from, const vector3& to)
{
	mOrigin=from;
	mDirection.difference(from, to);
	mLength=mDirection.length();

	if (mLength<std::numeric_limits<m_real>::epsilon())
	{
		mDirection=vector3(1,0,0);
		mLength=mLength<std::numeric_limits<m_real>::epsilon();
	}
	else
		mDirection/=mLength;
}

m_real LineSegment::minDistTime(vector3 const& pos) const
{
	// line          x
	// --------------------------
	//               |
	//              pos

	vector3 const& d=dir();
	vector3 const& o=origin();

	// x=o+a*d

	// condition:
	// (x-pos).d=0
	// (o+a*d-pos).d=0

	// (ox+a*dx-px)*dx+(oy+a*dy-py)*dy+(oz+a*dz-pz)*dz=0

	// a*(d.d)=pos.d-o.d

	return (pos%d-o%d)/mLength;
}
// 가장 가까운 두 점 사이의 거리.
m_real LineSegment::minDist(LineSegment const& other) const
{
	//  line 1     x1
	// .......................
	//             |
	// ----------------------
	//             x2        line 2

	vector3 const& d1=dir();
	vector3 const& o1=origin();
	vector3 const& d2=other.dir();
	vector3 const& o2=other.origin();
	// x1=o1 + a*d1
	// x2=o2 + b*d2

	// conditions:
	// (x2-x1).d1=0 and (x2-x1).d2=0

	// 1.     (o2+b*d2 - o1 - a*d1). d1 =0
	// 2.     (o2+b*d2 - o1 - a*d1). d2 =0

	// -> o2d1 + bd2d1 -o1d1 -ad1d1=0

	// (-d1d1, d2d1)(a) = o1d1-o2d1
    // (-d1d2, d2d2)(b) = o1d2-o2d2

	matrixn A(2,2);
	vectorn B(2), vx;

	A(0,0)=d1%d1*-1;
	A(0,1)=d2%d1;
	A(1,0)=d1%d2*-1;
	A(1,1)=d2%d2;
	B(0)=o1%d1-o2%d1;
	B(1)=o1%d2-o2%d2;

	m::LUsolve(A, B, vx);

	m_real a=vx[0];
	m_real b=vx[1];

	if((0<=a && a<=length()) && (0<=b && b<=other.length()))
		return pos(a).distance(other.pos(b));

	vector3 t1=target();
	vector3 t2=other.target();
	m_real dist1=o1.distance(o2);
	m_real dist2=o1.distance(t2);
	m_real dist3=t1.distance(o2);
	m_real dist4=t1.distance(t2);

	return std::min(std::min(dist1, dist2), std::min(dist3, dist4));
}

std::pair<m_real, vector3> LineSegment::minDistDir(LineSegment const& other) const
{
	ASSERT(0);
	return std::pair<m_real, vector3>(0.0, vector3(0,0,0));
}

intersectionTest::LineSegment Ray2LineSegment(::Ray const& r)
{
	return intersectionTest::LineSegment (r.origin(), r.getPoint(FLT_MAX));
}
}
/*
*         
*  Triangle-Triangle Overlap Test Routines        
*  July, 2002                                                          
*  Updated December 2003                                                
*                                                                       
*  This file contains C implementation of algorithms for                
*  performing two and three-dimensional triangle-triangle intersection test 
*  The algorithms and underlying theory are described in                    
*                                                                           
* "Fast and Robust Triangle-Triangle Overlap Test 
*  Using Orientation Predicates"  P. Guigue - O. Devillers
*                                                 
*  Journal of Graphics Tools, 8(1), 2003                                    
*                                                                           
*  Several geometric predicates are defined.  Their parameters are all      
*  points.  Each point is an array of two or three double precision         
*  floating point numbers. The geometric predicates implemented in          
*  this file are:                                                            
*                                                                           
*    int tri_tri_overlap_test_3d(p1,q1,r1,p2,q2,r2)                         
*    int tri_tri_overlap_test_2d(p1,q1,r1,p2,q2,r2)                         
*                                                                           
*    int tri_tri_intersection_test_3d(p1,q1,r1,p2,q2,r2,
*                                     coplanar,source,target)               
*                                                                           
*       is a version that computes the segment of intersection when            
*       the triangles overlap (and are not coplanar)                        
*                                                                           
*    each function returns 1 if the triangles (including their              
*    boundary) intersect, otherwise 0                                       
*                                                                           
*                                                                           
*  Other information are available from the Web page                        
*  http://www.acm.org/jgt/papers/GuigueDevillers03/                         
*                                                                           
*/


/* function prototype */

// Three-dimensional Triangle-Triangle Overlap Test
int tri_tri_overlap_test_3d(const double p1[3], const double q1[3], const double r1[3], 
          const double p2[3], const double q2[3], const double r2[3]);

namespace intersectionTest
{
	bool testTriTriOverlap(vector3 const& p1,  vector3 const& q1, vector3 const& r1,
			vector3 const& p2,  vector3 const& q2, vector3 const& r2)
	{
		return tri_tri_overlap_test_3d(&p1.x, &q1.x, &r1.x,
				&p2.x, &q2.x, &r2.x);
	}
}


// Three-dimensional Triangle-Triangle Overlap Test
// additionaly computes the segment of intersection of the two triangles if it exists. 
// coplanar returns whether the triangles are coplanar, 
// source and target are the endpoints of the line segment of intersection 
int tri_tri_intersection_test_3d(double p1[3], double q1[3], double r1[3], 
								 double p2[3], double q2[3], double r2[3],
								 int * coplanar, 
								 double source[3],double target[3]);


int coplanar_tri_tri3d(const double  p1[3], const double  q1[3], const double  r1[3],
           const double  p2[3], const double  q2[3], const double  r2[3],
           double  N1[3], double  N2[3]);


// Two dimensional Triangle-Triangle Overlap Test
int tri_tri_overlap_test_2d(double p1[2], double q1[2], double r1[2], 
          double p2[2], double q2[2], double r2[2]);






/* some 3D macros */

#define CROSS(dest,v1,v2)                       \
               dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
               dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
               dest[2]=v1[0]*v2[1]-v1[1]*v2[0];

#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])
 


#define SUB(dest,v1,v2) dest[0]=v1[0]-v2[0]; \
                        dest[1]=v1[1]-v2[1]; \
                        dest[2]=v1[2]-v2[2]; 


#define SCALAR(dest,alpha,v) dest[0] = alpha * v[0]; \
                             dest[1] = alpha * v[1]; \
                             dest[2] = alpha * v[2];



#define CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2) {\
  SUB(v1,p2,q1)\
  SUB(v2,p1,q1)\
  CROSS(N1,v1,v2)\
  SUB(v1,q2,q1)\
  if (DOT(v1,N1) > 0.0f) return 0;\
  SUB(v1,p2,p1)\
  SUB(v2,r1,p1)\
  CROSS(N1,v1,v2)\
  SUB(v1,r2,p1) \
  if (DOT(v1,N1) > 0.0f) return 0;\
  else return 1; }



/* Permutation in a canonical form of T2's vertices */

#define TRI_TRI_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2) { \
  if (dp2 > 0.0f) { \
     if (dq2 > 0.0f) CHECK_MIN_MAX(p1,r1,q1,r2,p2,q2) \
     else if (dr2 > 0.0f) CHECK_MIN_MAX(p1,r1,q1,q2,r2,p2)\
     else CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2) }\
  else if (dp2 < 0.0f) { \
    if (dq2 < 0.0f) CHECK_MIN_MAX(p1,q1,r1,r2,p2,q2)\
    else if (dr2 < 0.0f) CHECK_MIN_MAX(p1,q1,r1,q2,r2,p2)\
    else CHECK_MIN_MAX(p1,r1,q1,p2,q2,r2)\
  } else { \
    if (dq2 < 0.0f) { \
      if (dr2 >= 0.0f)  CHECK_MIN_MAX(p1,r1,q1,q2,r2,p2)\
      else CHECK_MIN_MAX(p1,q1,r1,p2,q2,r2)\
    } \
    else if (dq2 > 0.0f) { \
      if (dr2 > 0.0f) CHECK_MIN_MAX(p1,r1,q1,p2,q2,r2)\
      else  CHECK_MIN_MAX(p1,q1,r1,q2,r2,p2)\
    } \
    else  { \
      if (dr2 > 0.0f) CHECK_MIN_MAX(p1,q1,r1,r2,p2,q2)\
      else if (dr2 < 0.0f) CHECK_MIN_MAX(p1,r1,q1,r2,p2,q2)\
      else return coplanar_tri_tri3d(p1,q1,r1,p2,q2,r2,N1,N2);\
     }}}
  


/*
*
*  Three-dimensional Triangle-Triangle Overlap Test
*
*/


int tri_tri_overlap_test_3d(const double p1[3], const double q1[3], const double r1[3], 

          const double p2[3], const double q2[3], const double r2[3])
{
  double dp1, dq1, dr1, dp2, dq2, dr2;
  double v1[3], v2[3];
  double N1[3], N2[3]; 
  
  /* Compute distance signs  of p1, q1 and r1 to the plane of
     triangle(p2,q2,r2) */


  SUB(v1,p2,r2)
  SUB(v2,q2,r2)
  CROSS(N2,v1,v2)

  SUB(v1,p1,r2)
  dp1 = DOT(v1,N2);
  SUB(v1,q1,r2)
  dq1 = DOT(v1,N2);
  SUB(v1,r1,r2)
  dr1 = DOT(v1,N2);
  
  if (((dp1 * dq1) > 0.0f) && ((dp1 * dr1) > 0.0f))  return 0; 

  /* Compute distance signs  of p2, q2 and r2 to the plane of
     triangle(p1,q1,r1) */

  
  SUB(v1,q1,p1)
  SUB(v2,r1,p1)
  CROSS(N1,v1,v2)

  SUB(v1,p2,r1)
  dp2 = DOT(v1,N1);
  SUB(v1,q2,r1)
  dq2 = DOT(v1,N1);
  SUB(v1,r2,r1)
  dr2 = DOT(v1,N1);
  
  if (((dp2 * dq2) > 0.0f) && ((dp2 * dr2) > 0.0f)) return 0;

  /* Permutation in a canonical form of T1's vertices */


  if (dp1 > 0.0f) {
    if (dq1 > 0.0f) TRI_TRI_3D(r1,p1,q1,p2,r2,q2,dp2,dr2,dq2)
    else if (dr1 > 0.0f) TRI_TRI_3D(q1,r1,p1,p2,r2,q2,dp2,dr2,dq2)  
    else TRI_TRI_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2)
  } else if (dp1 < 0.0f) {
    if (dq1 < 0.0f) TRI_TRI_3D(r1,p1,q1,p2,q2,r2,dp2,dq2,dr2)
    else if (dr1 < 0.0f) TRI_TRI_3D(q1,r1,p1,p2,q2,r2,dp2,dq2,dr2)
    else TRI_TRI_3D(p1,q1,r1,p2,r2,q2,dp2,dr2,dq2)
  } else {
    if (dq1 < 0.0f) {
      if (dr1 >= 0.0f) TRI_TRI_3D(q1,r1,p1,p2,r2,q2,dp2,dr2,dq2)
      else TRI_TRI_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2)
    }
    else if (dq1 > 0.0f) {
      if (dr1 > 0.0f) TRI_TRI_3D(p1,q1,r1,p2,r2,q2,dp2,dr2,dq2)
      else TRI_TRI_3D(q1,r1,p1,p2,q2,r2,dp2,dq2,dr2)
    }
    else  {
      if (dr1 > 0.0f) TRI_TRI_3D(r1,p1,q1,p2,q2,r2,dp2,dq2,dr2)
      else if (dr1 < 0.0f) TRI_TRI_3D(r1,p1,q1,p2,r2,q2,dp2,dr2,dq2)
      else return coplanar_tri_tri3d(p1,q1,r1,p2,q2,r2,N1,N2);
    }
  }
};



int coplanar_tri_tri3d(const double p1[3], const double q1[3], const double r1[3],
           const double p2[3], const double q2[3], const double r2[3],
           double normal_1[3], double normal_2[3]){
  
  double P1[2],Q1[2],R1[2];
  double P2[2],Q2[2],R2[2];

  double n_x, n_y, n_z;

  n_x = ((normal_1[0]<0)?-normal_1[0]:normal_1[0]);
  n_y = ((normal_1[1]<0)?-normal_1[1]:normal_1[1]);
  n_z = ((normal_1[2]<0)?-normal_1[2]:normal_1[2]);


  /* Projection of the triangles in 3D onto 2D such that the area of
     the projection is maximized. */


  if (( n_x > n_z ) && ( n_x >= n_y )) {
    // Project onto plane YZ

      P1[0] = q1[2]; P1[1] = q1[1];
      Q1[0] = p1[2]; Q1[1] = p1[1];
      R1[0] = r1[2]; R1[1] = r1[1]; 
    
      P2[0] = q2[2]; P2[1] = q2[1];
      Q2[0] = p2[2]; Q2[1] = p2[1];
      R2[0] = r2[2]; R2[1] = r2[1]; 

  } else if (( n_y > n_z ) && ( n_y >= n_x )) {
    // Project onto plane XZ

    P1[0] = q1[0]; P1[1] = q1[2];
    Q1[0] = p1[0]; Q1[1] = p1[2];
    R1[0] = r1[0]; R1[1] = r1[2]; 
 
    P2[0] = q2[0]; P2[1] = q2[2];
    Q2[0] = p2[0]; Q2[1] = p2[2];
    R2[0] = r2[0]; R2[1] = r2[2]; 
    
  } else {
    // Project onto plane XY

    P1[0] = p1[0]; P1[1] = p1[1]; 
    Q1[0] = q1[0]; Q1[1] = q1[1]; 
    R1[0] = r1[0]; R1[1] = r1[1]; 
    
    P2[0] = p2[0]; P2[1] = p2[1]; 
    Q2[0] = q2[0]; Q2[1] = q2[1]; 
    R2[0] = r2[0]; R2[1] = r2[1]; 
  }

  return tri_tri_overlap_test_2d(P1,Q1,R1,P2,Q2,R2);
    
};



/*
*                                                                
*  Three-dimensional Triangle-Triangle Intersection              
*
*/

/*
   This macro is called when the triangles surely intersect
   It constructs the segment of intersection of the two triangles
   if they are not coplanar.
*/

#define CONSTRUCT_INTERSECTION(p1,q1,r1,p2,q2,r2) { \
  SUB(v1,q1,p1) \
  SUB(v2,r2,p1) \
  CROSS(N,v1,v2) \
  SUB(v,p2,p1) \
  if (DOT(v,N) > 0.0f) {\
    SUB(v1,r1,p1) \
    CROSS(N,v1,v2) \
    if (DOT(v,N) <= 0.0f) { \
      SUB(v2,q2,p1) \
      CROSS(N,v1,v2) \
      if (DOT(v,N) > 0.0f) { \
  SUB(v1,p1,p2) \
  SUB(v2,p1,r1) \
  alpha = DOT(v1,N2) / DOT(v2,N2); \
  SCALAR(v1,alpha,v2) \
  SUB(source,p1,v1) \
  SUB(v1,p2,p1) \
  SUB(v2,p2,r2) \
  alpha = DOT(v1,N1) / DOT(v2,N1); \
  SCALAR(v1,alpha,v2) \
  SUB(target,p2,v1) \
  return 1; \
      } else { \
  SUB(v1,p2,p1) \
  SUB(v2,p2,q2) \
  alpha = DOT(v1,N1) / DOT(v2,N1); \
  SCALAR(v1,alpha,v2) \
  SUB(source,p2,v1) \
  SUB(v1,p2,p1) \
  SUB(v2,p2,r2) \
  alpha = DOT(v1,N1) / DOT(v2,N1); \
  SCALAR(v1,alpha,v2) \
  SUB(target,p2,v1) \
  return 1; \
      } \
    } else { \
      return 0; \
    } \
  } else { \
    SUB(v2,q2,p1) \
    CROSS(N,v1,v2) \
    if (DOT(v,N) < 0.0f) { \
      return 0; \
    } else { \
      SUB(v1,r1,p1) \
      CROSS(N,v1,v2) \
      if (DOT(v,N) >= 0.0f) { \
  SUB(v1,p1,p2) \
  SUB(v2,p1,r1) \
  alpha = DOT(v1,N2) / DOT(v2,N2); \
  SCALAR(v1,alpha,v2) \
  SUB(source,p1,v1) \
  SUB(v1,p1,p2) \
  SUB(v2,p1,q1) \
  alpha = DOT(v1,N2) / DOT(v2,N2); \
  SCALAR(v1,alpha,v2) \
  SUB(target,p1,v1) \
  return 1; \
      } else { \
  SUB(v1,p2,p1) \
  SUB(v2,p2,q2) \
  alpha = DOT(v1,N1) / DOT(v2,N1); \
  SCALAR(v1,alpha,v2) \
  SUB(source,p2,v1) \
  SUB(v1,p1,p2) \
  SUB(v2,p1,q1) \
  alpha = DOT(v1,N2) / DOT(v2,N2); \
  SCALAR(v1,alpha,v2) \
  SUB(target,p1,v1) \
  return 1; \
      }}}} 

                

#define TRI_TRI_INTER_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2) { \
  if (dp2 > 0.0f) { \
     if (dq2 > 0.0f) CONSTRUCT_INTERSECTION(p1,r1,q1,r2,p2,q2) \
     else if (dr2 > 0.0f) CONSTRUCT_INTERSECTION(p1,r1,q1,q2,r2,p2)\
     else CONSTRUCT_INTERSECTION(p1,q1,r1,p2,q2,r2) }\
  else if (dp2 < 0.0f) { \
    if (dq2 < 0.0f) CONSTRUCT_INTERSECTION(p1,q1,r1,r2,p2,q2)\
    else if (dr2 < 0.0f) CONSTRUCT_INTERSECTION(p1,q1,r1,q2,r2,p2)\
    else CONSTRUCT_INTERSECTION(p1,r1,q1,p2,q2,r2)\
  } else { \
    if (dq2 < 0.0f) { \
      if (dr2 >= 0.0f)  CONSTRUCT_INTERSECTION(p1,r1,q1,q2,r2,p2)\
      else CONSTRUCT_INTERSECTION(p1,q1,r1,p2,q2,r2)\
    } \
    else if (dq2 > 0.0f) { \
      if (dr2 > 0.0f) CONSTRUCT_INTERSECTION(p1,r1,q1,p2,q2,r2)\
      else  CONSTRUCT_INTERSECTION(p1,q1,r1,q2,r2,p2)\
    } \
    else  { \
      if (dr2 > 0.0f) CONSTRUCT_INTERSECTION(p1,q1,r1,r2,p2,q2)\
      else if (dr2 < 0.0f) CONSTRUCT_INTERSECTION(p1,r1,q1,r2,p2,q2)\
      else { \
        *coplanar = 1; \
  return coplanar_tri_tri3d(p1,q1,r1,p2,q2,r2,N1,N2);\
     } \
  }} }
  

/*
   The following version computes the segment of intersection of the
   two triangles if it exists. 
   coplanar returns whether the triangles are coplanar
   source and target are the endpoints of the line segment of intersection 
*/

int tri_tri_intersection_test_3d(double p1[3], double q1[3], double r1[3], 
         double p2[3], double q2[3], double r2[3],
         int * coplanar, 
         double source[3], double target[3] )
         
{
  double dp1, dq1, dr1, dp2, dq2, dr2;
  double v1[3], v2[3], v[3];
  double N1[3], N2[3], N[3];
  double alpha;

  // Compute distance signs  of p1, q1 and r1 
  // to the plane of triangle(p2,q2,r2)


  SUB(v1,p2,r2)
  SUB(v2,q2,r2)
  CROSS(N2,v1,v2)

  SUB(v1,p1,r2)
  dp1 = DOT(v1,N2);
  SUB(v1,q1,r2)
  dq1 = DOT(v1,N2);
  SUB(v1,r1,r2)
  dr1 = DOT(v1,N2);
  
  if (((dp1 * dq1) > 0.0f) && ((dp1 * dr1) > 0.0f))  return 0; 

  // Compute distance signs  of p2, q2 and r2 
  // to the plane of triangle(p1,q1,r1)

  
  SUB(v1,q1,p1)
  SUB(v2,r1,p1)
  CROSS(N1,v1,v2)

  SUB(v1,p2,r1)
  dp2 = DOT(v1,N1);
  SUB(v1,q2,r1)
  dq2 = DOT(v1,N1);
  SUB(v1,r2,r1)
  dr2 = DOT(v1,N1);
  
  if (((dp2 * dq2) > 0.0f) && ((dp2 * dr2) > 0.0f)) return 0;

  // Permutation in a canonical form of T1's vertices


  if (dp1 > 0.0f) {
    if (dq1 > 0.0f) TRI_TRI_INTER_3D(r1,p1,q1,p2,r2,q2,dp2,dr2,dq2)
    else if (dr1 > 0.0f) TRI_TRI_INTER_3D(q1,r1,p1,p2,r2,q2,dp2,dr2,dq2)
  
    else TRI_TRI_INTER_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2)
  } else if (dp1 < 0.0f) {
    if (dq1 < 0.0f) TRI_TRI_INTER_3D(r1,p1,q1,p2,q2,r2,dp2,dq2,dr2)
    else if (dr1 < 0.0f) TRI_TRI_INTER_3D(q1,r1,p1,p2,q2,r2,dp2,dq2,dr2)
    else TRI_TRI_INTER_3D(p1,q1,r1,p2,r2,q2,dp2,dr2,dq2)
  } else {
    if (dq1 < 0.0f) {
      if (dr1 >= 0.0f) TRI_TRI_INTER_3D(q1,r1,p1,p2,r2,q2,dp2,dr2,dq2)
      else TRI_TRI_INTER_3D(p1,q1,r1,p2,q2,r2,dp2,dq2,dr2)
    }
    else if (dq1 > 0.0f) {
      if (dr1 > 0.0f) TRI_TRI_INTER_3D(p1,q1,r1,p2,r2,q2,dp2,dr2,dq2)
      else TRI_TRI_INTER_3D(q1,r1,p1,p2,q2,r2,dp2,dq2,dr2)
    }
    else  {
      if (dr1 > 0.0f) TRI_TRI_INTER_3D(r1,p1,q1,p2,q2,r2,dp2,dq2,dr2)
      else if (dr1 < 0.0f) TRI_TRI_INTER_3D(r1,p1,q1,p2,r2,q2,dp2,dr2,dq2)
      else {
  // triangles are co-planar

  *coplanar = 1;
  return coplanar_tri_tri3d(p1,q1,r1,p2,q2,r2,N1,N2);
      }
    }
  }
};





/*
*
*  Two dimensional Triangle-Triangle Overlap Test    
*
*/


/* some 2D macros */

#define ORIENT_2D(a, b, c)  ((a[0]-c[0])*(b[1]-c[1])-(a[1]-c[1])*(b[0]-c[0]))


#define INTERSECTION_TEST_VERTEX(P1, Q1, R1, P2, Q2, R2) {\
  if (ORIENT_2D(R2,P2,Q1) >= 0.0f)\
    if (ORIENT_2D(R2,Q2,Q1) <= 0.0f)\
      if (ORIENT_2D(P1,P2,Q1) > 0.0f) {\
  if (ORIENT_2D(P1,Q2,Q1) <= 0.0f) return 1; \
  else return 0;} else {\
  if (ORIENT_2D(P1,P2,R1) >= 0.0f)\
    if (ORIENT_2D(Q1,R1,P2) >= 0.0f) return 1; \
    else return 0;\
  else return 0;}\
    else \
      if (ORIENT_2D(P1,Q2,Q1) <= 0.0f)\
  if (ORIENT_2D(R2,Q2,R1) <= 0.0f)\
    if (ORIENT_2D(Q1,R1,Q2) >= 0.0f) return 1; \
    else return 0;\
  else return 0;\
      else return 0;\
  else\
    if (ORIENT_2D(R2,P2,R1) >= 0.0f) \
      if (ORIENT_2D(Q1,R1,R2) >= 0.0f)\
  if (ORIENT_2D(P1,P2,R1) >= 0.0f) return 1;\
  else return 0;\
      else \
  if (ORIENT_2D(Q1,R1,Q2) >= 0.0f) {\
    if (ORIENT_2D(R2,R1,Q2) >= 0.0f) return 1; \
    else return 0; }\
  else return 0; \
    else  return 0; \
 };



#define INTERSECTION_TEST_EDGE(P1, Q1, R1, P2, Q2, R2) { \
  if (ORIENT_2D(R2,P2,Q1) >= 0.0f) {\
    if (ORIENT_2D(P1,P2,Q1) >= 0.0f) { \
        if (ORIENT_2D(P1,Q1,R2) >= 0.0f) return 1; \
        else return 0;} else { \
      if (ORIENT_2D(Q1,R1,P2) >= 0.0f){ \
  if (ORIENT_2D(R1,P1,P2) >= 0.0f) return 1; else return 0;} \
      else return 0; } \
  } else {\
    if (ORIENT_2D(R2,P2,R1) >= 0.0f) {\
      if (ORIENT_2D(P1,P2,R1) >= 0.0f) {\
  if (ORIENT_2D(P1,R1,R2) >= 0.0f) return 1;  \
  else {\
    if (ORIENT_2D(Q1,R1,R2) >= 0.0f) return 1; else return 0;}}\
      else  return 0; }\
    else return 0; }}



int ccw_tri_tri_intersection_2d(double p1[2], double q1[2], double r1[2], 
        double p2[2], double q2[2], double r2[2]) {
  if ( ORIENT_2D(p2,q2,p1) >= 0.0f ) {
    if ( ORIENT_2D(q2,r2,p1) >= 0.0f ) {
      if ( ORIENT_2D(r2,p2,p1) >= 0.0f ) return 1;
      else INTERSECTION_TEST_EDGE(p1,q1,r1,p2,q2,r2)
    } else {  
      if ( ORIENT_2D(r2,p2,p1) >= 0.0f ) 
  INTERSECTION_TEST_EDGE(p1,q1,r1,r2,p2,q2)
      else INTERSECTION_TEST_VERTEX(p1,q1,r1,p2,q2,r2)}}
  else {
    if ( ORIENT_2D(q2,r2,p1) >= 0.0f ) {
      if ( ORIENT_2D(r2,p2,p1) >= 0.0f ) 
  INTERSECTION_TEST_EDGE(p1,q1,r1,q2,r2,p2)
      else  INTERSECTION_TEST_VERTEX(p1,q1,r1,q2,r2,p2)}
    else INTERSECTION_TEST_VERTEX(p1,q1,r1,r2,p2,q2)}
};


int tri_tri_overlap_test_2d(double p1[2], double q1[2], double r1[2], 
          double p2[2], double q2[2], double r2[2]) {
  if ( ORIENT_2D(p1,q1,r1) < 0.0f )
    if ( ORIENT_2D(p2,q2,r2) < 0.0f )
      return ccw_tri_tri_intersection_2d(p1,r1,q1,p2,r2,q2);
    else
      return ccw_tri_tri_intersection_2d(p1,r1,q1,p2,q2,r2);
  else
    if ( ORIENT_2D(p2,q2,r2) < 0.0f )
      return ccw_tri_tri_intersection_2d(p1,q1,r1,p2,r2,q2);
    else
      return ccw_tri_tri_intersection_2d(p1,q1,r1,p2,q2,r2);

};
