/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
//
// TriOverlap.cpp
//
#include "StdAfxColDet.h"
#define HIRUKAWA_DEBUG 0
#include "utilities.h"
extern dmatrix33 CD_Rot2;
extern dvector3 CD_Trans2;
extern double CD_s2;


#include <stdio.h>
#include <iostream>
using namespace std;
/**********************************************************	
  separability test by the supporting plane of a triangle
   return value 
   0   : not intersect
   1   : f1 or e1 doesn't intersect
   2   : f2 or e2 doesn't intersect
   3   : f3 or e3 doesn't intersect
**********************************************************/	

#define NOT_INTERSECT  0

/* used in normal_test */
#define  EDGE1_NOT_INTERSECT  1
#define  EDGE2_NOT_INTERSECT  2
#define  EDGE3_NOT_INTERSECT  3

/* used in cross_test */
#define INTERSECT  1

int
separability_test_by_face(dvector3 &nm)
{

  if(nm[0] < 0 && nm[1]<0 && nm[2]<0 ||
     nm[0] > 0 && nm[1]>0 && nm[2]>0){
    return NOT_INTERSECT;
  }
  if(nm[0] < 0 && nm[1]<0 && nm[2]>0 ||
     nm[0] > 0 && nm[1]>0 && nm[2] < 0){
    return EDGE1_NOT_INTERSECT;
  }
  if(nm[0] < 0 && nm[1]>0 && nm[2] > 0 ||
     nm[0] > 0 && nm[1]<0 && nm[2] < 0){
    return EDGE2_NOT_INTERSECT;
  }
  if(nm[0] > 0 && nm[1]<0 && nm[2]>0 ||
     nm[0] < 0 && nm[1]>0 && nm[2]<0){
    return EDGE3_NOT_INTERSECT;
  }
  return 0;
}

/**********************************************************
   triangle inside test:
   normal vector is cross product of ei*fj
***********************************************************/

int
triangle_inside_test(dvector3 &ef1,
		     dvector3 &ef2,
		     dvector3 &ef3,
		     dvector3 &P3,
		     dvector3 &P1,
		     dvector3 &P2,
		     dvector3 &Q)
{	

  double ef1P1 = innerProd(ef1,P1);	/*project P1 on ef1*/
  double ef1P3 = innerProd(ef1,P3);	/*project P3 on ef1*/
  double ef1Q = innerProd(ef1,Q);	/*project Q on ef1*/

  double ef2P2 = innerProd(ef2,P2);	/*project P2 on ef2*/
  double ef2P1 = innerProd(ef2,P1);	/*project P1 on ef2*/	
  double ef2Q = innerProd(ef2,Q);	/*project Q on ef2*/

  double ef3P3 = innerProd(ef3,P3);	/*project P3 on ef3*/	
  double ef3P2 = innerProd(ef3,P2);	/*project P2 on ef3*/		
  double ef3Q = innerProd(ef3,Q);  	/*project Q on ef3*/		

  if((ef1P3 > ef1P1  && ef1Q > ef1P1 	||
      ef1P3 < ef1P1 &&  ef1Q < ef1P1    ) 
     &&
     (ef2P1 > ef2P2 && ef2Q > ef2P2 	||
      ef2P1 < ef2P2 && ef2Q < ef2P2 	) 
     &&
     (ef3P2 > ef3P3 && ef3Q > ef3P3 	||
      ef3P2 < ef3P3 && ef3Q < ef3P3    ))
    {	
      return INTERSECT;
    }
  else{
    return NOT_INTERSECT;
  }
}

void
find_intersection_pt(dvector3 &ipt,
		     dvector3 &x1,
		     dvector3 &x2,
		     double mn1, 
		     double mn2)
{
  dvector3 x;
  
  if(mn1 == mn2) /*exit(0);*/ return;
  

  if(mn1 >0 && mn2 < 0){
    x = (-(mn2*x1) + mn1*x2)/(mn1-mn2);
    ipt = x;
  }
  else if(mn1 < 0 && mn2 > 0){
    x = (mn2*x1 - mn1*x2)/(-mn1+mn2);
    ipt = x;
  }
}

void
find_intersection_pt(dvector3 &ipt,
		     dvector3 &x1,
		     dvector3 &x2,
		     double p)
{
  ipt = (1-p) * x1 + p * x2;

#if 0
  cout << "v1 = " << x1[0] << ", " << x1[1] << ", " << x1[2] << endl; 
  cout << "v2 = " << x2[0] << ", " << x2[1] << ", " << x2[2] << endl;
  cout << "edge = " << x2[0]-x1[0] << ", " << x2[1]-x1[1] << ", "
       << x2[2]-x1[2] << endl;

  cout << "common pt = " << ipt[0] << " " << ipt[1] << " " << ipt[2] << endl;
#endif
}

//
// Calculate the depth of the intersection between two triangles
//
double calc_depth(dvector3 &ip1,
		  dvector3 &ip2,
		  dvector3 &n)
{
  dvector3 vec;

  // vecNormalize(n);
  vec = ip1 - ip2;
  if(HIRUKAWA_DEBUG) cout << "calc_depth 1 = " << innerProd(vec,n) << endl;
  return fabs(innerProd(vec, n));
}


double calc_depth(dvector3 &ip,
		  dvector3 &pt1,
		  dvector3 &pt2,
		  dvector3 &n)
{
  dvector3 vec;
  double d1, d2, depth;

  vec = ip - pt1; d1 = fabs(innerProd(vec,n));
  vec = ip - pt2; d2 = fabs(innerProd(vec,n));

  if(d1<d2) depth = d1; else depth = d2;
  if(HIRUKAWA_DEBUG) cout << "calc_depth 2 = " << depth << endl;

  return depth;
} 

double calc_depth(dvector3 &ip1,
		  dvector3 &ip2,
		  dvector3 &pt1,
		  dvector3 &pt2,
		  dvector3 &pt3,
		  dvector3 &n)
{
  // when a triangle penetrates another triangle at two intersection points
  // and the separating plane is the supporting plane of the penetrated triangle
  dvector3 vec;
  double depth, d1, d2, d3;

  // vecNormalize(n);

  vec = ip1 - pt1; d1 = fabs(innerProd(vec, n));
  vec = ip2 - pt2; d2 = fabs(innerProd(vec, n));
  vec = ip1 - pt3; d3 = fabs(innerProd(vec, n)); // ip1 can be either ip1 or ip2

  if(d1 < d2) depth = d2; else depth = d1;
  if(d3 < depth) depth = d3;

  if(HIRUKAWA_DEBUG) cout << "calc_depth 3 = " << depth << endl;
  return depth;
}

void find_foot(dvector3 &ip,
	       dvector3 &pt1,
	       dvector3 &pt2,
	       dvector3 &f)
{
  double u, v, w, p;

  u = pt2[0] - pt1[0]; v = pt2[1] - pt1[1]; w = pt2[2] - pt1[2];

  p = u * (ip[0] - pt1[0]) + v * (ip[1] - pt1[1]) + w * (ip[2] - pt1[2]);
  p /= u * u + v * v + w * w;

  f[0] = pt1[0] + u * p; f[1] = pt1[1] + v * p; f[2] = pt1[2] + w * p;
}

double calc_depth(dvector3 &ip,
		  dvector3 &pt1,
		  dvector3 &pt2,
		  dvector3 &pt3,
		  dvector3 &n)
{
  dvector3 f12, f23, f31, vec;
  double d1, d2, d3, depth; // dsum

  find_foot(ip,pt1,pt2,f12); find_foot(ip,pt2,pt3,f23); find_foot(ip,pt3,pt1,f31);
#if 0
  cout << "ip = " << ip[0] << " " << ip[1] << " " << ip[2] << endl;
  cout << "f12 = " << f12[0] << " " << f12[1] << " " << f12[2] << endl;
  cout << "f23 = " << f23[0] << " " << f23[1] << " " << f23[2] << endl;
  cout << "f31 = " << f31[0] << " " << f31[1] << " " << f31[2] << endl;
#endif
  // fabs() is taken to cope with numerical error of find_foot()
  vec = f12 - ip; d1 = fabs(innerProd(vec,n));
  vec = f23 - ip; d2 = fabs(innerProd(vec,n));
  vec = f31 - ip; d3 = fabs(innerProd(vec,n));

  // cout << "d1 d2 d3 = " << d1 << " " << d2 << " " << d3 << endl;
  // dsum = fabs(d1)+fabs(d2)+fabs(d3);
  // if(d1<0.0) d1=dsum; if(d2<0.0) d2=dsum; if(d3<0.0) d3=dsum;
  
  if(d1 < d2) depth = d1; else depth = d2;
  if(d3 < depth) depth = d3;

  if(HIRUKAWA_DEBUG) cout << "calc_depth 4 = " << depth << endl;
  return depth;
}


double calc_depth2(dvector3 &ip1,
		   dvector3 &ip2,
		   dvector3 &pt1,
		   dvector3 &pt2,
		   dvector3 &pt3,
		   dvector3 &n)
{
  // when a triangle penetrates another triangle at two intersecting points
  // and the separating plane is the supporting plane of the penetrating triangle

  double depth, depth1, depth2;
  dvector3 nn;

  nn = n; // vecNormalize(nn);

  depth1 = calc_depth(ip1,pt1,pt2,pt3,nn);
  depth2 = calc_depth(ip2,pt1,pt2,pt3,nn);

  // cout << "depth1 depth2 = " << depth1 << " " << depth2 << endl;
  if(depth1 < depth2) depth = depth2; else depth = depth1;

  if(HIRUKAWA_DEBUG) cout << "calc_depth 5 = " << depth << endl;
  return depth;
}

void calcNormal(dvector3 &vec,
		dvector3 &v1,
		dvector3 &v2,
		dvector3 &v3,
		double sgn)
{
  // find the vector from v1 to the mid point of v2 and v3 when 0<sgn

  vec = v2 + v3;
  vec = vec * 0.5;
  vec = v1 - vec;
  vecNormalize(vec);

  if(sgn<0) vec = -vec;
}

int
find_common_perpendicular(dvector3 &p1,
			  dvector3 &p2,
			  dvector3 &q1,
			  dvector3 &q2,
			  dvector3 &ip1,
			  dvector3 &ip2,
			  dvector3 &n1,
			  dvector3 &m1,
			  dvector3 &n_vector,
			  double *dp)
{
  double det, c11, c12, c21, c22, a, b, t1, t2;
  dvector3 e, f, g; //, v1, v2;
  double eps, vn;

  eps = 1.0e-3; // threshold to consider two edges are parallel
  vn = 1.0e-2;     // threshold to judge an intersecting point is near a vertex

  e = p2 - p1; f = q2 - q1;

  c11 = innerProd(e,e); c12 = - innerProd(e,f);
  c21 = - c12;      c22 = - innerProd(f,f);

  det = c11 * c22 - c12 * c21;
  // cout << "det = " << det << endl;

  if(fabs(det) < eps){
    return 0;
  }
  else{
    g = q1 - p1;
    a = innerProd(e,g);
    b = innerProd(f,g);
    t1 = ( c22 * a - c12 * b)/det;
    t2 = (-c21 * a + c11 * b)/det;

    // quit if the foot of the common perpendicular is not on an edge
    if(t1<0 || 1<t1 || t2<0 || 1<t2) return 0;

    // when two edges are in contact near a vertex of an edge
    // if(t1<vn || 1.0-vn<t1 || t2<vn || 1.0-vn<t2) return 0;

    // find_intersection_pt(v1, p1, p2, t1);
    // find_intersection_pt(v2, q1, q2, t2);
   
    *dp = calc_depth(ip1, ip2, n_vector); 

    return 1;
  }
}
    

// for vertex-face contact
int
get_normal_vector_test(dvector3 &ip1,
		       dvector3 &v1,
		       dvector3 &ip2,
		       dvector3 &v2,
		       dvector3 &n1,
		       dvector3 &m1)
{
  double eps_applicability;

  // ip1 and ip2 are the intersecting points
  // v1 and v2 are the vertices of the penetrating triangle
  // note that (v1-ip1) (v2-ip2) lies on the penetrating triangle

  // eps_applicability = 0.965926; // Cos(15) threshold to consider two triangles face
  eps_applicability = 0.5; // Cos(60) threshold to consider two triangles face

  // This condition should be checked mildly because the whole sole of a foot
  // may sink inside the floor and then no collision is detected.
  if(eps_applicability < innerProd(n1,m1)) return 0; else return 1;
}


// for edge-edge contact
int
get_normal_vector_test(dvector3 &n_vector,
		       dvector3 &ef,
		       dvector3 &ip,
		       dvector3 &iq,
		       dvector3 &v1,
		       dvector3 &v2,
		       dvector3 &n1,
		       dvector3 &m1,
		       dvector3 &va1,
		       dvector3 &va2,
		       dvector3 &vb1,
		       dvector3 &vb2)
{
  double eps_applicability, eps_length, ea_length, eb_length, eps_theta,
    theta1, theta2, theta3, theta4, theta12, theta34, theta;
  dvector3 v, sv1, sv2;
  double sv1_norm, sv2_norm;

  // ip is the intersecting point on triangle p1p2p3
  // iq is the intersecting point on triangle q1q2q3
  // v1 is the vertex of p1p2p3 which is not on the intersecting edge
  // v2 is the vertex of q1q2q3 which is not on the intersecting edge
  // note that (v1-ip) lies on triangle p1p2p3 and (v2-iq) on q1q2q3

  eps_applicability = 0.0; // 1.52e-2; // threshold to consider two triangles face
  eps_length = 1.0e-3; // 1mm: thereshold to consider the intersecting edge is short
  eps_theta = 1.0e-1;   // threshold to consider cos(theta) is too small 

  // quit if two triangles does not satifsy the applicability condition
  // i.e. two triangles do not face each other
  if(- eps_applicability < innerProd(n1,m1)) return 0;

  v = va1 - va2; ea_length = sqrt(innerProd(v,v));
  v = vb1 - vb2; eb_length = sqrt(innerProd(v,v));

  // return the normal vector of a triangle if an intersecting edge is too short
  if(ea_length < eps_length || eb_length < eps_length){
    // cout << "edge is too short" << endl;
    if(ea_length < eb_length)
      n_vector = m1;
    else
      n_vector = -n1;
    return 1;
  }

  sv1 = v1 - ip; sv1_norm = vecNorm(sv1);
  sv2 = v2 - iq; sv2_norm = vecNorm(sv2);

  if(eps_length < sv1_norm && eps_length < sv2_norm){
    // quit if two triangles do not satisfy the applicability conditions
    if(- eps_applicability < innerProd(sv1,sv2) / (vecNorm(sv1) * vecNorm(sv2)))
      return 0;
  }

  // now neither of two edges is not too short
  vecNormalize(ef);

  // Triangle p1p2p3
  theta1 = innerProd(ef,n1) / vecNorm(n1);
  if(eps_length < sv1_norm)
    theta2 = innerProd(ef,sv1) / vecNorm(sv1);
  else
    theta2 = 0.0;

  // triangle q1q2q3
  theta3 = innerProd(ef,m1) / vecNorm(m1);
  if(eps_length < sv2_norm)
    theta4 = innerProd(ef,sv2) / vecNorm(sv2);
  else
    theta4 = 0.0;

  if(sv1_norm < eps_length || sv2_norm < eps_length){
    // when sv1 or sv2 is too short
    // cout << "sv is too short" << endl;
    if(fabs(theta1) < fabs(theta3))
      n_vector = m1;
    else
      n_vector = -n1;
    return 1;    
  }

  if(fabs(theta2) < eps_theta && fabs(theta4) < eps_theta){
    // when two triangles are coplanar
    // proof.
    //  ef = (va2-va1)x(vb2-vb1) (1)
    //  sv1 * ef = 0             (2)
    //  sv2 * ef = 0             
    //  substituting (1) to (2),
    //    sv1 * (va2-va1) x (vb2-vb1) = 0
    //    (vb2 - vb1) * sv1 x (va2 - va1) = 0
    //  considering sv1 x (va2 - va1) = n,
    //    (vb2 - vb1) * n = 0
    //  in the same way
    //    (va2 - va1) * m = 0
    // q.e.d.

    if(fabs(theta1) < fabs(theta3)){
      n_vector = m1;
      return 1;
    }
    else{
      n_vector = -n1;
      return 1;
    }
  }

  // return 0 if the plane which passes through ip with normal vector ef
  // does not separate v1 and v2
  if(-eps_applicability < theta2 * theta4) return 0;

  //
  // regular case
  //
  if(fabs(theta1) < fabs(theta2))
    theta12 = theta2;
  else
    theta12 = -1.0 * theta1;

  if(fabs(theta3) < fabs(theta4))
    theta34 = -1.0 * theta4;
  else
    theta34 = theta3;

  if(fabs(theta12) < fabs(theta34))
    theta = theta34;
  else
    theta = theta12;

  if(0 < theta)
    n_vector = ef;
  else
    n_vector = -ef;

#if 0
    cout << "va1=" << va1[0] << " " << va1[1] << " " << va1[2] << endl;
    cout << "va2=" << va2[0] << " " << va2[1] << " " << va2[2] << endl;
    cout << "va3=" << v1[0] << " " << v1[1] << " " << v1[2] << endl;
    cout << "vb1=" << vb1[0] << " " << vb1[1] << " " << vb1[2] << endl;
    cout << "vb2=" << vb2[0] << " " << vb2[1] << " " << vb2[2] << endl;
    cout << "vb3=" << v2[0] << " " << v2[1] << " " << v2[2] << endl;
    cout << "n1=" << n1[0] << " " << n1[1] << " " << n1[2] << endl;
    cout << "m1=" << m1[0] << " " << m1[1] << " " << m1[2] << endl;
    cout << "ef=" << ef[0] << " " << ef[1] << " " << ef[2] << endl;
    cout << "sv1=" << sv1[0] << " " << sv1[1] << " " << sv1[2] << endl;
    cout << "sv2=" << sv2[0] << " " << sv2[1] << " " << sv2[2] << endl;
    cout << endl;
#endif

    if(innerProd(n_vector,sv1) < eps_applicability || - eps_applicability < innerProd(n_vector,sv2)){
      // when the separating plane separates the outsides of the triangles
      return 0;
    }
    else
      return 1;
}

//
// find the collision info when a vertex penetrates a face
//
int find_collision_info(dvector3 &p1,
			dvector3 &p2,
			dvector3 &p3,
			double mp0,
			double mp1,
			double mp2,
			dvector3 &q1,
			dvector3 &q2,
			dvector3 &q3,
			dvector3 &f1,
			dvector3 &f2,
			dvector3 &f3,
			dvector3 &n1,
			dvector3 &m1,
			dvector3 &ip3,
			dvector3 &ip4,
			dvector3 &ip5,
			dvector3 &ip6,
			collision_data *col_p, double pq)
{
  dvector3 ip, ip1, ip2, vec, nv;
  double dp;
 
  find_intersection_pt(ip,p1,p2,mp0,mp1); ip1 = ip;
  find_intersection_pt(ip,p3,p1,mp2,mp0); ip2 = ip;
  if(get_normal_vector_test(ip1,p2,ip2,p3,m1,n1)){

    calcNormal(vec,p1,p2,p3,mp0);
    col_p->n_vector = m1 * pq;

    //
    // The depth is estimated in InsertCollisionPair.cpp
    // The following depth calculation is done only for debugging purpose
    //
    col_p->depth = calc_depth(ip1, ip2, p2, p3, p1, col_p->n_vector);
    nv = -n1 * pq; dp = calc_depth2(ip1, ip2, q1, q2, q3, nv);
    if(dp<col_p->depth) col_p->depth = dp;

    ip3 = ip1; ip4 = ip2;
    col_p->num_of_i_points = 2;

    return 1;
  }
  else
    return 0;
}

//
// find the collision info when an edges penetrate a face each other 
//
int find_collision_info(dvector3 &p1,
			dvector3 &p2,
			dvector3 &p3,
			double mp0,
			double mp1,
			dvector3 &q1,
			dvector3 &q2,
			dvector3 &q3,
			double nq0,
			double nq1,
			dvector3 &ef11,
			dvector3 &n1,
			dvector3 &m1,
			dvector3 &ip3,
			dvector3 &ip4,
			collision_data *col_p)
{
  dvector3 ip, ip1, ip2;
  double dp;
  find_intersection_pt(ip,q1,q2,nq0,nq1); ip1 = ip;
  find_intersection_pt(ip,p1,p2,mp0,mp1); ip2 = ip;

  if(get_normal_vector_test(col_p->n_vector,ef11,ip2,ip1,p3,q3,n1,m1,p1,p2,q1,q2) &&
     find_common_perpendicular(p1,p2,q1,q2,ip1,ip2,n1,m1,col_p->n_vector,&dp)){

    ip3 = ip1; ip4 = ip2;
    col_p->num_of_i_points = 2;
    col_p->depth = dp;
    return 1;
  }
  else
    return 0;
}

// very robust triangle intersection test
// uses no divisions
// works on coplanar triangles

int 
tri_tri_overlap(dvector3 &P1,
		dvector3 &P2,
		dvector3 &P3,
		dvector3 &Q1,
		dvector3 &Q2,
		dvector3 &Q3,
		collision_data *col_p) 
{
  /*
     One triangle is (p1,p2,p3).  Other is (q1,q2,q3).
     Edges are (e1,e2,e3) and (f1,f2,f3).
     Normals are n1 and m1
     Outwards are (g1,g2,g3) and (h1,h2,h3).
     
     We assume that the triangle vertices are in the same coordinate system.

     First thing we do is establish a new c.s. so that p1 is at (0,0,0).

     */
  dvector3 p1, p2, p3;
  dvector3 q1, q2, q3;
  dvector3 e1, e2, e3;
  dvector3 f1, f2, f3;
  // dvector3 g1, g2, g3;
  // dvector3 h1, h2, h3;
  dvector3 n1, m1;
  dvector3 z;
  dvector3 nq, mp;

  int triP,triQ;
  int edf1,edf2,edf3,ede1,ede2,ede3;
  int FV, VF, EE;

  dvector3 ef11, ef12, ef13;
  dvector3 ef21, ef22, ef23;
  dvector3 ef31, ef32, ef33;

  /* intersection point   R is a flag which tri P & Q correspond or not  */
  dvector3 ip,ip3,ip4,ip5,ip6;
  dvector3 i_pts_w[4];
  
  FV = 1; // face-vertex contact type
  VF = 2; // vertex-face contact type
  EE = 3; // edge-edge contact type

  z[0] = 0.0;  z[1] = 0.0;  z[2] = 0.0;
  
  p1 = P1 - P1;
  p2 =  P2 -  P1;
  p3 =  P3 -  P1;
  
  q1 =  Q1 -  P1;
  q2 =  Q2 -  P1;
  q3 =  Q3 -  P1;
  
  e1 =  p2 -  p1;
  e2 =  p3 -  p2;
  e3 =  p1 -  p3;

  f1 =  q2 -  q1;
  f2 =  q3 -  q2;
  f3 =  q1 -  q3;

  outerProd(n1, e1, e2);
  outerProd(m1, f1, f2);

  // now begin the series of tests

 /*************************************
        separability test by face
  ************************************/

  nq[0] = innerProd(n1,q1);
  nq[1] = innerProd(n1,q2);
  nq[2] = innerProd(n1,q3);
  triQ = separability_test_by_face(nq);

  if(triQ == NOT_INTERSECT) return 0;

  double mq = innerProd(m1,q1);
  mp[0] = innerProd(m1,p1) - mq;
  mp[1] = innerProd(m1,p2) - mq;
  mp[2] = innerProd(m1,p3) - mq;
  triP = separability_test_by_face(mp);
  if(triP == NOT_INTERSECT) return 0;

  outerProd(ef11, e1, f1);
  outerProd(ef12, e1, f2);
  outerProd(ef13, e1, f3);
  outerProd(ef21, e2, f1);
  outerProd(ef22, e2, f2);
  outerProd(ef23, e2, f3);
  outerProd(ef31, e3, f1);
  outerProd(ef32, e3, f2);
  outerProd(ef33, e3, f3);

  edf1 = 0; edf2 = 0; edf3 = 0; ede1 = 0; ede2 = 0; ede3 = 0;

  /********************************
	 triangle inside test
  *********************************/	
  switch(triQ)
    {
    case NOT_INTERSECT:
      return 0;

    case EDGE1_NOT_INTERSECT:    
      edf2 = triangle_inside_test(ef12,ef22,ef32,p3,p1,p2,q2);
      edf3 = triangle_inside_test(ef13,ef23,ef33,p3,p1,p2,q3);
      break;

    case EDGE2_NOT_INTERSECT:	  
      edf1 = triangle_inside_test(ef11,ef21,ef31,p3,p1,p2,q1);	
      edf3 = triangle_inside_test(ef13,ef23,ef33,p3,p1,p2,q3);
      break;

    case EDGE3_NOT_INTERSECT:	
      edf1 = triangle_inside_test(ef11,ef21,ef31,p3,p1,p2,q1);	
      edf2 = triangle_inside_test(ef12,ef22,ef32,p3,p1,p2,q2);
      break;
    }

  int num_of_edges = edf1 + edf2 + edf3;
  if(num_of_edges == 3){
    //exit(1);
    return 0;
  }
 
  if(num_of_edges < 2){
    switch(triP)
      {
      case EDGE1_NOT_INTERSECT:
	ede2 = triangle_inside_test(ef21,ef22,ef23,q3,q1,q2,p2);
	ede3 = triangle_inside_test(ef31,ef32,ef33,q3,q1,q2,p3);
	if(ede2+ede3==2){
	  edf1= NOT_INTERSECT;
	  edf2= NOT_INTERSECT;
	  edf3= NOT_INTERSECT;
	}
	break;
	
      case EDGE2_NOT_INTERSECT:
	ede1 = triangle_inside_test(ef11,ef12,ef13,q3,q1,q2,p1);
	ede3 = triangle_inside_test(ef31,ef32,ef33,q3,q1,q2,p3);
	if(ede1+ede3==2){
	  edf1= NOT_INTERSECT;
	  edf2= NOT_INTERSECT;
	  edf3= NOT_INTERSECT;
	}
	break;    
	
      case EDGE3_NOT_INTERSECT:
	ede1 = triangle_inside_test(ef11,ef12,ef13,q3,q1,q2,p1);
	ede2 = triangle_inside_test(ef21,ef22,ef23,q3,q1,q2,p2);
	if(ede1+ede2 == 2){
	  edf1= NOT_INTERSECT;
	  edf2= NOT_INTERSECT;
	  edf3= NOT_INTERSECT;
	}
	break;
      }
    if(num_of_edges == 0 && ede1+ede2+ede3 == 3){
      //exit(1);
      return 0;
    }
  }
  
  int num = edf1+edf2+edf3+ede1+ede2+ede3;
  if(num == 0){
    // cout << "no edge intersect" << endl;
    return 0;
  }
  else if(num > 2){
    printf("err of edge detection....");
    //exit(1);
    return 0;
  }

  vecNormalize(n1);
  vecNormalize(m1);

  /*********************************
    find intersection points
    **********************************/
  if(num==1){
    if(edf1==INTERSECT){
      find_intersection_pt(ip,q1,q2,nq[0],nq[1]);
      ip3 = ip;
      col_p->n_vector = -n1;
      col_p->depth = 0.0;
      col_p->c_type = FV;
    }
    else if(edf2==INTERSECT){
      find_intersection_pt(ip,q2,q3,nq[1],nq[2]);
      ip3 = ip;
      col_p->n_vector = -n1;
      col_p->depth = 0.0;
      col_p->c_type = FV;
    }
    else if(edf3==INTERSECT){
      find_intersection_pt(ip,q3,q1,nq[2],nq[0]);
      ip3 = ip;
      col_p->n_vector = -n1;
      col_p->depth = 0.0;
      col_p->c_type = FV;
    }
    else if(ede1==INTERSECT){
      find_intersection_pt(ip,p1,p2,mp[0],mp[1]);
      ip3 =  ip;
      col_p->n_vector = m1;
      col_p->depth = 0.0;
      col_p->c_type = VF;
    }
    else if(ede2==INTERSECT){
      find_intersection_pt(ip,p2,p3,mp[1],mp[2]);
      ip3 =  ip;
      col_p->n_vector = m1;
      col_p->depth = 0.0;
      col_p->c_type = VF;
    }
    else if(ede3==INTERSECT){
      find_intersection_pt(ip,p3,p1,mp[2],mp[0]);
      ip3 =  ip;
      col_p->n_vector = m1;
      col_p->depth = 0.0;
      col_p->c_type = VF;
    }
    col_p->num_of_i_points = 1;
  }
  else if(num==2)
    {
      if(edf1==INTERSECT && edf2==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "f1 f2" << endl;
	col_p->c_type = FV;
	if(!find_collision_info(q2,q1,q3,nq[1],nq[0],nq[2],p1,p2,p3,e1,e2,e3,
				m1,n1,ip3,ip4,ip5,ip6,col_p,-1.0))
	  return 0;
      }
      else if(edf1==INTERSECT && edf3==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "f1 f3" << endl;
	col_p->c_type = FV;
	if(!find_collision_info(q1,q2,q3,nq[0],nq[1],nq[2],p1,p2,p3,e1,e2,e3,
				m1,n1,ip3,ip4,ip5,ip6,col_p,-1.0))
	  return 0;
      }
      else if(ede1==INTERSECT && edf1==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "e1 f1" << endl;
	col_p->c_type = EE;
	if(!find_collision_info(p1,p2,p3,mp[0],mp[1],q1,q2,q3,nq[0],nq[1],ef11,
				n1,m1,ip3,ip4,col_p))
	  return 0;
      }
      else if(ede2==INTERSECT && edf1==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "e2 f1" << endl;
	col_p->c_type = EE;
	if(!find_collision_info(p2,p3,p1,mp[1],mp[2],q1,q2,q3,nq[0],nq[1],ef21,
                                n1,m1,ip3,ip4,col_p))
          return 0;
      }
      else if(ede3==INTERSECT && edf1==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "e3 f1" << endl;
	col_p->c_type = EE;
	if(!find_collision_info(p3,p1,p2,mp[2],mp[0],q1,q2,q3,nq[0],nq[1],ef31,
				n1,m1,ip3,ip4,col_p))
	  return 0;
      }
      else if(edf2==INTERSECT && edf3==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "f2 f3" << endl;
	col_p->c_type = FV;
        if(!find_collision_info(q3,q2,q1,nq[2],nq[1],nq[0],p1,p2,p3,e1,e2,e3,
                                m1,n1,ip3,ip4,ip5,ip6,col_p,-1.0))
          return 0;
      }
      else if(ede1==INTERSECT && edf2==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "e1 f2" << endl;
	col_p->c_type = EE;
	if(!find_collision_info(p1,p2,p3,mp[0],mp[1],q2,q3,q1,nq[1],nq[2],ef12,
				n1,m1,ip3,ip4,col_p))
	  return 0;
      }
      else if(ede2==INTERSECT && edf2==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "e2 f2" << endl;
	col_p->c_type = EE;
	if(!find_collision_info(p2,p3,p1,mp[1],mp[2],q2,q3,q1,nq[1],nq[2],ef22,
				n1,m1,ip3,ip4,col_p))
	  return 0;
      }
      else if(ede3==INTERSECT && edf2==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "e3 f2" << endl;
	col_p->c_type = EE;
	if(!find_collision_info(p3,p1,p2,mp[2],mp[0],q2,q3,q1,nq[1],nq[2],ef32,
				n1,m1,ip3,ip4,col_p))
	  return 0;
      }
      else if(ede1==INTERSECT && edf3==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "e1 f3" << endl;
	col_p->c_type = EE;
	if(!find_collision_info(p1,p2,p3,mp[0],mp[1],q3,q1,q2,nq[2],nq[0],ef13,
				n1,m1,ip3,ip4,col_p))
	  return 0;
      }
      else if(ede2==INTERSECT && edf3==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "e2 f3" << endl;
	col_p->c_type = EE;
	if(!find_collision_info(p2,p3,p1,mp[1],mp[2],q3,q1,q2,nq[2],nq[0],ef23,
				n1,m1,ip3,ip4,col_p))
	  return 0;
      }
      else if(ede3==INTERSECT && edf3==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "e3 f3" << endl;
	col_p->c_type = EE;
	if(!find_collision_info(p3,p1,p2,mp[2],mp[0],q3,q1,q2,nq[2],nq[0],ef33,
				n1,m1,ip3,ip4,col_p))
	  return 0;
      }
      else if(ede1==INTERSECT && ede2==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "e1 e2" << endl;
	col_p->c_type = VF;
        if(!find_collision_info(p2,p1,p3,mp[1],mp[0],mp[2],q1,q2,q3,f1,f2,f3,
                                n1,m1,ip3,ip4,ip5,ip6,col_p,1.0))
          return 0;
      }
      else if(ede1==INTERSECT && ede3==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "e1 e3" << endl;
	col_p->c_type = VF;
	if(!find_collision_info(p1,p2,p3,mp[0],mp[1],mp[2],q1,q2,q3,f1,f2,f3,
                                n1,m1,ip3,ip4,ip5,ip6,col_p,1.0))
          return 0;
      }
      else if(ede2==INTERSECT && ede3==INTERSECT){
	if(HIRUKAWA_DEBUG) cout << "e2 e3" << endl;
	col_p->c_type = VF;
	if(!find_collision_info(p3,p2,p1,mp[2],mp[1],mp[0],q1,q2,q3,f1,f2,f3,
                                n1,m1,ip3,ip4,ip5,ip6,col_p,1.0))
          return 0;
      }
    }    

  if(col_p->num_of_i_points == 1){
      col_p->i_points[0] = ip3 + P1;
  }
  else if(col_p->num_of_i_points == 2){
      col_p->i_points[0] = ip3 + P1;
      col_p->i_points[1] = ip4 + P1;
  }
  else if(col_p->num_of_i_points == 3){
      col_p->i_points[0] = ip3 + P1;
      col_p->i_points[1] = ip4 + P1;
      col_p->i_points[2] = ip5 + P1;
  }
  else if(col_p->num_of_i_points == 4){
      col_p->i_points[0] = ip3 + P1;
      col_p->i_points[1] = ip4 + P1;
      col_p->i_points[2] = ip5 + P1;
      col_p->i_points[3] = ip5 + P1;
  }

    col_p->n = n1;
    col_p->m = m1;
  if(HIRUKAWA_DEBUG){
    dvector3 p1w, p2w, p3w, q1w, q2w, q3w;
    p1w = CD_s2 * (vecProd(CD_Rot2, P1) + CD_Trans2);
    p2w = CD_s2 * (vecProd(CD_Rot2, P2) + CD_Trans2);
    p3w = CD_s2 * (vecProd(CD_Rot2, P3) + CD_Trans2);
    q1w = CD_s2 * (vecProd(CD_Rot2, Q1) + CD_Trans2);
    q2w = CD_s2 * (vecProd(CD_Rot2, Q2) + CD_Trans2);
    q3w = CD_s2 * (vecProd(CD_Rot2, Q3) + CD_Trans2);
    cout << "P1 = " << p1w[0] << " " << p1w[1] << " " << p1w[2] << endl;
    cout << "P2 = " << p2w[0] << " " << p2w[1] << " " << p2w[2] << endl;
    cout << "P3 = " << p3w[0] << " " << p3w[1] << " " << p3w[2] << endl;
    cout << "Q1 = " << q1w[0] << " " << q1w[1] << " " << q1w[2] << endl;
    cout << "Q2 = " << q2w[0] << " " << q2w[1] << " " << q2w[2] << endl;
    cout << "Q3 = " << q3w[0] << " " << q3w[1] << " " << q3w[2] << endl;

    for(int i=0; i<col_p->num_of_i_points; i++){
      i_pts_w[i] = CD_s2 * (vecProd(CD_Rot2, col_p->i_points[i]) + CD_Trans2);
      cout << i << "-th intersecting point = ";
      cout << i_pts_w[i][0] << " " << i_pts_w[i][1] << " " << i_pts_w[i][2] << endl;
    }

    cout << "n1 = " << n1[0] << " " << n1[1] << " " << n1[2] << endl;
    cout << "m1 = " << m1[0] << " " << m1[1] << " " << m1[2] << endl;
    cout << "mp[0] mp[1] mp[2] = " << mp[0] << " " << mp[1] << " " << mp[2] << endl;
    cout << "nq[0] nq[1] nq[2] = " << nq[0] << " " << nq[1] << " " << nq[2] << endl;
    cout << "n_vector = " << col_p->n_vector[0] << " " << col_p->n_vector[1]
	 << " " << col_p->n_vector[2] << endl;
    cout << "depth = " << col_p->depth << endl << endl;;

  }

#if TRACE1
  printf("intersect point...in tri_contact..\n");
  printf("    ip1x = %f ip1y = %f ip1z = %f\n    ip2x = %f ip2y = %f ip2z = %f\n", col_p->i_points[0][0],col_p->i_points[0][1],col_p->i_points[0][2], col_p->i_points[1][0],col_p->i_points[1][1],col_p->i_points[1][2]);

  printf("normal vector....it tri_conctact..\n");
  printf("N[0] = %f,N[1] = %f,N[2] = %f\n",col_p->n_vector[0],col_p->n_vector[1],col_p->n_vector[2]);
#endif

    return 1;
}

