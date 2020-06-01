// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
/** \file
	\author Shin'ichiro Nakaoka
*/

#ifndef TRL_BODY_H_INCLUDED
#define TRL_BODY_H_INCLUDED

#include <map>
#include <vector>
#include <ostream>
#include "../../BaseLib/math/matrix3.h"
typedef matrixn dmatrix;
typedef vectorn dvector;
typedef matrix3 matrix33;
inline void getVector3(vector3& out, matrix3 const& m, int i, int j){ ASSERT(i==0); out.x=m(0,j); out.y=m(1,j); out.z=m(2,j);}
inline void setVector3(vector3 const& v3, dmatrix & m, int row, int col){ m(row++, col) = v3.x; m(row++, col) = v3.y; m(row, col) = v3.z; }
inline void setVector3(vector3 const& v3, matrix3 & m, int row, int col){ m(row++, col) = v3.x; m(row++, col) = v3.y; m(row, col) = v3.z; }
template <class V> void setVector3(vector3 const& in, V & m, int i){m[i++]=in.x; m[i++]=in.y; m[i]=in.z;}
template <class V> void getVector3(vector3& v3, V const& v, int top){ v3.x = v[top++]; v3.y = v[top++]; v3.z = v[top]; }
matrix3 rodrigues(vector3 const& a, double q);
inline vector3 normalize(vector3 const& m){ vector3 temp; temp.normalize(m); return temp;}
inline matrix3 trans(matrix3 const& mm){matrix3 out=mm; out.transpose(); return out;}
inline matrix3 identity33() { matrix3 out; out.identity(); return out;}
inline matrix3 hat(vector3 const& c){matrix3 out; out.setTilde(c.x, c.y, c.z); return out;}
inline matrix3 VVt_prod(vector3 const& a, vector3 const& b){matrix3 out; out.setFromOuterProduct(a,b); return out;}
inline vector3 Mtx_prod(matrix3 const& a, vector3 const& b) { return a*b;}
inline vector3 cross(vector3 const& a, vector3 const& b) { return a.cross(b);}
inline double dot(vector3 const& a, vector3 const& b) { return a%b;}

#include "Link.h"


namespace TRL {
	class Body;
	class JointPath;
}


namespace TRL {


	class Link;
	
    class Body {

    public:


		std::string modelName;
		std::string name;
		std::vector<Link*> linkArray;

		Body();
		Body(const Body& org);

		virtual ~Body();

		void setRootLink(Link* link);

		/**
		   This function must be called when the structure of the link tree is changed.
		*/
		void updateLinkTree();

		/**
		   The number of the links that work as a joint.
		   Note that the acutal value is the maximum joint ID plus one.
		   Thus there may be a case where the value does not correspond
		   to the actual number of the joint-links.
		   In other words, the value represents the size of the link sequence
		   obtained by joint() function.
		*/
		inline int numJoints() const {
			return numLinks()-1;
		}

		/**
		   This function returns a link that has a given joint ID.
		   If there is no link that has a given joint ID,
		   the function returns a dummy link object whose ID is minus one.
		   The maximum id can be obtained by numJoints().
		*/
		inline Link* joint(int id) const {
			return link(id+1);
		}

		/**
		   The number of all the links the body has.
		   The value corresponds to the size of the sequence obtained by link() function.
		*/
		inline int numLinks() const {
			//return linkTraverse_.numLinks();
			return (int)linkArray.size();
		}

		/**
		   This function returns the link of a given index in the whole link sequence.
		   The order of the sequence corresponds to a link-tree traverse from the root link.
		   The size of the sequence can be obtained by numLinks().
		*/
		inline Link* link(int index) const {
			//return linkTraverse_.link(index);
			return linkArray[index];
		}
		inline Link* operator[](int index) const { return link(index);}

		/*
		   //LinkTraverse object that traverses all the links from the root link
		inline const LinkTraverse& linkTraverse() const {
			return linkTraverse_;
		}
		*/


		/**
		   The root link of the body
		*/
		inline Link* rootLink() const {
			return rootLink_;
		}


		/**
		   This function returns true when the whole body is a static, fixed object like a floor.
		*/
		inline bool isStatic() {
			return isStatic_;
		}

		double calcTotalMass();

		inline double totalMass() {
			return totalMass_;
		}

		vector3 calcCM();

		/*
		   The motion equation for calcMassMatrix()
		  |       |   | dv   |   |    |   | fext      |
		  | out_M | * | dw   | + | b1 | = | tauext    |
		  |       |   |ddq   |   |    |   | u         |
		*/
		void calcMassMatrix(dmatrix& out_M, dmatrix& b1, vector3 const& g );

		void setColumnOfMassMatrix(dmatrix& M, int column);

		void calcInverseDynamics(Link* link, vector3& out_f, vector3& out_tau);

		void calcTotalMomentum(vector3& out_P, vector3& out_L);

        void setDefaultRootPosition(const vector3& pos, const matrix33& att);

		void initializeConfiguration();

		void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false);

		void clearExternalForces();



		void putInformation(std::ostream &out);


		struct LinkConnection {
			Link* link[2];
			vector3 point[2];
			int numConstraintAxes;
			vector3 constraintAxes[3];
		};
		typedef std::vector<LinkConnection> LinkConnectionArray;

		LinkConnectionArray linkConnections;

		
	private:


		bool isStatic_;
		Link* rootLink_;

		typedef std::vector<Link*> LinkArray;

		//LinkTraverse linkTraverse_; // no longer in use.



		double totalMass_;

        vector3 defaultRootPosition;
        matrix33 defaultRootAttitude;



		void initialize();
		void setVirtualJointForcesSub();

    };

	typedef TRL::Body* BodyPtr;

};




std::ostream &operator<< (std::ostream& out, TRL::Body& body);


#endif
