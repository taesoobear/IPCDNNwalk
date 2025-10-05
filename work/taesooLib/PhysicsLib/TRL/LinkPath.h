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
    \brief The header file of the LinkPath and JointPath classes
    \author Shin'ichiro Nakaoka
*/

#ifndef TRL_LINK_PATH_H_INCLUDED
#define TRL_LINK_PATH_H_INCLUDED

#include <vector>
#include <ostream>
#include "LinkTraverse.h"
#include "Body.h"



namespace TRL {

	class LinkPath : public LinkTraverse
	{
	public:
		LinkPath();
		LinkPath(Link* root, Link* end);
		/// set path from the root link
		LinkPath(Link* end);
		
		/**
		   true when the path is not empty
		*/
		inline operator bool() const {
			return !links.empty();
		}

		inline Link* endLink() const {
			return links.back();
		}
		
		bool find(Link* root, Link* end);
		void findPathFromRoot(Link* end);
		
	private:
		bool findPathSub(Link* link, Link* prev, Link* end, bool isForwardDirection);
		void findPathFromRootSub(Link* link);
	};


	class JointPath : public LinkPath
    {
    public:
		
		JointPath();
		JointPath(Link* root, Link* end);
		/// set path from the root link
		JointPath(Link* end);
		virtual ~JointPath();
		
		bool find(Link* root, Link* end);
		bool findPathFromRoot(Link* end);
		
		inline int numJoints() const {
			return joints.size();
		}

		inline Link* joint(int index) const {
			return joints[index];
		}
		
		inline bool isJointDownward(int index) const {
			return (index >= numUpwardJointConnections);
		}
		
		void setMaxIKError(double e);
		
		void calcJacobian(dmatrix& out_J) const;
		void calcJacobian(dmatrix& out_J, vector3 const& targetPos) const;
		void calcJdot(dmatrix& out_J, vector3 const& targetPos) const;
		void calcAngularJacobian(dmatrix& out_J) const;
		
		inline dmatrix Jacobian() const {
			dmatrix J;
			calcJacobian(J);
			return J;
		}
		
		/**
		   @deprecated use operator<<
		*/
		void putInformation(std::ostream& os) const;
		
    protected:
		
		virtual void onJointPathUpdated();
		
		double maxIkErrorSqr;
		
    private:
		
		void initialize();
		void extractJoints();
		
		std::vector<Link*> joints;
		int numUpwardJointConnections;
    };

	
};


std::ostream& operator<<(std::ostream& os, TRL::LinkTraverse& traverse);


#endif
