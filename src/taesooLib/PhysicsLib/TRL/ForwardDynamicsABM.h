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
/**
   \file
   \author S.NAKAOKA
*/

#ifndef TRL_FORWARD_DYNAMICS_ABM_H_INCLUDED
#define TRL_FORWARD_DYNAMICS_ABM_H_INCLUDED

#include <vector>
#include "Link.h"

template <class M>
void setMatrix33(const matrix33& m33, M& m, int row , int col ){
	m(row, col) = m33(0, 0); m(row, col+1) = m33(0, 1); m(row, col+2) = m33(0, 2);
	++row;
	m(row, col) = m33(1, 0); m(row, col+1) = m33(1, 1); m(row, col+2) = m33(1, 2);
	++row;
	m(row, col) = m33(2, 0); m(row, col+1) = m33(2, 1); m(row, col+2) = m33(2, 2);
}

template <class M>
void setTransMatrix33(const matrix33& m33, M& m, int row , int col ){
	m(row, col) = m33(0, 0); m(row, col+1) = m33(1, 0); m(row, col+2) = m33(2, 0);
	++row;
	m(row, col) = m33(0, 1); m(row, col+1) = m33(1, 1); m(row, col+2) = m33(2, 1);
	++row;
	m(row, col) = m33(0, 2); m(row, col+1) = m33(1, 2); m(row, col+2) = m33(2, 2);
}
namespace TRL
{
    class Body;
    typedef Body* BodyPtr;

    class LinkTraverse;

    /**
	   Forward dynamics calculation using Featherstone's Articulated Body Method (ABM)
    */
    class ForwardDynamicsABM {

    public:
        
        ForwardDynamicsABM(BodyPtr body);
		virtual ~ForwardDynamicsABM();
        
		void _updatePositionEuler();
		void _updatePositionAndVelocityEuler();
        virtual void initialize() { calcABMFirstHalf();}
        virtual void calcNextState();

        void setGravityAcceleration(const vector3& g);
        void setEulerMethod();
        void setTimeStep(double timeStep);
        
        inline void calcMotionWithEulerMethod()
		{
			// collect forces, and calculate accelerations
			calcABMLastHalf();
			_updatePositionAndVelocityEuler();
		}
		
		void calcAccelFK();
		// position and velocity FK 
		inline void calcABMPhase1()
		{
			calcPositionAndVelocityFK();
			addGravity();
		}
		// collect forces 
		void calcABMPhase2();
			// A part that can be calculated before external forces are given
			void calcABMPhase2Part1();
			// collect forces
			void calcABMPhase2Part2();
		// calculate accelerations
        void calcABMPhase3();

        inline void calcABMFirstHalf()
		{
			calcABMPhase1();
			calcABMPhase2Part1();
		}
		inline void calcABMLastHalf()
		{
			calcABMPhase2Part2();
			calcABMPhase3();
		}

		void calcPositionAndVelocityFK();
		inline void addGravity()
		{
			//const LinkTraverse& traverse = body->linkTraverse();
			for(int i=0, n=body->numLinks(); i < n; ++i){
				Link* link = body->link(i);
				link->Ivv .setValue( link->m, 0.0,     0.0, 0.0,     link->m, 0.0, 0.0,     0.0,     link->m);
				vector3 fg(-link->m * g);
				link->pf -= fg; link->ptau -= cross(link->wc, fg);
			}
		}

    protected:

		static void SE3exp(vector3& out_p, matrix33& out_R,
						   const vector3& p0, const matrix33& R0,
						   const vector3& w, const vector3& vo, double dt);
		
        BodyPtr body;
        vector3 g;
        double timeStep;

        enum { EULER_METHOD, RUNGEKUTTA_METHOD } integrationMode;
    };

    class ForwardDynamicsABM_rungeKutta : public ForwardDynamicsABM 
	{
		public:
        ForwardDynamicsABM_rungeKutta(BodyPtr body);
		virtual ~ForwardDynamicsABM_rungeKutta();

        virtual void calcNextState();
        void setRungeKuttaMethod();
		void integrateRungeKuttaOneStep(double r, double dt);
        void calcMotionWithRungeKuttaMethod();
        // Buffers for the Runge Kutta Method
		vector3 p0;
		matrix33 R0;
		vector3 vo0;
		vector3 w0;
		std::vector<double> q0;
		std::vector<double> dq0;
		
		vector3 vo;
		vector3 w;
		vector3 dvo;
		vector3 dw;
		std::vector<double> dq;
		std::vector<double> ddq;
	};
	
	typedef ForwardDynamicsABM* ForwardDynamicsPtr;
};

#endif
