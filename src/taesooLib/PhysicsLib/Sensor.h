// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, AIST, the University of Tokyo and General Robotix Inc.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * The University of Tokyo
 * National Institute of Advanced Industrial Science and Technology (AIST)
 * General Robotix Inc. 
 */
#ifndef OPENHRP_SENSOR_HEADER
#define OPENHRP_SENSOR_HEADER

/**
   @file DynamicsSimulator/server/Sensor.h
 */

#include "hrpModelExportDef.h"
#include <string>
#include <iostream>
#include "fMatrix3.h"
#include "chain.h"
#ifdef USE_SENSOR
namespace OpenHRP {

	class Sensor
	{
	public:

		enum SensorType {
			COMMON = 0,
			FORCE,
			RATE_GYRO,
			ACCELERATION,
			PRESSURE,
			PHOTO_INTERRUPTER,
			VISION,
			TORQUE,
			NUM_SENSOR_TYPES
		};

		static const int TYPE = COMMON;
		
        Sensor(); 
        virtual ~Sensor();

        static Sensor* create(int type);
		static void destroy(Sensor* sensor);

		virtual void operator=(const Sensor& org);

        virtual void clear();
		
		TString name;
		int type;
		int id;
		Joint* joint;
		fMat33 localR;
		fVec3 localPos;

		virtual void putInformation(std::ostream& os);

	};


	class ForceSensor : public Sensor
	{
	public:
		static const int TYPE = FORCE;
		
        ForceSensor();
		fVec3 f;
		fVec3 tau;

        virtual void clear();
		virtual void putInformation(std::ostream& os);
	};


	class RateGyroSensor : public Sensor
	{
	public:
		static const int TYPE = RATE_GYRO;

        RateGyroSensor();
		fVec3 w;

        virtual void clear();
		virtual void putInformation(std::ostream& os);
	};


	class AccelSensor : public Sensor
	{
	public:
		static const int TYPE = ACCELERATION;

        AccelSensor();

		fVec3 dv;

        virtual void clear();
		virtual void putInformation(std::ostream& os);

	};

};

#endif
#endif
