#pragma once

#include "DynamicsSimulator.h"
#include "../MainLib/OgreFltk/objectList.h"

namespace OpenHRP
{
	class DynamicsSimulator_penaltyMethod : public DynamicsSimulator
	{
	public:
	private:
	public:
		TString _debugInfo; // only for debugging
		///////////////////////////////////////////
		// contact handling
		///////////////////////////////////////////
		std::vector<ContactForce> _externalForces;


		ObjectList mObjectContactVisualization;

		virtual void drawDebugInformation() ;

		DynamicsSimulator_penaltyMethod(bool useSimpleColdet=true);
		DynamicsSimulator_penaltyMethod(const char* coldet);
		virtual ~DynamicsSimulator_penaltyMethod(){}

		virtual void setSimulatorParam(const char* string, vectorn const& value);

		virtual void registerCollisionCheckPair
		(
		 const char* char1, 
		 const char* name1, 
		 const char* char2,
		 const char* name2,
		 vectorn const& param
		 );
	};
}
