#pragma once

namespace MotionUtil
{
	void exportVRML(Motion const& mot, const char* filename, int start=0, int end=INT_MAX);
	void exportVRMLforRobotSimulation(Motion const& mot, const char* filename, const char* robotname, double cylinder_radius=0.025);
	void exportVRMLforRobotSimulation(MotionLoader const& skel, const char* filename, const char* robotname, double cylinder_radius=0.025);
}
