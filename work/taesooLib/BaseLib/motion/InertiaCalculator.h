#ifndef _INERTIA_CALC_H
#define  _INERTIA_CALC_H
#pragma once

namespace OBJloader
{
	class Mesh;
}
// calculate inertia from the convex hull of a mesh. 
// depends on Bullet
class InertiaCalculator
{
	void* _data;	// simplifies dependency.
public:
	InertiaCalculator(int precision=20);	// create 20 by 20 by 20 sampling volume.
	~InertiaCalculator();

	// assumes uniform density.
	
	void calculateFromFile(const char* objfilename);
	void calculateFromMesh(OBJloader::Mesh const &mesh);

	// for debugging purposes. (We know the right answer);
	void calculateFromBox(m_real sizeX, m_real sizeY, m_real sizeZ);	// sphere, cylinder, 
	void calculateFromCylinder(m_real radius, m_real height);

	// com will be at the origin when translate==0
	void drawSamplingGrid(m_real radius, vector3 const& translate);
	const vector3& centerOfMass() const;
	const matrix3& inertia() const;	// at the center of mass.
	m_real volume() const;	// == mass assuming uniform and unit density.
};

// works only for geometric primitives such as a Box
class InertiaCalculatorAnalytic
{
	void* _data;	// simplifies dependency.
	public:
	InertiaCalculatorAnalytic();	
	~InertiaCalculatorAnalytic();

	// assumes uniform density.
	void calculateFromMesh(OBJloader::Geometry const &mesh);

	// for debugging purposes. (We know the right answer);
	void calculateFromBox(m_real sizeX, m_real sizeY, m_real sizeZ);	// sphere, cylinder, 
	void calculateFromCylinder(m_real radius, m_real height);

	// com will be at the origin when translate==0
	const vector3& centerOfMass() const;
	const matrix3& inertia() const;	// at the center of mass.
	m_real volume() const;	// == mass assuming uniform and unit density.
};
#endif
