#ifndef __MRDPLOT_HPP
#define __MRDPLOT_HPP


class MRDplot
{	
public:

	TStrings names;
	TStrings units;
	matrixn data;
	m_real frequency;
	MRDplot();
	virtual ~MRDplot();

	// names and units by default will be set as CHANNEL%d, m, respectively. 
	// You can always modify them later.
	void initMRD(int n_channels, int numpoints, m_real frequency);
	
	int numPoints() const;
	int numChannels() const;

	void setAllUnits(const char* unitname);

	void load(const char* filename);
	void save(const char* filename) const;

	void addPoint(const vectorn& vv);
	void addChannel(const vectorn& vv, const char* name, const char* unit);
};


#endif