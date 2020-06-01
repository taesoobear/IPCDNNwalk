#include "stdafx.h"
#include "mrdplot.hpp"
#include "mrdplot.h"
  
	
MRDplot::MRDplot(){}
MRDplot::~MRDplot(){}

// names and units by default will be set as CHANNEL%d, m, respectively. 
// You can always modify them later.
void MRDplot::initMRD(int n_channels, int numpoints, m_real freq)
{
	names.resize(n_channels);
	units.resize(n_channels);
	data.resize(numpoints, n_channels);
	setAllUnits("m");
	frequency=freq;
	for(int i=0; i<n_channels; i++)
		names[i].format("channel%d", i);
}

int MRDplot::numPoints() const
{
	return data.rows();
}

int MRDplot::numChannels() const
{
	return data.cols();
}

void MRDplot::setAllUnits(const char* unitname)
{
	for(int i=0; i<numChannels(); i++)
	{
		units[i]=unitname;
	}
}

void MRDplot::load(const char* filename)
{
	MRDPLOT_DATA* dd=read_mrdplot(filename);
	initMRD(dd->n_channels, dd->n_points, dd->frequency);

	for(int i=0; i<dd->n_channels; i++)
	{
		units[i]=dd->units[i];
		names[i]=dd->names[i];
	}

	for(int i=0; i<data.rows(); i++)
		for(int j=0; j<data.cols(); j++)
			data[i][j]=dd->data[i*data.cols()+j];
	free_mrdplot(dd);

}
void MRDplot::save(const char* filename) const
{
	MRDPLOT_DATA* dd=malloc_mrdplot_data(numChannels(), numPoints());

	for(int i=0; i<dd->n_channels; i++)
	{
		TString name=names[i];
		TString unit=units[i];
		name.replace(' ', '_');
		unit.replace(' ', '_');
		printf("%s\n", name.ptr());
		printf("%s\n", unit.ptr());
		dd->units[i]=strdup(name.ptr());
		dd->names[i]=strdup(unit.ptr());
	}

	for(int i=0; i<data.rows(); i++)
		for(int j=0; j<data.cols(); j++)
			dd->data[i*data.cols()+j]=(float)data[i][j];

	dd->filename=strdup(filename);
	dd->frequency=frequency;

	write_mrdplot_file(dd);
	free_mrdplot(dd);
}


void MRDplot::addPoint(const vectorn& vv)
{
	data.pushBack(vv);
}

void MRDplot::addChannel(const vectorn& vv, const char* name, const char* unit)
{
	if (vv.size()!=data.rows() && data.rows()!=0)
		throw std::runtime_error("addChannel not possible");

	data.resize(data.rows(), data.cols()+1);

	data.column(data.cols()-1).assign(vv);

	names.pushBack(name);
	units.pushBack(unit);

	if(names.size()!=units.size() ||
		names.size()!=numChannels())
		throw std::runtime_error("addChannel2");
}
