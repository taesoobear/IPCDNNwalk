#pragma once

#include <math.h>
// numpy wrapper functions.
// see np::abs in luna_mainlib.lua also.
namespace np
{
	inline double mean(const vectorn & in){ return in.avg();}
	inline vectorn abs(const vectorn & in) { vectorn out(in.size()); for(int i=0,n=in.size(); i<n; i++) out[i]=ABS(in[i]); return out;}
	inline bool isnan(const vectorn & in) { for(int i=0,n=in.size(); i<n; i++) if(in[i]!=in[i]) return true; return false;}
	inline vectorn sqrt(const vectorn & in) { vectorn out(in.size()); for(int i=0,n=in.size(); i<n; i++) out[i]=::sqrt(in[i]); return out;}
	inline vectorn clip(const vectorn & in, double minv, double maxv) { vectorn out(in.size()); for(int i=0,n=in.size(); i<n; i++) out[i]=CLAMP(in[i], minv, maxv); return out;}
};
