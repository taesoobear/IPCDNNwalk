#ifndef _MOTION_WRAP_H
#define _MOTION_WRAP_H

#include "MotionDOF.h"
class Motion;
class MotionWrap;
class MotionLoader;
class BinaryFile;

struct PoseWrap
{
	MotionWrap* _src;
	int iframe;
	public:
	PoseWrap() { _src=NULL; iframe=-1;}
	PoseWrap(const MotionWrap * const p, int i) { _src=(MotionWrap*)p; iframe=i;}
};

class MotionDOFcontainer
{
	public:
		MotionDOF mot; // mot can be converted from and to an instance of Motion. See MotionDOF.h
		boolN discontinuity, conL, conR;
		MotionDOFcontainer(MotionDOFinfo const& info, const char* filename);
		MotionDOFcontainer(MotionDOFinfo const& info);
		MotionDOFcontainer(MotionDOF const& mot);

		// .dof file format
		void loadMotion(const char* fn);
		// .dof file format
		void saveMotion(const char* fn);
		void _importMot(BinaryFile& file);
		void _exportMot(BinaryFile& file); 

		void resize(int nframes);
		void concat(MotionDOF const& mot);
		inline const vectornView row(int i) const 	{ return mot.row(i);}
		inline vectornView row(int i) 				{ return mot.row(i);}
		int numFrames() const;
		bool isConstraint(int iframe, int con) const;
		void setConstraint(int iframe, int con, bool bSet=true);
		bool isContinuous(int startTime) const;
		bool isValid(int startTime, int endTime) const;
		bool isValid(int startTime) const;
};
//  a MotionWrap instance is either a Motion or a MotionDOF instance.
class MotionWrap 
{
	Motion* _mot;
	MotionDOFcontainer* _motdof;
	bool isReference;
	void _release();
	public:
	MotionWrap(); // new Motion
	MotionWrap(const Motion& mot);
	MotionWrap(const MotionDOF& mot);
	MotionWrap(const MotionDOFcontainer& mot);
	~MotionWrap();

	void setMotionDOF(MotionDOFinfo const& info);
	void setMotionDOF(MotionDOF& mot);
	void setMotion(Motion& mot) 		{ _release(); _mot=&mot;isReference=true;}
	TString getIdentifier() const;
	// automatic type conversions
	inline Motion& mot() 						{ if (!_mot || _motdof) throw(std::runtime_error("mot is NULL or motdof is not NULL!")); return *_mot;}
	inline Motion const& mot() const 			{ if (!_mot || _motdof) throw(std::runtime_error("mot is NULL or motdof is not NULL!")); return *_mot;}
	MotionDOF& motdof();
	MotionDOF const& motdof() const;
	MotionDOFcontainer const& motdofc() const;
	MotionDOFcontainer & motdofc() ;
	inline operator Motion const& () const 	{ return mot();}
	inline operator Motion & () 			{ return mot();}
	inline operator MotionDOF const& () const { return motdof();}
	inline operator MotionDOF& () 			{ return motdof();}
	inline bool hasMotionDOF() const 		{ return _motdof!=NULL;}
	int numFrames() const;
	inline int numFrames(float second) const	{ return ROUND(second/frameTime())+1;}
	float frameTime() const;
	PoseWrap pose(int i) const { return PoseWrap(this, i);}
	MotionLoader& skeleton() const;
	bool isConstraint(int iframe, int con) const;
	void setConstraint(int iframe, int con, bool bSet=true);
	bool isContinuous(int startTime) const;
	bool isValid(int startTime, int endTime) const;
	boolN getDiscontinuity() const;
};
#endif
