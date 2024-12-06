#include "stdafx.h"
#include "Motion.h"
#include "MotionDOF.h"
#include "MotionLoader.h"
#include "MotionWrap.h"
//#include "../../MainLib/OgreFltk/RE.h"

static void extractConstraints(Motion const& mot, int type, boolN & discontinuity)
{
   discontinuity.resize(mot.numFrames());

   for(int i=0;i<mot.numFrames();i++)
      discontinuity.set(i, mot.isConstraint(i, type));
}
void MotionDOFcontainer::loadMotion(const char* fn)
{
	TString filename=fn;
	if( filename.right(3).toUpper()=="MOT" )
	{
		MotionDOFinfo const& info=mot.mInfo;
		Motion tempMot(&info.skeleton());
		info.skeleton().loadAnimation(tempMot, filename);

		extractConstraints(tempMot,IS_DISCONTINUOUS, discontinuity);
		extractConstraints(tempMot,CONSTRAINT_LEFT_FOOT, conL);
	    extractConstraints(tempMot,CONSTRAINT_RIGHT_FOOT, conR);
	    mot.set(tempMot);
	}
	else if(filename.right(3).toUpper()=="BVH")
	{
		printf("Error! BVH loading has not been ported to cpp yet\n");
		/*
		double dofScale=0.01;// -- millimeters to meters
		quater dofRot=quater(TO_RADIAN(-90), vector3(0,1,0))
		*quater(TO_RADIAN(-90), vector3(1,0,0));// -- change to Y_up

		MotionLoader* loader1=RE::motionLoader(filename);
		MotionLoader* loader2=RE::motionLoader(filename);
		rotateSkeleton(loader1, loader2,dofRot)

		//-- convert to meter (millimeters, Z_up)
		loader2->scale(dofScale);
	    mot.set(loader2.mMotion);
		*/
	}
	else
	{
	    BinaryFile binaryFile;
	    binaryFile.openRead(filename);
		_importMot(binaryFile);
	    binaryFile.close();
	}
}
MotionDOFcontainer::MotionDOFcontainer(MotionDOFinfo const& info, const char* fn)
	:mot(info)
{
	loadMotion(fn);
}
MotionDOFcontainer::MotionDOFcontainer(MotionDOFinfo const& info, const std::string& fn)
	:mot(info)
{
	loadMotion(fn.c_str());
}
MotionDOFcontainer::MotionDOFcontainer(MotionDOFinfo const& info)
	:mot(info)
{
}
MotionDOFcontainer::MotionDOFcontainer(MotionDOF const& _mot)
	:mot(_mot)
{
	resize(_mot.numFrames());
}

void MotionDOFcontainer::_importMot(BinaryFile& binaryFile)
{
	    int version=binaryFile.unpackInt();
	    resize(binaryFile.unpackInt());
	    int numDOF=binaryFile.unpackInt();

	    if( numDOF!=mot.mInfo.numDOF())
		{
	       printf("Warning! numDOF doesn't match! importing data to self.mat. %d, %d", numDOF, mot.mInfo.numDOF());
	       matrixn mat;
	       binaryFile.unpack(mat);
		}
	    else
	       binaryFile.unpack(mot._matView().lval());

	    binaryFile.unpack(discontinuity);
	    binaryFile.unpack(conL);
	    binaryFile.unpack(conR);
		if( version==2)
		{
			while (1)
			{
				TString k=binaryFile.unpackStr();
				if (k=="(end)") break ;
				matrixn mat;
				binaryFile.unpack(mat);
				printf("ignoring signal %s\n", k.ptr());
			}
		}
		else
			assert(version==1);
		if (conL.size() != mot.rows())
		{
			int n=mot.rows();
			conL.resize(n);
			conR.resize(n);
			discontinuity.resize(n);
		}
}
void MotionDOFcontainer::saveMotion(const char* fn)
{
	BinaryFile temp;
	temp.openWrite(fn);
	_exportMot(temp);
	temp.close();
}

void MotionDOFcontainer::_exportMot(BinaryFile& binaryFile)
{
	MotionDOFcontainer& self=*this;
	binaryFile.packInt(2);// -- version
	binaryFile.packInt(self.mot.numFrames());
	binaryFile.packInt(self.mot.mInfo.numDOF());
	binaryFile.pack(self.mot);;
	binaryFile.pack(self.discontinuity);
	binaryFile.pack(self.conL);
	binaryFile.pack(self.conR);
	binaryFile.pack("(end)");
}	
void MotionDOFcontainer::resize(int nframes)
{
   mot.resize(nframes);
   discontinuity.resize(nframes);
   conL.resize(nframes);
   conR.resize(nframes);
}
void MotionDOFcontainer::concat(MotionDOF const& mot)
{
}
int MotionDOFcontainer::numFrames() const
{
	return mot.numFrames();
}
bool MotionDOFcontainer::isConstraint(int iframe, int con) const
{
	if(con==CONSTRAINT_LEFT_FOOT)
		return conL(iframe);
	else if(con==CONSTRAINT_RIGHT_FOOT)
		return conR(iframe);
	return false;
}
void MotionDOFcontainer::setConstraint(int iframe, int con, bool bSet)
{
	if(con==CONSTRAINT_LEFT_FOOT)
		conL.set(iframe, bSet);
	else if(con==CONSTRAINT_RIGHT_FOOT)
		conR.set(iframe, bSet);
}
bool MotionDOFcontainer::isContinuous(int startTime) const
{
	return startTime<numFrames() && !discontinuity(startTime);
}
bool MotionDOFcontainer::isValid(int iframe) const
{
	return iframe>=0 && iframe<numFrames();
}

bool MotionDOFcontainer::isValid(int startFrame, int endFrame) const
{
	if(!isValid(startFrame)) return false;
	for(int i=startFrame+1; i<endFrame; i++)
	{
		if(!isValid(i)) return false;
		if(!isContinuous(i)) return false;
	}
	return true;
}

MotionWrap::MotionWrap()
{ 
	_motdof=NULL; _mot=new Motion(); isReference=false;
}
MotionWrap::MotionWrap(const Motion& mot)
{
	_motdof=NULL;
	_mot=(Motion*)&mot;
	isReference=true;
}

MotionWrap::MotionWrap(const MotionDOF& mot)
{
	_mot=NULL;
	_motdof=new MotionDOFcontainer(mot);
	isReference=false;
}
MotionWrap::MotionWrap(const MotionDOFcontainer& mot)
{
	_mot=NULL;
	_motdof=(MotionDOFcontainer*)&mot;
	isReference=true;
}
MotionWrap::~MotionWrap() 
{ 
	_release();
}
void MotionWrap::_release()
{
	if(!isReference) { delete _motdof; delete _mot;} 
	_motdof=NULL;
	_mot=NULL;
}

TString MotionWrap::getIdentifier() const
{
	if(_motdof)
	{
		return motdof().mInfo.skeleton().getName();
	}
	else
	{
		if(mot().GetIdentifier()==NULL)
			((Motion&)mot()).SetIdentifier("(unnamed)");
		return TString(mot().GetIdentifier());
	}
	return TString();
}

MotionLoader& MotionWrap::skeleton() const
{
	if(_motdof)
		return motdof().mInfo.skeleton();
	return mot().skeleton();
}
int MotionWrap::numFrames() const
{
	if(_motdof)
		return motdof().numFrames();
	return mot().numFrames();
}
void MotionWrap::setMotionDOF(MotionDOF& mot)	 	{ _release(); _motdof=new MotionDOFcontainer(mot);isReference=false;}
void MotionWrap::setMotionDOF(MotionDOFinfo const& info)	 	
{ 
	_release(); 
	_motdof=new MotionDOFcontainer(info);
	isReference=false;
}
MotionDOF& MotionWrap::motdof() 				{ if (!_motdof) throw (std::runtime_error("motdof is NULL!")); return _motdof->mot;}
MotionDOF const& MotionWrap::motdof() const 	{ if (!_motdof) throw (std::runtime_error("motdof is NULL!")); return _motdof->mot;}
MotionDOFcontainer const& MotionWrap::motdofc() const { if (!_motdof) throw (std::runtime_error("motdof is NULL!")); return *_motdof;}
MotionDOFcontainer & MotionWrap::motdofc() { if (!_motdof) throw (std::runtime_error("motdof is NULL!")); return *_motdof;}

bool MotionWrap::isConstraint(int iframe, int con) const
{
	if(!_motdof)
		return mot().isConstraint(iframe, con);
	else
		return motdofc().isConstraint(iframe, con);
}
void MotionWrap::setConstraint(int iframe, int con, bool bSet) 
{
	if(!_motdof)
		mot().setConstraint(iframe, con, bSet);
	else
		motdofc().setConstraint(iframe, con, bSet);
}
bool MotionWrap::isContinuous(int startTime) const
{
	if(!_motdof)
		return mot().IsContinuous(startTime);
	else
		return motdofc().isContinuous(startTime);
}
float MotionWrap::frameTime() const
{
	if(!_motdof)
		return mot().frameTime();
	else
		return 1.0/motdof().mInfo.mFrameRate;
}

bool MotionWrap::isValid(int startTime, int endTime) const
{
	if(!_motdof)
		return mot().IsValid(startTime, endTime);
	else
		return motdofc().isValid(startTime, endTime);
}
boolN MotionWrap::getDiscontinuity() const
{
	boolN t;
	t.resize(numFrames());
	for(int i=0; i<numFrames(); i++)
		t.set(i, !isContinuous(i)); return t;
}
