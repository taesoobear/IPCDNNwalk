
#include "stdafx.h"
#include "VRMLloader.h"
#include "MotionWrap.h"
#include "../utility/TextFile.h"
#include "../utility/BinaryFile.h"
#include "../utility/operatorString.h"
#include "../MainLib/OgreFltk/RE.h"
#include "VRMLloader_internal.h"
#include "VRMLexporter.h"
//#define TEST_MEMORYLEAK
using namespace std;

_HRP_JOINT::_HRP_JOINT()
	{
		jointType=HRP_JOINT::FREE;
		jointAxis="Z";
		jointStartId=-1;
		jointEndId=-1;
		jointAxis2 = NULL;
		jointRangeMin=-180.f;
		jointRangeMax=180.f;
	}
_HRP_JOINT::~_HRP_JOINT()
{
	delete[] jointAxis2;
}
bool _HRP_JOINT::_jointRangeIsLimited()
{
	if (jointRangeMin!=-180.f)
		return true;
	if (jointRangeMax!=180.f)
		return true;
	return false;
}
HRP_SEGMENT::HRP_SEGMENT()
{
	centerOfMass=vector3(0,0,0);
#ifdef ZERO_MASS_SUPPORTED
	mass=0;
	inertia.zero();
#else
	mass=0.001;
	momentsOfInertia.identity();
	momentsOfInertia*=0.00001;
#endif

	/*
	if (RE::ogreSceneManager()->getShadowTextureSelfShadow() )
		material="lightgrey_depthshadow";
	else
	*/
	material="lightgrey";
}

static void getShapeFN(TString& shapeFn, TString const& url, TString const& nameid)
{
	shapeFn=url;
	sz0::filename()(shapeFn);

	const bool useLocal=true;
	shapeFn=url.left(-4)+"_sd/"+nameid+".obj";
	shapeFn.replace(":", "_"); // : in a filename is not supported in a Windows system.
}
static void pack_HRP_SEGMENT(BinaryFile& bf, HRP_SEGMENT * mSegment, TString const& url,TString const& nameid, OBJloader::Geometry const & mesh )
{
	bf.packInt(1);
	vector3 & com=mSegment->centerOfMass;
	matrix3 & inertia=mSegment->momentsOfInertia;
	bf.pack(com);
	bf.packFloat(mSegment->mass);
	bf.packFloat(inertia._11);
	bf.packFloat(inertia._12);
	bf.packFloat(inertia._13);
	bf.packFloat(inertia._21);
	bf.packFloat(inertia._22);
	bf.packFloat(inertia._23);
	bf.packFloat(inertia._31);
	bf.packFloat(inertia._32);
	bf.packFloat(inertia._33);

	TString shapeFn;
	getShapeFN(shapeFn, url, nameid);
	bf.pack(shapeFn);
	mesh.pack(bf);
}
//////////////////////////////////////////////////////////////////////////
// Mesh Bone Implementation
//////////////////////////////////////////////////////////////////////////
VRMLTransform::VRMLTransform()
 :Bone()
{
  mJoint=NULL;
  mSegment=NULL;
  mShape=NULL;
  mTransform=NULL;
}

VRMLTransform::~VRMLTransform()
{
  delete mJoint;
  delete mSegment;
  delete mShape;
  delete mTransform;
}
void VRMLTransform::printHierarchy(int depth)
{
	for(int ii=0; ii<depth; ii++) Msg::print(" ");

	if (treeIndex()>=1 && getSkeleton().dofInfo._sharedinfo)
	{
		Msg::print("%s : %d, R: %d, T: %d %s:%s ", NameId, treeIndex(), rotJointIndex(), transJointIndex(), getRotationalChannels().ptr(), getTranslationalChannels().ptr() );
		Msg::print("startT: %d endR: %d ",getSkeleton().dofInfo.startT(treeIndex()), getSkeleton().dofInfo.endR(treeIndex()));
		Msg::print("jointId: %d numJoints:%d ", HRPjointIndex(0), numHRPjoints());
		Msg::print("offset: %s \n",getOffsetTransform().translation.output().c_str());
	}

	for(Node *i=m_pChildHead; i!=NULL; i=i->m_pSibling)
		i->printHierarchy(depth+1);
}
static void unexpectedToken(CTextFile& file, TString const& token, const char* msg)
{
	  Msg::msgBox("unexpected token %s! %s. next 10 lines will be printed to console.", token.ptr(),msg);
	  for (int i=0; i<10; i++)
		  Msg::print("%s\n", file.GetLine());
	  ASSERT(0);
}

vector3 VRMLTransform::localCOM() const
{
  VRMLTransform const& bone=*this;
  if(bone.mSegment)
    {
      return bone.mSegment->centerOfMass;
    }
  return vector3(0,0,0);
}

TString Geometry_packShape(OBJloader::Geometry const& _mesh, const char* dir, const char* shapeFn, transf const& toGlobal);


void VRMLTransform::translateMesh( vector3 const& trans)
{ 
	VRMLTransform* bone=this;
	matrix4 m;
	m.setTranslation(trans, false);
	bone->mShape->mesh.scaleAndRigidTransform(m);
	bone->mSegment->centerOfMass+=trans;
}

void VRMLTransform::setJointPosition(vector3 const& trans)
{
	VRMLTransform* bone=this;
	bone->_getOffsetTransform().translation=trans;
	bone->initBones();
}
void VRMLTransform::translateBone(vector3 const& trans)
{
	VRMLTransform* bone=this;
	bone->_getOffsetTransform().translation+=trans;
	bone->initBones();
}

// m: with respect to global coordinate.
void VRMLTransform::transformMesh(matrix4 const& mm)
{
	// gpos= T * lpos
	// lpos'= invT * mm * T * lpos
	matrix4 m=mm;
	VRMLTransform* bone=this;
	m.rightMult(matrix4(bone->getFrame()));
	m.leftMult(matrix4(bone->getFrame().inverse()));
	bone->mShape->mesh.scaleAndRigidTransform(m);
	bone->mSegment->centerOfMass.leftMult(m);

}
void VRMLTransform::transformMeshLocal(matrix4 const& m)
{
	VRMLTransform* bone=this;
	if (bone->mShape)
		bone->mShape->mesh.scaleAndRigidTransform(m);
	bone->mSegment->centerOfMass.leftMult(m);
}

// without scaling mass
void VRMLTransform::scaleMesh( vector3 const& scale)
{
	VRMLTransform& bone=*this;
	if(bone.mSegment) {
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				bone.mSegment->momentsOfInertia.m[i][j]*=scale[i]*scale[j];
			}
		}
		if(bone.mShape)
			bone.mShape->mesh.scale(scale);
		bone.mSegment->centerOfMass.x*=scale.x;
		bone.mSegment->centerOfMass.y*=scale.y;
		bone.mSegment->centerOfMass.z*=scale.z;
	}
}

// without scaling volume
void VRMLTransform::scaleMass( m_real scalef)
{
	VRMLTransform& bone=*this;
	if(bone.mSegment)
	{
		bone.mSegment->mass*=scalef;
		bone.mSegment->momentsOfInertia*=scalef;
	}
}
void VRMLTransform::setLocalCOM(vector3 const& com)
{
  VRMLTransform& bone=*this;
  if(bone.mSegment)
    {
      bone.mSegment->centerOfMass=com;
	}
}

double VRMLTransform::mass()
{
  VRMLTransform& bone=*this;
  if(bone.mSegment)
    {
      return bone.mSegment->mass;
    }
  return 0;
}

void VRMLTransform::setMass(double m)
{
	VRMLTransform& bone=*this;
	if(bone.mSegment)
	{
		bone.mSegment->mass=m;
	}
}
vector3 VRMLTransform::inertia() const
{
  VRMLTransform const& bone=*this;
  if(bone.mSegment)
    {
      return vector3(
		     bone.mSegment->momentsOfInertia._11,
		     bone.mSegment->momentsOfInertia._22,
		     bone.mSegment->momentsOfInertia._33);
    }

  return vector3(0,0,0);
}

matrix3 const& VRMLTransform::momentsOfInertia() const
{
	return mSegment->momentsOfInertia;
}
void VRMLTransform::setInertia(double ix, double iy, double iz)
{
	VRMLTransform& bone=*this;
	if(bone.mSegment)
	{
		bone.mSegment->momentsOfInertia._11=ix;
		bone.mSegment->momentsOfInertia._22=iy;
		bone.mSegment->momentsOfInertia._33=iz;
	}
}
static TString nameId(Bone& bone)
{
  static int curSite=0;
  TString out(bone.NameId);

  if(TString("Site")==bone.NameId)
    {
      out.add("%d", curSite++);
    }
  return out;
}

static TString space(int level)
{
  TString temp;
  temp.empty();
  temp.add("\n");
  for(int i=0; i<level; i++)
    temp.add("  ");
  return temp;
}

void VRMLTransform::pack(BinaryFile& bf)
{
	int version=1; // VRMLtransform binary format version 1. (the second version)
	bf.packInt(version);
	bf.pack(nameId(*this));

	bf.packInt((int)mJoint->jointType);
	bf.pack(mJoint->jointAxis);
	if(mJoint->jointAxis2)
	{
		bf.packInt(1);
		bf.packInt(mJoint->AxisNum);
		for(int i=0;i<mJoint->AxisNum;i++)
		{
			//bf.pack(mJoint->jointAxis2Angle[i]);
			bf.packFloat(0.0);
			bf.pack(mJoint->jointAxis2[i]);
		}
	}
	else
		bf.packInt(0);

	if(mJoint->_jointRangeIsLimited())
	{
		bf.packInt(2); // field code.
		bf.packFloat(mJoint->jointRangeMin);
		bf.packFloat(mJoint->jointRangeMax);
	}

	if(version>=1)
		bf.packInt(111);// seperating token for future addition of fields.

	bf.pack(getOffsetTransform().translation);

	if(mSegment)
    {
		if(mShape)
			pack_HRP_SEGMENT(bf, mSegment,((VRMLloader*)&(getSkeleton()))->url ,nameId(*this), mShape->mesh);
		else
		{
			OBJloader::Geometry g;
			pack_HRP_SEGMENT(bf, mSegment,((VRMLloader*)&(getSkeleton()))->url ,nameId(*this), g);
		}
		if(version>=1)
		{
			if(mSegment->material.length()>0)
			{
				bf.packInt(1);
				bf.pack(mSegment->material);
			}
			bf.packInt(222);// seperating token for future addition of fields.
		}
	}
	else
		bf.packInt(0);

	if(version>=1)
		bf.packInt(333);// seperating token for future addition of fields.

	if(m_pChildHead)
    {
		bf.packInt(1);
		for(Bone* node=(Bone*)m_pChildHead; node!=NULL; node=(Bone*)(node->m_pSibling))
		{
			bf.packInt(1);
			((VRMLTransform*)node)->pack( bf);
		}
		bf.packInt(0);
    }
	else
		bf.packInt(0);

	bf.pack(nameId(*this));
}

void VRMLTransform::setJointRange(int i, double min_deg, double max_deg)
{
	Msg::verify(mJoint->jointType==HRP_JOINT::ROTATE, "joint range of ball, universal, or free joints are not supporte yet");
	Msg::verify(i==0, "hinge joint has only one dof");

	mJoint->jointRangeMin=min_deg;
	mJoint->jointRangeMax=max_deg;
}
void VRMLTransform::pack(FILE* file, int level)
{
  TString transform;
  TString sp=space(level);

  fprintf(file, "%sDEF %s Joint %s{%s", sp.ptr(), nameId(*this).ptr(),sp.ptr(),sp.ptr());

  switch(mJoint->jointType)
    {
    case HRP_JOINT::FREE:
      fprintf(file,"  jointType \"free\"%s",sp.ptr());
      break;
	case HRP_JOINT::BALL:
      fprintf(file,"  jointType \"ball\"%s",sp.ptr());
      break;
    case HRP_JOINT::ROTATE:
      fprintf(file,"  jointType \"rotate\"%s",sp.ptr());
      break;
    case HRP_JOINT::SLIDE:
      fprintf(file,"  jointType \"slide\"%s",sp.ptr());
      break;
    case HRP_JOINT::GENERAL:
      fprintf(file,"  jointType \"general\"%s",sp.ptr());
      break;
    case HRP_JOINT::FIXED:
      fprintf(file,"  jointType \"fixed\"%s",sp.ptr());
      break;
    }

  fprintf(file,"  jointAxis \"%s\"%s", mJoint->jointAxis.ptr(),sp.ptr());

  if(mJoint->jointAxis2)
  {
	  fprintf(file,"  jointAxis2");
	  //Msg::print("length():%d\n",mJoint->AxisNum);
	  for(int i=0;i<mJoint->AxisNum;i++)
	  {
		  //fprintf(file," %f %f %f %f",mJoint->jointAxis2Angle[i],mJoint->jointAxis2[i].x,mJoint->jointAxis2[i].y,mJoint->jointAxis2[i].z);
		  fprintf(file," %f %f %f %f",0.0,mJoint->jointAxis2[i].x,mJoint->jointAxis2[i].y,mJoint->jointAxis2[i].z);
	  }
	  fprintf(file,"%s",sp.ptr());
  }

  if(mJoint->_jointRangeIsLimited())
  {
	  fprintf(file,"  jointRange %f %f%s", mJoint->jointRangeMin, mJoint->jointRangeMax, sp.ptr());
  }

  vector3 const & translation=getOffsetTransform().translation;
  fprintf(file,"  translation %f %f %f%s", translation.x, translation.y, translation.z,sp.ptr());

  fprintf(file,"  children [");

  if(mSegment)
    {
      TString shape;
      vector3 com=mSegment->centerOfMass;
      matrix3 & inertia=mSegment->momentsOfInertia;
      shape.add("\nSegment \n{\n  centerOfMass %f %f %f\n  mass %f\n  momentsOfInertia [%f %f %f %f %f %f %f %f %f]\n", 
		com.x, com.y, com.z, 
		mSegment->mass, 
		inertia._11, inertia._12, inertia._13,
		inertia._21, inertia._22, inertia._23,
		inertia._31, inertia._32, inertia._33);
	  if(mSegment->material.length()>0)
		  shape.add("  material \"%s\"\n", mSegment->material.ptr());
		
      TString url=((VRMLloader*)&(getSkeleton()))->url;
	  TString shapeFn;
	  getShapeFN(shapeFn, url, nameId(*this));
      const bool useLocal=false;

	  if (mShape)
		  shape.concat(Geometry_packShape(mShape->mesh, url.left(-4)+"_sd/",shapeFn, getFrame()));

	  shape.add("\n}");

      shape.op0(sz0::replace("\n", space(level+2)));
      fprintf(file, "%s", shape.ptr());
    }

  if(m_pChildHead)
    {
      //if(level==0)
      //packAnimation(bone, file, level+1, mot, start, end);
		

      for(Bone* node=(Bone*)m_pChildHead; node!=NULL; node=(Bone*)(node->m_pSibling))
	{
	  ((VRMLTransform*)node)->pack( file, level+2);
	}
    }

  transform.add("\n  ]\n");
  transform.add("} # %s\n", nameId(*this).ptr());
  transform.op0(sz0::replace("\n", space(level)));
  fprintf(file, "%s", transform.ptr());
}

static void getVec3(CTextFile& file, vector3& out)
{
  for(int i=0; i<3; i++)
    out[i]=atof(file.GetToken());
}

static void getVec4(CTextFile& file, vector4& out)
{
  for(int i=0; i<4; i++)
    out[i]=atof(file.GetToken());

  vector3 temp;
  temp.x=out[0];
  temp.y=out[1];
  temp.z=out[2];
  /*
  temp.normalize();
  out[0]=temp.x;
  out[1]=temp.y;
  out[2]=temp.z;*/
}

static void getMat3(CTextFile& file, matrix3& out)
{
	TString temp;
	temp=file.GetToken();
  VERIFY(temp=="[");
  for(int i=0; i<9; i++)
    out[i]=atof(file.GetToken());
	temp=file.GetToken();
  VERIFY(temp=="]");
}

static void unpackExposedField(CTextFile& file, TString& name, vectorn& out)
{
  TString token;
  token=file.GetToken();

  if(token=="SFFloat" || token=="SFInt32")
    {
      name=file.GetToken();
      out.setSize(1);
      out[0]=atof(file.GetToken());
    }
  else if(token=="SFVec3f")
    {
      name=file.GetToken();
      out.setSize(3);
      for(int i=0; i<3; i++)
	out[i]=atof(file.GetToken());
    }
  else if(token=="SFNode")
    {
      file.GetToken();
      file.GetToken();
    }
  else if(token=="SFRotation")
    {
      name=file.GetToken();
      out.setSize(4);
      for(int i=0; i<4; i++)
	out[i]=atof(file.GetToken());
    }
  else if(token=="MFFloat" || token=="MFNode"||token=="MFString")
    {
      name=file.GetToken();
      token=file.GetToken();
      out.resize(0);
		
      if(token=="[")
	{
	  while(1)
	    {
	      token=file.GetToken();

	      if(token=="]")
		break;
	      else
		out.pushBack(atof(token));
	    }
	}
      else if(token!="[]")
	file.Undo();
    }
  else if(token=="SFString")
    {
      name=file.GetToken();
      token=file.GetQuotedText();
      out.resize(token.length());
      for(int i=0; i<token.length(); i++)
	out[i]=(double)token[i];
    }
  else {
	  unexpectedToken(file, token, "SF* or MF* were expected.");
  }
}
static void skipNode(CTextFile& file, const char* brace="{}")
{
  char t1[2];
  t1[0]=brace[0];
  t1[1]=0;
  char t2[2];
  t2[0]=brace[1];
  t2[1]=0;
	
  TString token=file.GetToken();
	
  int count=0;
  if(token==t1)
    count++;
  while(1)
    {
      token=file.GetToken();
      if(token==t1)
	count++;
      else if(token==t2)
	{
	  count--;
	  if(count==0)
	    return;
	}
      else if(token.length())
	continue;
      else
	Msg::error("EOF");
    }
}
VRMLloader::~VRMLloader()
{

#ifdef TEST_MEMORYLEAK
	
  Msg::msgBox("dtor loader %s", url.ptr());
#endif 	


	
}
void VRMLloader::operator=(VRMLloader const& other)
{

	MemoryFile m;
	other._exportBinary(m);
	_importBinary(m);
}

void VRMLloader::insertChildJoint(Bone& parent, const char* trans_channels, const char* rot_channels, const char* nameId, bool bMoveChildren)
{
	vector3 offset(0,0,0);
	insertChildJoint( parent,  trans_channels,  rot_channels,  nameId, bMoveChildren,  offset);
}
void VRMLloader::insertChildJoint(Bone& parent, const char* trans_channels, const char* rot_channels, const char* nameId, bool bMoveChildren, const vector3& offset)
{
	VRMLTransform* newChild=new VRMLTransform();

	newChild->mVRMLtype="Joint";
	newChild->mJoint=new _HRP_JOINT();
	newChild->_getOffsetTransform().translation=offset;

	TString tch(trans_channels);
	TString rch(rot_channels);
	if (tch.length()==3 && rch.length()==3)
	  {
		newChild->mJoint->jointType=HRP_JOINT::FREE;
	  }
	else if(tch.length()==0 )
	{
		newChild->mJoint->jointType=HRP_JOINT::ROTATE;
		newChild->mJoint->jointAxis=rot_channels;
	}
	else if(rch.length()==0)
	{
		newChild->mJoint->jointType=HRP_JOINT::SLIDE;
		newChild->mJoint->jointAxis=trans_channels;
	}
	else
	  Msg::error("not yet supported");

	newChild->initBones();
	newChild->SetNameId(nameId);

	std::list<Node*> children;
	if(bMoveChildren)
		parent.detachAllChildren(children);

	parent.AddChild(newChild);

	if(bMoveChildren)
	  {
		newChild->addChildren(children);
		VRMLTransform* tt=(VRMLTransform*)&parent;
		newChild->mSegment=tt->mSegment;
		newChild->mShape=tt->mShape;
		tt->mSegment=NULL;
		tt->mShape=NULL;
	  }


	_initDOFinfo();
}
void VRMLloader::setChannels(Bone& bone, const char* translation_axis, const char* rotation_axis)
{
	VRMLTransform* b=(VRMLTransform*)&bone;
	Msg::verify(b->mJoint, "Error! this VRMLTransform has no joint");

	if(strlen(translation_axis)==0)
	{
		b->mJoint->jointType=HRP_JOINT::ROTATE;
		b->mJoint->jointAxis=TString(rotation_axis);
	}
	else if(bone.treeIndex()==1 && strlen(translation_axis)==3 && strlen(rotation_axis)==3)
	{
		b->mJoint->jointType=HRP_JOINT::FREE;
	}
	else
	{
		b->mJoint->jointType=HRP_JOINT::GENERAL;
		b->mJoint->jointAxis=TString(translation_axis)+"_"+TString(rotation_axis);
	}

	b->setChannels(translation_axis, rotation_axis);
	_initDOFinfo();
}
static HRP_JOINT::jointType_T _stringToJointType(TString const& type)
{
  if(type=="free")
    return HRP_JOINT::FREE;
  else if(type=="ball")
	  return HRP_JOINT::BALL;
  else if(type=="rotate")
    return HRP_JOINT::ROTATE;
  else if(type=="slide")
    return HRP_JOINT::SLIDE;
  else if(type=="general")
    return HRP_JOINT::GENERAL;
  else if(type=="fixed")
    return HRP_JOINT::FIXED;
  else 
  {
	  Msg::error("unknown joint type %s\n", type.ptr());
	  ASSERT(0);
  }

  return HRP_JOINT::FREE;
}
void VRMLTransform::unpack(VRMLloader& l, BinaryFile & bf)
{
	int version=bf.unpackInt(); // for binary backward compatibility.
	Msg::verify(version==0 || version==1, "version error");
	
	SetNameId(bf.unpackStr());
	mVRMLtype="Joint";
	mJoint=new _HRP_JOINT();
	mJoint->jointType=(HRP_JOINT::jointType_T)bf.unpackInt();
	bf.unpack(mJoint->jointAxis);
	if (bf.unpackInt()==1)
	{
		mJoint->AxisNum=bf.unpackInt();
		mJoint->jointAxis2 = new vector3[mJoint->AxisNum];
		//mJoint->jointAxis2Angle = new m_real[mJoint->AxisNum];
		for(int i=0;i<mJoint->AxisNum;i++)
		{
			//mJoint->jointAxis2Angle[i]=bf.unpackFloat();
			double unused=bf.unpackFloat();
			if (unused!=0.0)
				Msg::print("warning! non-zero default angle is no longer supported!\n");
			bf.unpack(mJoint->jointAxis2[i]);
		}
	}

	if(version>=1)
	{
		while(1){
			int sep=bf.unpackInt();
			if(sep==111)
				break;

			if(sep==2)
			{
				mJoint->jointRangeMin=bf.unpackFloat();
				mJoint->jointRangeMax=bf.unpackFloat();
			}
		}
	}

	vector3 joint_translation;
	bf.unpack(joint_translation);
	_getOffsetTransform().translation=joint_translation ;

	if(bf.unpackInt())
	{
		mSegment=new HRP_SEGMENT();
		vector3 &com=mSegment->centerOfMass;
		matrix3 & inertia=mSegment->momentsOfInertia;
		bf.unpack(com);
		mSegment->mass=bf.unpackFloat();
		inertia._11=bf.unpackFloat();
		inertia._12=bf.unpackFloat();
		inertia._13=bf.unpackFloat();
		inertia._21=bf.unpackFloat();
		inertia._22=bf.unpackFloat();
		inertia._23=bf.unpackFloat();
		inertia._31=bf.unpackFloat();
		inertia._32=bf.unpackFloat();
		inertia._33=bf.unpackFloat();

		mShape=new HRP_SHAPE();
		TString shapefn=bf.unpackStr();
		mShape->mesh.unpack(bf);

		if(mShape->mesh.numVertex()==0)
		{
			delete mShape;
			mShape=NULL;
		}
		if(version>=1)
		{
			int sep=bf.unpackInt();
			if(sep==1)
			{
				bf.unpack(mSegment->material);
				sep=bf.unpackInt();
			}
			Msg::verify(sep==222,"incorrect seperating token for future addition of fields.");
		}
	}
	if(version>=1)
		Msg::verify(bf.unpackInt()==333,"incorrect seperating token for future addition of fields.");

	if(bf.unpackInt())
	{
		while(bf.unpackInt())
		{
			VRMLTransform* child=new VRMLTransform();
			child->unpack(l, bf);
			AddChild(child);
		}
	}
	Msg::verify(bf.unpackStr()==nameId(*this), "VRMLtransform unpack binary");
}
void VRMLTransform::Unpack(VRMLloader& l, CTextFile& file)
{
  TString token=file.GetToken();
#ifdef _DEBUG
  Msg::print("token: %s", token.ptr());
#endif
  if(token=="DEF")
    {
      SetNameId(file.GetToken());
      token=file.GetToken();
    }
#ifdef _DEBUG
  Msg::print(" %s %s\n", NameId, token.ptr());
#endif

  mVRMLtype=token;
  if(token=="Joint")
    {
      token=file.GetToken();
      ASSERT(token=="{");
      mJoint=new _HRP_JOINT();

      while(1)
	{
	  token=file.GetToken();
#ifdef _DEBUG
	  Msg::print("Joint token: %s", token.ptr());
#endif
	  if(token=="translation")
	  {
		  vector3 joint_translation;
	    getVec3(file,joint_translation);
		_getOffsetTransform().translation=joint_translation;
	  }
	  else if (token=="jointRange" )
	  {
		  mJoint->jointRangeMin=atof(file.GetToken());
		  mJoint->jointRangeMax=atof(file.GetToken());
	  }
	  else if(token=="jointType")
	    mJoint->jointType=_stringToJointType(file.GetQuotedText());
	  else if(token=="jointAxis")
	    mJoint->jointAxis=file.GetQuotedText();
	  else if(token=="axis")
	  {
		  if (!mJoint->jointAxis2)
		  {
			  mJoint->jointAxis2 = new vector3[3];
			  //mJoint->jointAxis2Angle = new m_real[3];
			  mJoint->AxisNum = 1;
			  mJoint->jointAxis="A";
		  }
		  else
		  {
			  mJoint->AxisNum++;
			  mJoint->jointAxis=mJoint->jointAxis+"A";
		  }
		  int i=mJoint->AxisNum-1;
		  //mJoint->jointAxis2Angle[i] = 0.0;
		  getVec3(file,mJoint->jointAxis2[i]);
	  }
	  else if(token=="jointAxis2")//if(token=="jointAxis")에 넣는게 나을 것 같 다.
	 	{
			// deprecated. use "axis" instead.
			mJoint->AxisNum = mJoint->jointAxis.length();
			mJoint->jointAxis2 = new vector3[mJoint->AxisNum];
			//mJoint->jointAxis2Angle = new m_real[mJoint->AxisNum];
			for(int i=0;i<mJoint->AxisNum;i++)				
			{
				double unused=atof(file.GetToken());
				if (unused!=0.0)
					Msg::print("warning! non-zero default angle is no longer supported!\n");
				//mJoint->jointAxis2Angle[i] = 
		  	    getVec3(file,mJoint->jointAxis2[i]);
			}
		}
	  else if(token=="jointId")
	    {
	      int jointId_unused=atoi(file.GetToken());
	    }
	  else if(token=="children")
	    UnpackChildren(l, file);
	  else if(token=="rotation")
	  {
		  vector4 jr;
		  getVec4(file,jr);
		  _getOffsetTransform().rotation.setRotation(vector3(jr.x(), jr.y(), jr.z()), jr.w());
	  }
	  else if(token=="translation")
	  {
	    getVec3(file,_getOffsetTransform().translation);
	  }
	  else if(token=="}")
	    return;
	  else unexpectedToken(file, token, "children, rotation, translation, } .. are expected");
	};
    }
  else if(token=="AccelerationSensor")
    skipNode(file);
  else if(token=="Gyro")
    skipNode(file);
  else if(token=="VisionSensor")
    skipNode(file);
  else if(token=="Segment")
    {
      token=file.GetToken();
#ifdef _DEBUG
	  Msg::print("segment_token: %s", token.ptr());
#endif
      ASSERT(token=="{");
      mSegment=new HRP_SEGMENT();
      while(1)
	{
	  token=file.GetToken();
	  if(token=="centerOfMass")
	    getVec3(file,mSegment->centerOfMass);
	  else if(token=="mass")
	    mSegment->mass=atof(file.GetToken());
	  else if(token=="momentsOfInertia")
	    getMat3(file, mSegment->momentsOfInertia);
	  else if(token=="children")
	    UnpackChildren(l,file);
	  else if(token=="material")
	    mSegment->material=file.GetQuotedText();
	  else if(token=="}")
	    return;
	  else unexpectedToken(file, token, "children, material,}... expected");
	};
    }
  else if(token=="Transform")
    {
      token=file.GetToken();
      ASSERT(token=="{");
      mTransform=new VRML_TRANSFORM();
      while(1)
	{
	  token=file.GetToken();
	  if(token=="rotation")
	    getVec4(file,mTransform->rotation);
	  else if(token=="translation")
	    getVec3(file,mTransform->translation);
	  else if(token=="scale")
	    getVec3(file, mTransform->scale);
	  else if(token=="children")
	    UnpackChildren(l,file);
	  else if(token=="}")
	    return;
	  else unexpectedToken(file, token, "translation, scale, children, },... expected");
	};
    }
  else if(token=="Shape")
    {
      token=file.GetToken();
      ASSERT(token=="{");
      mShape=new HRP_SHAPE();
      while(1)
	{
	  token=file.GetToken();
	  if(token=="appearance")
	    {
	      token=file.GetToken();
	      if(token=="DEF")
		{
		  token=file.GetToken();// name
		  token=file.GetToken();// Appearance
		}
				

	      if(token=="Appearance")
		skipNode(file);
	      else if(token=="USE")
		{
		  token=file.GetToken();// use name
		}
	      else unexpectedToken(file, token, "??");
	    }
	  else if(token=="geometry")
	  {
		  TString geometryType=file.GetToken();

		  float r=1.f,g=1.f,b=1.f, a=0.f;
		  bool hasColor=false;

		  if(geometryType=="Box" || geometryType=="Ellipsoid" || geometryType=="Plane")
		  {
			  token=file.GetToken();
			  ASSERT(token=="{");
			  token=file.GetToken();
			  ASSERT(token=="size");
			  vector3 size;
			  size.x=atof(file.GetToken());
			  size.y=atof(file.GetToken());
			  size.z=atof(file.GetToken());

			  //Msg::print("Box size %s\n", size.output().ptr());
			  token=file.GetToken();

			  if (token=="color")
			  {
				  hasColor=true;
				  r=atof(file.GetToken());
				  g=atof(file.GetToken());
				  b=atof(file.GetToken());
				  a=atof(file.GetToken());

				  token=file.GetToken();
			  }
			  ASSERT(token=="}");

			  if(geometryType=="Box")
				  mShape->mesh.initBox(vector3(size.x, size.y, size.z));
			  else if(geometryType=="Plane")
				  mShape->mesh.initPlane(size.x, size.z);
			  else
				  mShape->mesh.initEllipsoid(vector3(size.x, size.y, size.z));
		  }
		  else if(geometryType=="Cylinder" || geometryType=="Capsule")
		  {
			  token=file.GetToken();
			  ASSERT(token=="{");
			  token=file.GetToken();
			  double radius;
			  double height;
			  if (token=="radius")
			  {
				  radius=atof(file.GetToken());
				  token=file.GetToken();
				  if(token!="height") unexpectedToken(file, token, "height expected");
				  height=atof(file.GetToken());
			  }
			  else  unexpectedToken(file, token, "radius expected");
			  token=file.GetToken();
			  int numDivision=10;
			  while (token!="}")
			  {
				  if(token=="numDivision") 
				  {
					  numDivision=atoi(file.GetToken());
				  }
				  else if (token=="color")
				  {
					  hasColor=true;
					  r=atof(file.GetToken());
					  g=atof(file.GetToken());
					  b=atof(file.GetToken());
					  a=atof(file.GetToken());
				  }
				  else
					  unexpectedToken(file, token, "numDivision expected");

				  token=file.GetToken();
			  }

			  if (geometryType=="Cylinder")
				  mShape->mesh.initCylinder( radius, height, numDivision);
			  else
				  mShape->mesh.initCapsule( radius, height);
		  }
		  else if(geometryType=="OBJ" || geometryType=="OBJ_no_classify_tri")
		  {
			  token=file.GetQuotedText();

			  try{
				  mShape->mesh.loadObj(token);
			  }
			  catch(std::runtime_error& e)
			  {
				  // try with relative path.
				  TString wrlPath=sz1::parentDirectory(l.url);
#ifdef _DEBUG
				  Msg::print("%s %s\n", wrlPath.ptr(), token.ptr());
#endif
				  try{
					  mShape->mesh.loadObj(wrlPath+token);
				  }
				  catch(std::runtime_error& e)
				  {
					  TString dir;
					  TString fn=sz1::filename(token, dir);
					  TString tryfn=l.url.left(-4)+"_sd/"+fn;
#ifdef _DEBUG
					  Msg::print("%s\n", tryfn.ptr());
#endif
					  if(!mShape->mesh.loadObj(tryfn))
						  Msg::error("%s not found", token.ptr());
				  }
			  }
			  mShape->mesh.calculateVertexNormal();
			  if (geometryType!="OBJ_no_classify_tri")
				  mShape->mesh.classifyTriangles();
		  }
		  else skipNode(file);

		  // also set colors
		  auto& mesh=mShape->mesh;
		  int numColor=1;
		  mesh.resize(mesh.numVertex(), mesh.numNormal(), mesh.numTexCoord(), numColor, mesh.numFace());
		  mesh.getColor(0).x()=r;
		  mesh.getColor(0).y()=g;
		  mesh.getColor(0).z()=b;
		  mesh.getColor(0).w()=a;
		  for(int f=0; f<mesh.numFace(); f++){
			  auto& ff=mesh.getFace(f);
			  ff.setIndex(0,0,0, OBJloader::Buffer::COLOR);
		  }
	  }
	  else if(token=="}")
	    return;
	  else unexpectedToken(file, token, "geometry, }, ... expected");
	};
    }
  else if(token=="Humanoid")
    {
      token=file.GetToken();
      ASSERT(token=="{");
      while(1)
	{
	  token=file.GetToken();
#ifdef _DEBUG
	  Msg::print("humanoid token: %s\n", token.ptr());
#endif
	  
	  if(token=="name")
	    l.name=file.GetQuotedText();
	  else if(token=="url")
	    l.url=file.GetQuotedText();
	  else if (token=="frameRate")
	  {
		  l._frameRate=atof(file.GetToken());
		  //Msg::print("frameRate::%f\n", l._frameRate);
	  }
	  else if(token=="version")
	    l.version=file.GetQuotedText();
	  else if(token=="info")
	    {
	      token=file.GetToken();
	      ASSERT(token=="[");
	      while(1)
		{
		  token=file.GetToken();
		  if(token=="]")
		  {
#ifdef _DEBUG
			  Msg::print("info last line: %s\n", l.info.back().ptr());
#endif
		    break;
		  }
		  else
		    {
		      file.Undo();
		      token=file.GetQuotedText();
#ifdef _DEBUG
			  Msg::print("info line: %s\n", token.ptr());
#endif
		      l.info.pushBack(token);
		    }
		}
	    }
	  else if(token=="joints")
	    skipNode(file, "[]");
	  else if(token=="segments")
	    skipNode(file, "[]");
	  else if(token=="humanoidBody")
	    UnpackChildren(l,file);
	  else if(token=="}")
	    return;
	  else
		unexpectedToken(file, token, "joints, segments, humanoidBody,... expected");
	}
    }
  else unexpectedToken(file, token, "Shape, humanoid, ... expected");
}

static void VRML_TRANSFORM_identity(VRML_TRANSFORM* t)
{
	t->translation=vector3(0,0,0);
	t->rotation=vector4(0,0,1,0);
	t->scale=vector3(1,1,1);
}
VRML_TRANSFORM::VRML_TRANSFORM()
{
	VRML_TRANSFORM_identity(this);
}
// only for parsing
void VRML_TRANSFORM_setTransform(VRML_TRANSFORM* t, matrix4& mat)
{
  mat.setScaling(t->scale.x, t->scale.y, t->scale.z);

  mat.leftMultRotation(
		       vector3(t->rotation.x(),t->rotation.y(),t->rotation.z()),
		       t->rotation.w());
  mat.leftMultTranslation(t->translation);
}

// only for parsing
static void VRML_JOINT_setTransform(VRML_TRANSFORM* transform, Bone* bone)
{

  ASSERT(isSimilar(transform->scale.x,1.0));
  ASSERT(isSimilar(transform->scale.y,1.0));
  ASSERT(isSimilar(transform->scale.z,1.0));
  matrix4 temp;
  VRML_TRANSFORM_setTransform(transform, temp);

  bone->_getOffsetTransform()=temp;
}

void VRMLTransform::setJointAxes(const char* axes)
{
  if(mVRMLtype=="Joint")
  {
	  if(mJoint->jointType==HRP_JOINT::ROTATE ||
			  mJoint->jointType==HRP_JOINT::BALL||
			  mJoint->jointType==HRP_JOINT::FIXED)
	  {
		  if (strlen(axes)==0)
			  mJoint->jointType=HRP_JOINT::FIXED;
		  else
			  mJoint->jointType=HRP_JOINT::ROTATE;
		  mJoint->jointAxis=axes;
		  setChannels("", mJoint->jointAxis);//
		  return;
	  }
	  else if(mJoint->jointType==HRP_JOINT::SLIDE)
	  {
		  if (strlen(axes)==0)
			  mJoint->jointType=HRP_JOINT::FIXED;
		  else
			  mJoint->jointAxis=axes;
		  
		  setChannels( mJoint->jointAxis, "");//
		  return;
	  }
	  else if(mJoint->jointType==HRP_JOINT::FREE)
	  {
		  if (strlen(axes)==0)
		  {
			  mJoint->jointType=HRP_JOINT::FIXED;
			  setChannels( "", "");//
		  }
		  else
		  {
			  Msg::msgBox("Error! cannot change axes (case1 - not implemented yet). %s:%s:%d", NameId , mVRMLtype.ptr(), mJoint->jointType);
		  }
	  }
  }
  else
	  Msg::msgBox("Error! cannot change axes. %s:%s:%d", NameId , mVRMLtype.ptr(), mJoint->jointType);
}
void VRMLloader::changeAll3DOFjointsToSpherical()
{
	Posture pose;
	getPose(pose);
	for(int i=1; i<numBone(); i++)
	{
		auto& bone=VRMLbone(i);
		if( bone.mJoint->jointType==HRP_JOINT::ROTATE && bone.getRotationalChannels().length()==3)
			bone.mJoint->jointType=HRP_JOINT::BALL;
	}

	_initDOFinfo(); 
	setPose(pose);

}
void VRMLloader::changeAllMultiDOFjointsToSpherical()
{
	Posture pose;
	getPose(pose);
	for(int i=1; i<numBone(); i++)
	{
		auto& bone=VRMLbone(i);
		if( bone.mJoint->jointType==HRP_JOINT::ROTATE && bone.getRotationalChannels().length()>1)
			bone.mJoint->jointType=HRP_JOINT::BALL;
	}

	_initDOFinfo(); 
	setPose(pose);
}

void VRMLloader::changeAllJointsToSpherical()
{
	Posture pose;
	getPose(pose);
	for(int i=1; i<numBone(); i++)
	{
		auto& bone=VRMLbone(i);
		if( bone.mJoint->jointType==HRP_JOINT::ROTATE )
			bone.mJoint->jointType=HRP_JOINT::BALL;
	}

	_initDOFinfo(); 
	setPose(pose);
}
void VRMLTransform::initBones()
{
  if(mVRMLtype=="Joint")
    {
      ASSERT(mJoint);
      if(mJoint->jointType==HRP_JOINT::FREE)
	{
		if(getRotationalChannels().length()!=3)
			m_rotChannels="ZYX";

	  setChannels("XYZ", getRotationalChannels());
	}
	  else if (mJoint->jointType==HRP_JOINT::BALL)
	  {
		if(getRotationalChannels().length()!=3)
			m_rotChannels="ZYX";
		  setChannels("",getRotationalChannels());
	  }
      else if(mJoint->jointType==HRP_JOINT::ROTATE)
	{
		if(mJoint->jointAxis.findChar(0,'A') != -1)
		{
		  setChannels("", mJoint->jointAxis);
		  setArbitraryAxes(mJoint->jointAxis2);
		  //if( mJoint->jointAxis2Angle!=0)

		}
		else
		  setChannels("", mJoint->jointAxis);//
	}
      else if(mJoint->jointType==HRP_JOINT::SLIDE)
	{
	  setChannels(mJoint->jointAxis, "");
	}
      else if(mJoint->jointType==HRP_JOINT::GENERAL)
	{
	  int index=mJoint->jointAxis.findChar(0, '_');
	  setChannels(mJoint->jointAxis.left(index), mJoint->jointAxis.right(mJoint->jointAxis.length()-index-1));
	}
      else
	{
	  ASSERT(mJoint->jointType==HRP_JOINT::FIXED);
	  setChannels("","");
	}
    }
  else if(mVRMLtype=="Transform")
    {
      ASSERT(mTransform);
      VRML_JOINT_setTransform(mTransform, this);
      setChannels("","");
    }

  for(Node::child_iterator i=begin(); i!=end(); ++i)
    {
      ((VRMLTransform*)(*i))->initBones();
    }

}

static VRMLTransform* findTransform(VRMLTransform* parent, TString const& what)
{
  if(what==parent->NameId && parent->mVRMLtype!="USE")
    return parent;
  if(parent->mShape && parent->mShape->name==what)
    return parent;

  if(parent->child())
    {
      VRMLTransform* res;
      res=findTransform((VRMLTransform*)parent->child(), what);
      if(res) return res;
    }

  if(parent->sibling())
    return findTransform((VRMLTransform*)parent->sibling(), what);

  return NULL;
};

static VRMLTransform* copyTransform(VRMLTransform* other)
{
  VRMLTransform *out=new VRMLTransform ();
  out->copyFrom(*other);
  out->SetNameId(TString("copy_of_")+other->NameId);

  for(Node* c=other->m_pChildHead; c; c=c->m_pSibling)
    out->AddChild(copyTransform((VRMLTransform*)c));

  return out;
};

static void mergeMesh(VRMLTransform* parent, VRMLTransform* child)
{
  if(child->mShape)
    {
      if(parent->mShape)
	{
	  parent->mShape->mesh.merge(
				     parent->mShape->mesh, 
				     child->mShape->mesh);
	}
      else
	{
	  parent->mShape=child->mShape;
	  child->mShape=NULL;
	}
    }
}
static void mergeShapes_subroutine(VRMLTransform* node, matrix4 transform)
{
  ASSERT(node->mTransform);
	
  matrix4 transf;
  VRML_TRANSFORM_setTransform(node->mTransform,transf);
  transform*=transf;

  if(node->mShape)
    node->mShape->mesh.scaleAndRigidTransform(transform);		
	
  VRML_TRANSFORM_identity(node->mTransform);

  for(VRMLTransform* child=(VRMLTransform*)node->m_pChildHead;
      child; 
      child=(VRMLTransform*)child->m_pSibling)
    {
      mergeShapes_subroutine(child, transform);
      mergeMesh(node, child);
    }
  std::list<Node*> children;
  node->detachAllChildren(children);
  for(std::list<Node*>::iterator i=children.begin(); i!=children.end(); ++i)
    delete *i;

	
}

static void makeIdentityTransform(VRMLTransform* node)
{
  if(node->mVRMLtype=="Transform")
    {
      matrix4 t;
      t.setIdentityRot();
      mergeShapes_subroutine(node, t);
		
      VRMLTransform* parent=(VRMLTransform*)node->parent();
    }
  else
    {
      if(node->m_pChildHead)
	makeIdentityTransform((VRMLTransform*)node->m_pChildHead);
    }
	
  if(node->m_pSibling)
    makeIdentityTransform((VRMLTransform*)node->m_pSibling);
}

static bool VRML_TRANSFORM_isIdentity(VRML_TRANSFORM *t)
{
  return t->translation==vector3(0,0,0) &&
    t->rotation.w()==0 &&
    t->scale==vector3(1,1,1);
}

static void removeTransform(VRMLTransform* node)
{
  if(node->child())
    for(Bone* child=node->child(); child; )
      {
	VRMLTransform* c=(VRMLTransform*) child;
	if(c->mVRMLtype=="Transform")
	  {
	    ASSERT(!c->mShape || VRML_TRANSFORM_isIdentity(c->mTransform));

	    mergeMesh(node, (VRMLTransform* )child);
			
	    Bone* rchild=child;
	    child=child->sibling();

	    node->RemoveChild(rchild);
	  }
	else
	  child=child->sibling();
      }


  if(node->child())
    for(Bone* child=node->child(); child;child=child->sibling() )
      removeTransform((VRMLTransform*)child);
}
static void mergeShapes(VRMLTransform* rootnode)
{
  makeIdentityTransform(rootnode);
  removeTransform(rootnode);
}

static void printHierarchy(VRMLTransform* node, int depth)
{
  for(int ii=0; ii<depth; ii++) Msg::print(" ");

  if(node->mShape)
    Msg::print("%s (%s)-shape %s jointid %d\n", node->NameId, node->mVRMLtype.ptr(), node->mShape->name.ptr(), node->HRPjointIndex(0));
  else
    Msg::print("%s (%s) jointid %d\n", node->NameId, node->mVRMLtype.ptr(), node->HRPjointIndex(0));

  for(Node *i=node->m_pChildHead; i!=NULL; i=i->m_pSibling)
    printHierarchy((VRMLTransform*)i,depth+1);
}

static void replaceUseNode(VRMLTransform* node, VRMLTransform* root)
{
  if(node->m_pChildHead)
    replaceUseNode((VRMLTransform*)node->m_pChildHead, root);

  if(node->m_pSibling)
    replaceUseNode((VRMLTransform*)node->m_pSibling, root);

  if(node->mVRMLtype=="USE")
    {
      VRMLTransform* found=findTransform(root, node->NameId);
      ASSERT(found);

      if(TString(node->NameId)==found->NameId)	// use transform.
	{
	  VRMLTransform* child=copyTransform(found);
	  Node* parent=node->m_pParent;
	  parent->RemoveChild(node);
	  parent->AddChild(child);
	}
      else	// use shape
	{
	  VRMLTransform* parent=(VRMLTransform* )node->m_pParent;
	  parent->RemoveChild(node);
			
	  ASSERT(parent->mShape==NULL);

	  parent->mShape=new HRP_SHAPE();
	  *parent->mShape=*found->mShape;
	}
    }
}
bool VRMLTransform::hasShape() const {return mShape!=NULL;}
OBJloader::Geometry& VRMLTransform::getMesh() const { Msg::verify(hasShape(), "error! no shape!!!");return mShape->mesh;}
void VRMLTransform::createNewShape()
{
	delete mShape;
	mShape=new HRP_SHAPE();
}
void VRMLTransform::removeShape()
{
	delete mShape;
	mShape=NULL;
}
void VRMLTransform::UnpackChildren(VRMLloader& l, CTextFile& file)
{
  TString token=file.GetToken();
  int multipleChildren=true;

  if(token!="[")
    multipleChildren=false;
  else
    token=file.GetToken();	


  while(1)
    {
      if(token=="USE")
	{
	  token=file.GetToken();//use what..
	  VRMLTransform* child;
	  child=new VRMLTransform();
	  child->mVRMLtype="USE";
	  child->SetNameId(token);
	  AddChild(child);
	}
      else
	{
	  file.Undo();	// so that the TYPE identifier can be read in the Unpack function.

	  VRMLTransform* child;
	  child=new VRMLTransform();
	  child->Unpack(l,file);

	  if(child->mVRMLtype=="Joint"||child->mVRMLtype=="Transform")
	    AddChild(child);
	  else if(child->mVRMLtype=="Segment")
	    {
	      ASSERT(numChildren()==0);
	      ASSERT(mSegment==0);
	      ASSERT(mShape==0 || child->mShape==0);
	      mSegment=child->mSegment;
	      if(child->mShape)
		mShape=child->mShape;

	      std::list<Node*> children;
	      child->detachAllChildren(children);

	      addChildren(children);
	    }
	  else if(child->mVRMLtype=="Shape")
	    {
	      //				ASSERT(numChildren()==0);
	      ASSERT(child->numChildren()==0);
	      ASSERT(mShape==0);
	      mShape=child->mShape;
		  child->mShape=NULL;
	      mShape->name=child->NameId;
		  delete child;
	    }
	  else if(child->mVRMLtype=="Humanoid")
	    {
	      ASSERT(0);
	    }
	  else
	    {
	      ASSERT(child->numChildren()==0);
	      delete child;
	    }
	}
      if(!multipleChildren)
	break; 
      token=file.GetToken();
      if(token=="]")
	break;
    }
}

int VRMLTransform::numHRPjoints() const
{
  if(mJoint)
    return mJoint->jointEndId-mJoint->jointStartId;
  return 0;
}
int VRMLTransform::HRPjointIndex(int i) const
{
  if(!mJoint) return -1;

  return mJoint->jointStartId+i;
}

int VRMLTransform::DOFindex(int i) const
{
  if(!mJoint) return -1;

  VRMLloader* skel=(VRMLloader* )&getSkeleton();
  return skel->dofInfo.startT(treeIndex())+i;
}
int VRMLTransform::DQindex(int i) const
{
  if(!mJoint) return -1;

  VRMLloader* skel=(VRMLloader* )&getSkeleton();
  return skel->dofInfo.startDQ(treeIndex())+i;
}

HRP_JOINT::jointType_T VRMLTransform::HRPjointType(int i) const
{
  if(mJoint->jointType==HRP_JOINT::GENERAL)
    {
      if(i<getTranslationalChannels().length())
	return HRP_JOINT::SLIDE;
      else
	return HRP_JOINT::ROTATE;
    }
  return mJoint->jointType;
}

TString VRMLTransform::HRPjointAxis(int i) const
{
	if (i==-1)
		return mJoint->jointAxis;

  if(mJoint->jointType==HRP_JOINT::GENERAL)
    {
      int tl=getTranslationalChannels().length();
      if(i<tl)
	return mJoint->jointAxis.subString(i,i+1);
      else
	return getRotationalChannels().subString(i-tl,i-tl+1);
    }
  return mJoint->jointAxis.subString(i,i+1);
}

TString VRMLTransform::HRPjointName(int i) const
{
  int numJoints=numHRPjoints();
  if(!numJoints) return TString ("(NULL)");

  if(numJoints==1 || i==numJoints-1)
    return TString (NameId);

  TString f;
  f.format("%s_%d", NameId, i);
  return f;
}

// convert a position in joint local frame to body local frame.
void VRMLTransform::jointToBody(vector3& lposInout) const
{
  lposInout-=mSegment->centerOfMass;
}
// convert a position in body local frame to joint local frame.
void VRMLTransform::bodyToJoint(vector3& lposInout) const
{
  lposInout+=mSegment->centerOfMass;
}


int VRMLloader::numHRPjoints()   	
{ 
	return VRMLbone(numBone()-1).mJoint->jointEndId;
}

VRMLloader::VRMLloader(VRMLloader const& other)
	:MotionLoader()
{
	_frameRate=30;
	_terrain=NULL;
	MemoryFile m;
	other._exportBinary(m);
	_importBinary(m);
	url=other.url;
}
VRMLloader::VRMLloader()
 :MotionLoader()
{
	_frameRate=30;
	_terrain=NULL;
	m_pTreeRoot=new VRMLTransform();
	m_pTreeRoot->SetNameId("HumanoidBody");
	VRMLTransform* root=new VRMLTransform();
	MemoryFile m;

	// VRMLTransform::pack
	m.packInt(0);
	m.pack("ROOT");
	m.packInt(HRP_JOINT::FREE);
	m.pack("Z");
	m.packInt(0);
	m.pack(vector3(0,0,0)); //joint->translation
	OBJloader::Geometry g;
	g.initEllipsoid(vector3(0.1,0.1,0.1)); // 10cm sphere

	url.format("FREEBODY_%s", RE::generateUniqueName().ptr());
	HRP_SEGMENT mSegment;
	pack_HRP_SEGMENT(m, &mSegment, url, TString("root"), g);
	m.packInt(0);
	m.pack("ROOT");
	root->unpack(*this, m);
	m_pTreeRoot->AddChild(root);
	_initDOFinfo(); 
}
VRMLloader::VRMLloader(MotionLoader const& skel, double cylinder_radius)
	:MotionLoader()
{
	_terrain=NULL;
	// TODO: remove file IO.
	MotionUtil::exportVRMLforRobotSimulation(skel, "__temp.wrl",RE::generateUniqueName().ptr(), cylinder_radius);
	CTextFile file;
	if(!file.OpenReadFile("__temp.wrl"))
		Msg::msgBox("error opening %s", "__temp.wrl");
	_importVRML(file);
	dofInfo.setFrameRate(skel.dofInfo.frameRate());
}
void VRMLloader::_clear()
{
	_terrain=NULL;
	if(m_pTreeRoot) delete m_pTreeRoot;
	m_pTreeRoot=NULL;
	delete dofInfo._sharedinfo;
	dofInfo._sharedinfo=NULL;
}
VRMLloader::VRMLloader(OBJloader::Geometry const& mesh, bool useFixedJoint)
	:MotionLoader()
{
	_frameRate=30;
	_terrain=NULL;

	m_pTreeRoot=new VRMLTransform();
	m_pTreeRoot->SetNameId("floor");
	VRMLTransform* root=new VRMLTransform();
	MemoryFile m;

	// VRMLTransform::pack
	m.packInt(0);
	m.pack("WAIST");
	if(useFixedJoint)
	{
		m.packInt(HRP_JOINT::FIXED);
		m.pack("");
	}
	else
	{
		m.packInt(HRP_JOINT::FREE);
		m.pack("Z");
	}
	m.packInt(0);
	m.pack(vector3(0,0,0)); //joint->translation

	url.format("BODY_%s", RE::generateUniqueName().ptr());
	HRP_SEGMENT mSegment;
	pack_HRP_SEGMENT(m, &mSegment, url, TString("WAIST"), mesh);
	m.packInt(0);
	m.pack("WAIST");
	root->unpack(*this, m);
	m_pTreeRoot->AddChild(root);
	_initDOFinfo(); 
	name=RE::generateUniqueName();
	url=name+".wrl";

	_initDOFinfo();
	//VRMLloader_updateMeshEntity(*l);
}
VRMLloader::VRMLloader(OBJloader::Terrain* terrain)
	:MotionLoader()
{
	_frameRate=30;
	_terrain=terrain;

	m_pTreeRoot=new VRMLTransform();
	m_pTreeRoot->SetNameId("floor");
	VRMLTransform* root=new VRMLTransform();
	MemoryFile m;

	// VRMLTransform::pack
	m.packInt(0);
	m.pack("WAIST");
	m.packInt(HRP_JOINT::FIXED);
	m.pack("");

	m.packInt(0);
	m.pack(vector3(0,0,0)); //joint->translation

	url.format("BODY_%s", RE::generateUniqueName().ptr());
	HRP_SEGMENT mSegment;
	OBJloader::Geometry mesh;
	mesh.assignTerrain(*terrain, vector3(0,0,0));
	pack_HRP_SEGMENT(m, &mSegment, url, TString("WAIST"), mesh);
	m.packInt(0);
	m.pack("WAIST");
	root->unpack(*this, m);
	m_pTreeRoot->AddChild(root);
	_initDOFinfo(); 
	name=RE::generateUniqueName();
	url=name+".wrl";

	_initDOFinfo();
	//VRMLloader_updateMeshEntity(*l);
}
VRMLloader::VRMLloader(const char* filename)
 :MotionLoader()
{
  _frameRate=30;
	_terrain=NULL;
#ifdef TEST_MEMORYLEAK
  Msg::msgBox("loader%s", filename);
#endif 

  url=filename;
  if(!(url.right(4).toUpper()==".WRL"))
  {
	  BinaryFile bf(false, filename);
	  _importBinary(bf);
	  return;
  }
  CTextFile file;
  if(!file.OpenReadFile(filename))
    Msg::msgBox("error opening %s", filename);
  _importVRML(file);
}
VRMLloader::VRMLloader(const std::string & filename)
 :MotionLoader()
{
  _frameRate=30;
	_terrain=NULL;
#ifdef TEST_MEMORYLEAK
  Msg::msgBox("loader%s", filename.c_str());
#endif 

  url=filename.c_str();
  if(!(url.right(4).toUpper()==".WRL"))
  {
	  BinaryFile bf(false, filename.c_str());
	  _importBinary(bf);
	  return;
  }
  CTextFile file;
  if(!file.OpenReadFile(filename.c_str()))
    Msg::msgBox("error opening %s", filename.c_str());
  _importVRML(file);
}
VRMLloader::VRMLloader(CTextFile& vrmlFile)
{
	_frameRate=30;
	_terrain=NULL;
	url="CTextFile";
	_importVRML(vrmlFile);
}


void VRMLloader::_initDOFinfo()
{
	((VRMLTransform*)m_pTreeRoot)->initBones();
	int numChannel;
	MakeBoneArrayFromTree(numChannel);

	int curJoint=0;
	for(int i=1;i<numBone(); i++)
	{
		VRMLTransform* ll=((VRMLTransform*)&getBoneByTreeIndex(i));

		ASSERT(ll->mJoint);
		// override joint index.
		ll->mJoint->jointStartId=curJoint;
		if(ll->mJoint->jointType==HRP_JOINT::FREE	  ||
				ll->mJoint->jointType==HRP_JOINT::FIXED  ||
				ll->mJoint->jointType==	HRP_JOINT::BALL)
			curJoint++;
		else 
		{
			curJoint+=ll->getRotationalChannels().length();
			curJoint+=ll->getTranslationalChannels().length();
		}

		ll->mJoint->jointEndId=curJoint;
	}

	// DOF information
	bitvectorn useSpherical;
	useSpherical.resize(numBone());
	useSpherical.clearAll();
	for(int i=1;i<numBone(); i++)
	{
		VRMLTransform* ll=((VRMLTransform*)&getBoneByTreeIndex(i));
		ASSERT(ll->mJoint);
		if(ll->mJoint->jointType==HRP_JOINT::FREE || ll->mJoint->jointType==HRP_JOINT::BALL)
			useSpherical.setAt(i);
	}

	if(!dofInfo._sharedinfo)
	   	dofInfo._sharedinfo=new MotionDOFinfo::SharedInfo;
	dofInfo._sharedinfo->init(*this, useSpherical);
	dofInfo.mFrameRate=_frameRate;
#ifdef _DEBUG
	for(int i=1;i<numBone(); i++)
	{
		if(useSpherical(i))
			ASSERT(VRMLbone(i).numHRPjoints()==1);
	}
#endif
	//Msg::print("frameRate: %f\n", _frameRate);
}

void VRMLTransform::copyFrom(VRMLTransform const& bone)
{
	SetNameId(bone.name());
	VRMLTransform const* other=&bone;
	VRMLTransform* out=this;

	out->mVRMLtype=other->mVRMLtype;

	if(other->mJoint)
	{
		out->mJoint=new _HRP_JOINT();
		*out->mJoint=*other->mJoint;
	}

	if(other->mSegment)
	{
		out->mSegment=new HRP_SEGMENT();
		*out->mSegment=*other->mSegment;
	}

	if(other->mShape)
	{
		out->mShape=new HRP_SHAPE();
		*out->mShape=*other->mShape;
	}

	if(other->mTransform)
	{
		out->mTransform=new VRML_TRANSFORM();
		*out->mTransform=*other->mTransform;
	}

}

void VRMLloader::setCurPoseAsInitialPose()
{
	// backup global orientations
	quaterN qo(numBone());
	for (int i=1; i<numBone(); i++)
		qo(i)=bone(i).getFrame().rotation;

	MotionLoader::setCurPoseAsInitialPose();

	vector3N o(numBone());
	for (int i=1; i<numBone(); i++)
		bone(i).getOffset(o(i));

	for (int i=1; i<numBone(); i++)
		VRMLbone(i).setJointPosition(o(i));

	for (int i=1; i<numBone(); i++)
	{
		matrix4 tf;
		tf.identity();
		tf.leftMultRotation(qo(i));
		if (VRMLbone(i).hasShape() )
			VRMLbone(i).transformMeshLocal(tf);
	}
}

// does not scale mass
void VRMLloader::scale(float fScale, Motion& mot)
{
  MotionLoader::scale(fScale, mot);
  TString temp;
  temp.format("%s:scale%.2f", url.ptr(), fScale);
  url=temp;

  m_real totalMass=0;
  for(int i=1; i<numBone(); i++)
    {
      VRMLTransform* bone=&VRMLbone(i);
      Msg::verify(bone->mJoint, "scaling unsupported for this mesh");
	  bone->scaleMesh(vector3(fScale, fScale, fScale));
      totalMass+=(bone->mSegment)?bone->mSegment->mass:0;
    }
  //setTotalMass( totalMass);   
  _checkMass();

  for (int i=0; i<constraints.size(); i++)
	{
		constraints[i].localpos1*=fScale;
		constraints[i].localpos2*=fScale;
	}
}
VRMLTransform& VRMLloader::VRMLbone(int treeIndex) const
{
  return (VRMLTransform& )bone(treeIndex);
}



#include "InertiaCalculator.h"

void VRMLloader::setTotalMass(m_real totalMass)
{
	VRMLloader & l=*this;
	vectorn mass;
	std::vector<matrix3> inertia;

	mass.resize(l.numBone());
	inertia.resize(l.numBone());

	InertiaCalculator ic;
	//InertiaCalculatorAnalytic ic;
	mass[0]=0.0;
	for(int b=1; b<l.numBone(); b++)
	{
		VRMLTransform& bone=l.VRMLbone(b);

		if(bone.mShape)
		{
			ic.calculateFromMesh(bone.mShape->mesh);
			mass[b]=ic.volume()*1000.0; // water density
			inertia[b]=ic.inertia()*1000.0;
			bone.mSegment->centerOfMass=ic.centerOfMass();
		}
		else
		{			
			mass[b]=0;
			inertia[b].zero();
		}
	}

	m_real scale=totalMass/mass.sum();

	if(totalMass==0.0) scale=1.0;

	for(int b=1; b<l.numBone(); b++)
	{
		VRMLTransform& bone=l.VRMLbone(b);

		if(bone.mSegment)
		{
			bone.mSegment->mass=mass[b]*scale;
			bone.mSegment->momentsOfInertia.mult(inertia[b],scale);
		}
	}
	_checkMass();
}
void VRMLloader::_checkMass()
{
	VRMLloader& l=*this;
	double min_mass=0.1; //100g
	// 0.524*(2*L)^3=mass/1000 assuming sphere with water density where L is the half radius
	double L=pow(min_mass/0.524/1000, 1/3)/2;
	double min_inertia=2.0/5.0*min_mass*L*L;
	for(int b=1; b<l.numBone(); b++)
	{
		VRMLTransform& bone=l.VRMLbone(b);

		if(bone.mSegment)
		{
			if (bone.mSegment->mass<min_mass)
				bone.mSegment->mass	=min_mass;

			if (bone.mSegment->momentsOfInertia._11<min_inertia)
				bone.mSegment->momentsOfInertia._11=min_inertia;
			if (bone.mSegment->momentsOfInertia._22<min_inertia)
				bone.mSegment->momentsOfInertia._22=min_inertia;
			if (bone.mSegment->momentsOfInertia._33<min_inertia)
				bone.mSegment->momentsOfInertia._33=min_inertia;
		}
	}
}
void VRMLloader::printDebugInfo()
{
	VRMLloader & l=*this;
	for (int i=1; i<l.numBone(); i++)
	{
		if(l.VRMLbone(i).mShape)
		{
			Msg::print("%s\n", l.VRMLbone(i).name().ptr());
			OBJloader::Geometry& mesh=l.VRMLbone(i).mShape->mesh;

			for(int j=0; j<mesh.numVertex(); j++)
			{
				Msg::print("%d: %s\n", j,mesh.getVertex(i).output().c_str());
			}
		}
	}
}

void VRMLloader::changeTotalMass( m_real newtotalMass)
{
	VRMLloader & l=*this;
	vectorn mass;
	mass.resize(l.numBone());
	mass[0]=0.0;
	for(int b=1; b<l.numBone(); b++)
	{
		VRMLTransform& bone=l.VRMLbone(b);

		if(bone.mSegment)
		{
			mass[b]=bone.mSegment->mass;
		}
	}

	m_real totalMass=mass.sum();
	double scale=newtotalMass/totalMass;

	for(int b=1; b<l.numBone(); b++)
	{
		VRMLTransform& bone=l.VRMLbone(b);

		if(bone.mSegment)
		{
			bone.mSegment->mass*=scale;
			bone.mSegment->momentsOfInertia*=scale;
		}
	}
}
static void projectAngle(m_real& angle)
{
	// represent angle in [-pi, pi]
	int max_iter=1000;
	int i;
	for (i=0; i<max_iter && angle>M_PI+FERR; i++)
		angle-=2.0*M_PI;
	if (i==max_iter) throw std::runtime_error("projectAngle");
	for (i=0; i<max_iter && angle<-M_PI-FERR; i++)
		angle+=2.0*M_PI;
	if (i==max_iter) throw std::runtime_error("projectAngle");
}

void VRMLloader::loadAnimation(Motion& mot, const char* filename) const
{
	TString fn=filename;
	if(fn.right(4).toUpper()==".DOF")
	{
		MotionDOFcontainer temp(dofInfo, filename);
		mot=Motion(temp);
	}
	else MotionLoader::loadAnimation(mot, filename);
}

void VRMLloader::projectAngles(vectorn & temp1)
{
	for(int i=0; i<temp1.size(); i++)
		projectAngle(temp1[i]);		
}


void VRMLloader::removeAllRedundantBones()
{
	MotionLoader::removeAllRedundantBones();
	bool bChanged;
	do
	{
		bChanged=false;
		for(int i=2; i<numBone() ; i++)
		{
			Bone& target=bone(i);
			if(target.getRotationalChannels()==0
				&& target.getTranslationalChannels()==0 
				&& target.numChildren()==0) // remove Nubs too
			{
				Msg::print("Removing redundant bone %s\n", target.NameId);
				removeBone(target);
				bChanged=true;
				break;
			}
		}
	}
	while(bChanged);
	_initDOFinfo(); 
}
void VRMLloader::removeBone(Bone& target)
{
	// move meshes and segments to its parent before removing the bone.
	VRMLTransform &v=(VRMLTransform&)target;	
	Msg::verify(!v.mJoint || v.mJoint->jointType==HRP_JOINT::FIXED, "???cannot remove bone with non-fixed joints. (jointtype %d). use VRMLbone(ti):setJointAxes("") to fix a joint ", v.mJoint?(int)v.mJoint->jointType:-1 );
	Msg::verify(!v.mTransform, "mergeShapes first!");
	// v.mJoint->translation == v.getOffsetTransform().translation
	if (v.mSegment)
	{
		Msg::print("ignoring mass and inertia of %s\n", v.NameId);
	}
	VRMLTransform* parent=(VRMLTransform*)v.m_pParent;
	Msg::verify(parent, "???");
	if (v.mShape)
	{
		v.mShape->mesh.scaleAndRigidTransform(v.getOffsetTransform());
		mergeMesh(parent, &v);
	}
	delete v.mShape; v.mShape=NULL;
	delete v.mSegment; v.mSegment=NULL;

	//cout << "offset1:"<<parent->getOffsetTransform().translation;
	//cout << "+"<<v.getOffsetTransform().translation<<endl;
	//cout << "+"<<v.child()->getOffsetTransform().translation<<endl;

	MotionLoader::removeBone(target);
	//cout << "offset2:"<<parent->getOffsetTransform().translation<<endl;
	//cout << "+"<<parent->child()->getOffsetTransform().translation<<endl;
	//cout << parent->child()->NameId<<endl;


}

void VRMLloader::exportVRML(const char* filename)
{
	TString url_old=url;
	Msg::print("%s\n", url.ptr());
	Msg::print("%s\n", filename);
	url=filename;
	createDirectory(url.left(-4)+"_sd/");
  FILE* file=fopen(filename, "wt");
  fprintf(file, "DEF SampleRobot Humanoid { \n name \"%s\"\n", name.ptr());
  fprintf(file, " humanoidBody [\n");

  VRMLbone(1).pack(file, 1);
  fprintf(file, " ]\n}\n");
  fclose(file);
  url=url_old;
}
void VRMLloader::_exportBinary(BinaryFile& bf) const
{
	int version=0;
	bf.packInt(version);
	bf.pack(name);
	VRMLbone(1).pack(bf);
}
void VRMLloader::_importBinary(BinaryFile& bf)
{
	_clear();
	  int version=bf.unpackInt();
	  bf.unpack(name);
	  m_pTreeRoot=new VRMLTransform();
	  VRMLTransform* root=new VRMLTransform();
	  root->unpack(*this,bf);
	  m_pTreeRoot->SetNameId("HumanoidBody");
	  m_pTreeRoot->AddChild(root);

	  VRMLTransform* n=((VRMLTransform*)(m_pTreeRoot->m_pChildHead));
	  n->_getOffsetTransform().translation.zero();
	  _initDOFinfo(); 
}
void VRMLloader::exportBinary(const char* filename)
{
	TString url_old=url;
	url=filename;
	BinaryFile bf(true, filename);
	_exportBinary(bf);
	bf.close();
	url=url_old;
}
void VRMLloader::_importVRML(CTextFile& file)
{
	_clear();
  file.setSingleCharacterTokens("{}[]");
  TString token;

  while(1)
    {
      token=file.GetToken();

#ifdef _DEBUG
	  Msg::print("loader token: %s\n", token.ptr());
#endif
      if(token=="PROTO")
	{
	  TString proptoName=file.GetToken();
	  TString token;
	  token=file.GetToken();

	  ASSERT(token=="[");

			
	  while(1)
	    {
	      token=file.GetToken();
	      if(token=="exposedField" || token=="field" || token=="eventIn")
		{
		  TString name;
		  vectorn value;
		  unpackExposedField(file, name, value);
		}
	      else break;
	    }

	  ASSERT(token=="]");

	  skipNode(file);
	}
      else if(token=="NavigationInfo")
	skipNode(file);
      else if(token=="Background")
	skipNode(file);
      else if(token=="Viewpoint")
	skipNode(file);
      else if(token=="DEF")
	{
	  file.Undo();
	  // dummy.
	  m_pTreeRoot=new VRMLTransform();
	  ((VRMLTransform*)m_pTreeRoot)->Unpack(*this,file);
	}
      else if(token.length()==0)
	break;
      else unexpectedToken(file, token, "DEF, Background, ... expected");
    }

  if(name=="")
    name=m_pTreeRoot->NameId;

//  printHierarchy((VRMLTransform*)m_pTreeRoot,0);
  replaceUseNode((VRMLTransform*)m_pTreeRoot, (VRMLTransform*)m_pTreeRoot);

  // optional. greatly simplifies tree structures. (Prerequisties for VRMLloaderView)
  mergeShapes((VRMLTransform*)m_pTreeRoot);

  VRMLTransform* n=((VRMLTransform*)(m_pTreeRoot->m_pChildHead));
  n->_getOffsetTransform().translation.zero();
  _initDOFinfo(); 
}


static void make_extra(VRMLTransform* node,VRMLTransform* out)
{
  while(out->m_pSibling)
	out = (VRMLTransform*)(out->m_pSibling);

  VRMLTransform* child;
  child=new VRMLTransform();
  child->copyFrom(*node);

  out->AddChild(child);

  if(node->m_pChildHead)
	make_extra((VRMLTransform*)node->m_pChildHead,(VRMLTransform*)out->m_pChildHead);

  if(node->m_pSibling)
	make_extra((VRMLTransform*)node->m_pSibling,(VRMLTransform*)out);
}


void VRMLloader::addRelativeConstraint(int ibone1, vector3 const& lpos1, int ibone2, vector3 const& lpos2)
{
	constraints.resize(constraints.size()+1);
	auto& con=constraints[constraints.size()-1];
	con.ibone1=ibone1;
	con.ibone2=ibone2;
	con.localpos1=lpos1;
	con.localpos2=lpos2;
}
void VRMLloader::_getAllRelativeConstraints(intvectorn& ibone, vector3N& localpos) const
{
	ibone.setSize(constraints.size()*2);
	localpos.setSize(constraints.size()*2);
	for (int icon=0; icon<constraints.size(); icon++)
	{
		int iconx2=icon*2;
		auto& c=constraints[icon];
		ibone[iconx2]=c.ibone1;
		ibone[iconx2+1]=c.ibone2;
		localpos[iconx2]=c.localpos1;
		localpos[iconx2+1]=c.localpos2;
	}
}
VRMLloader_subtree* VRMLloader::makesubtree(int treeIndex) const
{
  //VRMLTransform* found=findTransform((VRMLTransform*)m_pTreeRoot, startname);
  VRMLTransform* found=&VRMLbone(treeIndex);
  ASSERT(found);

  VRMLloader_subtree* extra_VRMLloader = new VRMLloader_subtree();
  extra_VRMLloader->m_pTreeRoot = new VRMLTransform();

  if(TString(m_pTreeRoot->NameId) != found->name())
  {
	extra_VRMLloader->url = url+TString("_subtree_")+found->name();
	extra_VRMLloader->name = name;
	extra_VRMLloader->version = version;
	extra_VRMLloader->info = info;

	VRMLTransform* extra = ((VRMLTransform*)extra_VRMLloader->m_pTreeRoot);
	extra->copyFrom(*(&VRMLbone(0)));

 	VRMLTransform* m_child;	
	m_child=new VRMLTransform();

	if(1)
	{
		m_child->copyFrom(*found);
		extra->AddChild(m_child);
		Msg::verify(found->m_pChildHead, "%s has no child", found->NameId);
		make_extra((VRMLTransform*)(found->m_pChildHead),(VRMLTransform*)(extra->m_pChildHead));		
	}
	else
	{
		// copy found to m_child
		MemoryFile m;
		found->pack(m);
		m_child->unpack((VRMLloader&)*this, m);
		extra->AddChild(m_child);
	}

	extra_VRMLloader->_initDOFinfo(); 
	}
  else
  {
	  Msg::print("\nWarning : You don't need extractSubtree! (Same VRMLloader)\n");		
	return NULL;
  }
  extra_VRMLloader->fullbody = this;
  extra_VRMLloader->mtreeindex=treeIndex;
  extra_VRMLloader->mdof = 0;

  const VRMLloader* fullbody=this;
  for(int i =1; i < treeIndex; i++)
	  extra_VRMLloader->mdof+=fullbody->dofInfo.numDOF(i);

  extra_VRMLloader->FullbodyPoseToSubpose();
  Bone* bone = fullbody->VRMLbone(treeIndex).parent();

  if(bone!=NULL)	
  {
	  bone->getTranslation(extra_VRMLloader->origindofpos); 
	  bone->getRotation(extra_VRMLloader->origindofori);
  }
  return extra_VRMLloader;
}
 

VRMLloader_subtree::VRMLloader_subtree()
	:VRMLloader()
{
}

void VRMLloader_subtree::FullbodyPoseToSubpose()
{
	vectorn mPose,subPose;
	fullbody->getPoseDOF(mPose);


	fullbodyPoseToSubpose(mPose, subPose);

	setPoseDOF(subPose);
}
void VRMLloader_subtree::fullbodyPoseToSubpose(vectorn const & fullPose, vectorn &subPose) const
{
	getPoseDOF(subPose);
	for(int i = 0; i<subPose.size();i++)
		subPose.set(i, fullPose(i + mdof ));
}
void VRMLloader_subtree::subPoseToFullpose(vectorn const & subPose, vectorn &fullPose) const
{
	for(int i=0;i<subPose.size();i++)
		fullPose.set(i + mdof , subPose(i));
}

void VRMLloader_subtree::subPoseToFullpose()
{
	vectorn mPose,subPose;
	fullbody->getPoseDOF(mPose);
	getPoseDOF(subPose);
	subPoseToFullpose(subPose, mPose);
	fullbody->setPoseDOF(mPose);
}


VRMLloader_subtree::~VRMLloader_subtree()
{
	//fullbody is not created by this.
#ifdef TEST_MEMORYLEAK	
	Msg::msgBox("dtor VRMLloader_subtree %s", url.ptr());
#endif 	
	
}

int VRMLloader_subtree::subTreeindex(int fulltreeindex) const 
{ 
	int ii=
	fulltreeindex-mtreeindex+1;
	if (ii>=numBone() || ii<1)
		return -1;
	return ii;
}

void VRMLloader::setPosition(const vector3 & pos)
{
	auto& root=VRMLbone(1);
	if (root.getRotationalChannels().length()>0 ||
			root.getTranslationalChannels().length()>0)
		Msg::msgBox("VRMLloader::setPosition function works only for wrl files having a fixed root joint")	;
	else
	{
		VRMLbone(1).setJointPosition(pos);
		VRMLbone(1)._getLocalFrame().translation=pos;
		fkSolver().forwardKinematics();
	}
}
