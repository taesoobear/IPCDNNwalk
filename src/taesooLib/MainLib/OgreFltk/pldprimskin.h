#ifndef _INC_PLDPRIMSKIN_38BF502900AA_INCLUDED
#define _INC_PLDPRIMSKIN_38BF502900AA_INCLUDED
#if defined (_MSC_VER) && (_MSC_VER >= 1000)
#pragma once
#endif

#include "../../BaseLib/math/mathclass.h"
class MotionLoader;
class Interpolator;
class TimeSensor;
class Posture;
class BoneForwardKinematics;
class MotionManager;
class Motion;
class MotionDOFinfo;

#include "AnimationObject.h"
#include "RE.h"

class PLDPrimSkin : public AnimationObject
{
public:
	PLDPrimSkin();
	virtual ~PLDPrimSkin();

	// reimplement this.
	virtual void SetPose(const Posture & posture, const MotionLoader& skeleton){}
	virtual void setPoseDOF(const vectorn& poseDOF, MotionDOFinfo const& info);

	virtual void updateBoneLength(MotionLoader const& loader){}
	virtual void setPose(int iframe);
	virtual void setPose(const Motion& mot, int iframe);
	virtual void ApplyAnim(const Motion& mot);
	virtual void applyAnim(const MotionDOF& motion);
	virtual void detachAnim();
	virtual void setThickness(float thick){}

	virtual void setDrawConstraint(int con, float radius, RE::Color c){}
	virtual void setDrawOrientation(int ijoint){}
	void scale(double x, double y, double z);
	virtual void setScale(double x, double y, double z);

	struct DrawCallback
	{
		virtual void draw(const Motion& mot, int iframe){}
		virtual void draw(const Posture& posture, const MotionLoader& skeleton){}
	};

	void setDrawCallback(DrawCallback* pCallback)	{ mDrawCallback=pCallback; }
	void setBeforeDrawCallback(DrawCallback* pCallback)	{ mBeforeDrawCallback=pCallback; }
	virtual void setMaterial(const char* mat){Msg::error("not implemented yet");}
protected:
	DrawCallback* mDrawCallback;
	DrawCallback* mBeforeDrawCallback;
};
#include "../../BaseLib/utility/TArray.h"
namespace RE
{
	class SkinArray : public TArray<PLDPrimSkin>
	{
		bool m_bHideFootPrint;
		PLDPrimSkinType m_type;
	public:
		SkinArray(MotionLoader* skeleton, PLDPrimSkinType t=PLDPRIM_SKIN, bool bHideFoot=true);
		SkinArray(){}	
		virtual~SkinArray();	
		
		void changeColor(Color c);
		void show();
		void hide();
		void setSkeleton(MotionLoader* skeleton, PLDPrimSkinType t=PLDPRIM_SKIN, bool bHideFoot=true);	
	};
}

#endif /* _INC_PLDPRIMSKIN_38BF502900AA_INCLUDED */
