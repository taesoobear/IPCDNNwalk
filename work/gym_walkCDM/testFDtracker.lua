

require("config")
require("module")
require("common")

package.projectPath='./gym_walkCDM'
package.path=package.path..";../../taesooLib/Samples/scripts/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/RigidBodyWin/subRoutines/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/?.lua" --;"..package.path
require("module")
package.path=package.path..";../Samples/scripts/RigidBodyWin/subRoutines/?.lua" --;"..package.path
require("Timeline")
fc=require("gym_walk/footCoord")
require("gym_walkCDM/FDtracker")

useSimpleTraj=false



function ctor()
	mEventReceiver=EVR()
	-- input ={skel='a.wrl', mot='a.dof', motionType='run'}
	mMot=fc.loadMotion({
		skel="../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T.wrl",
		mot="../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_wd2_2foot_walk_turn2.dof",
	})

	mLoader= MainLib.VRMLloader('box.wrl')
	mSkin2=RE.createVRMLskin(mLoader, false)
	mSkin2:setMaterial('lightgrey_transparent')
	mSkin2:scale(100,100,100)
	mSkin2:setTranslation(0,0,0)

	g_mot_=mMot.motionDOFcontainer.mot:matView():sub(0,0,0,7):copy()
	--g_mot_=g_mot_:sub(100,0,0,0):copy()


	if useSimpleTraj then
		for i=0, g_mot_:rows()-1 do
			MotionDOF.setRootTransformation(g_mot_:row(i), 
			transf(quater(1,0,0,0), vector3(0, 1, sop.map(i, 0, g_mot_:rows()-1, 0, 3))))
		end
	end





	mTimeline=Timeline("Timeline", 1000000, 1/30)
	RE.motionPanel():motionWin():playFrom(0)
	mSkin=RE.createVRMLskin(mLoader, false)
	mSkin:scale(100,100,100)


	this:create("Check_Button", "simulation", "simulation", 0, 2,0)
	this:widget(0):checkButtonValue(0) -- 1 for imediate start
	this:widget(0):buttonShortcut("FL_ALT+s")
	mTracker=FDTracker( vector3(0,9.8,0))

	g_output=mTracker:processInput(g_mot_)


end
function dtor()
	if mSkin~=nill then
		RE.remove(mSkin)
		mSkin=nil
	end
	if mTimeline then
		mTimeline:dtor()
		mTimeline=nil
	end
end
function handleRendererEvent()
	return 0
end
function frameMove() 
	return 0
end
function onCallback(w, userid)
end

EVR=LUAclass(EventReceiver)
function EVR:__init()
end
function EVR:onFrameChanged(win, iframe)
	if iframe<g_mot_:rows() then
		mSkin2:setPoseDOF(g_mot_:row(iframe))
		if not g_output:row(iframe):isnan() then
			mSkin:setPoseDOF(g_output:row(iframe))
			else
				dbg.console()
		end
	end
end
