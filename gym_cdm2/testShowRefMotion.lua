require('module')
require('common')
require('subRoutines/Timeline')
require('subRoutines/VRMLexporter')
require('subRoutines/WRLloader')
fc=require('gym_cdm2/module/CDMTraj')

envName='walk2-v1'
--motion='run3' useHanyangInsteadOfHyunwoo=true
motion='walk4' useHanyangInsteadOfHyunwoo=true
--motion='fastwalk' useHanyangInsteadOfHyunwoo=true
--motion='runjump'
--motion='runjump2'
--motion='run2' useHanyangInsteadOfHyunwoo=true
--motion='run180'
--motion='run90'
--motion='run90R'
--motion='fastwalk'
--motion='walk'
--motion='backflip'

RL_step=1/60
model=require('gym_cdm2/defaultRLsetting')(motion)
motionInfo=model._motionInfo
model.frame_rate=30
model.inertia=model.inertia or vector3(9.683,2.056,10.209)
model.collectRefTraj=false
model.loopMotion=true
model.start=model.start or 0
model.doNotSolveIK=false
removeFeetPenetration =true
drawCDM=false
drawReference=true
local downsample=model.downsample or 1
RL_step=1/30/downsample

randomRestartInfo={prevStart=model.start, freq=1, limitLength=3/RL_step}


feet_config={
	-- this local positions have to be same as those defined in showHuman.lua (input.limbs[1][3] and input.limbs[2][3]))
	toe=input.limbs[1][3], 
	heel=input.limbs[2][3],
}


RagdollSim=LUAclass()

-- actually recorded playbacks.
function RagdollSim:__init(loader, drawSkeleton, simulatorParam)

	self.impulse=0
	self.CDM={ }
	self.CDM.loader= MainLib.WRLloader(
	{
		name='CDM',
		body={
			name='box',
			jointType="free",
			translation =vector3(0.0,0.0,0.0 ), -- joint의 위치
			geometry ={ 
				{
					'Box',
					translation=vector3(0,0,0),
					size=VRMLexporter.boxSizeFromInertia(60, model.inertia.x, model.inertia.y, model.inertia.z),
					mass=60,
				},
			},
		}
	}
	)
	self.CDM.skin=RE.createVRMLskin(self.CDM.loader, false)
	self.CDM.skin:setThickness(0.03)
	self.CDM.skin:scale(100,100,100)
	self.CDM.skin:setMaterial("lightgrey_transparent")
	self.CDM.skin:setVisible(drawCDM)

	local option={
		bones=input.bones,
		debugDraw=false,
	}

	self.loader_ref_fullbody=loader
	self.loader_ref=self.CDM.loader

	g_info={
		loader=mLoader,
		mot=mMotionDOF,
	}

	if model.option_refCDMtraj then
		table.mergeInPlace(option, model.option_refCDMtraj, true)
	end
	g_info.loader:setVoca(option.bones)
	g_info.CDMtraj=fc.CDMTraj(g_info.loader, g_info.mot, option)

	if motionInfo.fixDOF_part2 then
		motionInfo.fixDOF_part2 (mLoader, mMotionDOF)
	end
	model.motionData=g_info
end
function RagdollSim:prepareMotionForSim()
	model.touchDown=motionInfo.contact.touchDown
	model.touchOff=motionInfo.contact.touchOff

	g_info.loader_ref_fullbody=self.loader_ref_fullbody
	g_info.loader_ref=self.loader_ref
	fc.prepareMotionsForSim(g_info, 0, model)
end

require('gym_cdm2/module/showHuman')

function onCallback(w)
	if w:id()=='attach camera' then
		mEventReceiver:attachCameraToMotion(mLoader, g_info.motionDOF_fullbody)
	elseif w:id()=='show sim skin' then
		mSkin:setVisible(w:checkButtonValue())
	end
end
function ctor()
	mEventReceiver=EVR()

	this:create('Button', 'attach camera', 'attach camera')
	this:create('Check_Button', 'show sim skin', 'show sim skin')
	this:widget(0):checkButtonValue(true)
	this:updateLayout()
	mLoader=MainLib.VRMLloader ( motionInfo.skel)
	mLoader:setVoca(input.bones)

	if not motionInfo.isDeepmimicMot then

		mMotionDOFcontainer=MotionDOFcontainer(mLoader.dofInfo, motionInfo.fn)
		mMotionDOF=mMotionDOFcontainer.mot

		mMotionDOFcontainer=nil
		motionInfo.fixDOF(mLoader, mMotionDOF)
	else
		local file_name='gym_deepmimic/module/humanoid3d.txt'
		local mot_file=motionInfo.fn
		require('gym_cdm/motions/DeepMimicLoader')
		if useHanyangInsteadOfHyunwoo then
			g_info=loadDeepMimicMotionToHanyang(file_name, mot_file)
		else
			g_info=loadDeepMimicMotionToHyunwoo(file_name, mot_file)
		end

		info1=g_info.dmloader
		local frameTime=g_info.frametime
		mMotion=info1.mot
		mMotionDOF=g_info.mot
		mLoader:setVoca(input.bones)
		motionInfo.fixDOF(mLoader, mMotionDOF)
	end

	mSkin= RE.createVRMLskin(mLoader, false);	-- to create character
	mSkin:scale(100,100,100);					-- motion data is in meter unit while visualization uses cm unit.
	mSkin:setMaterial('lightgrey_transparent')
	mSkin:setVisible(drawReference)



	mRagdoll=RagdollSim(mLoader)

	-- showHuman
	mRagdoll.skin_ref=RE.createVRMLskin(mLoader, false)
	mRagdoll:prepareMotionForSim()

	mSkin:applyMotionDOF(g_info.motionDOF_fullbody)
	--mSkin:applyMotionDOF(mMotionDOF)
	RE.motionPanel():motionWin():addSkin(mSkin)

	if envName then
		local fn2='gym_cdm2/spec/refTraj_'..envName..'.dat.processed'
		if fn2 and os.isFileExist(fn2) then
			refTraj=util.loadTable(fn2)
			math.changeChartPrecision(100) -- height of a panel
			local sp=RE.motionPanel():scrollPanel()
			local function concat2(A)
				return (A:T().. A:sub(1,0,0,0):T()):T():copy()
			end
			sp:addPanel(concat2(refTraj.comdof_error))
			sp:setLabel('comdof_err')

			sp:addPanel(concat2(refTraj.bodyVel))
			sp:setLabel('bodyVel')

			sp:addPanel(concat2(refTraj.CDMtoRoot))
			sp:setLabel('CDMtoRoot')

			sp:addPanel(concat2(refTraj.CDMfeetToHumanFeetPositions:sub(0,0,0,-1)))
			sp:setLabel('CDMfeetToHumanFeetPositions')
			local refFrames=refTraj.CDMfeetToHumanFeetPositions:column(refTraj.CDMfeetToHumanFeetPositions:cols()-1)
			local drefFrames=refFrames:column():derivative(1)
			sp:addPanel(concat2(drefFrames))
			sp:setLabel('refFrames')
		end
	end
end

function frameMove()


	return 0
end
function handleRendererEvent()
	return 0
end

function EVR:onFrameChanged(win, i)
	if noFrameChange or not mRagdoll then return end


	local COMtraj=g_info.CDMtraj.COMtraj
	mRagdoll.skin_fullbody:setPoseDOF(g_info.motionDOF_fullbody:row(i))

	self:moveCamera(i)
	--dbg.namedDraw('Axes', transf(quater(1,0,0,0), COMtraj:row(i)), 'root', 100)
end
function renderOneFrame()
	noFrameChange=true
	RE.renderOneFrame(true)
	noFrameChange=false
end
-- cdm_gf : global frame
-- cdm_dq
