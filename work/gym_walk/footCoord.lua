arc=require('gym_walk/arc')
local fc={}

function MotionUtil.FullbodyIK_MotionDOF:_setNormalConstraint(i, bone, localpos, normal, gpos)
	local plane=Plane(-1*normal, gpos)
	return self:_setHalfSpaceConstraint(i, bone, localpos, plane.normal, plane.d)
end
fc.debugDraw=false
local function quatToVec(q)
	return CT.vec(q.w, q.x, q.y,q.z)
end
input={
	skel="../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T.wrl",
	mot="../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof",
	motionFrameRate=30,
	limbs={
		{'LeftKnee', 'LeftAnkle', vector3(0, -0.05, 0.14), reversed=false},
		{'LeftKnee', 'LeftAnkle', vector3(0, -0.10, -0.06), reversed=false},
		{'RightKnee', 'RightAnkle', vector3(0, -0.05, 0.14), reversed=false},
		{'RightKnee', 'RightAnkle', vector3(0, -0.10, -0.06), reversed=false},
	},
	head={'Neck', vector3(0,0.05,0), vector3(0,0.7,0)}, -- name, local pos, local pos WRT COM
	lowerbody={2,9},
	bones={
		left_ankle="LeftAnkle",
		right_ankle="RightAnkle",
		left_shoulder='LeftShoulder',
		right_shoulder='RightShoulder',
		left_hip='LeftHip',
		right_hip='RightHip',
	},
	-- L, R
	legs={ { 'LeftAnkle', 'RightElbow'}, { 'RightAnkle', 'LeftElbow'},}, -- this defines the bones related to Lpose and Rpose
	skinScale=100,
	--     phase
	--              H   HT     T     F     H  ...  
	--            0    1    2    3    0     1    2    3     0
	--                             == 4     5    6    
	walk={
		contacts=
		{
			--          262                 298
			{{252, 254, 270,274},  {288, 291, 305, 310},},  --L
			--      280                  
			{{270,273,287,291}, {306, 309, 324, 327},}, -- R
		},
		minHalfStride=0.6,  --
		speed=1.15, -- original speed
		maxspeed=1.7,
		--minspeed=2e-3,
		minspeed=0.6, -- don't go to stop mode when generating randomly
		pendstrength=4,
		basePolyDur=0.15,
		filterSize=0,
		strideWeight=50.0,
		intersectionWeight=5.0,
		cfWeight=10.0,
		COMoffsetScale=5.5,
		defaultErrorOffsetQ=quater(1,0,0,0),
		-- COM down using only the hip.
		COMy_down_pose={
			CT.vec(0,-0.04,-0.19)..
			quatToVec(quater(math.rad(40), vector3(1,0,0)))..
			CT.vec(0,0,
			-0.846564,0, 0,0.356504,
			0,0,
			-0.846564,0, 0,0.356504,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0),
			0.08, -- COM down amount
		},
		COMy_down_pose_z={
			CT.vec(0.19,-0.04,0)..
			quatToVec(quater(math.rad(40), vector3(0,0,1)))..
			CT.vec(0,
			 -0.846564,0,0, 0,0.356504,
			0,
			 -0.846564,0,0, 0,0.356504,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0),
			0.08, -- COM down amount
		}
	},
	bar_handwalk={
		contacts=
		{
			{{252, 254, 270,274},  {288, 291, 305, 310},},
			{{270,273,287,291}, {306, 309, 324, 327},},
		},
		speed=1.15, -- original speed
		basePolyDur=0.15,
		filterSize=5,
		useMMoffsetQ=true,
		limbs={
			{'LeftShoulder', 'LeftElbow', vector3(0.24, 0, 0), reversed=false},
			{'RightShoulder', 'RightElbow', vector3(-0.24, 0, 0), reversed=false},
		},
		customOffsets=require('gym_walk/offset_humanRun'),
		hands_up_pose=
			CT.vec(0,0,0)..
			quatToVec(quater(1,0,0,0))..
			CT.vec(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1.57,0,-3.14,0,1.57,0,3.14,0,0,0,0),
		hands_down_pose=
			CT.vec(0,0,0)..
			quatToVec(quater(1,0,0,0))..
			CT.vec(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1.57,0,0,0,1.57,0,0,0,0,0,0)
	},
	-- run
	run={
		contacts={
			{{754,756,761,765}, {778,780,784,789},   },
			{{766,768,773,777}, {790,792,796,801},   },
		},
		--customOffsets=require('module/offset_humanRunR'),
		minHalfStride=0.8,  --
		maxspeed=10,
		minspeed=1.6, -- motion quality would not be good enough below this speed
		speed=1.76,
		basePolyDur=0.08,
		strideWeight=0.85,
		maxScale=8,
		COMoffsetScale=5,
		offsetPose=CT.vec(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.498262),
		offsetQ=quater(math.rad(3), vector3(0,1,0))*quater(math.rad(-1), vector3(0,0,1))*quater(math.rad(5), vector3(1,0,0)),
		defaultErrorOffsetQ=quater(math.rad(5), vector3(0,1,0)), -- fastrun
		filterSize=0,
		initialHeight=-0.02,
		pendstrength=8,
		renderFreq=30,
		COMy_knee_down_pose={
			CT.vec(0,0.791593-0.95,-0.07)..
			quatToVec(quater(math.rad(10), vector3(1,0,0)))..
			CT.vec(0,0,
			-0.946564,1.417285, 0,-0.486504,
			0,0,
			-0.946564,1.417285, 0,-0.486504,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0),
			0.83535 -- COM height of this pose.
		},
		hand_forward_pose={
			CT.vec(0,0.056917,-0.108187,0.998951,-0.015283,-0.042595,-0.006986,
			0.137785,0.056569,-0.224437,0.362941,0.135235,0.245545, -0.012410,
			-0.040622, -0.271763,0.422651,-0.104579,0.316912,0.160330,0.024543,
			-0.269614,
			-1.5+1.57,0,-2.3,1.5,
			1.5-1.57,0,2.3,-1.5,
			 0,0,0)
		},
		hand_backward_pose={
			CT.vec(0,0.056917,-0.108187)..
			quatToVec(quater(1,0,0,0))..
			CT.vec(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			-2.0+1.57,0,1.3,0,
			2.0-1.57,0,-1.3,-0,
			 0,0,0)
		},
		hands_down_pose={
			CT.vec(0,0,0)..
			quatToVec(quater(1,0,0,0))..
			CT.vec(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1.57,0,0,0,1.57,0,0,0,0,0,0)
		},
		-- COM down using only the hip.
		COMy_down_pose={
			CT.vec(0,-0.04,-0.19)..
			quatToVec(quater(math.rad(40), vector3(1,0,0)))..
			CT.vec(0,0,
			-0.846564,0, 0,0.356504,
			0,0,
			-0.846564,0, 0,0.356504,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0),
			0.08, -- COM down amount
		},
		COMy_down_pose_z={
			CT.vec(0.19,-0.04,0)..
			quatToVec(quater(math.rad(40), vector3(0,0,1)))..
			CT.vec(0,
			 -0.846564,0,0, 0,0.356504,
			0,
			 -0.846564,0,0, 0,0.356504,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0),
			0.08, -- COM down amount
		}
	},
	jump={
		--mot="../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_justin_jump.dof",
		mot="../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_jump3.dof",
		--              steady  heel  toe        toe  heel   steady
		contacts={
			{{52, 54, 126, 132,},{148  ,150, 221, 228}},
			{{52, 54, 126, 132,},{148  ,150, 221, 228}},
		},
		speed=0.25,
		basePolyDur=0.05, -- 0.15 works for most cases but ...
		isJump=true,
		filterSize=5,
		useMMoffsetQ=true,
	},
	turn={
		mot="../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_wd2_2foot_walk_turn2.dof",
		--              89                 123 
		contacts={
			{{79,81, 92,99}, {114, 116, 125, 132},},
			--               106.5             140
			{{97,99,107,116}, {131, 134, 143, 149},},
		},
		basePolyDur=0.15,
		filterSize=3,
		speed=0,
		doNotUseOffsetQ=true,
		filterFD=3
	},
}

if useWoody then
	require("gym_walk/wd_all")
end
-- swing / stance duration ratio: 
--    walk: 14/22
--    run:  13/11

local function frameToPhase(frame, contact)
	local thr=1e-3
	local function isSupport(frame, spprt)
		if frame>=spprt[1]-thr and frame<=spprt[4]+thr then
			if frame<spprt[2] then
				return sop.map(frame, spprt[1], spprt[2], 0, 1)
			elseif frame<spprt[3] then
				local midFrame=spprt[1]*0.5+spprt[4]*0.5

				if frame<midFrame then
					return sop.map(frame, spprt[2], midFrame, 1, 1.5)
				else
					return sop.map(frame, midFrame, spprt[3], 1.5, 2)
				end
			else
				return sop.map(frame, spprt[3], spprt[4], 2, 3)
			end
		end
		return -1
	end
	local phase= isSupport(frame, contact[1])
	if phase==-1 then
		phase=isSupport(frame, contact[2])
	end
	if phase==-1 then
		assert(frame>contact[1][4]-thr and frame<contact[2][1]+thr)
		return sop.map(frame, contact[1][4], contact[2][1], 3,4)
	end
	return phase
end

-- assumes that heel first touches the ground.
function fc.heelAndToeContactToSupportPhase(heelC, toeC)
	if heelC then
		if toeC then
			return 1 --- heel and toe  
		else
			return 0 --- heel
		end
	else
		if toeC then
			return 2 --- toe
		else
			return 3 --- swing
		end
	end
end

function fc.extractCOMroot(loader, mot, bones, inputm)
	local COMtraj=vector3N(mot:numFrames())
	for currFrame=0, mot:numFrames()-1 do
		loader:setPoseDOF(mot:row(currFrame))
		COMtraj(currFrame):assign( loader:calcCOM())
	end
	local global_positions=mot:extractGlobalPositionsFromVoca(
	MotionLoader.LEFTSHOULDER, MotionLoader.RIGHTSHOULDER,
	MotionLoader.LEFTHIP, MotionLoader.RIGHTHIP
	)
    local across = (
        (global_positions:page(0) - global_positions:page(1)) + 
        (global_positions:page(2) - global_positions:page(3)))

	local forwardDir=across:vec3ViewCol(0):copy()

	for i=0, forwardDir:size()-1 do
		local v=forwardDir(i)
		v:cross(v:copy(), vector3(0,1,0))
		v.y=0
		v:normalize()
	end

	math.gaussFilter(inputm.filterFD or 20, forwardDir:matView())

	local rotY=quaterN(mot:numFrames())
	for currFrame=0, mot:numFrames()-1 do
		rotY(currFrame):setAxisRotation(vector3(0,1,0), vector3(0,0,1), forwardDir(currFrame))
	end
	return COMtraj, rotY
end
function fc.extractCOMroot_raw(loader, mot, bones, inputm)
	local COMtraj=vector3N(mot:numFrames())
	for currFrame=0, mot:numFrames()-1 do
		loader:setPoseDOF(mot:row(currFrame))
		COMtraj(currFrame):assign( loader:calcCOM())
	end
	local forwardDir=COMtraj:copy()

	for i=0, forwardDir:size()-1 do
		local v=forwardDir(i)
		v:rotate(mot:row(i):toQuater(3):rotationY(), vector3(0,0,1))
		v.y=0
		v:normalize()
	end

	math.gaussFilter(inputm.filterFD or 20, forwardDir:matView())

	local rotY=quaterN(mot:numFrames())
	for currFrame=0, mot:numFrames()-1 do
		rotY(currFrame):setAxisRotation(vector3(0,1,0), vector3(0,0,1), forwardDir(currFrame))
	end
	return COMtraj, rotY
end

fc.PoseSampler=LUAclass()

local function loader_getIndices(loader, boneName)
	local indices=intvectorn()
	local bone=loader:getBoneByName(boneName)
	while bone:treeIndex()~=0 do
		local dofInfo=loader.dofInfo
		local sr=dofInfo:startT(bone:treeIndex())
		local et=dofInfo:endR(bone:treeIndex())
		for i=et-1,sr,-1 do
			indices:pushFront(i)
		end
		bone=bone:parent()
	end
	return indices
end
local function blendPose(a, weight_a, b, weight_b)
	local pose= a*weight_a+b*weight_b
	pose:setQuater(3, pose:toQuater(3):Normalize())
	return pose
end
local function diffPose(a, b)
	local pose= b-a
	local qd=quater()
	qd:difference(a:toQuater(3), b:toQuater(3))
	pose:setQuater(3, qd)
	return pose
end
local function addPose(a, b)
	local pose= a+b
	local q=b:toQuater(3)*a:toQuater(3)
	pose:setQuater(3, q)
	return pose
end
local function scalePose(a,s)
	local pose= a*s
	local q=a:toQuater(3)
	q:scale(s)
	pose:setQuater(3, q)
	return pose
end
fc.addPose=addPose
fc.scalePose=scalePose
fc.blendPose=blendPose
-- uses input.Lleg, input.Rleg
function fc.PoseSampler:__init(effectors, loader, mot, input, motionType, debugDraw)
	local motionType=motionType or 'run'
	local NDOF=loader.dofInfo:numDOF()
	self.contact={
		input[motionType].contacts, 
		input[motionType].offsetQ   
	}

	local weightsO=vectorn(NDOF) weightsO:setAllValue(1)
	if not input.legs then
		input.legs={input.Lleg, input.Rleg}
	end
	input[motionType].numLegs=#input.legs

	local legs=input.legs
	local weights={}
	for i=1, #input.legs do
		local weightsL=vectorn(NDOF) weightsL:setAllValue(0)
		weights[i]=weightsL
	end

	for ileg, leg in ipairs(legs) do
		for i,v in ipairs(leg) do
			local Lindices=loader_getIndices(loader, v)
			local weightsL=weights[ileg]
			weightsL:assignSelective(Lindices, CT.ones(Lindices:size()))
			weightsO:assignSelective(Lindices, CT.zeros(Lindices:size()))
		end
	end
	local sum=vectorn(NDOF)
	sum:setAllValue(0)
	for ileg, leg in ipairs(legs) do
		sum:radd(weights[ileg])
	end

	for ileg, leg in ipairs(legs) do
		local weightsL=weights[ileg]
		for i=0,weightsL:size()-1 do
			if sum(i)>1e-3 then
				weightsL:set(i,weightsL(i)/sum(i))
			end
		end
	end
	self.weights={weights, weightsO}
	self.loader=loader
	self.mot=mot:copy()
	if self.mot:rows()==1 then
		self.mot:resize(100)
		for i=1, 99 do
			self.mot:row(i):assign(self.mot:row(0))
		end
	end
	self.MMhelper={
		LtoT=MotionUtil.LoaderToTree(self.loader, false, false),
		chain=BoneForwardKinematics(self.loader),
	}

	self.MMhelper.chain:init()
	local initialHeight=input[motionType].initialHeight 
	if initialHeight then
		for i=0, self.mot:numFrames()-1 do
			self.mot:row(i):set(1, self.mot:row(i)(1)+initialHeight)
		end
	end
	-- convert poses to COM local coord (without actually modifying self.mot)
	local inputm=input[motionType]
	local COMtraj,rotY

	if input.bones then
		loader:setVoca(input.bones) -- for drawing spheres. see dbg.drawSphere below.
		mot.dofInfo:skeleton():setVoca(input.bones)
	end

	if #legs==2 and not inputm.useRawRoot then
		COMtraj,rotY= fc.extractCOMroot(loader, self.mot, input.bones, inputm)
	else
		COMtraj,rotY= fc.extractCOMroot_raw(loader, self.mot, input.bones, inputm)
	end

	if debugDraw then
		local input=input[motionType]
		local thickness=5
		print('minHeight', COMtraj:matView():sub(input.Lcontact[1][1], input.Rcontact[2][4]):column(1):avg())
		local COMx100=COMtraj:matView():sub(input.Lcontact[1][1], input.Rcontact[2][4])*100
		COMx100:column(1):setAllValue(1)
		dbg.namedDraw('Traj', COMx100, 'goal2', 'blueCircle', thickness, 'QuadListY')
	end
	self.COMtraj=COMtraj
	--print(self.COMtraj(input.Lcontact[1][1])*0.5+self.COMtraj(input.Lcontact[1][4])*0.5)
	--print(self.COMtraj(input.Rcontact[1][1])*0.5+self.COMtraj(input.Rcontact[1][4])*0.5)
	--print(self.COMtraj(input.Lcontact[2][1])*0.5+self.COMtraj(input.Lcontact[2][4])*0.5)
	--print(self.COMtraj(input.Rcontact[2][1])*0.5+self.COMtraj(input.Rcontact[2][4])*0.5)

	self.rotY=rotY

	local motionFrameRate=input.motionFrameRate

	do
		local input=input[motionType]
		self.input=input

		assert(input.strides==nil)
		assert(input.aOffsetQ==nil)
		input.strides={}
		input.aOffsetQ={}
		input.speeds={}
		input.offsets={}
		input.midPos={}
		input.dists={}
		input.L3poses={}
		input.neutralPoses={}
		local nleg=#input.contacts
		assert(nleg==input.numLegs)
		for ileg, Lcontact in ipairs(input.contacts) do

			local speed=COMtraj(Lcontact[1][2]):distance(COMtraj(Lcontact[2][2]))
			/(Lcontact[2][2]-Lcontact[1][2])*30.0
			input.speeds[ileg]=speed

			local Lstride={ Lcontact[1][1]*0.5+Lcontact[1][4]*0.5, Lcontact[2][1]*0.5+Lcontact[2][4]*0.5, }
			input.strides[ileg]=Lstride

			local isL=false
			if math.fmod(ileg-1,2)==0 then
				isL=true
			end

			if input.useMMoffsetQ then
				-- momentum-based computation of offset q.
				input.aOffsetQ[ileg]=self:computeOffsetQ_MM(Lcontact, rotY, COMtraj)
			elseif input.doNotUseOffsetQ then
				-- preserves original offsetq
				input.aOffsetQ[ileg]=self:identityOffsetQ(Lcontact, rotY, COMtraj)
			else
				if effectors:size()==nleg*2 then
					local i=(ileg-1)*2
					input.aOffsetQ[ileg]=self:computeOffsetQ(effectors(i+0), effectors(i+1), Lstride, Lcontact, rotY, COMtraj, isL) 
				else
					assert(false)
				end
			end

			local offsetQ=input.aOffsetQ[ileg]
			if effectors:size()==nleg*2 then
				local i=(ileg-1)*2
				input.offsets[ileg], input.midPos[ileg]=self:calcOffset(offsetQ[2], effectors(i+0), effectors(i+1), Lstride, Lcontact, rotY, COMtraj, isL, debugDraw, input)
			else
				input.offsets[ileg], input.midPos[ileg]=self:calcOffset_noToes(offsetQ[2], effectors(ileg-1), Lstride, Lcontact, rotY, COMtraj, isL, debugDraw, input)
			end
			local midSwing=Lstride[1]*0.5+Lstride[2]*0.5
			local dist_z=(rotY(midSwing):inverse()*(self.COMtraj(Lstride[2])-self.COMtraj(Lstride[1]))).z
			local dist_x=(rotY(midSwing):inverse()*(self.COMtraj(Lstride[2])-self.COMtraj(Lstride[1]))).x

			input.dists[ileg]={dist_z, dist_x}
		end


		-- remove global rotY and COM position components from self.mot
		for i=0, self.mot:numFrames()-1 do
			local roottf=MotionDOF.rootTransformation(self.mot:row(i))
			local comtf=transf(rotY:row(i), COMtraj:row(i))
			--comtf.translation.y=0
			-- root=com*delta
			local localtf=comtf:inverse()*roottf
			localtf.rotation:align(quater(1,0,0,0))
			MotionDOF.setRootTransformation(self.mot:row(i), localtf)

			if input.offsetPose then
				self.mot:row(i):assign(self.mot:row(i)+input.offsetPose)
			end
		end


		self.dmot=self.mot:calcDerivative(motionFrameRate)

		if #input.contacts==2 and not input.useMMoffsetQ then
			-- remove offsetQ
			local a=input.contacts[1][1][1]
			local b=input.contacts[2][1][1]
			local c=input.contacts[1][2][4]
			local d=input.contacts[2][2][4]
			local leg_a=1
			local leg_b=2
			if not (input.contacts[1][1][1]<input.contacts[2][1][1]) then
				a,b=b,a
				c,d=d,c
				leg_a=2
				leg_b=1
			end
			assert(a<b and b<c and c<d)

			local function removeOffsetQ(pose, dpose, offset_q)
				local roottf=MotionDOF.rootTransformation(pose)
				local invoq=offset_q:inverse()

				-- roottf1= rotY*oq*remaining_delta
				-- 		  = oq*remaining_delta
				-- roottf2= invoq*oq*remaining_delta
				MotionDOF.setRootTransformation( pose, transf( invoq, vector3(0, 0,0)) *roottf)

				dpose:setVec3(0, invoq*dpose:toVector3(0))
				dpose:setVec3(4, invoq*dpose:toVector3(4))
			end

			assert(input.aOffsetQ[leg_a][1]==a)
			for iframe=a, b-1 do
				local oq=input.aOffsetQ[leg_a][2](iframe-a)
				removeOffsetQ(self.mot:row(iframe), self.dmot:row(iframe), oq)
			end
			assert(input.aOffsetQ[leg_b][1]==b)
			for iframe=b, c-1 do
				local oq1=input.aOffsetQ[leg_a][2](iframe-a)
				local oq2=input.aOffsetQ[leg_b][2](iframe-b)
				local q=quater()
				q:safeSlerp(oq1, oq2, sop.map(iframe, b, c-1, 0, 1))
				removeOffsetQ(self.mot:row(iframe),self.dmot:row(iframe),  q)
			end
			for iframe=c, d-1 do
				local oq=input.aOffsetQ[leg_b][2](iframe-b)
				removeOffsetQ(self.mot:row(iframe), self.dmot:row(iframe), oq)
			end

			for iframe=a,d-1 do
				local x=self.mot:matView()(iframe,0)
				assert(x==x)
			end

			for ileg, Lcontact in ipairs(input.contacts) do

				local offsetQ= input.aOffsetQ[ileg][2]
				input.aOffsetQ[ileg][2]=nil

				--[[
				for ii=0, offsetQ:size()-1 do
					offsetQ(ii):identity()
				end
				]]
			end
		end


		for ileg, Lcontact in ipairs(input.contacts) do
			local samplePose1=self:_samplePose(ileg, 3)
			local samplePose2=self:_samplePose(ileg, 0)
			local neutralPose=blendPose(samplePose1, 0.5, samplePose2, 0.5)
			input.neutralPoses[ileg]=neutralPose

			local dist_z=input.dists[1][1]
			input.L3poses[ileg]={
				diffPose(neutralPose, samplePose1),
				diffPose(neutralPose, samplePose2)
			}
			self.dist_z=math.max(dist_z,0.1)
		end

		if fc.debugDraw then
			skin=fc.createSkin(loader, config.skinScale)
			skin:setPoseDOF(addPose(input.neutralPoses[1],input.L3poses[1][1]))
			skin:setTranslation(-100,0,0)
			skin2=fc.createSkin(loader, config.skinScale)
			skin2:setPoseDOF(self:linearRegressedPose({3,0}, 1.0))
			--local pose=self:_linearRegressedPose(1, 3.0, 14.0) skin2:setPoseDOF(pose)
			skin2:setTranslation(-200,0,0)
			skin3=fc.createSkin(loader, config.skinScale)
			skin3:setPoseDOF(self:linearRegressedPose({0,3}, 1.0))
			skin3:setTranslation(-300,0,0)
			--skin:setPoseDOF(input.neutralPoses[1])
			--skin3:setPoseDOF(input.L3poses[1][1]) -- start of L swing
		end
	end

	local inputm=input[motionType]
	if inputm.useMMoffsetQ or inputm.doNotUseOffsetQ then

		local minf=inputm.contacts[1][1][1]
		local maxf=inputm.contacts[1][2][4]
		for ileg, Lcontact in ipairs(inputm.contacts) do
			minf=math.min(minf, Lcontact[1][1])
			maxf=math.max(maxf, Lcontact[2][4])
		end

		local rootTraj=matrixn(maxf-minf,7)


		local function _getOffsetQ(iframe, oL, oR)
			if oL[1]>oR[1] then
				local oT=oL
				oL=oR
				oR=oT
			end
			assert(oL[1]<=oR[1])
			local Lmax=oL[1]+oL[2]:rows()
			assert(Lmax<=oR[1]+oR[2]:rows())
			if iframe<oR[1] then
				return oL[2](iframe-oL[1])
			elseif iframe>=Lmax then
				return oR[2](iframe-oR[1])
			else
				local w=sop.map(iframe, oR[1], Lmax, 0, 1)
				local q=quater()
				q:interpolate(w, oL[2](iframe-oL[1]), oR[2](iframe-oR[1]))
				return q
			end
		end
		local function getOffsetQ(iframe)
			return quater(1,0,0,0) -- already removed

			--[[
			local oL=inputm.aOffsetQ[1]
			local oR=inputm.aOffsetQ[2]
			if nlegs==2 then
				return _getOffsetQ(iframe,oL, oR)
			elseif nlegs==4 then
				if iframe>=oL[1]+oL[2]:size() and 
					iframe>=oR[1]+oR[2]:size() then 
					return _getOffsetQ(iframe, inputm.aOffsetQ[3], inputm.aOffsetQ[4])
				end
				return _getOffsetQ(iframe,oL, oR)
			else
				assert(false)
			end
			]]
		end
		for i=minf, maxf-1 do
			rootTraj:row(i-minf):setVec3(0, self.COMtraj(i))

			local qO=getOffsetQ(i)
			local qY=self.rotY(i)
			rootTraj:row(i-minf):setQuater(3, qY*qO)
		end
		if debugDraw then
			for i=minf, maxf-1 do
				local tf=MotionDOF.rootTransformation(rootTraj:row(i-minf))
				dbg.draw('Axes', tf, 'rootcom_'..i, 100,0.5)
			end
		end
		local dmot=MotionDOF.calcDerivative(rootTraj, 30)

		if debugDraw then
			local tf=MotionDOF.rootTransformation(rootTraj:row(0))
			--tf.translation.z=tf.translation.z+0.05
			for i=minf, maxf-2 do
				dbg.draw('Axes', tf, 'rootcomi_'..i, 100,0.5)
				tf:integrateBodyVel(dmot:row(i-minf):toVector3(4), dmot:row(i-minf):toVector3(0), 1/30)
				--tf:integrate({w=dmot:row(i-minf):toVector3(4), v=dmot:row(i-minf):toVector3(0)}, 1/30)
			end
		end

		self.rootTraj={minf, rootTraj, dmot}
	end
end


local function decomposePhase(phase)
	if phase>4-1e-3 then
		phase=4-1e-3
	end
	local Lphase=math.floor(phase)
	local Lweight=phase-Lphase
	return Lphase, Lweight
end
local function decomposePhase_raw(phase)
	local Lphase=math.floor(phase)
	local Lweight=phase-Lphase
	return Lphase, Lweight
end
-- weight always goes from 0 to 1
-- phase == 0, 1 (heel and toe), 2, or 3 (swing)
-- returns frame, swing
local function getFrame(weight, phase, contact)
	if weight>1 then weight =1 end
	if phase==0 then
		return sop.map(weight, 0, 1, contact[1][1], contact[1][2]), 1-weight
	elseif phase==1 then

		local midSpprt=contact[1][1]*0.5+contact[1][4]*0.5

		if weight<0.5 then
			return sop.map(weight, 0, 0.5, contact[1][2], midSpprt), 0  
		else
			return sop.map(weight, 0.5, 1, midSpprt, contact[1][3]), 0  
		end
	elseif phase==2 then
		return sop.map(weight, 0, 1, contact[1][3], contact[1][4]), weight
	elseif phase==3 then
		return sop.map(weight, 0, 1, contact[1][4], contact[2][1]), 1
	elseif phase==4 then
		return sop.map(weight, 0, 1, contact[2][1], contact[2][2]), 1-weight
	elseif phase==5 then
		local midSpprt=contact[2][1]*0.5+contact[2][4]*0.5

		if weight<0.5 then
			return sop.map(weight, 0, 0.5, contact[2][2], midSpprt), 0  
		else
			return sop.map(weight, 0.5, 1, midSpprt, contact[2][3]), 0  
		end
	elseif phase==6 then
		return sop.map(weight, 0, 1, contact[2][3], contact[2][4]), weight
	else
		assert(false)
	end
end
function fc.PoseSampler:computeOffsetQ(effToe, effHeel, stride, contact,  rotY, COMtraj, isL)
	local midOffsetQ={}
	local loader=self.loader
	local mot=self.mot

	for str=1,2 do
		local f
		if str==1 then
			f=math.round(stride[1])
		else
			f=math.round(stride[2])
		end
		loader:setPoseDOF(mot:row(f))
		local toePos=effToe.bone:getFrame()*effToe.localpos
		local heelPos=effHeel.bone:getFrame()*effHeel.localpos

		local midPos=toePos*0.5+heelPos*0.5
		midOffsetQ[str]=quater()
		midOffsetQ[str]:axisToAxis(vector3(0,1,0), COMtraj(f)-midPos)
	end
	local offsetQ=quaterN(contact[2][4]-contact[1][1])
	for f=contact[1][1], contact[2][4]-1 do
		local phase=frameToPhase(f, contact)
		local oq
		local q
		if f>=contact[2][1]-1e-3 then
			oq=midOffsetQ[2]
			q=rotY(stride[2])
		elseif phase<3 then
			oq=midOffsetQ[1]
			q=rotY(stride[1])
		else
			oq=quater()
			q=quater()
			oq:safeSlerp(midOffsetQ[1], midOffsetQ[2], phase-3)
			q:safeSlerp(rotY(stride[1]), rotY(stride[2]), phase-3)
		end
		local ii=f-contact[1][1]
		-- q*local_oq=oq*q
		-- -> local_oq= q:inverse()*oq*q
		offsetQ(ii):assign(q:inverse()*oq*q)
	end
	return {contact[1][1], offsetQ}
end
function fc.PoseSampler:computeOffsetQ_MM(contact, rotY, COMtraj)
	local loader=self.loader
	local startF=contact[1][1]
	local endF=contact[2][4]

	loader:setPoseDOF(self.mot:row(startF))

	local simulator=Physics.DynamicsSimulator_TRL_QP('libccd')
	simulator:registerCharacter(loader)
	simulator:init(1/30, Physics.DynamicsSimulator.EULER)

	local mMotionDOF=self.mot
	local dmot=calcDerivative(mMotionDOF)
	dmot:rmult(1.0/4.0)  -- 120hz vs 30hz

	local tf=transf(quater(1,0,0,0), COMtraj(startF))
	local COMori=quaterN(endF-startF)

	--dbg.draw('Axes', tf, 'root0', 100)
	for i=startF, endF-1 do
		simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, mMotionDOF:row(i))
		simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dmot:row(i))
		simulator:initSimulation() -- very important

		local vel=simulator:calcMomentumCOM(0)

		local I=vectorn()
		simulator:calcInertia(0, mMotionDOF:row(i), I)
		
		local m=I(6)
		local r=vector3(I(7), I(8), I(9))	
		local I6=CT.mat(6,6,I(0),I(3), I(4), 0, -r.z, r.y,
		I(3),I(1), I(5), r.z, 0,  -r.x,
		I(4),I(5), I(2), -r.y, r.x, 0 ,
		0, r.z, -r.y, m,   0 , 0,
		-r.z,0,  r.x, 0,  m,  0,
		r.y, -r.x, 0, 0,  0,  m)

		invI=CT.inverse(I6)
		local M=vectorn(6)
		M:setVec3(0, vel:M())
		M:setVec3(3, vel:F())
		local v=(invI*M:column()):column(0):copy()
		local V=Liegroup.se3(v:toVector3(0), v:toVector3(3))
		local invR=tf.rotation:inverse()
		V.w:rotate(invR)
		V.v:rotate(invR)


		COMori(i-startF):assign(tf.rotation)
		tf:integrate(V, 1/30)
	end

	local m1=math.round(contact[1][2]*0.5+contact[1][3]*0.5)
	local m2=math.round(contact[2][2]*0.5+contact[2][3]*0.5)

	local q1=COMori(m1-startF):inverse()
	local q2=COMori(m2-startF):inverse()

	if true then	
		-- remove accumulated error
		for i=startF, endF-1 do
			local COM=COMtraj(i)

			if i<m1 then
				COMori(i-startF):leftMult(q1)
			elseif i>=m2 then
				COMori(i-startF):leftMult(q2)
			else
				local q=quater()
				q:interpolate(sop.map(i, m1, m2, 0, 1), q1, q2)
				COMori(i-startF):leftMult(q)
			end
			local tf=transf(COMori(i-startF), COM)

			if debugDraw and math.fmod(i,3)==0 then
				dbg.draw('Sphere', COM*100, 'com'..i,'blue', 1 )
				dbg.draw('Axes', tf, 'root_'..i, 100,0.5)
			end
			local q=rotY(i)
			-- q*local_oq=COMori*q
			-- local_oq=q:inverse()*COMori
			COMori(i-startF):assign(q:inverse()*COMori(i-startF)*q)
		end
	end
	return {startF, COMori}
end
function fc.PoseSampler:identityOffsetQ(contact, rotY, COMtraj)
	local startF=contact[1][1]
	local endF=contact[2][4]

	local COMori=quaterN(endF-startF)

	for i=startF, endF-1 do
		COMori(i-startF):identity()
	end

	return {startF, COMori}
end

function fc.PoseSampler:calcOffset_noToes( offsetQ, effBall, stride, contact,  rotY, COMtraj, isL, debugDraw, input)
	local loader=self.loader
	local mot=self.mot
	local ballOffset=vector3N(contact[2][4]-contact[1][1])

	local midPos={}
	local midOffsetQ={}
	local ofootpos
	local g_footLen

	local ofoott=math.floor((stride[1]+stride[2])*0.5)
	for f=contact[1][1], contact[2][4]-1 do
		loader:setPoseDOF(mot:row(f))
		local ballPos=effBall.bone:getFrame()*effBall.localpos
		g_footLen=0

		local ii=f-contact[1][1]
		ballOffset(ii):assign(ballPos)
		if f==math.round(stride[1]) then
			midPos[1]=ballPos
			midOffsetQ[1]=offsetQ(f-contact[1][1])
		elseif f==math.round(stride[2]) then
			midPos[2]=ballPos
			midOffsetQ[2]=offsetQ(f-contact[1][1])
		elseif f==math.round(ofoott) then
			ofootpos=ballPos
		end
	end
	midPos[1].y=0
	midPos[2].y=0
	ofootpos.y=0
	for f=contact[1][1], contact[2][4]-1 do
		local phase=frameToPhase(f, contact)
		local ii=f-contact[1][1]
		local ballPos=ballOffset(ii):copy()
		local footMidPos
		local q
		local oq
		if f>=contact[2][1]-1e-3 then
			footMidPos=midPos[2]
			q=rotY(stride[2])
			oq=midOffsetQ[2]
		elseif phase<3 then
			footMidPos=midPos[1]
			q=rotY(stride[1])
			oq=midOffsetQ[1]
		else
			local rotY1=rotY(stride[1])
			local rotY2=rotY(stride[2])
			local rotYO=rotY(ofoott)
			local function sc(p,oq, q)
				return fc.composeSupportCoordinate(p, oq, q)
			end


			if input.isJump then
				local rotY=self.rotY(f)
				local COMori=self.offsetQ_L[2](f-self.offsetQ_L[1])
				local currCOM=transf(COMori*rotY, self.COMtraj(f))

				--local COM1=self.COMtraj(contact[1][4]).y
				--local COM2=self.COMtraj(contact[2][1]).y
				if scenario=='humanJump' then
					p, shear_q, _q=fc.calcSwingFootCoord2(sc(midPos[1],midOffsetQ[1], rotY1), sc(midPos[2], midOffsetQ[2], rotY2), phase-3, currCOM)
				else
					p, shear_q, _q=fc.calcSwingFootCoordJump(sc(midPos[1],midOffsetQ[1], rotY1), sc(midPos[2], midOffsetQ[2], rotY2), phase-3, currCOM)
				end
			else
				p, shear_q, _q=fc.calcSwingFootCoord(sc(midPos[1],midOffsetQ[1], rotY1), sc(midPos[2], midOffsetQ[2], rotY2), sc(ofootpos, quater(1,0,0,0), rotYO), phase-3, isL, 0)
			end

			if debugDraw then
				dbg.draw('Sphere', p*100, 's'..tostring(isL)..'_'..tostring(f), 'red',2)
			end
			--local p, shear_q, _q=fc.calcSwingFootCoord(sc(midPos[1],rotY1), sc(midPos[2], rotY2), sc(ofootpos, rotYO), phase-3, nil)
			footMidPos=p
			q=_q
			oq=shear_q
		end
		local Rphase, Rweight=decomposePhase(phase)
		local Rframe, Rswing=getFrame(Rweight, Rphase, contact)
		--print(footMidPos)
		--todo
		ballOffset(ii):assign(rotate(ballPos-footMidPos, q:inverse()))
	end

	if true then
		local offset=contact[1][1]

		local function fixXZ(offsetPos, offsetFrame, s, e)
			local mx=offsetPos:x():range(s-offsetFrame, e-offsetFrame)
			mx:setAllValue(mx:avg())
			local mz=offsetPos:z():range(s-offsetFrame, e-offsetFrame)
			mz:setAllValue(mz:avg())
		end
		fixXZ(ballOffset, offset, contact[1][1], contact[1][4])
		fixXZ(ballOffset, offset, contact[2][1], contact[2][4])
	end

	midPos[1]=rotY(stride[1]):inverse()*(midPos[1]-COMtraj(stride[1]))
	midPos[2]=rotY(stride[2]):inverse()*(midPos[2]-COMtraj(stride[2]))
	return {contact[1][1], ballOffset, offsetQ}, midPos
end

function fc.PoseSampler:calcOffset( offsetQ, effToe, effHeel, stride, contact,  rotY, COMtraj, isL, debugDraw, input)
	local loader=self.loader
	local mot=self.mot
	local toeOffset=vector3N(contact[2][4]-contact[1][1])
	local heelOffset=vector3N(contact[2][4]-contact[1][1])

	local midPos={}
	local midOffsetQ={}
	local ofootpos
	local g_footLen

	local ofoott=math.floor((stride[1]+stride[2])*0.5)
	for f=contact[1][1], contact[2][4]-1 do
		loader:setPoseDOF(mot:row(f))
		local toePos=effToe.bone:getFrame()*effToe.localpos
		local heelPos=effHeel.bone:getFrame()*effHeel.localpos
		g_footLen=toePos:distance(heelPos)

		local ii=f-contact[1][1]
		heelOffset(ii):assign(heelPos)
		toeOffset(ii):assign(toePos)
		if f==math.round(stride[1]) then
			midPos[1]=toePos*0.5+heelPos*0.5
			midOffsetQ[1]=offsetQ(f-contact[1][1])
		elseif f==math.round(stride[2]) then
			midPos[2]=toePos*0.5+heelPos*0.5
			midOffsetQ[2]=offsetQ(f-contact[1][1])
		elseif f==math.round(ofoott) then
			ofootpos=toePos*0.5+heelPos*0.5
		end
	end
	midPos[1].y=0
	midPos[2].y=0
	ofootpos.y=0
	for f=contact[1][1], contact[2][4]-1 do
		local phase=frameToPhase(f, contact)
		local ii=f-contact[1][1]
		local heelPos=heelOffset(ii):copy()
		local toePos=toeOffset(ii):copy()
		local footMidPos
		local q
		local oq
		if f>=contact[2][1]-1e-3 then
			footMidPos=midPos[2]
			q=rotY(stride[2])
			oq=midOffsetQ[2]
		elseif phase<3 then
			footMidPos=midPos[1]
			q=rotY(stride[1])
			oq=midOffsetQ[1]
		else
			local rotY1=rotY(stride[1])
			local rotY2=rotY(stride[2])
			local rotYO=rotY(ofoott)
			local function sc(p,oq, q)
				return fc.composeSupportCoordinate(p, oq, q)
			end


			if input.isJump then
				local rotY=self.rotY(f)
				local offsetQ=input.aOffsetQ[1]
				local COMori=offsetQ[2](f-offsetQ[1])
				local currCOM=transf(COMori*rotY, self.COMtraj(f))

				--local COM1=self.COMtraj(contact[1][4]).y
				--local COM2=self.COMtraj(contact[2][1]).y
				if scenario=='humanJump' then
					p, shear_q, _q=fc.calcSwingFootCoord2(sc(midPos[1],midOffsetQ[1], rotY1), sc(midPos[2], midOffsetQ[2], rotY2), phase-3, currCOM)
				else
					p, shear_q, _q=fc.calcSwingFootCoordJump(sc(midPos[1],midOffsetQ[1], rotY1), sc(midPos[2], midOffsetQ[2], rotY2), phase-3, currCOM)
				end
			else
				p, shear_q, _q=fc.calcSwingFootCoord(sc(midPos[1],midOffsetQ[1], rotY1), sc(midPos[2], midOffsetQ[2], rotY2), sc(ofootpos, quater(1,0,0,0), rotYO), phase-3, isL, 0)
			end

			if debugDraw then
				dbg.draw('Sphere', p*100, 's'..tostring(isL)..'_'..tostring(f), 'red',2)
			end
			--local p, shear_q, _q=fc.calcSwingFootCoord(sc(midPos[1],rotY1), sc(midPos[2], rotY2), sc(ofootpos, rotYO), phase-3, nil)
			footMidPos=p
			q=_q
			oq=shear_q
		end
		local Rphase, Rweight=decomposePhase(phase)
		local Rframe, Rswing=getFrame(Rweight, Rphase, contact)
		--print(footMidPos)
		if input.isJump and scenario~='humanJump' then
			local offsetQ=input.aOffsetQ[1]
			local COMori=offsetQ[2](f-offsetQ[1])
			isHeel, hp,tp, qq= fc.getHeelAndToePosition(rotY(f), COMtraj(f), footMidPos, vector3(0,1,0), Rswing, g_footLen, input, q, COMori)
			heelOffset(ii):assign(rotate(heelPos-hp, qq:inverse()))
			toeOffset(ii):assign(rotate(toePos-tp, qq:inverse()))
		else
			isHeel, hp,tp= fc.getHeelAndToePosition(rotY(f), COMtraj(f), footMidPos, vector3(0,1,0), Rswing, g_footLen )
			heelOffset(ii):assign(rotate(heelPos-hp, q:inverse()))
			toeOffset(ii):assign(rotate(toePos-tp, q:inverse()))
		end

	end

	if true then
		local offset=contact[1][1]

		local function fixXZ(offsetPos, offsetFrame, s, e)
			local mx=offsetPos:x():range(s-offsetFrame, e-offsetFrame)
			mx:setAllValue(mx:avg())
			local mz=offsetPos:z():range(s-offsetFrame, e-offsetFrame)
			mz:setAllValue(mz:avg())
		end
		fixXZ(toeOffset, offset, contact[1][1], contact[1][4])
		fixXZ(toeOffset, offset, contact[2][1], contact[2][4])
		fixXZ(heelOffset, offset, contact[1][1], contact[1][3])
		fixXZ(heelOffset, offset, contact[2][1], contact[2][3])
	end

	midPos[1]=rotY(stride[1]):inverse()*(midPos[1]-COMtraj(stride[1]))
	midPos[2]=rotY(stride[2]):inverse()*(midPos[2]-COMtraj(stride[2]))
	return {contact[1][1], heelOffset, toeOffset, offsetQ}, midPos
end
function fc.PoseSampler:frameToPhase(frame)
	local phases={}
	for ileg, Lcontact in ipairs(self.contact[1]) do
		phases[ileg]=frameToPhase(frame, Lcontact)
	end
	return unpack(phases)
end
local function linearRegressedPose(phase, p1, p2, scale, neutralPose)
	local weight
	if phase>3 then
		weight=sop.map(phase,3,4, 1, 0)
	else
		weight=sop.map(phase,0,3, 0, 1)
	end
	local pose=blendPose(p1, weight, p2, 1-weight)
	pose=scalePose(pose, scale)
	return addPose(neutralPose, pose)
	--return neutralPose
end

function fc.PoseSampler:_linearRegressedPose(ileg, phaseL, Ldist_z)
	local input=self.input
	local LR3poses=input.L3poses[ileg]
	local neutralPose=input.neutralPoses[ileg]
	local Lpose=linearRegressedPose(phaseL, LR3poses[1], LR3poses[2], Ldist_z/self.dist_z, neutralPose)
	local weightsL=self.weights[1][ileg]
	return Lpose, weightsL
end
function fc.PoseSampler:linearRegressedPose(phases, dist_z)
	local maxScale=1.5
	if self.input.maxScale then
		maxScale=self.input.maxScale 
	end
	local dist_z=math.min(self.dist_z*maxScale, dist_z)  -- parameter : max scaling (signal processing)

	local weightsO=self.weights[2]

	local pose
	local avgPose
	for ileg, phaseL in ipairs(phases) do
		local Lpose, weightsL=self:_linearRegressedPose(ileg,phaseL, dist_z)
		if not avgPose then
			avgPose=vectorn(Lpose:size())
			avgPose:setAllValue(0)
			pose=avgPose:copy()
		end
		pose:radd(Lpose*weightsL)
		avgPose:radd(Lpose)
	end
	pose:radd(avgPose*(1.0/#phases)*weightsO)
	pose:setQuater(3, pose:toQuater(3):Normalize())
	return pose
end


function fc.PoseSampler:_sampleCOM(ileg, phaseL)
	local Lphase, Lweight=decomposePhase_raw(phaseL)
	local contacts, _offsetQ=unpack(self.contact)
	local Lcontact=contacts[ileg]
	local Lframe, Lswing=getFrame(Lweight, Lphase, Lcontact)

	local function sampleOffsetQ(Rframe, offsetsR)
		local r=Rframe-offsetsR[1]
		r=math.min(r, offsetsR[3]:rows()-1)
		local localoffset=offsetsR[4]:sampleRow(r)
		local rotY=self.rotY:sampleRow(Rframe)
		return rotY*localoffset 
	end

	local input=self.input
	local loffsetQ=sampleOffsetQ(Lframe, input.offsets[ileg])
	local lCOM=self.COMtraj:sampleRow(Lframe)
	return transf(loffsetQ, lCOM)
end
function fc.PoseSampler:sampleCOM(phaseL, phaseR)
	local loffset=self:_sampleCOM(1,phaseL)
	local roffset=self:_sampleCOM(2,phaseR)
	local offset=loffset:copy()
	offset.rotation:safeSlerp(loffset.rotation, roffset.rotation, 0.5)
	return offset
end

function fc.PoseSampler:sampleCOMvel(phaseL, phaseR)
	local Lphase, Lweight=decomposePhase_raw(phaseL)
	local Rphase, Rweight=decomposePhase_raw(phaseR)
	local contacts, _offsetQ=unpack(self.contact)
	local Lcontact=contacts[1]
	local Rcontact=contacts[2]
	local Lframe, Lswing=getFrame(Lweight, Lphase, Lcontact)
	local Rframe, Rswing=getFrame(Rweight, Rphase, Rcontact)


	function matrixn:sampleRow2(t)
		local out=vectorn()
		self:sampleRow(t, out)
		return out
	end

	local dposeL=self.rootTraj[3]:sampleRow2(Lframe-self.rootTraj[1])
	local dposeR=self.rootTraj[3]:sampleRow2(Rframe-self.rootTraj[1])
	if true then
		-- remove discontinuity
		if phaseL>=0 and phaseL<3 then
			local Lframe2, Lswing2=getFrame(Lweight, Lphase+4, Lcontact)
			local dposeL2=self.rootTraj[3]:sampleRow2(Lframe2-self.rootTraj[1])
			local w=sop.map(phaseL, 0, 3, 0, 1)
			dposeL=dposeL*w+dposeL2*(1-w)
		end
		if phaseR>=0 and phaseR<3 then
			local Rframe2, Rswing2=getFrame(Rweight, Rphase+4, Rcontact)
			local dposeR2=self.rootTraj[3]:sampleRow2(Rframe2-self.rootTraj[1])
			local w=sop.map(phaseR, 0, 3, 0, 1)
			dposeR=dposeR*w+dposeR2*(1-w)
		end
	end
	local dpose= dposeL*0.5+dposeR*0.5

	return dpose
end

-- phase goes continuously from 0 to 4
function fc.PoseSampler:samplePose(phaseL, phaseR, Ldist_z, Rdist_z)
	local a, b=self:samplePose2({phaseL, phaseR}, Ldist_z)
	return a, unpack(b)
end
-- 왼발과 오른발이 다른 pose를 다른 weight로 따라가는 경우.
function fc.PoseSampler:blendPose(refPose, poses, weights)
	local weightsO=self.weights[2]
	local poses2={}
	local pose
	local avgPose
	for ileg, w in ipairs(weights) do
		local rpose=poses[ileg]
		local Lpose=blendPose(refPose, w, rpose, 1-w)
		local weightsL=self.weights[1][ileg]
		if not avgPose then
			avgPose=vectorn(Lpose:size())
			avgPose:setAllValue(0)
			pose=avgPose:copy()
		end
		pose:radd(Lpose*weightsL)
		avgPose:radd(Lpose)
	end
	avgPose=avgPose*(1.0/#weights)
	avgPose:setQuater(3, avgPose:toQuater():Normalize())
	pose:radd(avgPose*weightsO)
	pose:setQuater(3, pose:toQuater(3):Normalize())
	return pose
end
function fc.PoseSampler:blendPose2(refPose, poses, weights)
	local weightsO=self.weights[2]
	local poses2={}
	local pose
	local avgPose

	for ileg, w in ipairs(weights) do
		local rpose=poses[ileg]
		local Lpose=blendPose(refPose, w, rpose, 1-w)
		local weightsL=self.weights[1][ileg]
		if not avgPose then
			avgPose=vectorn(Lpose:size())
			avgPose:setAllValue(0)
			pose=avgPose:copy()
		end
		pose:radd(Lpose*weightsL)
	end
	assert(#weights==2)
	local aw=math.max(unpack(weights))
	avgPose=blendPose(refPose, aw, poses[1], 1-aw)
	avgPose:radd(blendPose(refPose, aw, poses[2], 1-aw))
	avgPose=avgPose*(1.0/#weights)
	avgPose:setQuater(3, avgPose:toQuater():Normalize())
	pose:radd(avgPose*weightsO)
	pose:setQuater(3, pose:toQuater(3):Normalize())
	return pose
end

-- stride_phase goes from 0 to 1 (mid stance to next-mid stance)
function fc.PoseSampler:_sampleStridePose(ileg, stride_phase, dist_z, config)
	local s=1
	local dist_z_thr=1.5
	if config then
		dist_z_thr=config.dist_z_thr or dist_z_thr
	end
	if dist_z and dist_z>self.dist_z then
		s=dist_z/self.dist_z
		if s>dist_z_thr then
			dist_z=self.dist_z*dist_z_thr -- limit dist_z
		end
	end
	local contact=self.contact[1][ileg]
	local mid1=contact[1][2]*0.5+contact[1][3]*0.5
	local mid2=contact[2][2]*0.5+contact[2][3]*0.5
	local f=sop.map(stride_phase, 0, 1, mid1, mid2)
	local phase=math.max(frameToPhase(f, contact),0)
	--return self:_samplePose(ileg, phase)
	return self:_samplePose2(ileg, phase, dist_z)
end
function fc.PoseSampler:legAngleScale(pose, avgPose, dist_z, config)
	local s=1
	if dist_z>self.dist_z then
		s=dist_z/self.dist_z
	end
	if not config then 
		config={}
	end
	if s>1 and input.bones then
		local loader=mMot.loader
		local dofInfo=loader.dofInfo
		--local sh={MotionLoader.LEFTHIP, MotionLoader.RIGHTHIP, MotionLoader.LEFTSHOULDER, MotionLoader.RIGHTSHOULDER}
		local sh={MotionLoader.LEFTHIP, MotionLoader.RIGHTHIP}

		--if not g_min then
		--	g_min=100
		--	g_max=-100
		--end
		for ish, b in ipairs(sh) do
			local bone=loader:getBoneByVoca(b)

			local sf=config.hip_sf or 0.6 -- hip 
			local radianMid=-math.rad(7) -- 과장 중간점 (기본자세)

			-- 마지막 DOF가 진행방향 관련이라 가정하였음.
			local nchannels=bone:numChannels()
			local dofIndex=dofInfo:startR(bone:treeIndex())+nchannels-1
			pose:set(dofIndex, radianMid+(avgPose(dofIndex)-radianMid)*(1+ (s-1)*sf)) -- s가 2일때 30% 과장
			bone=bone:childHead()

			-- knee
			sf=config.knee_sf or sf
			radianMid=config.knee_mid or math.rad(20)
			local dofIndex=dofInfo:startR(bone:treeIndex())
			if avgPose(dofIndex)>radianMid then -- knee is an one-way joint
				pose:set(dofIndex, radianMid+(avgPose(dofIndex)-radianMid)*(1+ (s-1)*sf)) -- s가 2일때 30% 과장
			else
				pose:set(dofIndex, avgPose(dofIndex))
			end
		end

		local sh={MotionLoader.LEFTSHOULDER, MotionLoader.RIGHTSHOULDER}

		--if not g_min then
		--	g_min=100
		--	g_max=-100
		--end
		for ish, b in ipairs(sh) do
			local bone=loader:getBoneByVoca(b)

			local sf=0.4 -- shdr
			local radianMid=math.rad(16) -- 과장 중간점 (기본자세)
			if scenario=='fastgaps' then
				sf=sop.clampMap(dist_z,2,4, 0.4, 0.8)
				radianMid=math.rad(sop.clampMap(dist_z,2,4, 16,-0))
			end
			if ish==2 then
				radianMid=radianMid*-1
			end


			-- 마지막 DOF가 진행방향 관련이라 가정하였음.
			local nchannels=bone:numChannels()
			local dofIndex=dofInfo:startR(bone:treeIndex())+nchannels-1
			pose:set(dofIndex, radianMid+(avgPose(dofIndex)-radianMid)*(1+ (s-1)*sf)) -- s가 2일때 30% 과장
			bone=bone:childHead()

			-- elbow
			sf=0.0
			radianMid=config.knee_mid or math.rad(20)
			local dofIndex=dofInfo:startR(bone:treeIndex())
			if avgPose(dofIndex)>radianMid then -- knee is an one-way joint
				pose:set(dofIndex, radianMid+(avgPose(dofIndex)-radianMid)*(1+ (s-1)*sf)) -- s가 2일때 30% 과장
			else
				pose:set(dofIndex, avgPose(dofIndex))
			end
		end
	end
end
function fc.PoseSampler:sampleStridePose(stride_phases, dist_z, config)
	local weightsO=self.weights[2]
	local nleg=#stride_phases

	local pose
	local avgPose


	for ileg, phaseL in ipairs(stride_phases) do
		local Lpose =self:_sampleStridePose(ileg, phaseL, dist_z, config)
		local weightsL=self.weights[1][ileg]
		if not avgPose then
			avgPose=vectorn(Lpose:size())
			avgPose:setAllValue(0)
			pose=avgPose:copy()
		end
		pose:radd(Lpose*weightsL)
		avgPose:radd(Lpose)
	end
	avgPose=avgPose*(1.0/#stride_phases)
	avgPose:setQuater(3, avgPose:toQuater(3):Normalize())


	local I=vectorn()


	pose:radd(avgPose*weightsO)

	self:legAngleScale(pose, avgPose, dist_z, config)

	if not self.isJump then
		local solver=self.MMhelper.LtoT
		solver:setPoseDOF(self.loader.dofInfo, avgPose)
		solver:calcInertia(self.loader, I)

		local totalmass=I(6)
		-- use 6 by 6
		local m=totalmass
		local r=vector3(I(7), I(8), I(9))	
		local I6=CT.mat(6,6,I(0),I(3), I(4), 0, -r.z, r.y,
		I(3),I(1), I(5), r.z, 0,  -r.x,
		I(4),I(5), I(2), -r.y, r.x, 0 ,
		0, r.z, -r.y, m,   0 , 0,
		-r.z,0,  r.x, 0,  m,  0,
		r.y, -r.x, 0, 0,  0,  m)

		invI=CT.inverse(I6)
		self.MMhelper.chain:setPoseDOF(pose)
		local V=solver:calcMomentumCOMfromPose(self.loader, 1.0, self.MMhelper.chain)
		local mM=V:M()
		local mF=V:F()
		local m=CT.mat(6,1,mM.x, mM.y, mM.z, mF.x, mF.y, mF.z)
		local ww=invI*m
		local w=vector3(ww(0,0), ww(1,0), ww(2,0))
		local posErr=vector3(ww(3,0), ww(4,0), ww(5,0))
		local wq=quater()
		wq:setRotation(w)

		local tf=transf(wq, posErr)
		MotionDOF.setRootTransformation(pose, tf*MotionDOF.rootTransformation(pose))
	end


	return pose,  avgPose
end

-- using only one leg.
function fc.PoseSampler:_samplePose2(ileg, phase, dist_z)
	local avgPose, weightsL, swingInfoL=self:_samplePose(ileg, phase)

	local pose=avgPose
	if dist_z then
		local pose_o=self:_linearRegressedPose(ileg, phase, self.dist_z)
		local pose_d=diffPose(pose_o, pose)
		local pose_n=self:_linearRegressedPose(ileg, phase, dist_z)
		pose=addPose(pose_n, pose_d)
	end

	return pose, swingInfoL, avgPose
end
function fc.PoseSampler:samplePose2(phases, dist_z)

	local weightsO=self.weights[2]
	local nleg=#phases

	local pose
	local avgPose

	local swingInfo={}
	for ileg, phaseL in ipairs(phases) do
		local Lpose, weightsL, swingInfoL=self:_samplePose(ileg, phaseL)
		if not avgPose then
			avgPose=vectorn(Lpose:size())
			avgPose:setAllValue(0)
			pose=avgPose:copy()
		end
		pose:radd(Lpose*weightsL)
		avgPose:radd(Lpose)
		table.insert(swingInfo, swingInfoL)
	end
	avgPose=avgPose*(1.0/#phases)
	avgPose:setQuater(3, avgPose:toQuater(3):Normalize())

	local I=vectorn()


	pose:radd(avgPose*weightsO)


	if dist_z then
		local pose_o=self:linearRegressedPose(phases, self.dist_z)
		local pose_d=diffPose(pose_o, pose)
		local pose_n=self:linearRegressedPose(phases, dist_z)

		if fc.debugDraw then

			local  ileg=1
			local input=self.input
			local LR3poses=input.L3poses[ileg]
			skin:setPoseDOF(LR3poses[1])
			skin2:setPoseDOF(LR3poses[2])
			skin3:setPoseDOF(pose_n)
		end
		pose=addPose(pose_n, pose_d)
	end

	if not self.isJump then
		local solver=self.MMhelper.LtoT
		solver:setPoseDOF(self.loader.dofInfo, avgPose)
		solver:calcInertia(self.loader, I)

		local totalmass=I(6)
		-- use 6 by 6
		local m=totalmass
		local r=vector3(I(7), I(8), I(9))	
		local I6=CT.mat(6,6,I(0),I(3), I(4), 0, -r.z, r.y,
		I(3),I(1), I(5), r.z, 0,  -r.x,
		I(4),I(5), I(2), -r.y, r.x, 0 ,
		0, r.z, -r.y, m,   0 , 0,
		-r.z,0,  r.x, 0,  m,  0,
		r.y, -r.x, 0, 0,  0,  m)

		invI=CT.inverse(I6)
		self.MMhelper.chain:setPoseDOF(pose)
		local V=solver:calcMomentumCOMfromPose(self.loader, 1.0, self.MMhelper.chain)
		local mM=V:M()
		local mF=V:F()
		local m=CT.mat(6,1,mM.x, mM.y, mM.z, mF.x, mF.y, mF.z)
		local ww=invI*m
		local w=vector3(ww(0,0), ww(1,0), ww(2,0))
		local posErr=vector3(ww(3,0), ww(4,0), ww(5,0))
		local wq=quater()
		wq:setRotation(w)

		local tf=transf(wq, posErr)
		MotionDOF.setRootTransformation(pose, tf*MotionDOF.rootTransformation(pose))
	end
	return pose, swingInfo, avgPose
end
function fc.PoseSampler:sampleDPose(phases, dist_z)

	local weightsO=self.weights[2]
	local nleg=#phases

	local dpose
	local avgdPose

	for ileg, phaseL in ipairs(phases) do
		local Ldpose, weightsL=self:_sampleDPose(ileg, phaseL)
		if not avgdPose then
			avgdPose=vectorn(Ldpose:size())
			avgdPose:setAllValue(0)
			dpose=avgdPose:copy()
		end
		dpose:radd(Ldpose*weightsL)
		avgdPose:radd(Ldpose)
	end
	dpose:radd(avgdPose*(1.0/#phases)*weightsO)

	if dist_z then
		dpose:rmult(dist_z/self.dist_z)
	end
	return dpose
end
function fc.PoseSampler:_samplePose(ileg, phaseL)
	local input=self.input
	if #input.offsets[ileg]==3 then
		return self:_samplePose_noToes(ileg, phaseL)
	end

	local loader=self.loader
	local mot=self.mot
	local Lphase, Lweight=decomposePhase(phaseL)

	local weightsL=self.weights[1][ileg]

	local contacts, _offsetQ=unpack(self.contact)
	local Lcontact=contacts[ileg]

	local Lframe, Lswing=getFrame(Lweight, Lphase, Lcontact)

	local Lpose=vectorn()
	mot:samplePose(Lframe, Lpose)


	local function sampleOffset(Rframe, offsetsR)
		local r=Rframe-offsetsR[1]
		r=math.min(r, offsetsR[3]:rows()-1)
		local toePos=offsetsR[3]:sampleRow(r)
		local heelPos=offsetsR[2]:sampleRow(r)
		--local localoffset=offsetsR[4]:sampleRow(r)

		--return { heelPos, toePos, localoffset }
		return { heelPos, toePos }
	end

	local loffset=sampleOffset(Lframe, input.offsets[ileg])
	--[[
	local function removeOffsetQ(pose, offset_q, COM_y)
		local roottf=MotionDOF.rootTransformation(pose)
		MotionDOF.setRootTransformation( pose, transf(offset_q:inverse(), vector3(0, 0,0)) *roottf)
	end

	local COM_Ly=self.COMtraj:sampleRow(Lframe).y
	removeOffsetQ(Lpose, loffset[3], COM_Ly)
	]]

	if true then
		-- remove discontinuity
		if phaseL>=0 and phaseL<3 then
			local Lpose2=vectorn()
			local Lframe2, Lswing2=getFrame(Lweight, Lphase+4, Lcontact)
			mot:samplePose(Lframe2, Lpose2)
			local loffset2=sampleOffset(Lframe2, input.offsets[ileg])
			--[[

			local COM_Ly=self.COMtraj:sampleRow(Lframe2).y
			removeOffsetQ(Lpose2, loffset2[3], COM_Ly)
			]]

			local w=sop.map(phaseL, 0, 3, 0, 1)
			Lpose=Lpose*w+Lpose2*(1-w)

			loffset[1]=loffset[1]*w+loffset2[1]*(1-w)
			loffset[2]=loffset[2]*w+loffset2[2]*(1-w)

		end
	end

	if _offsetQ then
		Lpose:setQuater(3, _offsetQ*Lpose:toQuater(3):Normalize())
	else
		Lpose:setQuater(3, Lpose:toQuater(3):Normalize())
	end
	return Lpose, weightsL, {Lswing, Lweight, loffset}

end
function fc.PoseSampler:_sampleDPose(ileg, phaseL)
	local input=self.input

	local loader=self.loader
	local dmot=self.dmot
	local Lphase, Lweight=decomposePhase(phaseL)

	local weightsL=self.weights[1][ileg]

	local contacts, _offsetQ=unpack(self.contact)
	local Lcontact=contacts[ileg]

	local Lframe, Lswing=getFrame(Lweight, Lphase, Lcontact)

	local Ldpose=vectorn()
	dmot:sampleRow(Lframe, Ldpose)

	if true then
		-- remove discontinuity
		if phaseL>=0 and phaseL<3 then
			local Ldpose2=vectorn()
			local Lframe2, Lswing2=getFrame(Lweight, Lphase+4, Lcontact)
			dmot:sampleRow(Lframe2, Ldpose2)

			local w=sop.map(phaseL, 0, 3, 0, 1)
			Ldpose=Ldpose*w+Ldpose2*(1-w)

		end
	end

	return Ldpose, weightsL
end


function fc.PoseSampler:_saveOffset(ileg)

	local t=vectorn(15)
	t:linspace(0,4)
	local p1=vector3N(t:size())
	local p2=vector3N(t:size())
	local q=quaterN(t:size())
	for i=0, t:size()-1 do
		local p,w,s=self:_samplePose(ileg, t(i))
		p1(i):assign(s[3][1])
		p2(i):assign(s[3][2])
		q(i):assign(s[3][3])
	end
	util.saveTableToLua({p1,p2,q}, '__offsets.lua')
end
function fc.PoseSampler:_samplePose_noToes(ileg, phaseL)
	local input=self.input
	local loader=self.loader
	local mot=self.mot
	local Lphase, Lweight=decomposePhase(phaseL)

	local weightsL=self.weights[1][ileg]

	local contacts, _offsetQ=unpack(self.contact)
	local Lcontact=contacts[ileg]

	local Lframe, Lswing=getFrame(Lweight, Lphase, Lcontact)

	local Lpose=vectorn()
	mot:samplePose(Lframe, Lpose)

	local function sampleOffset(Rframe, offsetsR)
		local r=Rframe-offsetsR[1]
		r=math.min(r, offsetsR[2]:rows()-1)
		local ballPos=offsetsR[2]:sampleRow(r)
		--local localoffset=offsetsR[3]:sampleRow(r)

		--return { ballPos, localoffset }
		return { ballPos}
	end

	local loffset=sampleOffset(Lframe, input.offsets[ileg])
	--[[
	local function removeOffsetQ(pose, offset_q, COM_y)
		local roottf=MotionDOF.rootTransformation(pose)
		MotionDOF.setRootTransformation( pose, transf(offset_q:inverse(), vector3(0, 0,0)) *roottf)
	end

	local COM_Ly=self.COMtraj:sampleRow(Lframe).y
	removeOffsetQ(Lpose, loffset[2], COM_Ly)
	]]

	if true then
		-- remove discontinuity
		if phaseL>=0 and phaseL<3 then
			local Lpose2=vectorn()
			local Lframe2, Lswing2=getFrame(Lweight, Lphase+4, Lcontact)
			mot:samplePose(Lframe2, Lpose2)
			local loffset2=sampleOffset(Lframe2, input.offsets[ileg])

			--[[
			local COM_Ly=self.COMtraj:sampleRow(Lframe2).y
			removeOffsetQ(Lpose2, loffset2[2], COM_Ly)
			]]

			local w=sop.map(phaseL, 0, 3, 0, 1)
			Lpose=Lpose*w+Lpose2*(1-w)

			loffset[1]=loffset[1]*w+loffset2[1]*(1-w)

		end
	end

	if _offsetQ then
		Lpose:setQuater(3, _offsetQ*Lpose:toQuater(3):Normalize())
	else
		Lpose:setQuater(3, Lpose:toQuater(3):Normalize())
	end
	return Lpose, weightsL, {Lswing, Lweight, loffset}
end


function fc.createIKsolver(loader, config)
	local out={}
	local mEffectors=MotionUtil.Effectors()
	local numCon=#config
	mEffectors:resize(numCon);
	out.effectors=mEffectors
	out.numCon=numCon
	local kneeDofs=vectorn()

	for i=0, numCon-1 do
		local conInfo=config[i+1]
		local kneeInfo=1
		if #conInfo==2 then
			kneeInfo=0
		end
		mEffectors(i):init(loader:getBoneByName(conInfo[kneeInfo+1]), conInfo[kneeInfo+2])
		if kneeInfo~=0 then
			kneeDofs:pushBack(loader.dofInfo:startR(loader:getBoneByName(conInfo[kneeInfo]):treeIndex()))
		end
	end
	if kneeDofs:size()>0 then
		out.kneeDOFs=kneeDofs
	end
	--out.solver=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(loader.dofInfo, mEffectors,g_con);
	out.solver=MotionUtil.createFullbodyIk_MotionDOF_MultiTarget_lbfgs(loader.dofInfo);
	return out
end

function calcLocalFoot(pendState, pendRotY , conindex, i, footinfo, footinfo_corr, bone, localtoepos, localheelpos)

	local refFrame=i+footinfo.currSpprtFrame
	footinfo.currSpprt=getPendTransform(pendState:row(refFrame))*transf(pendRotY(refFrame), vector3(0,0,0))
	footinfo.duration=footinfo.refFrame-footinfo.currSpprtFrame
	local delta, footOriRefCoord, zmp_coord_ref=calcGlobalFootCoord(pendState, pendRotY, conindex, i, footinfo, footinfo_corr, localtoepos, localheelpos)

	local globalFoot=bone:getFrame():copy()
	globalFoot.translation:assign(bone:getFrame()*localtoepos)
	--globalFoot.translation=zmp_coord_ref*localPos+delta
	footinfo.localPos=zmp_coord_ref:inverse()*(globalFoot.translation-delta)
	--globalFoot.rotation=footOriRefCoord*localOri
	footinfo.localOri=footOriRefCoord:inverse()*globalFoot.rotation
end
function fc.calcGlobalFoot(pendState,pendRotY, conindex, i, footinfo, footinfo_corr, refPose, skel, bone, localpos, localheelpos)
	local foot=footinfo.localPos
	local footOri=footinfo.localOri


	local delta, footOriRefCoord, zmp_coord_ref=calcGlobalFootCoord(pendState,pendRotY, conindex, i, footinfo, footinfo_corr, localpos, localheelpos)
	local desiredProjPos=zmp_coord_ref*foot
	local currOri=footOriRefCoord*footOri
	return desiredProjPos+delta, currOri, zmp_coord_ref.rotation
end

function fc.shearM(shear_q)
	local shearDir=vector3(0,1,0)
	shearDir:rotate(shear_q)
	return CT.shearY(shearDir)
end

-- shear_q is represented in the world coord.
-- spprt=transf(shear_q, p)*transf(q, vector3(0,0,0))
function fc.calcSwingFootCoord(currSpprt, nextSpprt, ofootSpprt, weight, isL, _ofootmod, gamma)
	local p,shear_q,q=fc.decomposeSupportCoordinate(currSpprt)
	local p2,shear_q2,q2=fc.decomposeSupportCoordinate(nextSpprt)
	local ofootpos,oshear_q,oq=fc.decomposeSupportCoordinate(ofootSpprt)
	local optGamma=gamma or 1.26
	if true then
		-- use heuristic
		local d= currSpprt.translation:distance(nextSpprt.translation)
		if d<0.4 then
			optGamma=sop.clampMap(d, 0.4, 0.2, 1.26, 1)
			print(optGamma)
		end
	end
	weight=math.smoothTransition(math.pow(weight, optGamma))
	assert(weight==weight)

	--if isL ~=nil then
	--	--dbg.draw('Sphere', ofootpos*100, 'offot'..tostring(isL),'red',15)
	--	if isL then
	--		dbg.draw('Sphere', p*100, 'offot'..tostring(isL),'red',15)
	--		dbg.draw('Sphere', p2*100, 'oflfot'..tostring(isL),'green',15)
	--		dbg.draw('Sphere', ofootpos*100, 'orffot'..tostring(isL),'blue',15)
	--	end
	--end
	--p:assign(arc.arcInterpolation(weight, p:copy(), p2, ofootPos))
	--p:assign(arc.lineInterpolation(weight, p:copy(), p2))
	assert(ofootpos)


	local v_ofootmod=vector3(_ofootmod or 0.05 ,0,0)

	if isL ~=nil then
		if isL then 
			p:assign(arc.arcInterpolationL(weight, p:copy(), p2, ofootpos+2*(oq*v_ofootmod)))
		else
			p:assign(arc.arcInterpolationR(weight, p:copy(), p2, ofootpos-2*(oq*v_ofootmod)))
		end
	else
		p:assign(arc.lineInterpolation(weight, p:copy(), p2))
	end
	--if isL then
	--	p:assign(arc.arcInterpolationL(weight, p:copy(), p2, ofootPos))
	--else
	--	p:assign(arc.arcInterpolationR(weight, p:copy(), p2, ofootPos))
	--end

	q:safeSlerp(q:copy(), q2, weight)
	shear_q:safeSlerp(shear_q:copy(), shear_q2, weight)
	return p, shear_q, q
	--return p, shearM(shear_q), q
end
function fc.calcSwingFootCoord2(currSpprt, nextSpprt, weight, roottf)
	local p,shear_q,q=fc.decomposeSupportCoordinate(currSpprt)
	local p2,shear_q2,q2=fc.decomposeSupportCoordinate(nextSpprt)
	local optGamma=1.26
	if true then
		-- use heuristic
		local d= currSpprt.translation:distance(nextSpprt.translation)
		if d<0.4 then
			optGamma=sop.clampMap(d, 0.4, 0.2, 1.26, 1)
			print(optGamma)
		end
	end
	weight=math.smoothTransition(math.pow(weight, optGamma))
	assert(weight==weight)

	p:assign(arc.lineInterpolation(weight, p:copy(), p2))

	q:safeSlerp(q:copy(), q2, weight)
	shear_q:safeSlerp(shear_q:copy(), shear_q2, weight)
	return p, shear_q, q
	--return p, shearM(shear_q), q
end
function fc.calcSwingFootCoordJump(currSpprt, nextSpprt, weight, currCOM)

	local p1=currSpprt.translation
	local p2=nextSpprt.translation
	local currSpprtCOM=transf(quater(1,0,0,0), vector3(p1.x, 1, p1.z))
	local nextSpprtCOM=transf(quater(1,0,0,0), vector3(p2.x, 1, p2.z))

	-- currSpprt=COM*localSpprt
	local localSpprt1=currSpprtCOM:inverse()*currSpprt
	local localSpprt2=nextSpprtCOM:inverse()*nextSpprt

	local optGamma=1.26
	weight=math.smoothTransition(math.pow(weight, optGamma))
	assert(weight==weight)

	local localSpprt=transf()
	localSpprt:interpolate(weight, localSpprt1, localSpprt2)

	RE.output2("localspprt", localSpprt)
	RE.output2("currCOM", currCOM)


	local spprt=currCOM*localSpprt

	RE.output2('currspprt', currSpprt)
	RE.output2('spprt', spprt)
	RE.output2('nextspprt', nextSpprt)

	local p, shear_q, q=fc.decomposeSupportCoordinate(spprt)

	if weight<0.05 then
		local y=p.y
		p:interpolate(sop.map(weight,0,0.05, 0.1,1),  p1, p:copy())
		p.y=math.max(y,0)
	end
	if weight>0.9 then
		local y=p.y
		p:interpolate(sop.map(weight,0.9,1, 0,1),  p:copy(),  p2)
		p.y=math.max(y,0)
	end
	--[[

	local p,shear_q,q=fc.decomposeSupportCoordinate(currSpprt)
	local p2,shear_q2,q2=fc.decomposeSupportCoordinate(nextSpprt)
	p:assign(arc.lineInterpolation(weight, p:copy(), p2))
	q:safeSlerp(q:copy(), q2, weight)
	shear_q:safeSlerp(shear_q:copy(), shear_q2, weight)
	]]
	return p, shear_q, q
end

-- inverse of extractSupportCoordinate
function fc.decomposeSupportCoordinate(currSpprt)
	local pendOri=quater()
	local pendRotY=quater()
	local p=currSpprt.translation:copy()

	currSpprt.rotation:decomposeNoTwistTimesTwist(vector3(0,1,0), pendOri, pendRotY)

	return p, pendOri, pendRotY
end
function fc.composeSupportCoordinate(p, shear_q, q)
	--return transf(shear_q, p)*transf(q, vector3(0,0,0))
	return transf(shear_q*q, p)
end
function fc.composeL(rotY, footposL, compos, footOffset)
	local ZMP=footposL-rotY*footOffset
	local offset=quater()
	offset:axisToAxis(vector3(0,1,0), (compos-ZMP):Normalize())
	return fc.composeSupportCoordinate(footposL, offset, rotY)
end
function fc.composeR(rotY, footposR, compos, footOffset)
	local ZMP=footposR+rotY*footOffset
	local offset=quater()
	offset:axisToAxis(vector3(0,1,0), (compos-ZMP):Normalize())
	return fc.composeSupportCoordinate(footposR, offset, rotY)
end


function fc.calcGlobalFootCoord(spprt1, i, visualizeData)
	local p, shear_q, q=fc.decomposeSupportCoordinate(spprt1)
	local zmp_coord_ref= matrix4(quater(1,0,0,0), p)*  shear_q * matrix4(q, vector3(0,0,0))

	if visualizeData then
		dbg.namedDraw("Axes", transf(q, p+character2offset), 'refCoord'..visualizeData.conindex, 100)
	end

	return vector3(0,0,0),  q, zmp_coord_ref
end
-- input ={skel='a.wrl', mot='a.dof', motionType='run'}
function fc.loadMotion(input, createSkin, applyMotionDOF)
	local skel=input.skel
	local motion=input.mot

	if input.motionType and input[input.motionType].mot then
		motion=input[input.motionType].mot
	end

	local skinScale=input.skinScale

	local mot={}
	mot.loader=MainLib.VRMLloader (skel)
	if input.skel_scale then
		mot.loader:scale(input.skel_scale, mot.loader.mMotion)
	end
	mot.motionDOFcontainer=MotionDOFcontainer(mot.loader.dofInfo, motion)
	local loader=mot.loader
	for i=1, loader:numBone()-1 do
		if loader:VRMLbone(i):numChannels()==0 then
			loader:removeAllRedundantBones()
			--loader:removeBone(mLoader:VRMLbone(i))
			--loader:export(config[1]..'_removed_fixed.wrl')
			break
		end
	end
	loader:_initDOFinfo()
	mot.loader:bone(1):setChannels('XYZ', 'YZX') -- yaw roll pitch 
	if createSkin then
		mot.skin=fc.createSkin(mot.loader, input.skinScale)
		if applyMotionDOF then
			mot.skin:applyMotionDOF(mot.motionDOFcontainer.mot)
		end
	end
	return mot
end
function fc.createSkin(loader, s)
	s=s or 100
	local skin=RE.createVRMLskin(loader, false)
	skin:scale(s,s,s); -- motion data often is in meter unit while visualization uses cm unit.
	skin:setThickness(3/s)
	--skin:setMaterial('lightgrey_transparent')
	return skin
end
local function calcLegLean(rotY, footToCom)
	local front=rotY*vector3(0,0,1)
	local side=vector3()
	side:cross(front, vector3(0,1,0))
	local q=quater()
	q:axisToAxis(footToCom, vector3(0,1,0))
	return q:rotationAngleAboutAxis(side)
end
-- 0<=swingPhase<=1
function fc.getHeelAndToePosition(rotY, COM, footMidPos, contactNormal, swingPhase, g_footLen, input, q, rootq)

	local heelAndToe=function (rotY, footMidPos, normal, scale)
		local q=quater()
		q:axisToAxis(vector3(0,1,0), normal)
		q:scale(scale or 1)
		local delta=q*rotY*vector3(0,0, g_footLen*0.5)
		local toePos=footMidPos+delta
		local heelPos=footMidPos-delta
		return heelPos, toePos
	end
	local footToCom=COM-footMidPos
	footToCom:normalize()
	local isHeel=calcLegLean(rotY, footToCom)<0

	local h1, t1=heelAndToe(rotY, footMidPos, contactNormal)
	
	local normal=vector3()
	normal:interpolate(swingPhase, contactNormal, footToCom)

	local h2, t2
	if isHeel then
		h2,t2=heelAndToe(rotY, footMidPos, normal, 0.7)
	else
		h2,t2=heelAndToe(rotY, footMidPos, normal, 1.5)
	end

	local heelPos, toePos
	if isHeel then
		heelPos=h1
		toePos=h1-h2+t2
	else
		heelPos=t1-t2+h2
		toePos=t1
	end

	local function avoidFloor(toePos2, heelPos2)
		if heelPos2.y<0 then
			local delta=-heelPos2.y
			heelPos2.y=0
			toePos2.y=toePos2.y+delta
		end
		if toePos2.y<0 then
			local delta=-toePos2.y
			toePos2.y=0
			heelPos2.y=heelPos2.y+delta
		end
	end
	if input and input.isJump and swingPhase>0.0 then
		local delta=rootq*vector3(0,0,g_footLen*0.5)
		local toePos2=footMidPos+delta
		local heelPos2=footMidPos-delta

		avoidFloor(toePos2, heelPos2)
		avoidFloor(toePos, heelPos)
		assert(toePos.y>=0)
		assert(heelPos.y>=0)

		toePos:interpolate(swingPhase, toePos:copy(), toePos2)
		heelPos:interpolate(swingPhase, heelPos:copy(), heelPos2)
		local qq=quater()
		qq:safeSlerp(q, rootq, swingPhase)
		return isHeel, heelPos, toePos, qq
	end
	return isHeel, heelPos, toePos,q
end
function fc.getHeelAndToePosition_horse(rotY, COM, footMidPos, contactNormal, swingPhase, g_footLen,toeScale)

	local heelAndToe=function (rotY, footMidPos, normal, scale)
		local q=quater()
		q:axisToAxis(vector3(0,1,0), normal)
		q:scale(scale or 1)
		local delta=q*rotY*vector3(0,0, g_footLen*0.5)
		local toePos=footMidPos+delta
		local heelPos=footMidPos-delta
		return heelPos, toePos
	end
	local footToCom=COM-footMidPos
	footToCom:normalize()
	local isHeel=calcLegLean(rotY, footToCom)<0

	local h1, t1=heelAndToe(rotY, footMidPos, contactNormal)
	
	local normal=vector3()
	normal:interpolate(swingPhase, contactNormal, footToCom)

	local h2, t2
	if isHeel then
		h2,t2=heelAndToe(rotY, footMidPos, normal, 0)
	else
		h2,t2=heelAndToe(rotY, footMidPos, normal, toeScale)
	end

	local heelPos, toePos
	if isHeel then
		heelPos=h1
		toePos=h1-h2+t2
	else
		heelPos=t1-t2+h2
		toePos=t1
	end

	return isHeel, heelPos, toePos,q
end

-- spline=CT.mat(3,2, t0, v0, t1, v1, t2,v2) 
-- unlike getLinearSplineABS, t0, t1, t2 are the durations between key times.
function fc.getLinearSpline(t_global, spline)
	local durations=spline:row(0)
	local control1=spline:row(1)
	local control2=spline:row(2)
	local eps=1e-10
	assert(t_global>=0.0)
	local t=0
	local w
	local iseg=-1
	for i=0, durations:size()-1 do
		local d=durations(i)
		t=t+d
		if (t >= t_global-eps) then -- at junctions, returns previous spline (=)
			iseg=i
			w=sop.map(t_global, t-d, t, 0, 1)
			break
		end
	end
	--assert(iseg~=-1)
	if iseg==-1 then
		return control2(durations:size()-1)
	end

	RE.output2('iseg',iseg)
	return sop.map(w, 0, 1, control1(iseg), control2(iseg))
end

-- spline: tl.mat({{time0, value0}, {time1, value1}, ...{timen, valuen}})
function fc.getLinearSplineABS(t_global, spline)
	local time=spline:column(0)
	local control1=spline:column(1):range(0, spline:rows()-1)
	local control2=spline:column(1):range(1, spline:rows())
	local eps=1e-10
	assert(t_global>=0.0)
	local t=0
	local w
	local iseg=-1
	for i=0, time:size()-2 do
		t=time(i+1)
		local d=t-time(i)
		if (t >= t_global-eps) then -- at junctions, returns previous spline (=)
			iseg=i
			w=sop.map(t_global, t-d, t, 0, 1)
			break
		end
	end
	--assert(iseg~=-1)
	if iseg==-1 then
		return control2(control2:size()-1)
	end

	return sop.map(w, 0, 1, control1(iseg), control2(iseg))
end

--
-- spline: tl.mat{{time0, value0}, {time1, value1}, ...{timen, valuen}}
-- returns v, dv
function fc.getLinearSplineABSderiv(t_global, spline)
	local time=spline:column(0)
	local control1=spline:column(1):range(0, spline:rows()-1)
	local control2=spline:column(1):range(1, spline:rows())
	local eps=1e-10
	assert(t_global>=0.0)
	local t=0
	local w
	local iseg=-1
	for i=0, time:size()-2 do
		t=time(i+1)
		local d=t-time(i)
		if (t >= t_global-eps) then -- at junctions, returns previous spline (=)
			iseg=i
			w=sop.map(t_global, t-d, t, 0, 1)
			break
		end
	end
	--assert(iseg~=-1)
	if iseg==-1 then
		return control2(control2:size()-1),0
	end

	return sop.map(w, 0, 1, control1(iseg), control2(iseg)), (control2(iseg)-control1(iseg))/(spline(iseg+1,0)-spline(iseg,0))
end


-- tbl={
-- { time_offset, CT.vec(0, 1, 2)}, -- key
-- CT.mat(1,3,   value0, value1, value2), -- value
-- }
function fc.getLinearSpline2(t, tbl)
	local key, value=unpack(tbl)
	local t_global=t+key[1]
	local durations=key[2]
	local eps=1e-10
	assert(t_global>=0.0)
	local t=0
	local w
	local iseg=-1
	for i=0, durations:size()-1 do
		local d=durations(i)
		t=t+d
		if (t >= t_global-eps) then -- at junctions, returns previous spline (=)
			iseg=i
			w=sop.map(t_global, t-d, t, 0, 1)
			break
		end
	end
	assert(iseg~=-1)


	local out=vectorn()
	out:interpolate(w, value:column(iseg), value:column(iseg+1))
	return out
end
-- tbl={
-- { time_offset, CT.vec(0, 1, 2)}, -- key
-- CT.mat(1,3,   value0, value1, value2), -- value
-- }
function fc.getSmoothTransition(t, tbl)
	local key, value=unpack(tbl)
	local t_global=t+key[1]
	local durations=key[2]
	local eps=1e-10
	assert(t_global>=0.0)
	local t=0
	local w
	local iseg=-1
	for i=0, durations:size()-1 do
		local d=durations(i)
		t=t+d
		if (t >= t_global-eps) then -- at junctions, returns previous spline (=)
			iseg=i
			w=math.smoothTransition(sop.map(t_global, t-d, t, 0, 1))
			break
		end
	end
	assert(iseg~=-1)


	local out=vectorn()
	out:interpolate(w, value:column(iseg), value:column(iseg+1))
	return out
end
-- tbl={
-- { time_offset, CT.vec(0, 1, 2)}, -- key
-- CT.mat(1,3,   value0, value1, value2), -- position
-- CT.mat(1,3,   vel0, vel1, vel2), -- velocity
-- }
function fc.getHermiteSpline(t, tbl)
	local key, value, vel=unpack(tbl)
	local t_global=t+key[1]
	local durations=key[2]
	local eps=1e-10
	assert(t_global>=0.0)
	local t=0
	local w
	local iseg=-1
	for i=0, durations:size()-1 do
		local d=durations(i)
		t=t+d
		if (t >= t_global-eps) then -- at junctions, returns previous spline (=)
			iseg=i

			local out=vectorn()

			out:hermite(t_global-(t-d), d, value:column(iseg), vel:column(iseg), value:column(iseg+1), vel:column(iseg+1)) 
			return out
		end
	end
	assert(false)
end

function fc.summedTable(_phases)
	local phasesL=_phases:copy()
	-- build summed tables.
	for i=1,phasesL:size()-1 do
		phasesL:set(i, phasesL(i-1)+phasesL(i))
	end
	return phasesL
end

-- prev_pose will be updated to be the next pose.
function fc.solveIK(mSolverInfo, prev_pose, dotpose, desired_pose, comdof, footdof, effWeights, config) 
	-- world velocity
	local V=Liegroup.se3(comdof:toVector3(7), comdof:toVector3(10))
	
	RE.output2('V0', V.w, V.v)

	if true then
		local T=MotionDOF.rootTransformation(prev_pose)
		--V.w:radd(T.rotation*dotpose:toVector3(4))
		--V.v:radd(T.rotation*dotpose:toVector3(0))


		local T2=MotionDOF.rootTransformation(desired_pose)
		local V2=T:twist(T2 , 1/30)
		V2.v:rotate(T.rotation)
		V2.w:rotate(T.rotation)
		local a=0.9

		if config and config.wVelRoot then
			a=config.wVelRoot 
		end
		local b=1-a
		V.v:assign(V.v*a+V2.v*b)
		V.w:assign(V.w*a+V2.w*b)
	end
	local roottf=MotionDOF.rootTransformation(comdof)


	local pose=prev_pose

	local numCon=mSolverInfo.numCon
	local footPos=vector3N (numCon);

	for i=0, numCon-1 do
		footPos(i):assign(footdof:toVector3(3*i))
	end

	local mIK=mSolverInfo.solver
	local mEffectors=mSolverInfo.effectors


	local hasCOM=1
	local hasMM=1
	local wCOMy=0.0
	local wFullVel=0.01
	local wFullVelPose=0.001
	local wFullVelPose2=0.005
	local wMM=0.1
	if config then
		wCOMy=config.wCOMy or 0.0
		if config.wFullVel then wFullVel=config.wFullVel end
		if config.wFullVelPose then wFullVelPose=config.wFullVelPose wFullVelPose2=config.wFullVelPose*5 end
		if config.wFullVelPose2 then wFullVelPose2=config.wFullVelPose2 end
		if config.wMM then wMM=config.wMM end
	else
		config={}
	end
	--
	--local prev_roottf=MotionDOF.rootTransformation(pose)
	--local toLocal=prev_roottf:inverse()
	local toLocal=MotionDOF.rootTransformation(comdof):inverse()

	if not config.noPlannarProjection then
		toLocal.translation.y=0
		toLocal.rotation:assign(toLocal.rotation:rotationY())
	end

	local useHead=config.useHead or 1
	local hasVelCon=0
	if config.vel_con then
		hasVelCon=hasVelCon+#config.vel_con
	end

	if not config.v2_max then
		config.v2_max=math.max(V.v:length(), V.w:length(), 10)
	end
	if not config.effWeightsY then
		config.effWeightsY=0.1
	end
	local v2newpose=MotionDOF.calcVelocity(prev_pose, desired_pose, 30)
	v2newpose:clamp(config.v2_max)
	mIK:_changeNumEffectors(numCon)
	if effWeights then
	else
		effWeights=vectorn(numCon)
		effWeights:setAllValue(1.0)
	end
	for i=0, numCon-1 do mIK:_setEffectorWeight(i, math.pow(effWeights(i),config.effwPower or 2)*config.effWeightsY) end
	mIK:_changeNumConstraints(hasCOM+hasMM+useHead+4+hasVelCon)

	local COM=roottf.translation
	if hasCOM==1 then
		mIK:_setCOMConstraint(0, toLocal*COM, 1,wCOMy,1)
	end
	--if g_dbgdbg then dbg.console() end
	if hasMM==1 then
		V:Ad_ori(toLocal.rotation)
		local dt=1/30
		--local dt=1
		RE.output2('V', V.w, V.v, toLocal.rotation)
		mIK:_setMomentumConstraint(hasCOM, V.w*dt, V.v*dt, wMM)
	end
	mIK:setParam('damping_weight', 0,0)

	if false then
		-- 모션 품질 나빠짐.
		-- clamp dotpose
		local thr=math.rad(10)
		for i=7, dotpose:size()-1 do
			if prev_pose(i)> desired_pose(i)+thr and dotpose(i)>0  then
				dotpose:set(i,0)
			end
			if prev_pose(i)< desired_pose(i)-thr and dotpose(i)<0  then
				dotpose:set(i,0)
			end
		end
	end
	local kneeDOFs=mSolverInfo.kneeDOFs
	if kneeDOFs then
		-- knee damping
		local thr=0
		for ii=0, kneeDOFs:size()-1 do
			i=kneeDOFs(ii) 

			if prev_pose(i)< thr and dotpose(i)<0  then
				dotpose:set(i,0)
			end
		end
	end

	local nextPose1=MotionDOF.integrate(prev_pose, dotpose, 30)
	local nextPose2=MotionDOF.integrate(prev_pose, v2newpose, 30)


	local  headPos, headBone
	if useHead==1 then
		local loader=mMot.loader
		--local headRefPose=loader.dofInfo:blendPose( nextPose1, nextPose2, 0.1)
		--loader:setPoseDOF(headRefPose)
		headBone=loader:getBoneByName(input.head[1])
		--headOri=headBone:getFrame().rotation:copy()
		--local COMtoHead=headBone:getFrame()*input.head[2]- loader:calcCOM()
		local COMtf=MotionDOF.rootTransformation(comdof)
		headPos=COMtf*input.head[3]
		--dbg.draw('Sphere', headPos*100, 'head')
	end
	MotionDOF.setRootTransformation(pose, toLocal*MotionDOF.rootTransformation(pose))
	MotionDOF.setRootTransformation(nextPose1, toLocal*MotionDOF.rootTransformation(nextPose1))
	MotionDOF.setRootTransformation(nextPose2, toLocal*MotionDOF.rootTransformation(nextPose2))

	--for i=1, mMot.loader:numBone()-1 do print(i,mMot.loader:bone(i):name()) end
	mIK:_setPoseConstraint(hasCOM+hasMM, nextPose1, wFullVel, 2)  -- fullbody velocity constraint
	mIK:_setPoseConstraint(hasCOM+hasMM+1, nextPose2, wFullVelPose, 2)  -- fullbody poseconstraint
	if not input.lowerbody then
		input.lowerbody={2,10000}
	end
	mIK:_setPoseConstraint(hasCOM+hasMM+2, nextPose2, wFullVelPose2, input.lowerbody[1],input.lowerbody[2] ) -- lowerbody pose constraint
	for i=0,numCon-1 do
		mIK:_setEffector(i, mEffectors(i).bone, mEffectors(i).localpos)
		footPos(i):assign(toLocal*footPos(i))
	end
	if useHead==1 then
		local i=numCon
		mIK:_setPositionConstraint(hasCOM+hasMM+3, headBone, input.head[2], toLocal*headPos, 1,config.wHead_y or 0,config.wHead_z or 0.1 );
	end
	local ew=effWeights:copy()
	for i=0,ew:size()-1 do ew:set(i, math.pow(ew(i),config.effwPower or 2)) end
	
	mIK:_setEffectorYConstraint(hasCOM+hasMM+useHead+3, 1.0, ew)

	if config.vel_con then
		for i,v in ipairs(config.vel_con) do
			local loader=mMot.loader
			local bone=loader:getBoneByName(v[1][1])
			local lpos=v[1][2]
			local gpos=v[2]
			local offset=v[3]
			local normal=offset:copy()
			normal:normalize()
			--mIK:_setPositionConstraint(hasCOM+hasMM+4+i, bone, lpos, toLocal*(gpos+offset*1/30), 1,1,1)
			mIK:_setNormalConstraint(hasCOM+hasMM+4+i, bone, lpos, toLocal.rotation*normal, toLocal*(gpos+offset*1/30))
		end
	end

	mIK:_effectorUpdated()
	mIK:IKsolve(pose, footPos)


	MotionDOF.setRootTransformation(pose, toLocal:inverse()*MotionDOF.rootTransformation(pose))
	assert(prev_pose==pose)
end
function fc.solveIK_postprocess(mSolverInfo, prev_pose, comdof, footdof, effWeights, config) 
	local pose=prev_pose

	local numCon=mSolverInfo.numCon
	local footPos=vector3N (numCon);

	for i=0, numCon-1 do
		footPos(i):assign(footdof:toVector3(3*i))
	end

	local mIK=mSolverInfo.solver
	local mEffectors=mSolverInfo.effectors
	--local prev_roottf=MotionDOF.rootTransformation(pose)
	--local toLocal=prev_roottf:inverse()
	local toLocal=MotionDOF.rootTransformation(comdof):inverse()
	toLocal.translation.y=0
	toLocal.rotation:assign(toLocal.rotation:rotationY())


	local hasCOM=1
	local hasMM=0
	local wCOMy=0.0
	local wMM=0.1
	if config then
		wCOMy=config.wCOMy or 0.0
		if config.wMM then wMM=config.wMM end
	else
		config={}
	end
	local hasVelCon=0
	if config.vel_con then
		hasVelCon=hasVelCon+#config.vel_con
	end


	local useHead=config.useHead or 1

	if not config.effWeightsY then
		config.effWeightsY=0.1
	end
	mIK:_changeNumEffectors(numCon)
	if effWeights then
	else
		effWeights=vectorn(numCon)
		effWeights:setAllValue(1.0)
	end
	for i=0, numCon-1 do mIK:_setEffectorWeight(i, math.pow(effWeights(i),2)*config.effWeightsY) end
	mIK:_changeNumConstraints(hasCOM+hasMM+useHead+1+hasVelCon)

	local COM
	do
		local loader=mMot.loader
		loader:setPoseDOF(prev_pose)
		COM=loader:calcCOM()
	end

	if hasCOM==1 then
		mIK:_setCOMConstraint(0, toLocal*COM, 1,wCOMy,1)
	end
	--if g_dbgdbg then dbg.console() end
	if hasMM==1 then
		mIK:_setMomentumConstraint(hasCOM, vector3(0,0,0), vector3(0,0,0), wMM)
	end
	mIK:setParam('damping_weight', 0.01,0.01)

	local  headPos, headBone
	if useHead==1 then
		local loader=mMot.loader
		--local headRefPose=loader.dofInfo:blendPose( nextPose1, nextPose2, 0.1)
		--loader:setPoseDOF(headRefPose)
		headBone=loader:getBoneByName(input.head[1])
		headPos=headBone:getFrame()*input.head[2]
	end
	MotionDOF.setRootTransformation(pose, toLocal*MotionDOF.rootTransformation(pose))

	for i=0,numCon-1 do
		mIK:_setEffector(i, mEffectors(i).bone, mEffectors(i).localpos)
		footPos(i):assign(toLocal*footPos(i))
	end
	if useHead==1 then
		local i=numCon
		mIK:_setPositionConstraint(hasCOM+hasMM, headBone, input.head[2], toLocal*headPos, 1,config.wHead_y or 0,0.1 );
	end
	local ew=effWeights:copy()
	for i=0,ew:size()-1 do ew:set(i, math.pow(ew(i),2)) end
	
	mIK:_setEffectorYConstraint(hasCOM+hasMM+useHead, 1.0, ew)

	if config.vel_con then
		for i,v in ipairs(config.vel_con) do
			local loader=mMot.loader
			local bone=loader:getBoneByName(v[1][1])
			local lpos=v[1][2]
			local gpos=v[2]
			local offset=v[3]
			local normal=offset:copy()
			normal:normalize()
			--mIK:_setPositionConstraint(hasCOM+hasMM+4+i, bone, lpos, toLocal*(gpos+offset*1/30), 1,1,1)
			mIK:_setNormalConstraint(hasCOM+hasMM+1+i, bone, lpos, toLocal.rotation*normal, toLocal*(gpos+offset*1/30))
		end
	end

	mIK:_effectorUpdated()
	mIK:IKsolve(pose, footPos)


	MotionDOF.setRootTransformation(pose, toLocal:inverse()*MotionDOF.rootTransformation(pose))
	assert(prev_pose==pose)
end


fc.DiscontinuityRemover=LUAclass()
function fc.DiscontinuityRemover:__init(loader)
	self.map=vectorn()
	loader:getPoseDOF(self.map)
	self.map:setAllValue(0.0)
	self.weight=0.0
end
function fc.DiscontinuityRemover:resetDiscontinuity(prevPose, pose)
	self.map=diffPose(pose, prevPose)
	self.weight=1.0
end

function fc.DiscontinuityRemover:filterPose(pose)
	local weight=self.weight-0.2
	if weight<0 then weight=0 end
	self.weight=weight
	print(self.weight)
	local out=addPose(pose, scalePose(self.map,weight ))
	return out
end

-- linear
fc.DiscontinuityRemoverLin=LUAclass()
function fc.DiscontinuityRemoverLin:__init(ndim)
	self.map=vectorn(ndim)
	self.map:setAllValue(0.0)
	self.weight=0.0
end
function fc.DiscontinuityRemoverLin:resetDiscontinuity(prevPose, pose)
	self.map=prevPose-pose
	self.weight=1.0
end

function fc.DiscontinuityRemoverLin:filterPose(pose)
	local weight=self.weight-0.05
	if weight<0 then weight=0 end
	self.weight=weight
	local out=pose+self.map*weight
	return out
end

fc.ImpulseGen=LUAclass()
function fc.ImpulseGen:__init()
	self.weight=0.0
end
function fc.ImpulseGen:resetImpulse()
	self.weight=1.0
end

function fc.ImpulseGen:genImpulse()
	local weight=self.weight-0.2
	if weight<0 then weight=0 end
	self.weight=weight
	return self.weight
end

return fc


