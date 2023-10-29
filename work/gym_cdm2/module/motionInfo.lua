require("gym_cdm2/module/info_hyunwooLowDOF")

local motionInfos={}
motionInfos.run2={
	isDeepmimicMot=true,
	--skel="../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T3_boxfoot.wrl",
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	fn='gym_deepmimic/motions/humanoid3d_run.txt',
	fn2='gym_cdm2/spec/refTraj_run2.dat',
	fixDOF= input.fixDOF_humanoid3d_run,
	contact=input.contact_humanoid3d_run,
	downsample=2, -- because RL_step=60
	--contactThr=-0.05, -- transition doesn't work
	contactThr=0, 
	option_refCDMtraj={ usePendulum=true, filterCDMacc=15, },
}
motionInfos.backflip={
	isDeepmimicMot=true,
	--skel="../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T3_boxfoot.wrl",
	skel_deepmimic='gym_deepmimic/module/humanoid3d.txt',
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	fn='gym_deepmimic/motions/humanoid3d_backflip.txt',
	fn2='gym_cdm2/spec/refTraj_backflip.dat',
	fixDOF= input.fixDOF_backflip,
	contact=input.contact_backflip,
	downsample=2, -- because RL_step=60
	--contactThr=-0.05, -- transition doesn't work
	contactThr=0, 
	--option_refCDMtraj={ usePendulum=true, filterCDMacc=15, },
	option_refCDMtraj={ removeAccumulatedError=true, filterCDMori=5, constantAngVel={17,36, vector3(-9,0,0)}}
}
motionInfos.walk2={
	isDeepmimicMot=true,
	--skel="../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T3_boxfoot.wrl",
	skel_deepmimic='gym_deepmimic/module/humanoid3d.txt',
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	fn='gym_deepmimic/motions/humanoid3d_walk.txt',
	fn2='gym_cdm2/spec/refTraj_walk2.dat',
	fixDOF= input.fixDOF_walk,
	contact=input.contact_walk,
	strideThr=0.2,
	maxContactLen=0.9,
	start=0,
	downsample=2, -- because RL_step=60
	--contactThr=-0.05, -- transition doesn't work
	contactThr=0, 
	option_refCDMtraj={ usePendulum=true, filterFD=16, useMocapOffset=true },
	option_contactFilter={ useV2=true,  filterEndPhase=0.4, maxFootVel=3,
		logQ1=5.43388,
		logQ2=8.72924313578,
		logQm=6.800138,
	},
	contactThr=0, 
	delayedVis=3
}
motionInfos.run3={
	isDeepmimicMot=false,
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	fn="../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Run_F_Loop.fbx.dof" ,
	fn2='gym_cdm2/spec/refTraj_run3.dat',
	unlimitedLeglen=true,
	fixDOF= function (loader, motdof)
		local temp=motdof:copy()
		local temp2=motdof:copy()
		temp2:alignSimple(temp, temp)
		motdof:assign(temp2:range(6,24))
	end,
	fixDOF_part2= function (loader, motdof)
		-- fixDOF와 달리 CDM motion에 영향을 안미침.(CDM 경로 추출후 호출됨) 
		local startL=loader:getRotJointIndexByName('LeftCollar')
		local startR=loader:getRotJointIndexByName('RightCollar')
		local startC=loader:getRotJointIndexByName('Chest1')
		local startRH=loader:getRotJointIndexByName('RightHip')
		local mot=Motion(motdof)
		local nf=motdof:numFrames()
		local q=quater(math.rad(-10), vector3(0,0,1))
		local invq=q:inverse()
		local q2=quater(math.rad(20), vector3(0,1,0))
		for i=0, nf-1 do
			local pose= mot:pose(i)
			pose.rotations(startL):rightMult(q)
			pose.rotations(startL+1):leftMult(invq)
			pose.rotations(startR):rightMult(q)
			--pose.rotations(startR+1):leftMult(invq)
			pose.rotations(startC):rightMult(quater(math.rad(7), vector3(0,1,0)))

			-- Aglobal was originally R*H*K*A
			pose.rotations(startRH):rightMult(q2)
			-- now modified to =R*H*q2*K*A
			-- to recover
			-- q2*K*newA=K*A
			-- so new A=K:inverse()*q2:inverse()*K*A

			local K=pose.rotations(startRH+1)
			local A=pose.rotations(startRH+2)
			A:assign(K:inverse()*q2:inverse()*K*A*quater(math.rad(10), vector3(0,1,0)))
		end
		motdof:set(mot)
	end,
	start=0,
	maxContactY=0.80,
	--doNotUseConWeight =true,
	conWeightMap=function (_, i, startSwing, endSwing) 
		local w=sop.clampMap(i, startSwing, endSwing, -1, 1) 
		--return 1.0 
		return math.pow(w,2) *0.5+0.5
	end,
	doNotUseConHeight=true,
	contact=
	{
		touchDown=
		{
			{0, 17}, -- R touchdown moments
			{8}, -- L touchdown moments
		},
		touchOff={
			{5}, -- R touchoff moments
			{13}, -- L touchoff moments
		}
	},
	downsample=2, -- because RL_step=60
	--contactThr=-0.0,
	contactThr=0,
	option_refCDMtraj={ usePendulum=true, filterFD=16, filterCDMacc=12, scaleYdelta=0, useMocapOffset=true},
	option_poseSampler={ cyclicRoot=true, cyclicFoot=true},
	option_contactFilter={ 
		useV2=true, filterEndPhase=0.5, maxFootVel=3,
		logQ1=7.2589012398406,
		logQ2=11.08924313578,
		logQm=8.6188994846528,
	},
}
motionInfos.run4={
	isDeepmimicMot=false,
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	--fn="../Resource/motion/MOB1/hanyang_lowdof_T_lafan1_run.mot2" ,
	fn="../Resource/motion/MOB1/hanyang_lowdof_T_lafan1_run_3750.dof" ,
	fn2='gym_cdm2/spec/refTraj_run3.dat',
	unlimitedLeglen=true,
	fixDOF= function (loader, motdof)
		if false then
			-- 27740, 27760 -- fast but non-cyclic 
			-- 3750,
			local temp=motdof:range(3750, 3773+1):copy()
			temp:transform(temp:row(0):toTransf(0):project2D():inverse())
			temp:exportMot("../Resource/motion/MOB1/hanyang_lowdof_T_lafan1_run_3750.dof" )
			motdof:assign(temp)
		end
	end,
	fixDOF_part2= function (loader, motdof)
	end,
	start=0,
	maxContactY=0.80,
	--doNotUseConWeight =true,
	conWeightMap=function (_, i, startSwing, endSwing) 
		local w=sop.clampMap(i, startSwing, endSwing, -1, 1) 
		return math.pow(w,2) *0.5+0.5
	end,
	doNotUseConHeight=true,
	delayedVis=3,
	contact=
	{
		touchDown=
		{
			{11}, -- R touchdown moments
			{0, 23}, -- L touchdown moments
		},
		touchOff={
			{20}, -- R touchoff moments
			{8}, -- L touchoff moments
		}
	},
	downsample=2, -- because RL_step=60
	--contactThr=-0.0,
	contactThr=0,
	option_refCDMtraj={ usePendulum=true, filterFD=16, filterCDMacc=12, scaleYdelta=0, useMocapOffset=true},
	option_poseSampler={ cyclicRoot=true, cyclicFoot=true},
	option_contactFilter={ 
		useV2=true, filterEndPhase=0.5, maxFootVel=3,
		logQ1=7.2589012398406,
		logQ2=11.08924313578,
		logQm=8.6188994846528,
	},
}
motionInfos.jog={
	isDeepmimicMot=false,
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	fn="../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Jog_F_Loop.fbx.dof" ,
	fn2='gym_cdm2/spec/refTraj_jog.dat',
	fixDOF= function (loader, motdof)
		local startL=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.LEFTANKLE))
		local startR=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.RIGHTANKLE))
		local nf=motdof:rows()
		for i=0, nf-1 do
			local pose=motdof:row(i)
			-- ZYX
			pose:set(startL+2, pose(startL+2)+math.rad(-10)) --toe 내리기
			pose:set(startR+2, pose(startR+2)+math.rad(-10))

			local headSwing=-3
			local angle=sop.map(i,3,15, -headSwing, headSwing)
			pose:setQuater(3, pose:toQuater(3)*quater(math.rad(angle), vector3(0,0,1)))
		end

		local temp1=motdof:range(3,15):copy()
		local motL=Motion(temp1)
		local motR=Motion(loader)
		local LrootIndices=CT.ivec( loader:getTreeIndexByName('LeftHip'), loader:getTreeIndexByName('LeftCollar'))
		local RrootIndices=CT.ivec( loader:getTreeIndexByName('RightHip'), loader:getTreeIndexByName('RightCollar'))

		motR:mirrorMotion(motL, LrootIndices, RrootIndices)

		local temp1_mirrored=MotionDOF(loader.dofInfo)
		temp1_mirrored:set(motR)
		local temp2=temp1:copy()
		--temp2:alignSimple(temp1, temp1_mirrored)
		temp2:stitch(temp1, temp1_mirrored)

		local temp3=temp2:copy()
		temp3:alignSimple(temp2, temp1)
		temp3:range(temp2:rows()-temp1:rows(), temp3:rows()):stitch(temp2:range(temp2:rows()-temp1:rows(), temp2:rows()), temp1)

		--motdof:assign(temp2:range(4,27))
		motdof:assign(temp3:range(4, 27))

		nf=motdof:rows()
		for i=0, nf-1 do
			local pose=motdof:row(i)
			--pose:set(2, pose(2)*0.8) 
		end

		--makeCyclic(motdof)
		makeCyclic(motdof,5)
	end,
	fixDOF_part2= function (loader, motdof)
		-- CDM motion에 영향을 안미침. 
		local startL=loader:getRotJointIndexByName('LeftHip')
		local startR=loader:getRotJointIndexByName('RightHip')
		local mot=Motion(motdof)
		local nf=motdof:numFrames()
		local q=quater(math.rad(-3), vector3(0,0,1))
		local invq=q:inverse()
		for i=0, nf-1 do
			local pose= mot:pose(i)
			pose.rotations(startL):rightMult(invq)
			pose.rotations(startR):rightMult(q)
		end
		motdof:set(mot)
	end,
	start=0,
	maxContactY=0.80,
	--unlimitedLeglen=true,
	--doNotUseConWeight =true,
	--doNotSolveIK=true,
	conWeightMap=function (_, i, startSwing, endSwing) 
		local w=sop.clampMap(i, startSwing, endSwing, -1, 1) 
		--return 1.0 
		return math.pow(w,2) *0.5+0.5
	end,
	doNotUseConHeight=true,
	contact=
	{
		touchDown=
		{
			{0, 22}, -- R touchdown moments
			{11}, -- L touchdown moments
		},
		touchOff={
			{6}, -- R touchoff moments
			{17}, -- L touchoff moments
		}
	},
	downsample=2, -- because RL_step=60
	contactThr=0.05, -- adjust timing
	--maxContactY=0.9, -- no adjust timing
	--contactThr=0,
	option_contactFilter={ 
		useV2=true, filterEndPhase=0.5, maxFootVel=3,
		logQ1=5.7814012398406,
		logQ2=11.138924313578,
		logQm=7.6783594846528,
	},
	option_refCDMtraj={ usePendulum=true, filterFD=16, filterCDMacc=12, scaleYdelta=0, useMocapOffset=true},
	option_poseSampler={ cyclicRoot=true, cyclicFoot=true},
	delayedVis=3
}

motionInfos.walk3={
	isDeepmimicMot=false,
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	fn="../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Walk_F_Loop.fbx.dof" ,
	fn2='gym_cdm2/spec/refTraj_walk3.dat',
	minSwingPhase=0.25,
	fixDOF= function (loader, motdof)
		local temp=motdof:copy()
		local temp2=motdof:copy()
		print(temp2:rows())
		print(temp:matView():column(2))
		--temp2:transform(transf(quater(1,0,0,0), vector3(0,0.01,0)))
		temp2:alignSimple(temp, temp)
		motdof:assign(temp2:range(7,44))
		local startL=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.LEFTANKLE))
		local startR=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.RIGHTANKLE))
		local nf=motdof:rows()
		for i=0, nf-1 do
			local pose=motdof:row(i)
			-- ZYX
			pose:set(startL+2, pose(startL+2)+math.rad(-10)) --toe 내리기
			pose:set(startR+2, pose(startR+2)+math.rad(-10))
		end
	end,
	strideThr=0.2,
	maxContactLen=0.9,
	start=0,
	contact=
	{
		touchDown=
		{
			{0, 36}, -- R touchdown moments
			{0, 19}, -- L touchdown moments
		},
		touchOff={
			{24}, -- R touchoff moments
			{6}, -- L touchoff moments
		}
	},
	option_refCDMtraj={ usePendulum=true, filterFD=16, useMocapOffset=true },
	option_contactFilter={ useV2=true,  filterEndPhase=0.4, maxFootVel=3,
		logQ1=9.1795573930,
		logQ2=11.08924313578,
		logQm=2.040138,
	},
	contactThr=0, 
	--maxContactY=0.90,
	downsample=2, -- because RL_step=60
	delayedVis=4
}
motionInfos.walk4={
	isDeepmimicMot=false,
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	--fn="../Resource/motion/MOB1/hanyang_lowdof_T_lafan1_walk.mot2" ,
	fn="../Resource/motion/MOB1/hanyang_lowdof_T_lafan1_walk_2915.dof" ,
	fn2='gym_cdm2/spec/refTraj_walk3.dat',
	minSwingPhase=0.25,
	fixDOF= function (loader, motdof)
		if false then
			-- 2917 - 2948
			local temp=motdof:range(2916-1, 2949-1):copy()
			temp:transform(temp:row(0):toTransf(0):project2D():inverse())

			temp:exportMot("../Resource/motion/MOB1/hanyang_lowdof_T_lafan1_walk_2915.dof" )
			motdof:assign(temp)
		end
	end,
	fixDOF_part2= function (loader, motdof)
		makeCyclic(motdof,15)
	end,
	strideThr=0.45, -- todo: this quater-stride length needs to be calculated from the reference motion
	maxContactLen=0.9,
	start=0,
	contact=
	{
		touchDown=
		{
			{0, 16}, -- R touchdown moments
			{0, 32}, -- L touchdown moments
		},
		touchOff={
			{2}, -- R touchoff moments
			{18}, -- L touchoff moments
		}
	},
	option_refCDMtraj={ usePendulum=true, filterFD=16, useMocapOffset=true },
	option_contactFilter={ useV2=true,  filterEndPhase=0.4, maxFootVel=3,
		logQ1=9.1795573930,
		logQ2=11.08924313578,
		logQm=2.040138,
	},
	option_poseSampler={ cyclicRoot=true, cyclicFoot=true},
	contactThr=0, 
	--maxContactY=0.90,
	downsample=2, -- because RL_step=60
	delayedVis=4
}
motionInfos.fastwalk={
	isDeepmimicMot=false,
	doNotSolveIK=false,
	doNotUseConWeight=true,
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	fn="../Resource/motion/CMU/fastwalk/132_21.amc.dof" ,
	fn2='gym_cdm2/spec/refTraj_fastwalk.dat',
	minSwingPhase=0.25,
	fixDOF= function (loader, motdof)
		math.filter(motdof:matView(),10)
		motdof:resample(120,30)
		nf=motdof:rows()
		for i=0, nf-1 do
			local pose=motdof:row(i)
			pose:set(1, pose(1)+0.07)
			pose:setQuater(3, pose:toQuater(3):normalized())
		end
		motdof:assign(motdof:range(54,80):copy())

		local startL=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.LEFTANKLE))
		local startR=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.RIGHTANKLE))
		local nf=motdof:rows()
		for i=0, nf-1 do
			local pose=motdof:row(i)
			-- ZYX
			pose:set(startL, pose(startL)+math.rad(-5)) 
			pose:set(startR, pose(startR)+math.rad(10))
			pose:set(startL+1, pose(startL+1)+math.rad(10)) 
			pose:set(startR+1, pose(startR+1)+math.rad(-10))
		end
		--local startL=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.LEFTSHOULDER))
		--local startR=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.RIGHTSHOULDER))
		--local startC=loader.dofInfo:startR(loader:getRotJointIndexByName('Chest1'))
		--math.filter(motdof:matView():sub(0,0,startC-6, startL+3),20)
		--math.filter(motdof:matView():sub(0,0,startR-3, startR+3),20)

		--math.filter(motdof:matView():column(startL+1):column(),20)
		--math.filter(motdof:matView():column(startR+1):column(),20)

		local startL=loader:getRotJointIndexByName('LeftCollar')
		local startR=loader:getRotJointIndexByName('RightCollar')
		local startC=loader:getRotJointIndexByName('Chest1')
		local mot=Motion(motdof)
		local nf=motdof:numFrames()
		local q=quater(math.rad(-5), vector3(0,0,1))
		local invq=q:inverse()
		for i=0, nf-1 do
			local pose= mot:pose(i)
			pose.rotations(startL):rightMult(q)
			pose.rotations(startL+1):leftMult(invq)
			pose.rotations(startR):rightMult(q)
			pose.rotations(startR+1):leftMult(invq)
			pose.rotations(startC):rightMult(quater(math.rad(7), vector3(0,1,0)))
		end
		motdof:set(mot)
		makeCyclic(motdof,15)
		-- 26 frames 
	end,
	fixDOF_part2= function (loader, motdof)
		if false then
			local kernel=vectorn()
			math.getGaussFilter(5, kernel)
			local deconv=math.deconvolution(1, kernel, motdof:matView())
			for i=0, deconv:rows()-1 do
				local sum=deconv:row(i)
				sum:setQuater(3, sum:toQuater(3):Normalize())
			end
			motdof:matView():assign(deconv)
		end
	end,
	--fixDOF_part3=function(model)
	--	local CDMfeetToHumanFeetPositions=model.refTraj.CDMfeetToHumanFeetPositions
	--	assert(CDMfeetToHumanFeetPositions)
	--	local nf=CDMfeetToHumanFeetPositions:rows()
	--	CDMfeetToHumanFeetPositions:column(1):radd(CT.ones(nf)*0.05)
	--	CDMfeetToHumanFeetPositions:column(3+1):radd(CT.ones(nf)*0.05)
	--end,
	strideThr=0.2,
	maxContactLen=0.9,
	start=0,
	contact=
	{
		touchDown=
		{
			{0, 25}, -- R touchdown moments
			{0, 13}, -- L touchdown moments
		},
		touchOff={
			{14}, -- R touchoff moments
			{1, }, -- L touchoff moments
		}
	},
	--option_contactFilter={ refSwingAlpha=0.5, maxFootVel=0.3 },
	option_contactFilter={ useV2=true,  filterEndPhase=0.4, maxFootVel=3,
		logQ1=6.3169321547799, 
        logQ2=11.179156552627,
        logQm=8.1454344397493,
	},
	option_refCDMtraj={ usePendulum=true, filterFD=16, useMocapOffset=true },
	option_poseSampler={ cyclicRoot=true, cyclicFoot=true},
	contactThr=0, 
	--maxContactY=0.90,
	downsample=2, -- because RL_step=60
	delayedVis=3,
}
motionInfos['runjump2']={
	isDeepmimicMot=false,
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	fn="../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Run_F_Jump.dof" ,
	fn2='gym_cdm2/spec/refTraj_runjump2.dat',
	start=0,
	fixDOF= function (loader, motdof)
		input.fixDOF_runjump2(loader, motdof)
	end,
	fixDOF_part2= function (loader, motdof)

		-- make cyclic
		do
			local nf=motdof:rows()
			local temp2=motdof:range(0,10):copy()
			local temp3=motdof:range(nf-10, nf):copy()
			local temp4=temp2:copy()
			temp4:stitch(temp3, temp2)
			motdof:range(nf-10,nf):assign(temp4:range(0,10))

			local err=diffPose(temp4:row(10), motdof:row(0))
			err:set(0,0)
			err:set(2,0)

			local ws=10
			for i=nf-1-ws, nf-1 do
				local pose=motdof:row(i)
				local w=sop.smoothMap(i, nf-1-ws, nf-1, 0, 1)
				motdof:row(i):assign(addPose(pose, scalePose(err, w)))
			end
		end

		local startL=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.LEFTANKLE))
		local startR=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.RIGHTANKLE))
		local nf=motdof:rows()
		for i=0, nf-1 do
			local pose=motdof:row(i)
			-- ZYX
			pose:set(startL+2, pose(startL+2)+math.rad(-10)) --toe 내리기
			pose:set(startR+2, pose(startR+2)+math.rad(-10))
		end
	end,
	contact=input.contact_runjump,
	downsample=2, -- because RL_step=60
	contactThr=-0.05, -- adjust timing
	--maxContactY=0.9, -- no adjust timing
	option_refCDMtraj={  },
	--IKconfig={ hasMM=0, hasCOM=0, useHead=0, debugDraw=true },
	option_poseSampler={ cyclicRoot=true, cyclicFoot=true},
	conWeightMap=function (_, i, startSwing, endSwing) 
		local w=sop.clampMap(i, startSwing, endSwing, -1, 1) 
		--return 1.0 
		return math.pow(w,2) *0.5+0.5
	end,
	doNotUseConHeight=true,
	option_contactFilter={ 
		useV2=true, filterEndPhase=0.5, maxFootVel=3,
		logQ1=7.2589012398406,
		logQ2=11.08924313578,
		logQm=8.6188994846528,
	},
}


motionInfos['run180']={
	isDeepmimicMot=false,
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	fn="../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Run_L_180.dof" ,
	fn2='gym_cdm2/spec/refTraj_run180.dat',
	start=2,
	noLooping=true,
	--maxContactY=0.76,
	--start=35,
	--weight_COMLV=0,
	--maxMimicCost=62, -- 안주는게 더 잘 됨.
	--fixDOF= input.fixDOF_runjump2,
	contact=
	{
		touchDown={
			{2, 15, 29, 55, 71, 88}, -- R touchdown moments
			{9, 21, 30, 63, 80}, -- L touchdown moments
			--{9, 21, 63, 80}, -- L touchdown moments
		},
		touchOff={
			{5, 20, 44, 60, 76, 88}, -- R touchoff moments
			{12, 28, 51, 68, 84}, -- L touchoff moments
			--{12, 51, 68, 84}, -- L touchoff moments
		},
	},
	downsample=2, -- because RL_step=60
	option_refCDMtraj={ usePendulum=true, filterCDMacc=15, useMocapOffset=true},
	option_contactFilter=--{ refSwingAlpha=0.1, maxFootVel=3, usefilteredPosForSpportPhaseToo =true, },
	{ 
		useV2=true, filterEndPhase=0.5, maxFootVel=3,
		logQ1=7.2589012398406,
		logQ2=11.08924313578,
		logQm=8.6188994846528,
		usefilteredPosForSpportPhaseToo =true,
	},

	--doNotUseConWeight =true,
	doNotUseConHeight=true,
	conWeightMap=function (_, i, startSwing, endSwing) 
		local w=sop.clampMap(i, startSwing, endSwing, -1, 1) 
		--return 1.0 
		return math.pow(w,2) 
	end,
	--IKconfig={ hasMM=0, hasCOM=0, useHead=0, debugDraw=true },
	fixDOF=function (loader, motdof)
		-- frame 2 == frame 88
		-- so frame 0 == frame 86
		motdof:resize(87)
		local nf=motdof:numFrames()

		for i=0, nf-1 do
			-- move upward
			local pose=motdof:row(i)
			pose:set(1, pose(1)-0.0)
		end
		makeCyclic(motdof,15)
	end,
}
motionInfos['run180_1']=shallowCopyTable(motionInfos['run180'])
motionInfos['run180_1'].fn2='gym_cdm2/spec/refTraj_run180_1.dat'

motionInfos['run180_2']=shallowCopyTable(motionInfos['run180'])
motionInfos['run180_2'].start=21
motionInfos['run180_2'].fn2='gym_cdm2/spec/refTraj_run180_2.dat'

motionInfos['run180_3']=shallowCopyTable(motionInfos['run180'])
motionInfos['run180_3'].start=55
motionInfos['run180_3'].fn2='gym_cdm2/spec/refTraj_run180_3.dat'

motionInfos['run90']={
	isDeepmimicMot=false,
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	fn="../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Run_L_90.dof" ,
	fn2='gym_cdm2/spec/refTraj_run90.dat',
	start=2,
	noLooping=true,
	fixDOF= function (loader, motdof)
		local startL=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.LEFTANKLE))
		local startR=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.RIGHTANKLE))
		local nf=motdof:rows()
		for i=0, nf-1 do
			local pose=motdof:row(i)
			-- ZYX
			pose:set(startL+2, pose(startL+2)-math.rad(10)) --toe 내리기
			pose:set(startL+1, pose(startL+1)-math.rad(0)) 
			pose:set(startL+0, pose(startL+0)+math.rad(0)) 
			pose:set(startR+2, pose(startR+2)-math.rad(10))
		end
		motdof:resize(57)
	end,
	contact=
	{
		touchDown={
			{2, 12, 21, 41, 58, },-- R touchdown moments
			{7, 16, 31, 49, }, -- L touchdown moments
		},
		touchOff={
			{5, 17, 30, 47, 60,}, -- R touchoff moments
			{10, 24, 39, 55, }, -- L touchoff moments
		},
	},
	downsample=2, -- because RL_step=60
	option_refCDMtraj={ usePendulum=true, filterCDMacc=15, useMocapOffset=true },
	option_contactFilter=--{ refSwingAlpha=0.1, maxFootVel=3, usefilteredPosForSpportPhaseToo =true, },
	{ 
		useV2=true, filterEndPhase=0.5, maxFootVel=3,
		logQ1=7.2589012398406,
		logQ2=11.08924313578,
		logQm=8.6188994846528,
		usefilteredPosForSpportPhaseToo =true,
	},
}
motionInfos['run90R']={
	isDeepmimicMot=false,
	skel="../Resource/motion/MOB1/hanyang_lowdof_T.wrl",
	fn="../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Run_R_90.dof" ,
	fn2='gym_cdm2/spec/refTraj_run90R.dat',
	start=4,
	noLooping=true,
	fixDOF= function (loader, motdof)
		local startL=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.LEFTANKLE))
		local startR=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.RIGHTANKLE))
		local nf=motdof:rows()
		for i=0, nf-1 do
			local pose=motdof:row(i)
			-- ZYX
			pose:set(startL+2, pose(startL+2)-math.rad(10)) --toe 내리기
			pose:set(startL+1, pose(startL+1)-math.rad(0)) 
			pose:set(startL+0, pose(startL+0)+math.rad(0)) 
			pose:set(startR+2, pose(startR+2)-math.rad(10))
		end
	end,
	contact=
	{
		touchDown={
			{4, 16, 30, 49,  },-- R touchdown moments
			{11, 21, 41,  }, -- L touchdown moments
		},
		touchOff={
			{7, 22, 38, 51, }, -- R touchoff moments
			{14, 29, 46,  }, -- L touchoff moments
		},
	},
	downsample=2, -- because RL_step=60
	option_refCDMtraj={ usePendulum=true, filterCDMacc=15, useMocapOffset=true },
	option_contactFilter=--{ refSwingAlpha=0.1, maxFootVel=3, usefilteredPosForSpportPhaseToo =true, },
	{ 
		useV2=true, filterEndPhase=0.5, maxFootVel=3,
		logQ1=7.2589012398406,
		logQ2=11.08924313578,
		logQm=8.6188994846528,
		usefilteredPosForSpportPhaseToo =true,
	},
}

return motionInfos
