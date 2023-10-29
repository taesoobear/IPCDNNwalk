useHanyangInsteadOfHyunwoo=true
require("gym_cdm2/module/RetargetConfigPreset")

function diffPose(a, b)
	local pose= b-a
	local qd=quater()
	qd:difference(a:toQuater(3), b:toQuater(3))
	pose:setQuater(3, qd)
	return pose
end
function addPose(a, b)
	local pose= a+b
	local q=b:toQuater(3)*a:toQuater(3)
	pose:setQuater(3, q)
	return pose
end
function scalePose(a,s)
	local pose= a*s local q=a:toQuater(3)
	q:scale(s)
	pose:setQuater(3, q)
	return pose
end
function makeCyclic(motdof, spread)
	-- make exactly cyclic
	local nf=motdof:rows()
	spread=spread or math.round(motdof:rows()/2)
	local out=motdof:Stitch(motdof, spread)
	out=out:Stitch(motdof, spread)

	local initialTF=MotionDOF.rootTransformation(motdof:row(0))
	motdof:matView():assign(out:matView():sub(nf-1, nf-1+nf, 0,0))

	local currTF=MotionDOF.rootTransformation(motdof:row(0))
	motdof:transform((initialTF*currTF:inverse()):project2D())
end
input={
	limbs={
		{'LeftKnee', 'LeftAnkle', vector3(0, -0.12, 0.15), reversed=false},
		{'LeftKnee', 'LeftAnkle', vector3(0, -0.15, -0.03), reversed=false},
		{'RightKnee', 'RightAnkle', vector3(0, -0.11, 0.15), reversed=false},
		{'RightKnee', 'RightAnkle', vector3(0, -0.16, -0.03), reversed=false},
	},
	hands={
		{'LeftElbow', 'LeftWrist', vector3(0.05, 0, 0), reversed=true},
		{'RightElbow', 'RightWrist', vector3(-0.05, 0, 0), reversed=true},
	},
	head={'Neck', vector3(0,0.05,0), vector3(0,0.7,0)}, -- name, local pos, local pos WRT COM
	lowerbody={2,9},
	motionFrameRate=30,
	bones={
		chest='Chest',
		left_heel='LeftAnkle',
		left_ball='LeftAnkle',
		left_knee='LeftKnee',
		left_hip='LeftHip',
		right_heel='RightAnkle',
		right_ball='RightAnkle',
		right_knee='RightKnee',
		right_hip='RightHip',
		left_shoulder='LeftShoulder',
		left_elbow='LeftElbow',
		right_shoulder='RightShoulder',
		right_elbow='RightElbow',
		neck='Neck',
		hips='Hips',
	},
	-- L, R
	legs={ { 'LeftAnkle', 'RightElbow'}, { 'RightAnkle', 'LeftElbow'},}, -- this defines the bones related to Lpose and Rpose
	skinScale=100,
	-- the quality of the reference motion is too low, so I manually adjusted some angles which are expecially problematic.
	fixDOF_humanoid3d_run=function (loader, motdof)
		local nf=motdof:rows()
		local startL=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.LEFTANKLE))
		local startR=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.RIGHTANKLE))
		local nf=motdof:rows()
		for i=0, nf-1 do
			-- move forward
			local pose=motdof:row(i)
			-- ZYX
			pose:set(startL+2, pose(startL+2)+math.rad(10)) --toe 내리기
			pose:set(startR+2, pose(startR+2)+math.rad(10))

			-- 약간 뒤로 기울이기.
			pose:setQuater(3, pose:toQuater(3)*quater(math.rad(-5), vector3(1,0,0)))
		end
		math.filterQuat(motdof:matView():sub(0,0,3,7), 5)
		--math.filter(motdof:matView():sub(0,0,7,0), 3)

		makeCyclic(motdof)
	end,
	fixDOF_runjump=function (loader, motdof)
		local nf=motdof:numFrames()
		for i=0, nf-1 do
			-- move upward
			local pose=motdof:row(i)
			pose:set(1, pose(1)-0.12)
		end
		--makeCyclic(motdof)
	end,
	fixDOF_runjump2=function (loader, motdof)
		local src=motdof:copy()
		motdof:resize(motdof:numFrames()+2)
		motdof:range(0, src:numFrames()):assign(src)

		-- extrapolate the first and last frames 
		local ds=diffPose(src:row(1), src:row(0))
		local posen1=addPose(src:row(0), ds)

		local ns=src:rows()
		local de=diffPose(src:row(ns-2), src:row(ns-1))
		motdof:row(motdof:numFrames()-2):assign(addPose(src:row(src:numFrames()-1),de))
		motdof:row(motdof:numFrames()-1):assign(addPose(motdof:row(motdof:numFrames()-2),de))

		local nf=motdof:numFrames()

		--[[
		-- make cyclic
		local err=diffPose(motdof:row(nf-1), posen1)
		err:set(0,0)
		err:set(2,0)

		local ws=10
		for i=nf-1-ws, nf-1 do
			local pose=motdof:row(i)
			local w=sop.mapCos(i, nf-1-ws, nf-1, 0, 1)
			motdof:row(i):assign(addPose(pose, scalePose(err, w)))
		end
		]]


		for i=0, nf-1 do
			-- move upward
			local pose=motdof:row(i)
			pose:set(1, pose(1)-0.0)
		end
	end,
	fixDOF_runjump_part2=function (loader, motdof)

		local nf=motdof:rows()
		local startL=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.LEFTSHOULDER))
		local startR=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.RIGHTSHOULDER))
		local nf=motdof:rows()
		-- remove unnatural shoulder poses
		motdof:matView():sub(29,40, startL, startL+2):linspace(
		motdof:row(29), motdof:row(40))
		motdof:matView():column(startL+3):rmult(0.7) 
		-- reduce elbow angle
		local relbow=motdof:matView():column(startR+3)
		for i=0, nf-1 do
			if relbow(i)>1.2 then relbow:set(i, 1.2) end
		end



		if false then
			require("subRoutines/MatplotLib")
			plotter=MatplotLib()
			local xfn=CT.colon(0, nf)
			plotter:figure{1, nsubfig={1,1}, subfig_size={3,3}} -- one by one plotting area.
			plotter:add('grid(True)')
			plotter:subplot(0,0)
			plotter:plot(xfn, motdof:matView():column(startL+3))
			plotter:plot(xfn, motdof:matView():column(startR+3))
			plotter:plot(xfn, relbow)
			plotter:xlabel('x')
			plotter:ylabel('t')
			plotter:legends('y1(t)', 'y2(t)', 'y3(t)')
			plotter:savefig('plot.png')
			plotter:savefig('plot.eps')
			plotter:close()
			os.exit(0)
		end

		--makeCyclic(motdof)
	end,
	fixDOF_backflip=function (loader, motdof)
		local nf=motdof:numFrames()
		for i=0, nf-1 do local pose=motdof:row(i) pose:set(1, pose(1)-0.03) end
		
		motdof:resample(16, 30) -- make 30hz
	end,
	fixDOF_walk=function (loader, motdof)
		local nf=motdof:rows()
		local startL=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.LEFTANKLE))
		local startR=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.RIGHTANKLE))

		if true then
			-- improve motion quality!

			-- remove discontinuity at frame 18
			motdof:removeDiscontinuity(18)

			-- replace the problametic last pose with an extrapolated pose
			local lastF=motdof:rows()-1
			local delta=MotionDOF.deltaPose(motdof:row(lastF-2), motdof:row(lastF-1))
			motdof:row(lastF):assign(MotionDOF.addPose(motdof:row(lastF-1), delta))

			-- make cyclic
			do
				local temp=MotionDOF(loader.dofInfo)
				temp:alignSimple(motdof, motdof:range(0,18))
				local lastA=motdof:rows()-1
				temp:range(lastA-18, temp:numFrames()):removeDiscontinuity(lastA-(lastA-18))

				local A=motdof:rootTransformation(17)
				local B=temp:rootTransformation(motdof:rows()-1+17)
				local delta=transf()
				delta:difference(B,A)
				delta.rotation:assign(delta.rotation:rotationY())
				delta.translation.y=0

				motdof:range(0, 18):assign(temp:range(motdof:rows()-1, motdof:rows()-1+18))
				motdof:range(0, 18):transform(delta)

				motdof:range(lastA-18, lastA+1):assign(temp:range(lastA-18, lastA+1))

				-- remove lfoot sliding
				local lhipZ=7+1
				local twodeg=math.rad(2)
				for i=0, 38 do
					local pose=motdof:row(i)
					pose:set(lhipZ, pose(lhipZ)+sop.piecewiseLinearMap(i, {0, 15, 20, 34, 38}, {0, 0, -twodeg, twodeg, 0}))
				end
				local rhipZ=7+3+1+3+1
				for i=0, 38 do
					local pose=motdof:row(i)
					pose:set(rhipZ, pose(rhipZ)+twodeg)
				end
			end
		end

		for i=0, nf-1 do
			-- move forward
			local pose=motdof:row(i)
			-- ZYX
			--pose:set(startL+2, pose(startL+2)+math.rad(10)) --toe 내리기
			--pose:set(startR+2, pose(startR+2)+math.rad(10))
		end
	end,
	fixDOF_walk_part2=function (loader, motdof) -- part2 does not affect the reference CDM trajectory so that re-training is unnecessary -.-
		local nf=motdof:rows()
		local L=loader:getTreeIndexByVoca(MotionLoader.LEFTHIP)
		local R=loader:getTreeIndexByVoca(MotionLoader.RIGHTHIP)

		for i=0, nf-1 do
			-- move forward
			local pose=motdof:row(i)
			loader:setPoseDOF(pose)
			local tiltChest=quater(math.rad(-5), vector3(1,0,0))
			loader:bone(1):getLocalFrame().rotation:rightMult(tiltChest)
			loader:bone(L):getLocalFrame().rotation:leftMult(tiltChest:inverse())
			loader:bone(R):getLocalFrame().rotation:leftMult(tiltChest:inverse())
			loader:fkSolver():forwardKinematics()
			loader:getPoseDOF(pose)
			-- ZYX
			--pose:set(startL+2, pose(startL+2)+math.rad(10)) --toe 내리기
			--pose:set(startR+2, pose(startR+2)+math.rad(10))
		end
	end,
	contact_humanoid3d_run=
	{
		touchDown=
		{
			{0, 23}, -- R touchdown moments
			{11}, -- L touchdown moments
		},
		touchOff={
			{5}, -- R touchoff moments
			{17}, -- L touchoff moments
		}
	},
	contact_runjump=
	{
		touchDown={
			{0, 28, 43}, -- R touchdown moments
			{7, 31}, -- L touchdown moments
		},
		touchOff={
			{4,32}, -- R touchoff moments
			{12, 39}, -- L touchoff moments
		},
	},
	contact_backflip=
	{
		touchDown=
		{
			{0,12,36}, -- R touchdown moments
			{0,12,36}, -- L touchdown moments
		},
		touchOff={
			{4,21}, -- R touchoff moments
			{4, 21}, -- L touchoff moments
		}
	},
	contact_walk=
	{
		touchDown={
			{0, 35}, -- R touchdown moments
			{17}, -- L touchdown moments
		},
		touchOff={
			{19}, -- R touchoff moments
			{37}, -- L touchoff moments
		},
	},

}

