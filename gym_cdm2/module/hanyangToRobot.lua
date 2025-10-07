

function RE.createRobotFromHanyangLowDOF(mLoader1, modifyHanyangToMatchRobotProportion)
	-- B (렌더링 목적 외에는 쓰고 싶지 않은 이상한 스켈레톤)
	local mLoader2=RE.createLoader('Robot_TPOSE_fbx.mesh', {entityScale=1.05, skinScale=100, newRootBone='pelvis'})
	-- 보통은 B가 훨씬 많은 관절 수를 갖고 있음. 
	local poseAtoB=RE.createPoseTransferFromHanyangLowDOFtoRobot(mLoader1, mLoader2, modifyHanyangToMatchRobotProportion)

	local bindpose0=mLoader2.bindpose:copy()
	bindpose0:setVec3(0,vector3(0))
	mLoader2.loader:setPoseDOF(bindpose0) -- A-pose

	return { mLoader1, mLoader2, poseAtoB}
end


-- loader1: hanyang, loader2: robot in A-pose
function RE.createPoseTransferFromHanyangLowDOFtoRobot(mLoader1, mLoader2, modifyHanyangToMatchRobotProportion)
	local pose=mLoader2.loader:pose()
	pose.translations(0):zero()
	mLoader2.loader:setPose(pose)
	
	if not mLoader1.loader then
		mLoader1={ loader=mLoader1}
	end

	local boneCorrespondences ={
		-- map from name A to name B for all bones in A
		Hips='pelvis',
		LeftHip='thigh_l',
		LeftKnee='calf_l',
		LeftAnkle='foot_l',
		RightHip='thigh_r',
		RightKnee='calf_r',
		RightAnkle='foot_r',
		Chest='spine_01',
		Chest1='spine_02',
		Chest2='spine_03',
		LeftCollar='clavicle_l',
		LeftShoulder='upperarm_l',
		LeftElbow='lowerarm_l',
		LeftWrist='hand_l',
		RightCollar='clavicle_r',
		RightShoulder='upperarm_r',
		RightElbow='lowerarm_r',
		RightWrist='hand_r',
		Neck='neck_01',
		Head='head',
	}

	-- loader1과 2의 자세를 동일하게 만들기. 
	mLoader1.loader:updateInitialBone() -- T-pose


	local RT=require('retargetting/module/retarget_common')
	RT.setVoca(mLoader1.loader) -- 자동. 잘 안되면 일일이 지정해야함. 
	RT.setVoca(mLoader2.loader) -- 자동. 잘 안되면 일일이 지정해야함. 
	RT.gotoTpose(mLoader2.loader) -- T-pose

	if true then
		-- loader2의 등과 손가락 구부려주기.
		local bindPoseBedit={
			{'pelvis', quater(math.rad(6), vector3(1,0,0))},
			{'thigh_l', quater(math.rad(-2), vector3(1,0,0))},
			{'thigh_r', quater(math.rad(-2), vector3(1,0,0))},
			{'index_01_r', quater(math.rad(60), vector3(0,0,1))},
			{'index_02_r', quater(math.rad(60), vector3(0,0,1))},
			{'index_03_r', quater(math.rad(60), vector3(0,0,1))},
			{'middle_01_r',quater(math.rad(60), vector3(0,0,1))},
			{'middle_02_r',quater(math.rad(60), vector3(0,0,1))},
			{'middle_03_r',quater(math.rad(60), vector3(0,0,1))},
			{'ring_01_r',  quater(math.rad(65), vector3(0,0,1))},
			{'ring_02_r',  quater(math.rad(65), vector3(0,0,1))},
			{'ring_03_r',  quater(math.rad(65), vector3(0,0,1))},
			{'pinky_01_r', quater(math.rad(70), vector3(0,0,1))},
			{'pinky_02_r', quater(math.rad(70), vector3(0,0,1))},
			{'pinky_03_r', quater(math.rad(70), vector3(0,0,1))},
			{'index_01_l', quater(math.rad(-60), vector3(0,0,1))},
			{'index_02_l', quater(math.rad(-60), vector3(0,0,1))},
			{'index_03_l', quater(math.rad(-60), vector3(0,0,1))},
			{'middle_01_l',quater(math.rad(-60), vector3(0,0,1))},
			{'middle_02_l',quater(math.rad(-60), vector3(0,0,1))},
			{'middle_03_l',quater(math.rad(-60), vector3(0,0,1))},
			{'ring_01_l',  quater(math.rad(-65), vector3(0,0,1))},
			{'ring_02_l',  quater(math.rad(-65), vector3(0,0,1))},
			{'ring_03_l',  quater(math.rad(-65), vector3(0,0,1))},
			{'pinky_01_l', quater(math.rad(-70), vector3(0,0,1))},
			{'pinky_02_l', quater(math.rad(-70), vector3(0,0,1))},
			{'pinky_03_l', quater(math.rad(-70), vector3(0,0,1))},
			{'middle_01_l',  quater(math.rad(10), vector3(0,1,0))},
			{'ring_01_l',  quater(math.rad(20), vector3(0,1,0))},
			{'pinky_01_l', quater(math.rad(30), vector3(0,1,0))},
			{'middle_01_r',  quater(math.rad(-10), vector3(0,1,0))},
			{'ring_01_r',  quater(math.rad(-20), vector3(0,1,0))},
			{'pinky_01_r', quater(math.rad(-30), vector3(0,1,0))},
		}

		local loader=mLoader2.loader
		for k, v in ipairs(bindPoseBedit) do
			local shdr=loader:getBoneByName(v[1])
			loader:rotateBoneGlobal(shdr, v[2])
		end
	end
	if false then
		-- to draw the bind pose of mLoader1 and mLoader2
		mSkin3=RE.createSkin(mLoader2.loader)
		mSkin3:setScale(100)
		mSkin3:setTranslation(100,0,0)
		mSkin3:setPose(mLoader2.loader:pose())

		mSkin4=RE.createSkin(mLoader1.loader)
		mSkin4:setScale(100)
		mSkin4:setTranslation(100,0,0)
		mSkin4:setPose(mLoader1.loader:pose())
		mSkin4:setMaterial('lightgrey_transparent')
	end

	-- pose converter.
	local PoseAtoB=RT.createAngleRetarget(mLoader1.loader, mLoader2.loader, boneCorrespondences)


	-- PoseAtoB는 각도만 맞추므로 관절 위치가 정확하게 맞지 않음. 최대한 맞추기 위해 B에서 IK를 추가로 푸는 방법도 있지만, 
	-- 더 쉬운 방법은 A스켈레톤을 약간 변형해서 B에 강제로 맞춰버리는 방법임. 이경우 A와 B의 관절위치는 자세에 관계없이 정확하게 동일하게 됨.
	-- 물론 두 스켈레톤의 비율이나 현재 자세가 많이 다르면 A의 모양이 망가질 수 있음. 
	-- 아래 코드가 5초간 빨간 sphere들을 그림. 
	RT.fitSkeletonAtoB(mLoader1.loader,  mLoader2.loader, boneCorrespondences ) 


	return PoseAtoB
end
function RE.createRobotSkinForHanyangMotion(mRobotInfo)
	local mLoader1, mLoader2, PoseAtoB=unpack(mRobotInfo)

	local mSkin2=RE.createSkin(mLoader2, { boneScale={ ['neck_01']=0.80, -- applies to all children ... (너무 대두라)
		['ball_l']=0.01, -- applies to all children ... (너무 대두라)
		['ball_r']=0.01, -- applies to all children ... (너무 대두라)
	}})
	mSkin2:setScale(100)
	mSkin2:setPoseTransfer(PoseAtoB.pt)

	return mSkin2
end

