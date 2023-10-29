-- same as the taesooLib one.
-- 모션에 따라 살짝씩 세팅 다르게 하면 더 보기 좋게 만들 수 있음. 특히 bindPoseBpostprocess 가 유용
vocaHyunwooLowT={
	chest='Chest',
	left_heel='LeftAnkle',
	left_knee='LeftKnee',
	left_hip='LeftHip',
	right_heel='RightAnkle',
	right_knee='RightKnee',
	right_hip='RightHip',
	left_shoulder='LeftShoulder',
	left_elbow='LeftElbow',
	right_shoulder='RightShoulder',
	right_elbow='RightElbow',
	neck='Neck',
	hips='Hips',
}
vocaHanyangLowT={
	chest='Chest',
	chest2='Chest2',
	left_heel='LeftAnkle',
	left_knee='LeftKnee',
	left_hip='LeftHip',
	right_heel='RightAnkle',
	right_knee='RightKnee',
	right_hip='RightHip',
	left_collar='LeftCollar', -- not present in the skin mesh.
	left_shoulder='LeftShoulder',
	left_elbow='LeftElbow',
	left_wrist='LeftWrist',
	right_collar='RightCollar', -- not present in the skin mesh.
	right_shoulder='RightShoulder',
	right_elbow='RightElbow',
	right_wrist='RightWrist',
	neck='Neck',
	head='Head',
	hips='Hips',
}
function hyunwoo_lowdof_T_poseA_hook(loader, pose)
	local qScale=function(q, sf)
		local qq=q:copy()
		qq:scale(sf)
		return qq
	end
	local function smoothShdrTwist(voca)
		local shoulder=loader:getRotJointIndexByVoca(voca)

		local q=pose.rotations(shoulder):copy()
		local q_c=pose.rotations(shoulder+1):copy()
		-- q*q_c= q_new*q_cnew

		local shdr_twist=quater()
		local shdr_rotate=quater()
		q:decomposeNoTwistTimesTwist(vector3(1,0,0), shdr_rotate, shdr_twist)
		--shdr_twist:scale(0)
		local w=	0.6   -- twist의 40%를 팔꿈치로 이동.
		pose.rotations(shoulder):assign(shdr_rotate*qScale(shdr_twist, w))
		pose.rotations(shoulder+1):assign(qScale(shdr_twist, 1-w)*q_c)
		if false then
			RE.output('lsh', shdr_twist:rotationAngleAboutAxis(vector3(1,0,0)))
		end
	end
	smoothShdrTwist(MotionLoader.LEFTSHOULDER)
	smoothShdrTwist(MotionLoader.RIGHTSHOULDER)
end
function fixDOF_wd2_all(loader, motdof)
	local startL=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.LEFTANKLE))
	local startR=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.RIGHTANKLE))
	for i=0, motdof:rows()-1 do
		local pose=motdof:row(i)
		pose:set(startL, pose(startL)+math.rad(-16)) --toe 벌리기
		pose:set(startR, pose(startR)+math.rad(9)) 
		pose:set(startL+1, pose(startL+1)+math.rad(2-7)) --toe 내리기
		pose:set(startR+1, pose(startR+1)+math.rad(-7-7))
	end
end
function fixDOF_loco_hl(loader, motdof)
	for i=0, motdof:rows()-1 do
		local pose=motdof:row(i)
		pose:set(1, pose(1)-0.03) -- root 내리기
	end
end
function fixDOF_kickball(loader, motdof)
	local startL=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.LEFTANKLE))
	local startR=loader.dofInfo:startR(loader:getTreeIndexByVoca(MotionLoader.RIGHTANKLE))
	for i=0, motdof:rows()-1 do
		local pose=motdof:row(i)
		pose:set(1, pose(1)+sop.map(i, 0, motdof:rows()-1, 0.08, 0.12)) 
		local pose=motdof:row(i)
		pose:set(startR-2, pose(startR-2)-math.rad(8))  -- +: 허벅지 뒤로
		pose:set(startR-1, pose(startR-1)+math.rad(16))  -- +: 무릎 뒤로
		pose:set(startL-2, pose(startL-2)-math.rad(0))  -- +: 허벅지 뒤로
		pose:set(startL-1, pose(startL-1)+math.rad(0))  -- +: 무릎 뒤로
		pose:set(startL, pose(startL)+math.rad(0)) --toe 벌리기
		pose:set(startR, pose(startR)+math.rad(-0)) 
		pose:set(startL+1, pose(startL+1)+math.rad(4)) --toe 내리기
		pose:set(startR+1, pose(startR+1)+math.rad(0))
	end
end
skinConfigHanyangLowT={
	entity='man_tshirt_shortsWithPockets_FBX.mesh',
	file_name= "../Resource/motion/MOB1/hanyang_lowdof_T.wrl",  -- unused for skinning 
	--mot_file= "../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Run_F_Jump.dof", 
	mot_file= "../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Run_L_180.dof", 
	bones=vocaHanyangLowT,
	entityScale=1.05,
	buildEdgeList=true,
	setPoseBhook=function(rs, loaderB, pose)
		local loaderA=rs.loaderA
		--[[
		do
			-- chest smoothing
			local bi=rs.AtoB(loaderA:getTreeIndexByVoca(MotionLoader.CHEST))
			local ri=loaderB:bone(bi):rotJointIndex()
			local q=pose.rotations(ri):copy()
			q:scale(0.29) -- 0.33 is correct but slightly decreased spine curve.
			pose.rotations(ri-1):assign(rs.bindPoseB_orig.rotations(ri-1)*q) -- spine
			pose.rotations(ri):assign(q) -- spine1
			pose.rotations(ri+1):assign(q) --spine2
		end
		local function shoulderSmoothing(voca)
			local bi=rs.AtoB(loaderA:getTreeIndexByVoca(voca))
			local ri=loaderB:bone(bi):rotJointIndex()
			-- rotate clavicle
			local q=pose.rotations(ri):copy()
			q:scale(0.5)
			pose.rotations(ri-1):assign(rs.bindPoseB_orig.rotations(ri-1)*q)
			pose.rotations(ri):assign(q)
		end
		shoulderSmoothing(MotionLoader.LEFTSHOULDER)
		shoulderSmoothing(MotionLoader.RIGHTSHOULDER)
		loaderB:setPose(pose)
		]]
	end,
	posErrorHook=function(rs, loaderB, posError)
		local bi=rs.AtoB(loaderA:getTreeIndexByVoca(MotionLoader.LEFTSHOULDER))
		if posError then posError(bi-1):assign(posError(bi)*0.5) end
		bi=rs.AtoB(loaderA:getTreeIndexByVoca(MotionLoader.RIGHTSHOULDER))
		if posError then posError(bi-1):assign(posError(bi)*0.5) end
	end,
	posErrorHook=nil,
	boneCorrespondences={
		-- map from name A to name B for all bones in A
		Hips='mixamorig:Hips',
		LeftHip='mixamorig:LeftUpLeg',
		LeftKnee='mixamorig:LeftLeg',
		LeftAnkle='mixamorig:LeftFoot',
		RightHip='mixamorig:RightUpLeg',
		RightKnee='mixamorig:RightLeg',
		RightAnkle='mixamorig:RightFoot',
		Chest='mixamorig:Spine',
		Chest1='mixamorig:Spine1',
		Chest2='mixamorig:Spine2',
		LeftCollar='mixamorig:LeftShoulder',
		LeftShoulder='mixamorig:LeftArm',
		LeftElbow='mixamorig:LeftForeArm',
		LeftWrist='mixamorig:LeftHand',
		RightCollar='mixamorig:RightShoulder',
		RightShoulder='mixamorig:RightArm',
		RightElbow='mixamorig:RightForeArm',
		RightWrist='mixamorig:RightHand',
		Neck='mixamorig:Neck',
		Head='mixamorig:Head',
	},
	-- from nameA to localPosB.
	-- for all other bones, localPosB is asssumed to be zero.
	markers={
		-- 마커의 위치를 loaderA(skin)의 관절 위치와 맞춤. 
		LeftElbow={vector3(-0.05,-0.14,-0.04)+vector3(0.1,0.1,0)*0-vector3(0.1,-0.1,0)*0.9, rotate=true}, -- when rotate==true, the skin moves more intuitively aligned with world axes.
		RightElbow={vector3(0.05,-0.14,-0.04)+vector3(0.1,0.1,0)*0-vector3(-0.1,-0.1,0)*0.9, rotate=true},
		LeftWrist=vector3(0.10,-0.06,0.22),
		RightWrist=vector3(-0.10,-0.06,0.22),
		LeftArm=vector3(0,0,0),
		RightArm=vector3(0,0,0),
		LeftKnee=vector3(0,0,0.04),
		RightKnee=vector3(0,0,0.04),
		LeftAnkle=vector3(0,-0.02,0.02),
		RightAnkle=vector3(0,-0.02,0.02),
		Head={vector3(0,-0.02-0.04,0.00+0.03), rotate=true}, -- when rotate==true, the skin moves more intuitively aligned with world axes.
		Neck={vector3(0,-0.02-0.04,0.00+0.03), rotate=true}, -- when rotate==true, the skin moves more intuitively aligned with world axes.
		Chest1={ vector3(0,0.02,0.03), rotate=true}, -- spine1
		Chest2={ vector3(0,0.02,0.03+0.02), rotate=true}, -- spine2
		LeftCollar={ vector3(-0.02,0.02,0.03+0.02), rotate=true},
		RightCollar={ vector3(0.02,0.02,0.03+0.02), rotate=true},
		LeftShoulder={ vector3(-0.04,-0.02,0.02), rotate=true},
		RightShoulder={ vector3(0.04,-0.02,0.02), rotate=true},
	},
	Tpose_markers={
		-- used only for calculating T-pose
		-- 이 마커는 위 markers와 다른 경우에만 적으면 됨.
		-- 가능하면 메시의 관절 중심으로 넣을 것
		LeftShoulder=vector3(0,0,0),
		RightShoulder=vector3(0,0,0),
		LeftElbow=vector3(0.05,-0.03,0.11),
		RightElbow=vector3(-0.05,-0.03,0.11),
	},
	scale={
		['mixamorig:Head']=0.85, -- applies to all children ... (너무 대두라)
	},
	-- 이게 false이면 angle retarget만 함. true이면 위치까지 정합.
	enforceMarkerPos=true, 
	enforceMarkerDir={
		-- works only when enforceMarkerPos==true
		-- this is slow, so use for selected joints only.
		LeftShoulder=true,
		RightShoulder=true,
	},
	-- apply additional global rotations (applied in sequence)
	-- 스킨이 꼬인것 같으면 풀리는 방향으로 돌리면 됨 (z축 바라보는 T포즈 상상하면서)
	bindPoseBpostprocess={
		{'mixamorig:LeftUpLeg',quater(math.rad(-10), vector3(0,1,0)),},
		{'mixamorig:LeftFoot',quater(math.rad(-20), vector3(0,1,0)),},
		{'mixamorig:RightUpLeg',quater(math.rad(10), vector3(0,1,0)),},
		{'mixamorig:RightFoot',quater(math.rad(10), vector3(0,1,0)),},

		{'mixamorig:LeftFoot',quater(math.rad(-10), vector3(1,0,0)),}, -- toe 들어올리기
		{'mixamorig:RightFoot',quater(math.rad(-15), vector3(1,0,0)),},
		--{'mixamorig:LeftUpLeg',quater(math.rad(-30), vector3(0,1,0)),},

		{'mixamorig:LeftUpLeg',quater(math.rad(-4), vector3(0,0,1)),}, -- 다리 사이 틈 좁히기.
		{'mixamorig:RightUpLeg',quater(math.rad(4), vector3(0,0,1)),},
		{'mixamorig:LeftLeg',quater(math.rad(-5), vector3(0,0,1)),}, -- 발 사이 틈 좁히기.
		{'mixamorig:RightLeg',quater(math.rad(5), vector3(0,0,1)),},
		{'mixamorig:LeftFoot',quater(math.rad(9), vector3(0,1,0))*quater(math.rad(6), vector3(0,0,1)),}, -- 발바닥 수평잡기
		{'mixamorig:RightFoot',quater(math.rad(-6), vector3(0,0,1)),},

		{'mixamorig:LeftFoot',quater(math.rad(17), vector3(1,0,0)),}, -- toe 내리기
		{'mixamorig:RightFoot',quater(math.rad(20), vector3(1,0,0)),},

		{'mixamorig:Spine1',quater(math.rad(-10), vector3(1,0,0)),}, --
		{'mixamorig:Spine2',quater(math.rad(-10), vector3(1,0,0)),}, --
		{'mixamorig:Neck',quater(math.rad(3), vector3(1,0,0)),}, --
		{'mixamorig:Head',quater(math.rad(-3+12), vector3(1,0,0)),}, --시선 내리기
	},
}
skinConfigHanyangLowT_robot={
	entity='Robot_TPOSE_fbx.mesh',
	file_name= "../Resource/motion/MOB1/hanyang_lowdof_T.wrl",  -- unused for skinning 
	--mot_file= "../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Run_F_Jump.dof", 
	mot_file= "../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Run_L_180.dof", 
	bones=vocaHanyangLowT,
	entityScale=1.05,
	buildEdgeList=true,
	boneCorrespondences={
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
	},
	-- from nameA to localPosB.
	-- for all other bones, localPosB is asssumed to be zero.
	markers={
	},
	Tpose_markers={
	},
	scale={
		['neck_01']=0.80, -- applies to all children ... (너무 대두라)
		['ball_l']=0.01, -- applies to all children ... (너무 대두라)
		['ball_r']=0.01, -- applies to all children ... (너무 대두라)
		['index_01_r' ]=0.99,
		['index_02_r' ]=0.99,
		['index_03_r' ]=0.99,
		['middle_01_r']=0.99,
		['middle_02_r']=0.99,
		['middle_03_r']=0.99,
		['pinky_01_r' ]=0.99,
		['pinky_02_r' ]=0.99,
		['pinky_03_r' ]=0.99,
		['ring_01_r'  ]=0.99,
		['ring_02_r'  ]=0.99,
		['ring_03_r'  ]=0.99,
		['index_01_l' ]=0.99,
		['index_02_l' ]=0.99,
		['index_03_l' ]=0.99,
		['middle_01_l']=0.99,
		['middle_02_l']=0.99,
		['middle_03_l']=0.99,
		['pinky_01_l' ]=0.99,
		['pinky_02_l' ]=0.99,
		['pinky_03_l' ]=0.99,
		['ring_01_l'  ]=0.99,
		['ring_02_l'  ]=0.99,
		['ring_03_l'  ]=0.99,
	},
	-- 이게 false이면 angle retarget만 함. true이면 위치까지 정합.
	enforceMarkerPos=true, 
	enforceMarkerDir={
		-- works only when enforceMarkerPos==true
		-- this is slow, so use for selected joints only.
		LeftShoulder=true,
		RightShoulder=true,
	},
	-- apply additional global rotations (applied in sequence)
	-- 스킨이 꼬인것 같으면 풀리는 방향으로 돌리면 됨 (z축 바라보는 T포즈 상상하면서)
	bindPoseBpostprocess={
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
	},
}
skinConfigHanyangLowT_robot_straightFingers={
	entity='Robot_TPOSE_fbx.mesh',
	file_name= "../Resource/motion/MOB1/hanyang_lowdof_T.wrl",  -- unused for skinning 
	--mot_file= "../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Run_F_Jump.dof", 
	mot_file= "../Resource/motion/MOB1/hanyang_lowdof_T_MOB1_Run_L_180.dof", 
	bones=vocaHanyangLowT,
	entityScale=1.05,
	buildEdgeList=true,
	boneCorrespondences={
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
	},
	-- from nameA to localPosB.
	-- for all other bones, localPosB is asssumed to be zero.
	markers={
	},
	Tpose_markers={
	},
	scale={
		['neck_01']=0.80, -- applies to all children ... (너무 대두라)
		['ball_l']=0.01, -- applies to all children ... (너무 대두라)
		['ball_r']=0.01, -- applies to all children ... (너무 대두라)
		['index_01_r' ]=0.99,
		['index_02_r' ]=0.99,
		['index_03_r' ]=0.99,
		['middle_01_r']=0.99,
		['middle_02_r']=0.99,
		['middle_03_r']=0.99,
		['pinky_01_r' ]=0.99,
		['pinky_02_r' ]=0.99,
		['pinky_03_r' ]=0.99,
		['ring_01_r'  ]=0.99,
		['ring_02_r'  ]=0.99,
		['ring_03_r'  ]=0.99,
		['index_01_l' ]=0.99,
		['index_02_l' ]=0.99,
		['index_03_l' ]=0.99,
		['middle_01_l']=0.99,
		['middle_02_l']=0.99,
		['middle_03_l']=0.99,
		['pinky_01_l' ]=0.99,
		['pinky_02_l' ]=0.99,
		['pinky_03_l' ]=0.99,
		['ring_01_l'  ]=0.99,
		['ring_02_l'  ]=0.99,
		['ring_03_l'  ]=0.99,
	},
	-- 이게 false이면 angle retarget만 함. true이면 위치까지 정합.
	enforceMarkerPos=true, 
	enforceMarkerDir={
		-- works only when enforceMarkerPos==true
		-- this is slow, so use for selected joints only.
		LeftShoulder=true,
		RightShoulder=true,
	},
	-- apply additional global rotations (applied in sequence)
	-- 스킨이 꼬인것 같으면 풀리는 방향으로 돌리면 됨 (z축 바라보는 T포즈 상상하면서)
	bindPoseBpostprocess={
	},
}
skinConfigHyunwooLowT={
	entity='man_tshirt_shortsWithPockets_FBX.mesh',
	file_name= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T3.wrl",  -- unused for skinning 
	mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_wd2_all.dof", fixDOF=fixDOF_wd2_all,-- unused for skinning 
	--mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_MOB1_Run_F_Jump.dof", fixDOF=fixDOF_loco_hl,-- unused for skinning 
	--mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof",fixDOF=fixDOF_loco_hl,-- unused for skinning 
	--mot_file="../../gang_python/work/gym_gang/Resource_RL/kickball/kickball_hyunwoo.dof",fixDOF=fixDOF_kickball,
	--mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_wd2_2foot_walk_turn2.dof",fixDOF=fixDOF_loco_hl,-- unused for skinning 
	--mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_justin_jump.dof",-- unused for skinning 
	--mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_jump3.dof",fixDOF=fixDOF_loco_hl,-- unused for skinning 
	bones=vocaHyunwooLowT,
	entityScale=1.05,
	buildEdgeList=true,
	setPoseAhook=hyunwoo_lowdof_T_poseA_hook,
	setPoseBhook=function(rs, loaderB, pose)
		local loaderA=rs.loaderA
		do
			-- chest smoothing
			local bi=rs.AtoB(loaderA:getTreeIndexByVoca(MotionLoader.CHEST))
			local ri=loaderB:bone(bi):rotJointIndex()
			local q=pose.rotations(ri):copy()
			q:scale(0.29) -- 0.33 is correct but slightly decreased spine curve.
			pose.rotations(ri-1):assign(rs.bindPoseB_orig.rotations(ri-1)*q) -- spine
			pose.rotations(ri):assign(q) -- spine1
			pose.rotations(ri+1):assign(q) --spine2
		end
		local function shoulderSmoothing(voca)
			local bi=rs.AtoB(loaderA:getTreeIndexByVoca(voca))
			local ri=loaderB:bone(bi):rotJointIndex()
			-- rotate clavicle
			local q=pose.rotations(ri):copy()
			q:scale(0.5)
			pose.rotations(ri-1):assign(rs.bindPoseB_orig.rotations(ri-1)*q)
			pose.rotations(ri):assign(q)
		end
		shoulderSmoothing(MotionLoader.LEFTSHOULDER)
		shoulderSmoothing(MotionLoader.RIGHTSHOULDER)
		loaderB:setPose(pose)
	end,
	posErrorHook=function(rs, loaderB, posError)
		local bi=rs.AtoB(loaderA:getTreeIndexByVoca(MotionLoader.LEFTSHOULDER))
		if posError then posError(bi-1):assign(posError(bi)*0.5) end
		bi=rs.AtoB(loaderA:getTreeIndexByVoca(MotionLoader.RIGHTSHOULDER))
		if posError then posError(bi-1):assign(posError(bi)*0.5) end
	end,
	posErrorHook=nil,
	boneCorrespondences={
		Hips='mixamorig:Hips',
		LeftHip='mixamorig:LeftUpLeg',
		LeftKnee='mixamorig:LeftLeg',
		LeftAnkle='mixamorig:LeftFoot',
		RightHip='mixamorig:RightUpLeg',
		RightKnee='mixamorig:RightLeg',
		RightAnkle='mixamorig:RightFoot',
		Chest='mixamorig:Spine1',
		LeftShoulder='mixamorig:LeftArm',
		LeftElbow='mixamorig:LeftForeArm',
		RightShoulder='mixamorig:RightArm',
		RightElbow='mixamorig:RightForeArm',
		Neck='mixamorig:Head',
	},
	-- from nameA to localPosB.
	-- for all other bones, localPosB is asssumed to be zero.
	markers={
		-- 마커의 위치를 loaderA의 관절 위치와 맞춤.
		LeftElbow=vector3(0.05,-0.03,0.11),
		RightElbow=vector3(-0.05,-0.03,0.11),
		LeftArm=vector3(0,0,0),
		RightArm=vector3(0,0,0),
		LeftKnee=vector3(0,0,0.04),
		RightKnee=vector3(0,0,0.04),
		Neck={vector3(0,-0.05,0.03), rotate=true}, -- when rotate==true, the skin moves more intuitively aligned with world axes.
		Chest={ vector3(0,0.05,-0.01), rotate=true}, -- spine1
		--Chest={ vector3(0,-0.05,0), rotate=true}, -- spine2
		LeftShoulder={ vector3(-0.02,0,0), rotate=true},
		RightShoulder={ vector3(0.02,0,0), rotate=true},
	},
	Tpose_markers={
		-- used only for calculating T-pose
		-- 이 마커는 위 markers와 다른 경우에만 적으면 됨.
		-- 가능하면 메시의 관절 중심으로 넣을 것
		LeftShoulder=vector3(0,0,0),
		RightShoulder=vector3(0,0,0),
	},
	scale={
		['mixamorig:Head']=0.87, -- applies to all children ... (너무 대두라)
	},
	-- 이게 false이면 angle retarget만 함. true이면 위치까지 정합.
	enforceMarkerPos=true, 
	enforceMarkerDir={
		-- works only when enforceMarkerPos==true
		-- this is slow, so use for selected joints only.
		LeftShoulder=true,
		RightShoulder=true,
	},
	-- apply additional global rotations (applied in sequence)
	-- 스킨이 꼬인것 같으면 풀리는 방향으로 돌리면 됨 (z축 바라보는 T포즈 상상하면서)
	bindPoseBpostprocess={
		{'mixamorig:LeftUpLeg',quater(math.rad(-10), vector3(0,1,0)),},
		{'mixamorig:LeftFoot',quater(math.rad(-20), vector3(0,1,0)),},
		{'mixamorig:RightUpLeg',quater(math.rad(10), vector3(0,1,0)),},
		{'mixamorig:RightFoot',quater(math.rad(10), vector3(0,1,0)),},

		{'mixamorig:LeftFoot',quater(math.rad(-10), vector3(1,0,0)),}, -- toe 들어올리기
		{'mixamorig:RightFoot',quater(math.rad(-15), vector3(1,0,0)),},
		--{'mixamorig:LeftUpLeg',quater(math.rad(-30), vector3(0,1,0)),},

		{'mixamorig:LeftUpLeg',quater(math.rad(-4), vector3(0,0,1)),}, -- 다리 사이 틈 좁히기.
		{'mixamorig:RightUpLeg',quater(math.rad(4), vector3(0,0,1)),},
		{'mixamorig:LeftLeg',quater(math.rad(-5), vector3(0,0,1)),}, -- 발 사이 틈 좁히기.
		{'mixamorig:RightLeg',quater(math.rad(5), vector3(0,0,1)),},
		{'mixamorig:LeftFoot',quater(math.rad(9), vector3(0,1,0))*quater(math.rad(6), vector3(0,0,1)),}, -- 발바닥 수평잡기
		{'mixamorig:RightFoot',quater(math.rad(-6), vector3(0,0,1)),},

		{'mixamorig:LeftFoot',quater(math.rad(7), vector3(1,0,0)),}, -- toe 내리기
		{'mixamorig:RightFoot',quater(math.rad(7), vector3(1,0,0)),},

		{'mixamorig:Neck',quater(math.rad(-13), vector3(1,0,0)),}, --
		{'mixamorig:Head',quater(math.rad(13+12), vector3(1,0,0)),}, --시선 내리기
	},
}

skinConfigHyunwooLowT_muscle={
	entity='hyunwoo.mesh',
	file_name= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T.wrl",  -- unused for skinning 
	mot_file= "../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_wd2_all.dof",-- unused for skinning 
	bones=vocaHyunwooLowT,
	entityScale=0.0254*95,
	setPoseAhook=hyunwoo_lowdof_T_poseA_hook,
	boneCorrespondences={
		Hips='Bip01',
		LeftHip='Bip01_L_Thigh',
		LeftKnee='Bip01_L_Calf',
		LeftAnkle='Bip01_L_Foot',
		RightHip='Bip01_R_Thigh',
		RightKnee='Bip01_R_Calf',
		RightAnkle='Bip01_R_Foot',
		Chest='Bip01_Spine2', -- or Bip01_Spine1
		LeftShoulder='Bip01_L_UpperArm',
		LeftElbow='Bip01_L_Forearm',
		RightShoulder='Bip01_R_UpperArm',
		RightElbow='Bip01_R_Forearm',
		Neck='Bip01_Head',
	},
	-- from nameA to localPosB.
	-- for all other bones, localPosB is asssumed to be zero.
	-- when rotate==true, the skin moves more intuitively aligned with world axes.
	markers={
		Chest={vector3(0,-0.11,0), rotate=true}, -- localpos=R_bindpose:inverse()*localpos
		Neck={vector3(0,-0.02,0), rotate=true}, -- localpos=R_bindpose:inverse()*localpos
		LeftShoulder={vector3(-0.03,-0.02,0), rotate=true}, -- localpos=R_bindpose:inverse()*localpos
		RightShoulder={vector3(0.01,-0.05,0), rotate=true}, -- localpos=R_bindpose:inverse()*localpos
		LeftElbow={vector3(0.03,0,0), rotate=true}, -- shorten left forearm
		RightElbow={vector3(-0.03,0,0), rotate=true}, -- same
	},
	scale={
		-- works only when enforceMarkerPos==true
		Bip01_Spine1=0.93, -- applies to all children ...
	},
	-- 이게 false이면 angle retarget만 함. true이면 위치까지 정합.
	enforceMarkerPos=true, 
	-- apply additional global rotations (applied in sequence)
	-- 스킨이 꼬인것 같으면 풀리는 방향으로 돌리면 됨 (z축 바라보는 T포즈 상상하면서)
	bindPoseBpostprocess={
		{'Bip01_L_Thigh',quater(math.rad(-10), vector3(0,1,0)),},
		{'Bip01_L_Foot',quater(math.rad(-20), vector3(0,1,0)),},
		{'Bip01_R_Thigh',quater(math.rad(5), vector3(0,1,0)),},
		{'Bip01_R_Foot',quater(math.rad(5), vector3(0,1,0)),},
		{'Bip01_L_Foot',quater(math.rad(-6), vector3(1,0,0)),}, -- toe 들어올리기
		{'Bip01_R_Foot',quater(math.rad(-6), vector3(1,0,0)),},
	},
}
