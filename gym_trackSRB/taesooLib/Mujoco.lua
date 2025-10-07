
local Mujoco={ 
	-- will be updated.
	footIKconfig={
		{'LeftLeg', 'LeftFoot', vector3(0.000000,-0.093740,-0.04), childCon=2 },
		{'LeftToeBase', vector3(0,0.01,0),},
		{'RightLeg', 'RightFoot', vector3(0.000000,-0.094795,-0.04), childCon=4},
		{'RightToeBase', vector3(0,0.01,0),},
	}
}

Mujoco.lafan={
	retargetConfig={
		A={
			boneCorrespondences ={
				Hips ="root",
				LeftArm ="left_shoulder",
				LeftFoot ="left_ankle",
				LeftForeArm ="left_elbow",
				LeftLeg ="left_knee",
				LeftUpLeg ="left_hip",
				LeftHand ="left_wrist",
				Neck ="neck",
				Spine2="chest",
				RightArm ="right_shoulder",
				RightFoot ="right_ankle",
				RightForeArm ="right_elbow",
				RightUpLeg ="right_hip",
				RightHand ="right_wrist",
			},
			reusePreviousBinding =true,
			skel ="/Users/taesookwon/sample_SAMP/lafan1/aiming1_subject1.bvh",
			skinScale =0.979,
		},
		B={
			EE ={
			},
			motion ="",
			skel ="/Users/taesookwon/taesooLib/Resource/motion/Mixamo/passive_marker_man_T_nofingers.fbx.dat",
			skinScale =100,
		},
		bindposes_quat={{"__userdata", "Pose", {"__userdata", "matrixn", {{0, 91.882057189941, 0, }, }, }, {"__userdata", "matrixn", {{0.52119910967873, 0.46017326620957, 0.52772279121599, 0.48795564221281, }, {0.012858730592775, 0.053615630075146, -0.99830695602399, -0.018526705408766, }, {0.99460617274978, -0.07423455017246, -0.0069521643279197, -0.072107281878381, }, {0.80558105246345, -0.059453755195192, 0.0052697119039604, 0.58947149977036, }, {0.98252429453143, -5.0326236432175e-06, 2.6518197066786e-05, 0.18613438673972, }, {0.06584651601154, -0.069823457655907, 0.99292129325943, -0.069973041109717, }, {0.99178644312071, 0.089316696449715, 0.019140137998092, -0.089531190624814, }, {0.77591089995246, 0.07319217396565, -0.0037483537136183, 0.62657093042175, }, {0.98252428523247, 4.8985016190349e-06, -2.6747888170185e-05, 0.18613443579568, }, {0.99815987065668, 0.00095373135580639, -0.0036619901934932, 0.060519028701419, }, {0.99940249907397, 0.001643296274537, -0.0074265897010878, 0.033716319304842, }, {0.99949229347352, 0.001644196664769, -0.007420242334087, 0.030941750242496, }, {0.99739744453066, -0.023430001761392, -0.053119708386207, 0.042751248427781, }, {0.98099626137272, -0.052442597897504, 0.10008206180074, -0.15773297056804, }, {0.011111163153514, -0.70476449753853, 0.036754155043061, -0.7084014943164, }, {0.98864258207914, -0.088604542488528, 0.12131958137437, -0.0040791084110475, }, {0.97010412615921, -0.017965641117238, -0.13774142586436, -0.1990038184281, }, {0.96052618953813, 0.26741063980865, -0.0060305459148209, 0.076450123900832, }, {0.033784653401828, 0.69676659737746, -0.062631317982708, -0.71375921989233, }, {0.99192541777146, 0.053363918300428, -0.10736488816832, -0.041340519967136, }, {0.97198180944749, 0.016545145905086, 0.13793357147602, -0.18960999475245, }, {0.98966030269221, -0.12622055416549, 0.021448277757192, 0.06465932541067, }, }, }, }, {"__userdata", "Pose", {"__userdata", "matrixn", {{-0.0078004963459822, 0.90769010423738, 9.3795171733859e-05, }, }, }, {"__userdata", "matrixn", {{0.99995950027337, 0.0089992279574284, 0, -0.00010820902854154, }, {1, 3.75199235426e-19, -1.0578246730467e-22, -3.6715531079582e-21, }, {1, -1.359454018381e-18, -1.0578246730467e-22, -3.674352180196e-21, }, {0.96206743271693, -0.2491381631361, -0.070828978544049, 0.08567196958769, }, {0.9719733278534, -2.4164654314683e-18, 0.23509115241023, 1.1422018398947e-17, }, {0.95118705945468, -0.28300138501061, 0.053576984139254, -0.11082824900974, }, {0.9731908994082, -4.4641517007734e-17, -0.22999885501684, -1.3270858489323e-17, }, {0.99945065850584, -0.001556928433145, 0.011425377083753, -0.031071175461983, }, {0.99327098297449, 0.11581344645977, 1.8305296280022e-19, 2.1564311405351e-19, }, {0.97601905921572, -0.12843709722558, -0.15963036800316, 0.073544909511816, }, {0.999480619111, -0.014937367712007, -0.00375308582727, 0.028306914598766, }, {0.99677689337104, 0.080223592798942, -9.8031058412493e-20, -8.871833472216e-20, }, {0.99094157499273, -0.086509874578801, 0.072224921539798, -0.073036958177751, }, }, }, }, },
		bindposes_map={{['R']={['LeftFoot']=3, ['Hips']=0, ['LeftToe']=4, ['RightHand']=21, ['Spine']=9, ['Spine2']=11, ['Spine1']=10, ['RightShoulder']=18, ['RightFoot']=7, ['RightArm']=19, ['RightUpLeg']=5, ['Head']=13, ['LeftUpLeg']=1, ['RightForeArm']=20, ['LeftForeArm']=16, ['RightToe']=8, ['LeftHand']=17, ['LeftArm']=15, ['Neck']=12, ['LeftShoulder']=14, ['RightLeg']=6, ['LeftLeg']=2, }, ['T']={['Hips']=0, }, }, {['R']={['left_ankle']=12, ['left_shoulder']=5, ['right_hip']=7, ['root']=0, ['left_hip']=10, ['right_elbow']=4, ['left_elbow']=6, ['right_ankle']=9, ['right_shoulder']=3, ['neck']=2, ['chest']=1, ['right_knee']=8, ['left_knee']=11, }, ['T']={['root']=0, }, }, }
	},
	constraintMarkingConfig={
		filter_size=3,
		markers={
			-- testConstraintMarking.lua -> results go to ....fbx.con.lua or ...bvh.con.lua
			default_param ={
				thr_speed=0.3*111, -- speed limit
				thr_height=0.02*111,  -- height threshhold (the lower limit)
			},
			{
				"leftHeel", 
				bone='leftFoot', 
				lpos=vector3(0,-0.07,0)*111, -- local position
			},
			{
				"leftToe", 
				bone='LeftToe', 
				lpos=vector3(0,0,0),
			}, {
				"rightHeel", 
				bone='RightFoot',
				lpos=vector3(0,-0.07,0)*111,
			}, 
			{
				"rightToe", 
				bone='RightToe',
				lpos=vector3(0,0,0),
			}, 		
		}
	},
}

Mujoco.hanyang={
	retargetConfig={
		A={
			boneCorrespondences ={
				Hips ="root",
				LeftShoulder ="left_shoulder",
				LeftElbow ="left_elbow",
				LeftWrist ="left_wrist",
				RightShoulder ="right_shoulder",
				RightElbow ="right_elbow",
				RightWrist ="right_wrist",
				LeftHip ="left_hip",
				LeftKnee ="left_knee",
				LeftAnkle ="left_ankle",
				RightHip ="right_hip",
				RightKnee ="right_knee",
				RightAnkle ="right_ankle",
				Neck ="neck",
				Chest2="chest",
			},
			reusePreviousBinding =true,
			skel ="/Users/taesookwon/sample_SAMP/lafan1/aiming1_subject1.bvh",
			skinScale =100,
		},
		B={
			EE ={
			},
			motion ="",
			skel ="/Users/taesookwon/taesooLib/Resource/motion/Mixamo/passive_marker_man_T_nofingers.fbx.dat",
			skinScale =100,
		},
		bindposes_quat={
			{"__userdata", "Pose", {"__userdata", "matrixn", {{0, 0.92, 0, }, }, }, {"__userdata", "matrixn", {{1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, }, }, },
			{"__userdata", "Pose", {"__userdata", "matrixn", {{0, 0.85, 0, }, }, }, {"__userdata", "matrixn", {{1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, }, }, }
			--{"__userdata", "Pose", {"__userdata", "matrixn", {{0.015785667868466, 0.92298682847682, 0.0049433574981487, }, }, }, {"__userdata", "matrixn", {{0.99997350014971, -0.00024489799579049, -0.007075577260995, -0.0016956502402841, }, {0.99751747640332, -0.066396281973242, -0.016311300089651, 0.01686296235763, }, {0.98373771689059, 0.17961098064117, 2.168404344971e-18, -4.336808689942e-19, }, {0.9923057722943, -0.121064449408, 0.012263000079293, -0.022853275257668, }, {0.9970880466403, -0.07187566175875, -0.020298258444437, -0.01540445383319, }, {0.97933987986791, 0.20222116531241, 8.673617379884e-19, 2.6020852139652e-18, }, {0.98983462830921, -0.13129462972914, 0.051380419554061, 0.018686393196682, }, {1, -2.4089617019912e-18, 1.1865882858365e-15, -1.4781614314182e-16, }, {1, -1.675431169669e-18, 6.0195005201558e-16, -8.0885293715681e-18, }, {1, -1.1658561486008e-17, 1.1865206820194e-15, 5.4137687307285e-17, }, {0.99878408802742, 0.014682353390116, 0.033319581638563, 0.033235214480882, }, {0.72818623632013, 0.07547823001027, -0.014559335439599, -0.68105496678315, }, {1, 1.6306400674182e-16, 3.2612801348364e-16, -2.5944524306709e-14, }, {1, 1.2490009027033e-16, 2.1510571102112e-16, -8.0905334515213e-15, }, {1, -2.1412992906589e-18, 1.1865888152321e-15, -1.4825172983495e-16, }, {0.72382135741023, 0.018759018145713, -0.068331915495867, 0.68633919538351, }, {0.99999500000567, 0.00028551282995374, -0.00099999533334722, 0.0029863783122787, }, {1, 2.0816681711722e-16, 1.7364581994528e-15, -1.5853637846952e-14, }, {1, -2.1412992906589e-18, 1.1865888152321e-15, -1.4825172983495e-16, }, {1, -7.4369492768928e-19, 5.967473638953e-16, -7.6624717990992e-18, }, }, }, }, 
			--{"__userdata", "Pose", {"__userdata", "matrixn", {{-0.0078004963459822, 0.90769010423738, 9.3795171733859e-05, }, }, }, {"__userdata", "matrixn", {{0.99995950027337, 0.0089992279574284, 0, -0.00010820902854154, }, {1, 3.75199235426e-19, -1.0578246730467e-22, -3.6715531079582e-21, }, {1, -1.359454018381e-18, -1.0578246730467e-22, -3.674352180196e-21, }, {0.96206743271693, -0.2491381631361, -0.070828978544049, 0.08567196958769, }, {0.9719733278534, -2.4164654314683e-18, 0.23509115241023, 1.1422018398947e-17, }, {0.95118705945468, -0.28300138501061, 0.053576984139254, -0.11082824900974, }, {0.9731908994082, -4.4641517007734e-17, -0.22999885501684, -1.3270858489323e-17, }, {0.99945065850584, -0.001556928433145, 0.011425377083753, -0.031071175461983, }, {0.99327098297449, 0.11581344645977, 1.8305296280022e-19, 2.1564311405351e-19, }, {0.97601905921572, -0.12843709722558, -0.15963036800316, 0.073544909511816, }, {0.999480619111, -0.014937367712007, -0.00375308582727, 0.028306914598766, }, {0.99677689337104, 0.080223592798942, -9.8031058412493e-20, -8.871833472216e-20, }, {0.99094157499273, -0.086509874578801, 0.072224921539798, -0.073036958177751, }, }, }, }, 
		},
		bindposes_map={
			{['R']={['RightKnee']=5, ['Chest2']=9, ['Neck']=18, ['Chest1']=8, ['RightAnkle']=6, ['RightShoulder']=15, ['LeftElbow']=12, ['LeftCollar']=10, ['LeftShoulder']=11, ['Head']=19, ['RightElbow']=16, ['LeftHip']=1, ['Chest']=7, ['LeftKnee']=2, ['RightHip']=4, ['RightWrist']=17, ['LeftWrist']=13, ['Hips']=0, ['RightCollar']=14, ['LeftAnkle']=3, }, ['T']={['Hips']=0, }, },
			{['R']={['left_ankle']=12, ['left_shoulder']=5, ['right_hip']=7, ['root']=0, ['left_hip']=10, ['right_elbow']=4, ['left_elbow']=6, ['right_ankle']=9, ['right_shoulder']=3, ['neck']=2, ['chest']=1, ['right_knee']=8, ['left_knee']=11, }, ['T']={['root']=0, }, }, 
		}
	},
}

function Mujoco.createAngleRetargetFromLafan(mLafanLoader, mTargetLoader, _options)
	if not _options then
		_options={}
	end
	-- a retargetConfig generated from correspondenceTools_GUI.lua (which can also be used for models with fingers)
	local retargetConfig=Mujoco.lafan.retargetConfig
	
	local RET=require("retargetting/module/retarget_common")

	if true then
		-- further adjust bindPoseB manually.
		local pose2=Pose.fromTable(retargetConfig.bindposes_quat[2])

		local pose2_new, posedof2= RET.decodeBindPose(mTargetLoader, pose2, retargetConfig.bindposes_map[2])

		--pose2.translations(0).y=pose2.translations(0).y-0.05
		mTargetLoader:setPose(pose2_new)

		local ankle=11
		mTargetLoader:getBoneByName('left_ankle'):getLocalFrame().rotation:leftMult(quater(math.rad(ankle), vector3(1,0,0)))
		mTargetLoader:getBoneByName('right_ankle'):getLocalFrame().rotation:leftMult(quater(math.rad(ankle), vector3(1,0,0)))
		mTargetLoader:fkSolver():forwardKinematics()
		
		-- update bindpose
		retargetConfig.bindposes_quat[2]=mTargetLoader:pose()
		retargetConfig.bindposes_map[2]=RET.generateBindPoseMap(mTargetLoader)

		if _options.debugDraw then
			assert(_options.debugSkin)
			_options.debugSkin:setPose(mTargetLoader:pose())
		end
	end
	--RET.saveRetargetInfo2(retargetConfig) -- you can see the manual adjustment using correspondenceTools_GUI.lua

	retargetConfig.A.mot={
		loader=mLafanLoader
	}
	retargetConfig.B.mot={
		loader=mTargetLoader
	}

	local ret=RET.AngleRetarget(retargetConfig, { heightAdjust=-0.04})
	--local ret=RET.AngleRetarget(retargetConfig)
	return ret
end
function Mujoco.createAngleRetargetFromHanyang(mSourceLoader, mTargetLoader, _options)
	if not _options then
		_options={}
	end
	-- a retargetConfig generated from correspondenceTools_GUI.lua (which can also be used for models with fingers)
	local retargetConfig=Mujoco.hanyang.retargetConfig
	
	local RET=require("retargetting/module/retarget_common")

	if true then
		-- further adjust bindPoseB manually.
		local pose2=Pose.fromTable(retargetConfig.bindposes_quat[2])

		local pose2_new, posedof2= RET.decodeBindPose(mTargetLoader, pose2, retargetConfig.bindposes_map[2])

		--pose2.translations(0).y=pose2.translations(0).y-0.05
		mTargetLoader:setPose(pose2_new)

		local ankle=11
		mTargetLoader:getBoneByName('left_ankle'):getLocalFrame().rotation:leftMult(quater(math.rad(ankle), vector3(1,0,0)))
		mTargetLoader:getBoneByName('right_ankle'):getLocalFrame().rotation:leftMult(quater(math.rad(ankle), vector3(1,0,0)))
		mTargetLoader:fkSolver():forwardKinematics()
		
		-- update bindpose
		retargetConfig.bindposes_quat[2]=mTargetLoader:pose()
		retargetConfig.bindposes_map[2]=RET.generateBindPoseMap(mTargetLoader)

		if _options.debugDraw then
			assert(_options.debugSkin)
			_options.debugSkin:setPose(mTargetLoader:pose())
		end
	end
	--RET.saveRetargetInfo2(retargetConfig) -- you can see the manual adjustment using correspondenceTools_GUI.lua

	retargetConfig.A.mot={
		loader=mSourceLoader
	}
	retargetConfig.B.mot={
		loader=mTargetLoader
	}

	local ret=RET.AngleRetarget(retargetConfig, { heightAdjust=-0.04})
	--local ret=RET.AngleRetarget(retargetConfig)
	return ret
end


return Mujoco
