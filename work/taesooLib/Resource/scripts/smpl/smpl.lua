-- written by Taesoo. 
local smpl={}
local FBXloader=require("FBXloader")

smpl={
	vertex_ids = {
		smplh= {
			nose=		    332,
			reye=		    6260,
			leye=		    2800,
			rear=		    4071,
			lear=		    583,
			rthumb=		6191,
			rindex=		5782,
			rmiddle=		5905,
			rring=		6016,
			rpinky=		6133,
			lthumb=		2746,
			lindex=		2319,
			lmiddle=		2445,
			lring=		2556,
			lpinky=		2673,
			LBigToe=		3216,
			LSmallToe=	3226,
			LHeel=		3387,
			RBigToe=		6617,
			RSmallToe=    6624,
			RHeel=		6787
		},
		smplx= {
			nose=		    9120,
			reye=		    9929,
			leye=		    9448,
			rear=		    616,
			lear=		    6,
			rthumb=		8079,
			rindex=		7669,
			rmiddle=		7794,
			rring=		7905,
			rpinky=		8022,
			lthumb=		5361,
			lindex=		4933,
			lmiddle=		5058,
			lring=		5169,
			lpinky=		5286,
			LBigToe=		5770,
			LSmallToe=    5780,
			LHeel=		8846,
			RBigToe=		8463,
			RSmallToe= 	8474,
			RHeel=  		8635
		},
		mano= {
			thumb=		744,
			index=		320,
			middle=		443,
			ring=		    554,
			pinky=	671,
		}
	},
	SMPLX_JOINT_NAMES = {
		"pelvis",
		"left_hip",
		"right_hip",
		"spine1",
		"left_knee",
		"right_knee",
		"spine2",
		"left_ankle",
		"right_ankle",
		"spine3",
		"left_foot",
		"right_foot",
		"neck",
		"left_collar",
		"right_collar",
		"head",
		"left_shoulder",
		"right_shoulder",
		"left_elbow",
		"right_elbow",
		"left_wrist",
		"right_wrist",
		"jaw",
		"left_eye_smplhf",
		"right_eye_smplhf",
		"left_index1",
		"left_index2",
		"left_index3",
		"left_middle1",
		"left_middle2",
		"left_middle3",
		"left_pinky1",
		"left_pinky2",
		"left_pinky3",
		"left_ring1",
		"left_ring2",
		"left_ring3",
		"left_thumb1",
		"left_thumb2",
		"left_thumb3",
		"right_index1",
		"right_index2",
		"right_index3",
		"right_middle1",
		"right_middle2",
		"right_middle3",
		"right_pinky1",
		"right_pinky2",
		"right_pinky3",
		"right_ring1",
		"right_ring2",
		"right_ring3",
		"right_thumb1",
		"right_thumb2",
		"right_thumb3",
	},
	SMPLX_ADDITIONAL_NAMES={
		"nose",
		"right_eye",
		"left_eye",
		"right_ear",
		"left_ear",
		"left_big_toe",
		"left_small_toe",
		"left_heel",
		"right_big_toe",
		"right_small_toe",
		"right_heel",
		"left_thumb",
		"left_index",
		"left_middle",
		"left_ring",
		"left_pinky",
		"right_thumb",
		"right_index",
		"right_middle",
		"right_ring",
		"right_pinky",
		"right_eye_brow1",
		"right_eye_brow2",
		"right_eye_brow3",
		"right_eye_brow4",
		"right_eye_brow5",
		"left_eye_brow5",
		"left_eye_brow4",
		"left_eye_brow3",
		"left_eye_brow2",
		"left_eye_brow1",
		"nose1",
		"nose2",
		"nose3",
		"nose4",
		"right_nose_2",
		"right_nose_1",
		"nose_middle",
		"left_nose_1",
		"left_nose_2",
		"right_eye1",
		"right_eye2",
		"right_eye3",
		"right_eye4",
		"right_eye5",
		"right_eye6",
		"left_eye4",
		"left_eye3",
		"left_eye2",
		"left_eye1",
		"left_eye6",
		"left_eye5",
		"right_mouth_1",
		"right_mouth_2",
		"right_mouth_3",
		"mouth_top",
		"left_mouth_3",
		"left_mouth_2",
		"left_mouth_1",
		"left_mouth_5",  --# 59 in OpenPose output
		"left_mouth_4",  --# 58 in OpenPose output
		"mouth_bottom",
		"right_mouth_4",
		"right_mouth_5",
		"right_lip_1",
		"right_lip_2",
		"lip_top",
		"left_lip_2",
		"left_lip_1",
		"left_lip_3",
		"lip_bottom",
		"right_lip_3",
		--# Face contour
		"right_contour_1",
		"right_contour_2",
		"right_contour_3",
		"right_contour_4",
		"right_contour_5",
		"right_contour_6",
		"right_contour_7",
		"right_contour_8",
		"contour_middle",
		"left_contour_8",
		"left_contour_7",
		"left_contour_6",
		"left_contour_5",
		"left_contour_4",
		"left_contour_3",
		"left_contour_2",
		"left_contour_1",
	},
	SMPLX_LOCKEDHEAD_JOINT_NAMES = {
		"pelvis",
		"left_hip",
		"right_hip",
		"spine1",
		"left_knee",
		"right_knee",
		"spine2",
		"left_ankle",
		"right_ankle",
		"spine3",
		"left_foot",
		"right_foot",
		"neck",
		"left_collar",
		"right_collar",
		"head",
		"left_shoulder",
		"right_shoulder",
		"left_elbow",
		"right_elbow",
		"left_wrist",
		"right_wrist",
		"nose",
		"right_eye",
		"left_eye",
		"left_index1",
		"left_index2",
		"left_index3",
		"left_middle1",
		"left_middle2",
		"left_middle3",
		"left_pinky1",
		"left_pinky2",
		"left_pinky3",
		"left_ring1",
		"left_ring2",
		"left_ring3",
		"left_thumb1",
		"left_thumb2",
		"left_thumb3",
		"right_index1",
		"right_index2",
		"right_index3",
		"right_middle1",
		"right_middle2",
		"right_middle3",
		"right_pinky1",
		"right_pinky2",
		"right_pinky3",
		"right_ring1",
		"right_ring2",
		"right_ring3",
		"right_thumb1",
		"right_thumb2",
		"right_thumb3",
	},
	SMPLH_JOINT_NAMES = {
		"pelvis",
		"left_hip",
		"right_hip",
		"spine1",
		"left_knee",
		"right_knee",
		"spine2",
		"left_ankle",
		"right_ankle",
		"spine3",
		"left_foot",
		"right_foot",
		"neck",
		"left_collar",
		"right_collar",
		"head",
		"left_shoulder",
		"right_shoulder",
		"left_elbow",
		"right_elbow",
		"left_wrist",
		"right_wrist",
		"left_index1",
		"left_index2",
		"left_index3",
		"left_middle1",
		"left_middle2",
		"left_middle3",
		"left_pinky1",
		"left_pinky2",
		"left_pinky3",
		"left_ring1",
		"left_ring2",
		"left_ring3",
		"left_thumb1",
		"left_thumb2",
		"left_thumb3",
		"right_index1",
		"right_index2",
		"right_index3",
		"right_middle1",
		"right_middle2",
		"right_middle3",
		"right_pinky1",
		"right_pinky2",
		"right_pinky3",
		"right_ring1",
		"right_ring2",
		"right_ring3",
		"right_thumb1",
		"right_thumb2",
		"right_thumb3",
		"nose",
		"right_eye",
		"left_eye",
		"right_ear",
		"left_ear",
		"left_big_toe",
		"left_small_toe",
		"left_heel",
		"right_big_toe",
		"right_small_toe",
		"right_heel",
		"left_thumb",
		"left_index",
		"left_middle",
		"left_ring",
		"left_pinky",
		"right_thumb",
		"right_index",
		"right_middle",
		"right_ring",
		"right_pinky",
	},
	SMPLH_LOCKEDHEAD_JOINT_NAMES = {
		"pelvis",
		"left_hip",
		"right_hip",
		"spine1",
		"left_knee",
		"right_knee",
		"spine2",
		"left_ankle",
		"right_ankle",
		"spine3",
		"left_foot",
		"right_foot",
		"neck",
		"left_collar",
		"right_collar",
		"head",
		"left_shoulder",
		"right_shoulder",
		"left_elbow",
		"right_elbow",
		"left_wrist",
		"right_wrist",
		"left_index1",
		"left_index2",
		"left_index3",
		"left_middle1",
		"left_middle2",
		"left_middle3",
		"left_pinky1",
		"left_pinky2",
		"left_pinky3",
		"left_ring1",
		"left_ring2",
		"left_ring3",
		"left_thumb1",
		"left_thumb2",
		"left_thumb3",
		"right_index1",
		"right_index2",
		"right_index3",
		"right_middle1",
		"right_middle2",
		"right_middle3",
		"right_pinky1",
		"right_pinky2",
		"right_pinky3",
		"right_ring1",
		"right_ring2",
		"right_ring3",
		"right_thumb1",
		"right_thumb2",
		"right_thumb3",
	},
	SMPL_JOINT_NAMES = {
		"pelvis",
		"left_hip",
		"right_hip",
		"spine1",
		"left_knee",
		"right_knee",
		"spine2",
		"left_ankle",
		"right_ankle",
		"spine3",
		"left_foot",
		"right_foot",
		"neck",
		"left_collar",
		"right_collar",
		"head",
		"left_shoulder",
		"right_shoulder",
		"left_elbow",
		"right_elbow",
		"left_wrist",
		"right_wrist",
		"left_hand",
		"right_hand",
	},
}


smpl.taesooLib={
	voca={
		left_hip='left_hip',
		right_hip='right_hip',
		left_knee='left_knee',
		right_knee='right_knee',
		left_ankle='left_ankle',
		right_ankle='right_ankle',
		left_toes='left_foot',
		right_toes='right_foot',
		left_shoulder='left_shoulder',
		right_shoulder='right_shoulder',
		head='head',
	},
	SMPLHtoMixamo={ -- retarget config
		boneCorrespondences={
			head ="Head",
			left_ankle ="LeftFoot",
			left_collar ="LeftShoulder",
			left_elbow ="LeftForeArm",
			left_foot ="LeftToeBase",
			left_hip ="LeftUpLeg",
			left_index1 ="LeftHandIndex1",
			left_index2 ="LeftHandIndex2",
			left_index3 ="LeftHandIndex3",
			left_knee ="LeftLeg",
			left_middle1 ="LeftHandMiddle1",
			left_middle2 ="LeftHandMiddle2",
			left_middle3 ="LeftHandMiddle3",
			left_pinky1 ="LeftHandPinky1",
			left_pinky2 ="LeftHandPinky2",
			left_pinky3 ="LeftHandPinky3",
			left_ring1 ="LeftHandRing1",
			left_ring2 ="LeftHandRing2",
			left_ring3 ="LeftHandRing3",
			left_shoulder ="LeftArm",
			left_thumb1 ="LeftHandThumb1",
			left_thumb2 ="LeftHandThumb2",
			left_thumb3 ="LeftHandThumb3",
			left_wrist ="LeftHand",
			neck ="Neck",
			pelvis ="Hips",
			right_ankle ="RightFoot",
			right_collar ="RightShoulder",
			right_elbow ="RightForeArm",
			right_foot ="RightToeBase",
			right_hip ="RightUpLeg",
			right_index1 ="RightHandIndex1",
			right_index2 ="RightHandIndex2",
			right_index3 ="RightHandIndex3",
			right_knee ="RightLeg",
			right_middle1 ="RightHandMiddle1",
			right_middle2 ="RightHandMiddle2",
			right_middle3 ="RightHandMiddle3",
			right_pinky1 ="RightHandPinky1",
			right_pinky2 ="RightHandPinky2",
			right_pinky3 ="RightHandPinky3",
			right_ring1 ="RightHandRing1",
			right_ring2 ="RightHandRing2",
			right_ring3 ="RightHandRing3",
			right_shoulder ="RightArm",
			right_thumb1 ="RightHandThumb1",
			right_thumb2 ="RightHandThumb2",
			right_thumb3 ="RightHandThumb3",
			right_wrist ="RightHand",
			spine1 ="Spine",
			spine2 ="Spine1",
			spine3 ="Spine2",
		},
		bindposes_quat={{"__userdata", "Pose", {"__userdata", "matrixn", {{0, 0.94116611955176, 0, }, }, }, {"__userdata", "matrixn", {{1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, }, }, }, {"__userdata", "Pose", {"__userdata", "matrixn", {{0, 0.97, 0, }, }, }, {"__userdata", "matrixn", {{1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {0.99800270462893, -6.8941265576878e-09, -0.057580167227482, 0.02598318485873, }, {1, -7.9057948662488e-19, 3.6603997520169e-18, 1.3616673170182e-18, }, {1, -8.2712535810165e-19, 3.6474827817137e-18, 1.363755019322e-18, }, {1, -6.4683079508131e-19, 3.628920390618e-18, 1.3533320856133e-18, }, {0.99972735877713, 0.016167311577625, 0.013199285955669, 0.010469240617833, }, {0.99849339844475, -4.3466739796636e-08, 0.0094631067677645, -0.054049818432176, }, {1, 6.3725376523702e-18, 9.8596858660463e-18, 7.2967296050981e-18, }, {1, 6.7374264181075e-18, -1.1640551256801e-16, -6.7191025735008e-17, }, {0.99480059018807, 3.1676600287366e-05, 0.098503057804605, 0.025863726746334, }, {0.99631712253306, 1.966853131603e-08, 0.062153682424797, -0.059068698212147, }, {1, -2.5468133344787e-18, -1.6080393892699e-17, 2.3998278758417e-17, }, {1, 2.0692306633465e-19, -1.565501990754e-17, 5.1984921273446e-17, }, {0.97225494196646, 0.0051721016871432, 0.23248481715262, -0.025384778499112, }, {0.99931978354094, -4.5517793349922e-09, -0.017914459403066, -0.032234180119638, }, {1, 5.363481027267e-18, -1.1095684048293e-16, -1.595552225905e-18, }, {1, 1.3246065421371e-17, -1.3545376443542e-16, -6.4099690147556e-17, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {0.9991034658447, -0.016424907164384, -0.010113720653089, 0.037685535902971, }, {0.94432155037751, -0.20545059892808, 0.21488390469218, -0.14096726001788, }, {0.97963629396094, 0.11257373054042, -0.16318239350028, 0.031802408685574, }, {1, 4.2636107262473e-17, 1.2983606146798e-17, -1.1478433820908e-16, }, {0.99689674282835, -2.6475388073219e-08, 0.024455298127912, -0.074825280030905, }, {0.99595685846253, 1.6176353465143e-08, -0.026561426133849, 0.085816238109068, }, {1, 1.7336472306509e-17, -8.864386673454e-18, 5.3480237155134e-17, }, {1, 1.0838790132353e-17, 1.474431009634e-17, 5.3580207551547e-18, }, {0.99679336750589, 0.0015412257013155, -0.050475810016215, -0.062070924939216, }, {0.99805584317614, 4.8284303290926e-09, -0.016821940392093, 0.060012967127209, }, {1, 1.4244469683789e-17, 5.2594268146968e-18, 1.3646177118533e-17, }, {1, 6.6651213026524e-18, 5.6527604435828e-18, 7.2544177139994e-18, }, {0.99241056346742, 0.0030384788220546, -0.10668266084907, -0.061080692848816, }, {0.99532922619989, 5.2499777612705e-08, -0.067985126427896, 0.068540163823154, }, {1, -1.6872957918152e-17, 4.6705672623752e-17, 5.0587906286524e-17, }, {1, -2.4502336390688e-17, 1.9292254826692e-17, -6.2344580821413e-18, }, {0.96634118740809, 0.0091418552462446, -0.25708694003065, -0.0027278685939096, }, {0.99941808890826, -7.4320445132687e-09, 0.020895350059551, 0.02696048791924, }, {1, -1.7338829732779e-17, 3.9587786727464e-17, 6.9504264567796e-18, }, {1, -1.209265839191e-17, 3.8312078601461e-17, 5.6737831790333e-18, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, {1, 0, 0, 0, }, }, }, }, },
		bindposes_map={{['R']={['left_wrist']=17, ['left_ankle']=3, ['left_collar']=14, ['right_hip']=5, ['neck']=12, ['right_middle2']=41, ['right_ring2']=47, ['right_middle3']=42, ['left_ring2']=28, ['right_thumb1']=49, ['right_index3']=39, ['right_middle1']=40, ['right_thumb3']=51, ['left_middle2']=22, ['left_foot']=4, ['left_knee']=2, ['left_index2']=19, ['right_index1']=37, ['pelvis']=0, ['left_middle1']=21, ['left_shoulder']=15, ['right_thumb2']=50, ['right_foot']=8, ['left_ring1']=27, ['left_thumb2']=31, ['left_thumb1']=30, ['right_shoulder']=34, ['left_elbow']=16, ['left_pinky1']=24, ['left_hip']=1, ['right_index2']=38, ['left_pinky3']=26, ['right_collar']=33, ['right_wrist']=36, ['right_pinky2']=44, ['left_middle3']=23, ['left_index3']=20, ['spine2']=10, ['right_pinky3']=45, ['head']=13, ['left_ring3']=29, ['left_pinky2']=25, ['left_index1']=18, ['left_thumb3']=32, ['right_elbow']=35, ['right_knee']=6, ['right_ankle']=7, ['right_ring1']=46, ['spine3']=11, ['right_pinky1']=43, ['spine1']=9, ['right_ring3']=48, }, ['T']={['pelvis']=0, }, }, {['R']={['LeftHandIndex3']=15, ['LeftFoot']=54, ['LeftForeArm']=8, ['LeftToeBase']=55, ['LeftHandPinky4']=28, ['LeftHandPinky1']=25, ['RightHandMiddle1']=40, ['RightShoulder']=29, ['LeftHandRing1']=21, ['RightHandIndex4']=39, ['RightFoot']=58, ['RightHandRing4']=47, ['RightHandIndex2']=37, ['LeftHandIndex4']=16, ['RightUpLeg']=56, ['Head']=5, ['RightHandRing2']=45, ['LeftHandThumb2']=11, ['RightHandPinky1']=48, ['LeftHandIndex2']=14, ['LeftHandMiddle2']=18, ['RightHandThumb1']=33, ['LeftHand']=9, ['RightLeg']=57, ['RightHandRing1']=44, ['LeftHandRing3']=23, ['LeftHandRing4']=24, ['LeftLeg']=53, ['LeftHandThumb3']=12, ['LeftShoulder']=6, ['LeftHandPinky3']=27, ['Hips']=0, ['LeftHandMiddle3']=19, ['RightHand']=32, ['Spine']=1, ['Spine2']=3, ['RightHandIndex3']=38, ['Spine1']=2, ['LeftHandMiddle4']=20, ['RightToeBase']=59, ['LeftArm']=7, ['RightHandPinky4']=51, ['RightHandMiddle3']=42, ['RightHandMiddle2']=41, ['RightArm']=30, ['RightForeArm']=31, ['RightHandThumb2']=34, ['LeftHandMiddle1']=17, ['LeftUpLeg']=52, ['LeftHandRing2']=22, ['RightHandThumb3']=35, ['RightHandIndex1']=36, ['LeftHandIndex1']=13, ['RightHandRing3']=46, ['Neck']=4, ['LeftHandPinky2']=26, ['RightHandPinky3']=50, ['LeftHandThumb1']=10, ['RightHandPinky2']=49, ['RightHandMiddle4']=43, }, ['T']={['Hips']=0, }, }, },
	},
}
function smpl.taesooLib.createAngleRetargetToMixamo(mSmplLoader, mMixamoLoader,_bonename_prefix)
	-- a retargetConfig generated from correspondenceTools_GUI.lua (which can also be used for models with fingers)
	local retargetConfig={
		A={
			boneCorrespondences=smpl.taesooLib.SMPLHtoMixamo.boneCorrespondences,
			reusePreviousBinding=true,
			skinScale=100
		},
		B={ 
			--skel =(taesooLib_PATH.."Resource/motion/Mixamo/passive_marker_man_T.fbx.dat").string,
			skinScale=98
		},
		bindposes_quat=smpl.taesooLib.SMPLHtoMixamo.bindposes_quat,
		bindposes_map=smpl.taesooLib.SMPLHtoMixamo.bindposes_map
	}
	if _bonename_prefix then
		local prev= retargetConfig.A.boneCorrespondences
		retargetConfig.A.boneCorrespondences={}
		for k, v in pairs(prev) do
			retargetConfig.A.boneCorrespondences[k]=_bonename_prefix..v
		end
	end

	if true then
		-- further adjust bindPoseB manually.
		local pose2=Pose.fromTable(retargetConfig.bindposes_quat[2])
		--pose2.translations(0).y=pose2.translations(0).y-0.05
		mMixamoLoader.loader:setPose(pose2)

		local ankle=5
		mMixamoLoader.loader:getBoneByVoca(MotionLoader.LEFTANKLE):getLocalFrame().rotation:leftMult(quater(math.rad(ankle), vector3(1,0,0)))
		mMixamoLoader.loader:getBoneByVoca(MotionLoader.RIGHTANKLE):getLocalFrame().rotation:leftMult(quater(math.rad(ankle), vector3(1,0,0)))
		mMixamoLoader.loader:fkSolver():forwardKinematics()
		
		-- update bindpose
		retargetConfig.bindposes_quat[2]=mMixamoLoader.loader:pose()
	end

	local RET=require("retargetting/module/retarget_common")
	retargetConfig.A.mot={
		loader=mSmplLoader.loader
	}
	retargetConfig.B.mot={
		loader=mMixamoLoader.loader
	}

	local ret=RET.AngleRetarget(retargetConfig, { heightAdjust=-0.04})
	return ret
end
function smpl.flattenShapeDirs(dd)
	if dd.shapedirs_flat then
		-- all members are beta-independent.
		return 
	end
	-- so that rows appeaer in the following order
	-- x
	-- y
	-- z
	-- x
	-- y
	-- z
	assert(dd.shapedirs:rows()==3)
	dd.shapedirs_flat=dd.shapedirs:flattenAlongAxis(1)
	dd.JS_flat=dd.J_regressor:dotProduct(dd.shapedirs):flattenAlongAxis(1)
	dd.J_v_template_flat=(dd.J_regressor*dd.v_template):toVector()
end

-- input:
-- 		smpl_PATH=Path(scriptPath).parent.parent.parent..'sample_SAMP'
-- 		npz_motion='../airplane1.npz'
-- returns { fbxLoader=..., motion=..., dd=... }
function smpl.initAuto(smpl_PATH, npz_motion)
	local zfile=util.NPZfile(npz_motion)
	zfile=zfile:toTable()
	assert(zfile.gender)

	-- body model
	if zfile.poses:cols()==52*3 then -- smpl-h
		print('SMPL-H')
		bm_path=smpl_PATH:join('SMPLH/'..zfile.gender(0)..'/model.npz').string
	else
		print('SMPL-X')
		bm_path=smpl_PATH:join('SMPL-X/models_lockedhead/smplx/SMPLX_'..string.upper(zfile.gender(0))..'.npz').string
	end

	local dd=smpl.init(bm_path) -- dd contains a shape (mesh)
	if zfile.betas then
		dd.betas:range(0,zfile.betas:size()):assign(zfile.betas) -- change bodyshape (first 16 components)
	else
		print("warning! "..bm_path.." doesn't contain shape parameters.")
	end
	--mLoader=smpl.createSkeleton(dd) -- surface가 안필요한 경우. 
	local mSMPLloader=smpl.createFBXskeleton(dd) 

	if zfile.mocap_frame_rate then
		zfile.mocap_framerate=zfile.mocap_frame_rate
	end
	local mSourceMotion=smpl.loadMotion(dd, mSMPLloader.loader, zfile.mocap_framerate(0), zfile.trans, zfile.poses, true)
	return 
	{
		fbxLoader=mSMPLloader,
		motion=mSourceMotion,
		dd=dd,
	}
end

function smpl.init(bm_path)
	local dd=util.NPZfile(bm_path)
	dd=dd:toTable()
	if not dd.shapedirs then
		error('loading'..bm_path)
	end
	-- set shape parameter
	dd.betas = CT.zeros(dd.shapedirs:cols()) 
	if dd.kintree_table:cols()==55 then
		--dd.joint_names=smpl.SMPLX_LOCKEDHEAD_JOINT_NAMES 
		dd.joint_names=smpl.SMPLX_JOINT_NAMES 
	elseif dd.kintree_table:cols()==52 then
		dd.joint_names=smpl.SMPLH_LOCKEDHEAD_JOINT_NAMES 
	elseif dd.kintree_table:cols()==24 then
		dd.joint_names=smpl.SMPL_JOINT_NAMES
	else
		-- not implemented yet
		assert(false)
	end
	return dd
end

function smpl.createMesh(dd)
	local mesh=Mesh()

	--local vertices=dd.v_template

	mesh:resize(dd.v_template:rows(), dd.f:rows())
	smpl.updateShape(dd, mesh)
	for i=0, mesh:numFace()-1 do
		mesh:getFace(i):setIndex(dd.f(i,0), dd.f(i,1), dd.f(i,2))
	end
	mesh:calculateVertexNormal()
	return mesh
end

function smpl.updateShape(dd, mesh)

	-- vertices
	-- (6890,3)=(6890, 3, 16)*(16,1) +(6890,3)
	dd.v_shaped=dd.shapedirs:dotProduct(dd.betas)+dd.v_template

	-- joint locations
	-- (52,3)=(52, 6890)*(6890,3)
	dd.J=dd.J_regressor*dd.v_shaped

	-- beta fitting!
	-- assuming the same T-pose, 
	-- minimize (J_regressor*(dd.shapedirs:dotProduct(dd.betas)+dd.v_template)
	--  -  J_target )^2
	--  -> this is a very simple least-square minization problem.

	if mesh then
		local vertices=dd.v_shaped
		for i=0, vertices:rows()-1 do
			mesh:getVertex(i):assign(vertices:row(i):toVector3())
		end
	end

	--dd['J'] = dd['J_regressor']@v_shaped  # 24 by 3
end

-- this function assumes that smpl.updateShape is already done.
-- LBSvertices=vector3N(dd.v_shaped:rows())
function smpl.performLBS(dd, root_trans, smpl_pose, LBS_vertices, _convertYUP)
	local global_tf=smpl._forwardKinematics(dd, root_trans,smpl_pose, _convertYUP)

	for i=0, LBS_vertices :size()-1 do
		local v=smpl._computeLBS(dd, global_tf, i)
		LBS_vertices(i):assign(v)
	end
end
function smpl._calcFKmatrix(dd, smpl_pose, _convertYUP)
	local global_q={}
	local nj=dd.kintree_table:cols()
	for i=0, dd.kintree_table:cols()-1 do
		local pid=dd.kintree_table(0,i)
		assert(dd.kintree_table(1,i)==i and i>=pid)
		local q=quater()
		q:setRotation(smpl_pose:toVector3(i*3))
		if pid==-1 then
			assert(i==0)
			global_q[i+1]=q
			if _convertYUP then
				global_q[i+1]:leftMult(quater(math.rad(-90), vector3(1,0,0)))
			end
		else
			global_q[i+1]=global_q[pid+1]*q
		end
	end
	-- calcT*calcOffset*dd.J +root_trans-> joint global positions
	local calcT=CT.zeros(nj*3, nj*3)
	local calcOffset=CT.eye(nj*3, nj*3)
	local qM=matrixn(3,3)
	for i=0, dd.kintree_table:cols()-1 do
		local pid=dd.kintree_table(0,i)

		if pid==-1 then
			assert(i==0)
			--calcT:sub(i*3,i*3+3,i*3,i*3+1):assign33(global_q[i+1]:M())
			calcT:sub(i*3,i*3+3,i*3,i*3+1):setAllValue(0)
		else
			local q=quater()
			q:setRotation(smpl_pose:toVector3(pid*3))
			qM:assign33(q:M())
			calcOffset:sub(i*3, i*3+3, pid*3,pid*3+3):assign(-CT.eye(3))
			-- diagonal
			calcT:sub(i*3,i*3+3,i*3,i*3+1):assign33(global_q[pid+1]:M())
			calcT:sub(i*3,i*3+3, 0,0):radd(calcT:sub(pid*3,pid*3+3,0,0))
		end
	end
	return global_q, calcT*calcOffset
end

-- fksolver : loader:fkSolver() or skin:getState()
function smpl.convertToSMPLpose(dd, fksolver_or_pose)

	if dbg.lunaType(fksolver_or_pose)=='Pose' then
		print('not implemented yet')
		dbg.console()
	else
		local fksolver=fksolver_or_pose
		local nj=dd.J:rows()
		local smplPose=vectorn(nj*3)
		for i=0, nj-1 do
			smplPose:setVec3(i*3, fksolver:localFrame(dd.tree_indices(i)).rotation:rotationVector())
		end
		return smplPose
	end
end
-- slow. use performLBS instead. This function is only for educational purposes.
function smpl.performLBS_usingMatrices(dd, root_trans, smpl_pose, LBS_vertices, _convertYUP)

	local global_q, FKmat=smpl._calcFKmatrix(dd, smpl_pose, _convertYUP)

	local rootpos=root_trans:toVector3(0)
	if _convertYUP then
		rootpos=quater(math.rad(-90), vector3(1,0,0))*rootpos
	end
	
	-- 이제 아래 수식은 모두 행렬 곱으로 표현 가능하다.
	dd.v_shaped=dd.shapedirs:dotProduct(dd.betas)+dd.v_template
	dd.J=dd.J_regressor*dd.v_shaped

	local nj=dd.J:rows()
	local J=vectorn(nj*3)
	for i=0, nj-1 do
		J:setVec3(i*3, dd.J:row(i):toVector3())
	end
	local gJ=FKmat*J:column()
	
	-- removed pid dependency so that it can be written in a matrix form.
	local global_tf={}
	for i=0, dd.kintree_table:cols()-1 do
		global_tf[i+1]=transf(global_q[i+1], gJ:column(0):toVector3(i*3)+rootpos)
	end

	for i=0, LBS_vertices :size()-1 do
		local v=smpl._computeLBS(dd, global_tf, i)
		LBS_vertices(i):assign(v)
	end
end

function smpl._forwardKinematics(dd, root_trans, smpl_pose, _convertYUP)
	local global_tf={}
	if dbg.lunaType(root_trans)~='vector3' then
		root_trans=root_trans:toVector3()
	end
	for i=0, dd.kintree_table:cols()-1 do
		local pid=dd.kintree_table(0,i)
		assert(dd.kintree_table(1,i)==i and i>=pid)
		local q=quater()
		q:setRotation(smpl_pose:toVector3(i*3))
		if pid==-1 then
			assert(i==0)
			global_tf[i+1]=transf( q, root_trans+dd.J:row(0):toVector3())
			if _convertYUP then
				global_tf[i+1]:leftMult(transf(quater(math.rad(-90), vector3(1,0,0))))
			end
		else
			global_tf[i+1]=global_tf[pid+1]*transf( q, dd.J:row(i):toVector3()-dd.J:row(pid):toVector3())
		end
	end
	return global_tf
end
-- global_tf  : matrixn(#dd.joint_names,  7)
function smpl._computeLBS(dd, global_tf, vertex_index)
	local weights=dd.weights:row(vertex_index)
	local pos=vector3(0,0,0)
	local vertices=dd.v_shaped
	for j=0, weights:size()-1 do
		local wj=weights(j)
		if wj>1e-3 then
			local local_vertexpos=vertices:row(vertex_index):toVector3()-dd.J:row(j):toVector3()
			pos:radd(wj*(global_tf[j+1]*local_vertexpos))
		end
	end
	return pos
end

function smpl.createSkeleton(dd)
	smpl.updateShape(dd)
	local numJoint=dd.kintree_table:cols()
	local names=TStrings(numJoint)
	for i=0, numJoint-1 do
		names:set(i, dd.joint_names[i+1])
	end

	local pid=dd.kintree_table:row(0)
	local jointpos=vector3N(numJoint)
	jointpos:matView():assign(dd.J)
	local jointori=quaterN(numJoint)
	jointori:setAllValue(quater(1,0,0,0))

	local skinScale=100
	local tbl, currentPose, bones, parent=MotionUtil.generateWRLfromRawInfo(names(0),1/5/skinScale , names, pid, jointpos, jointori)
	local loader=FBXloader.motionLoaderFromTable(bones) --:toVRMLloader(0.25) -> vrmlloader conversion is slow and unnecesary.

	loader:setPose(currentPose)
	assert(#dd.joint_names==numJoint)
	dd.tree_indices=intvectorn(#dd.joint_names)
	dd.smpl_joint_index_from_tree_index=intvectorn(loader:numBone())
	dd.smpl_joint_index_from_tree_index:setAllValue(-1)
	for i, v in ipairs(dd.joint_names) do
		local ti=loader:getTreeIndexByName(v)
		dd.tree_indices:set(i-1,ti)
		dd.smpl_joint_index_from_tree_index:set(ti, i-1)
	end

	-- set axes for robust IK
	--loader:bone(1):setChannels('XYZ', 'YZX') -- trans, rot
	--for i=2, loader:numBone()-1 do
	--	loader:bone(i):setChannels('', 'YZX') -- trans, rot
	--end
	loader:getBoneByName('left_shoulder'):setChannels('', 'ZXY') -- trans, rot
	loader:getBoneByName('right_shoulder'):setChannels('', 'ZXY') -- trans, rot
	return loader
end
function smpl.createFBXskeleton(dd)
	local loader=smpl.createSkeleton(dd)

	local fbxloader=FBXloader(loader)

	local mesh=smpl.createMesh(dd)
	local meshInfo=fbxloader.fbxInfo[1]
	meshInfo[1]=mesh
	local skin=meshInfo.skin
	skin:resize(mesh:numVertex())
	assert(mesh:numVertex()==dd.weights:rows())
	assert(loader:numRotJoint()==dd.weights:cols())
	for i=0, mesh:numVertex()-1 do
		local weights=dd.weights:row(i)
		local w=skin:weights(i)
		local ti=skin:treeIndices(i)
		for j=0, weights:size()-1 do
			if weights(j)>1e-3 then
				local treeIndex= dd.tree_indices(j)
				ti:pushBack(treeIndex)
				w:pushBack(weights(j))
			end
		end
	end

	skin:calcLocalVertexPositions(loader, mesh)
	if mesh:numNormal()>0 then
		meshInfo.localNormal=vector3N(mesh:numNormal())
		for i=0, mesh:numNormal()-1 do
			meshInfo.localNormal(i):assign(mesh:getNormal(i))
		end
	end
	fbxloader.dd=dd

	return fbxloader
end

function smpl.loadMotion(dd, loader, mocap_framerate,trans, poses, _convertYUP)
	local mot=Motion(loader)
	mot:resize(poses:rows())
	mot:setFrameTime(1/mocap_framerate)
	local root_offset=dd.J:row(0):toVector3()
	for i=0, mot:numFrames()-1 do
		local pose=mot:pose(i)
		pose:assign(loader:pose())
		pose.translations(0):assign(trans:row(i):toVector3()+root_offset)
		assert(pose.rotations:size()*3==poses:cols())

		local frame=poses:row(i)
		local nSmplJoint=pose.rotations:size()
		for j=0, nSmplJoint-1 do
			local jj=loader:getRotJointIndexByTreeIndex(dd.tree_indices(j))
			assert(jj>=0)
			pose.rotations(jj):setRotation(frame:toVector3(j*3))
		end
	end
	if _convertYUP then
		local tf=transf(quater(math.rad(-90), vector3(1,0,0)))
		mot:transform(tf)
	end
	return mot

end


function smpl.redraw(dd, meshToEntity, node)
	local height=dd.J(array.locate(dd.joint_names, "left_foot")-1, 1)*-1 
	dd.draw_offset=vector3(0, height, 0)

	meshToEntity:updatePositions()
	node:setPosition(vector3(0,height*100,0))

	local nj=dd.J:rows()
	local pos=vector3N(nj)
	local lines=vector3N()
	local pid=dd.kintree_table:row(0)
	for i=0, dd.J:rows() -1 do
		pos(i):assign(dd.J:row(i):toVector3()+vector3(0,height, 0))
		dbg.draw('Text', pos(i)*100+vector3(0,5,0), dd.joint_names[i+1], vector3(0,0,0),1)
		if i>0 then
			lines:pushBack(pos(i))
			lines:pushBack(pos(pid(i)))
		end
	end
	local thickness=5
	dbg.drawBillboard( pos:matView()*100, 'joints', 'blueCircle', thickness, 'QuadListV')
	local thickness=2
	dbg.drawBillboard( lines:matView()*100, 'goal3', 'solidblue', thickness, 'BillboardLineList' )
	return height
end


-- fitting smpl to (partial) target T_pose joint positions
--jointNames: TStrings() should at least contain 'left_hip' and 'left_foot'
--jointPositions: vector3N()
function smpl.findBetasFromJointPositions(dd, jointNames, target_Tpose_jointPositions )
	-- vertices
	--dd.v_shaped=dd.shapedirs:dotProduct(dd.betas)+dd.v_template
	-- joint locations
	--dd.J=dd.J_regressor*dd.v_shaped
	if not dd.J_regress_shape then

		assert(dd.shapedirs:rows()==3)
		dd.J_regress_shape=hypermatrixn(3, dd.J_regressor:rows(), dd.betas:size())


		for i=0, dd.shapedirs:rows()-1 do
			dd.J_regress_shape:page(i):mult(dd.J_regressor, dd.shapedirs:row(i))
		end

		dd.J_regress_template=dd.J_regressor*dd.v_template

		-- dd.J:column(i)==dd.J_regress_shape:page(i)*dd.betas:column()+dd.J_regressor*dd.v_template:column(i)
		--
		local function genPair(n1, n2, offset)
			return { array.locate(SMPL.SMPL_JOINT_NAMES, n1)-1, osimLoader:getTreeIndexByName(n2), offset }
		end
	end

	local correspondences={}
	local targetJointIndexByName={}

	for i=0, jointNames:size()-1 do
		correspondences[i+1]=array.locate(smpl.SMPL_JOINT_NAMES, jointNames(i))-1
		targetJointIndex[jointNames(i)]=i
	end

	--
	local func=QuadraticFunction()
	local selected=boolN(dd.J:rows())
	selected:setAllValue(false)

	local desired=matrixn(dd.J:rows(),3)
	-- make fitting hip-relative
	local footPos=target_Tpose_jointPositions(targetJointIndexByName.left_foot)
	local hipPos=target_Tpose_jointPositions(targetJointIndexByName.left_hip)
	
	for i, v in ipairs(correspondences) do
		local jindex=v
		selected:set(jindex, true)
		desired:row(jindex):setVec3(0, target_Tpose_jointPositions(i-1))
		--desired:set(jindex,1,desired(jindex,1)-footPos.y)
		desired:set(jindex,1,desired(jindex,1)-hipPos.y)
	end

	do

		local nvar=dd.betas:size()
		
		--- w*(Ax-b)^2 (only for selected rows)
		-- desired x location
		-- dd.J_regress_shape:page(0)*betas +dd.J_regress_template:column(0)
		local b=desired:column(0)
		func:addSystemSelectedRows(1, dd.J_regress_shape:page(0), b-dd.J_regress_template:column(0), selected)


		-- desired z location
		local b=desired:column(2)
		func:addSystemSelectedRows(1, dd.J_regress_shape:page(2), b-dd.J_regress_template:column(2), selected)
		
		-- desired y location 
		-- y= dd.J_regress_shape:page(1)*x +dd.J_regress_template:column(1)
		-- footY= dd.J_regress_shape:page(1):row(dd.correspondences[1][1])*x +dd.J_regress_template:column(1)
		--local footIndex=dd.correspondences[1][1]
		local hipIndex=targetJointIndexByName.left_hip

		local A=dd.J_regress_shape:page(1):copy()
		local b=desired:column(1):copy()
		for i=0, A:rows()-1 do
			--A:row(i):rsub(dd.J_regress_shape:page(1):row(footIndex))
			A:row(i):rsub(dd.J_regress_shape:page(1):row(hipIndex))
			--b:set(i, b(i)+dd.J_regress_template( footIndex,1))
			b:set(i, b(i)+dd.J_regress_template( hipIndex,1))
		end

		func:addSystemSelectedRows(1, dd.J_regress_shape:page(1), b-dd.J_regress_template:column(1), selected)

		-- regularization
		func:addSystem(0.001, CT.eye(nvar), CT.zeros(nvar))

		local x=func:solve(nvar)
		dd.betas:assign(x)
		SMPL.updateShape(dd, mesh)
	end
end
-- markers={ { "markerName1", { baryCoeffs =vector3(0.40696105069268,0.52972872278993,0.06331022651739), faceIndex =90, }, 
--              ... }
--samples: {{ root_pos=vector3(), smpl_pose=vectorn(), marker_target=vector3N()},... }
function smpl.findBetasFromSurfacePositions(dd, markers, samples)
	local func=QuadraticFunction()

	smpl.flattenShapeDirs(dd)

	local function rep3(v, offset)
		local out=vectorn(v:size()*3)
		out:setAllValue(0)
		for i=0, v:size()-1 do
			out:set(i*3+offset,v(i) )
		end
		return out
	end

	local timerFK=util.PerfTimer()
	local timerMatMul=util.PerfTimer()
	local timerAdd=util.PerfTimer()

	--local V=vector3N()  -- for debug-drawing
	local errorSum=0
	
	for isample, pose_sample in ipairs(samples) do

		local root_pos=pose_sample.root_pos
		local smpl_pose=pose_sample.smpl_pose
		assert(root_pos)
		assert(pose_sample.marker_target and pose_sample.marker_target:size()==#markers)

		timerFK:start()
		local global_q, FKmat=smpl._calcFKmatrix(dd, smpl_pose, false)
		local nj=dd.kintree_table:cols()
		--local gR=CT.eye(nj*3, nj*3)
		--for i=0, nj-1 do
			--gR:sub(i*3,i*3+3,i*3,i*3+1):assign33(global_q[i+1]:M())
		--end
		local aR={}
		for i=0, nj-1 do
			aR[i+1]=matrixn(3,3)
			aR[i+1]:assign33(global_q[i+1]:M())
		end

		timerFK:pause()

		local FKmat_JS_flat=FKmat*dd.JS_flat
		local FKmat_J_v_template_flat=FKmat*dd.J_v_template_flat:column()

		for i, markerInfo in ipairs(markers) do
			local meshInfo=markerInfo[2]

			local fi=meshInfo.faceIndex
			local BW= 
			meshInfo.baryCoeffs(0)* dd.weights:row(dd.f(fi,0)) + 
			meshInfo.baryCoeffs(1)* dd.weights:row(dd.f(fi,1)) + 
			meshInfo.baryCoeffs(2)* dd.weights:row(dd.f(fi,2))  


			local coef, constant=smpl._computeCoef(dd, aR, fi, meshInfo.baryCoeffs)
			coef:radd(FKmat_JS_flat)
			constant:radd(FKmat_J_v_template_flat)

			timerAdd:start()

			local bw0=rep3(BW,0)
			local bw1=rep3(BW,1)
			local bw2=rep3(BW,2)
			-- markerPos(i)=bw(i):row()*(coef*betas+constant)+root_pos

			local targetPos=pose_sample.marker_target:row(i-1)
			func:addSystem(1, bw0:row()*coef, targetPos.x-((bw0:row()*constant)(0,0)+root_pos.x))
			func:addSystem(1, bw1:row()*coef, targetPos.y-((bw1:row()*constant)(0,0)+root_pos.y))
			func:addSystem(1, bw2:row()*coef, targetPos.z-((bw2:row()*constant)(0,0)+root_pos.z))
			--if isample==1 then
			local p=vector3()
			p.x=(bw0:row()*coef*dd.betas:column())(0,0)+((bw0:row()*constant)(0,0)+root_pos.x)
			p.y=(bw1:row()*coef*dd.betas:column())(0,0)+((bw1:row()*constant)(0,0)+root_pos.y)
			p.z=(bw2:row()*coef*dd.betas:column())(0,0)+((bw2:row()*constant)(0,0)+root_pos.z)
			errorSum=errorSum+p:squaredDistance(targetPos)
			--	p:radd(vector3(1,0,0))
			--	V:pushBack(p)
			--end


			timerAdd:pause()
		end
	end
	--print('_calcFKmat', timerFK:stop()/1e3)
	--print('matMul', timerMatMul:stop()/1e3)
	--print('addSystem', timerAdd:stop()/1e3)
	local nvar=dd.betas:size()
	-- regularization
	func:addSystem(0.001, CT.eye(nvar), CT.zeros(nvar))
	local timer2=util.Timer()
	timer2:start()
	local x, A, b=func:solve(nvar)
	--print('actualSolve', timer2:stop2()/1e6)

	-- original error
	--print('hessian error:', (A*dd.betas:column()-b:column()):column(0):length())
	print('MSE:', errorSum/(#samples*#markers))

	--dbg.drawBillboard (V:matView()*100, 'vvmarkerpos', 'blueCircle', 5, 'QuadListV')

	dd.betas:assign(x)
	-- new error doesn't mean much
	--print('new error:', (A*dd.betas:column()-b:column()):column(0):length())

	smpl.updateShape(dd, mesh)
end

function smpl._computeCoef(dd, aR, fi, baryCoeffs)
	local nj=dd.kintree_table:cols()
	local coef, constant
	local shape_sel=matrixn()
	local selIndex= CT.ivec(
	dd.f(fi,0)*3,  -- x0
	dd.f(fi,0)*3+1,  -- y0
	dd.f(fi,0)*3+2,   --- z0
	dd.f(fi,1)*3, 
	dd.f(fi,1)*3+1, 
	dd.f(fi,1)*3+2, 
	dd.f(fi,2)*3, 
	dd.f(fi,2)*3+1, 
	dd.f(fi,2)*3+2) 
	shape_sel:extractRows(dd.shapedirs_flat,selIndex)
	local v_template_sel=dd.v_template:toVector():extract(selIndex)

	local B=CT.zeros(3,9)
	for i=0,3-1 do
		B:row(i):set(0+i, baryCoeffs(0))
		B:row(i):set(3+i, baryCoeffs(1))
		B:row(i):set(6+i, baryCoeffs(2))
	end

	--local rep=CT.zeros(nj*3, 3)
	--for i=0, nj-1 do
	--	rep:set(i*3, 0, 1)
	--	rep:set(i*3+1,1,  1)
	--	rep:set(i*3+2,2, 1)
	--end

	local function leftmult_gR(mat)
		local nj=mat:rows()/3
		--return gR*mat  의 sparse implementation
		local out=matrixn(nj*3, mat:cols())
		for i=0, nj-1 do
			out:sub(i*3, i*3+3, 0,0):assign(aR[i+1]*mat:sub(3*i, 3*i+3, 0,0))
		end
		return out
	end

	local function leftmult_gR_rep(nj, mat)
		--return gR*rep*mat  의 sparse implementation
		local out=matrixn(nj*3, mat:cols())
		for i=0, nj-1 do
			out:sub(i*3, i*3+3, 0,0):assign(aR[i+1]*mat:sub(0, 3, 0,0))
		end
		return out
	end

	coef=leftmult_gR_rep(nj, B*shape_sel)-leftmult_gR(dd.JS_flat)

	constant=leftmult_gR_rep(nj, B*v_template_sel:column())-leftmult_gR(dd.J_v_template_flat:column())
	return coef, constant
end
function smpl.computeError(dd, markers, samples)
	if false then
		-- slow but easy to understand code
		local LBSvertices=vector3N(dd.v_shaped:rows())
		local err=0
		local c=0
		for isample, pose_sample in ipairs(samples) do

			local root_pos=pose_sample.root_pos
			local smpl_pose=pose_sample.smpl_pose
			assert(root_pos)
			assert(pose_sample.marker_target and pose_sample.marker_target:size()==#markers)

			-- slow!!! use FBXloader instead.
			smpl.performLBS(dd,root_pos, smpl_pose, LBSvertices, false)

			--dbg.draw("Traj", LBSvertices:matView()*100, 'points', '',0, 'PointList')
			--dbg.draw("Sphere", (dd.J:row(0):toVector3())*100, 'pointso')

			for i, markerInfo in ipairs(markers) do
				local meshInfo=markerInfo[2]

				local fi=meshInfo.faceIndex

				local vi0=dd.f(fi,0)
				local vi1=dd.f(fi,1)
				local vi2=dd.f(fi,2)


				local v=vector3(0)
				for j=0, 2 do
					v:radd( meshInfo.baryCoeffs(j)*LBSvertices(dd.f(fi,j)))
				end
				v:rsub(dd.J:row(0):toVector3())


				--dbg.draw("Sphere", v*100, 'pp'..i)
				local targetPos=pose_sample.marker_target:row(i-1)
				--dbg.draw("Sphere", targetPos*100, 'pp2_'..i,'red')
				err=err+targetPos:squaredDistance(v)
				c=c+1
			end
		end
		return err/c
	else

		smpl.flattenShapeDirs(dd)

		local function rep3(v, offset)
			local out=vectorn(v:size()*3)
			out:setAllValue(0)
			for i=0, v:size()-1 do
				out:set(i*3+offset,v(i) )
			end
			return out
		end

		local errorSum=0

		for isample, pose_sample in ipairs(samples) do

			local root_pos=pose_sample.root_pos
			local smpl_pose=pose_sample.smpl_pose
			assert(root_pos)
			assert(pose_sample.marker_target and pose_sample.marker_target:size()==#markers)

			local global_q, FKmat=SMPL._calcFKmatrix(dd, smpl_pose, false)
			local nj=dd.kintree_table:cols()
			local aR={}
			for i=0, nj-1 do
				aR[i+1]=matrixn(3,3)
				aR[i+1]:assign33(global_q[i+1]:M())
			end

			local FKmat_JS_flat=FKmat*dd.JS_flat
			local FKmat_J_v_template_flat=FKmat*dd.J_v_template_flat:column()

			for i, markerInfo in ipairs(markers) do
				local meshInfo=markerInfo[2]

				local fi=meshInfo.faceIndex
				local BW= 
				meshInfo.baryCoeffs(0)* dd.weights:row(dd.f(fi,0)) + 
				meshInfo.baryCoeffs(1)* dd.weights:row(dd.f(fi,1)) + 
				meshInfo.baryCoeffs(2)* dd.weights:row(dd.f(fi,2))  


				local coef, constant=smpl._computeCoef(dd, aR, fi, meshInfo.baryCoeffs)
				coef:radd(FKmat_JS_flat)
				constant:radd(FKmat_J_v_template_flat)

				local bw0=rep3(BW,0)
				local bw1=rep3(BW,1)
				local bw2=rep3(BW,2)
				-- markerPos(i)=bw(i):row()*(coef*betas+constant)+root_pos

				local targetPos=pose_sample.marker_target:row(i-1)
				--if isample==1 then
				local p=vector3()
				p.x=(bw0:row()*coef*dd.betas:column())(0,0)+((bw0:row()*constant)(0,0)+root_pos.x)
				p.y=(bw1:row()*coef*dd.betas:column())(0,0)+((bw1:row()*constant)(0,0)+root_pos.y)
				p.z=(bw2:row()*coef*dd.betas:column())(0,0)+((bw2:row()*constant)(0,0)+root_pos.z)
				errorSum=errorSum+p:squaredDistance(targetPos)
				--	p:radd(vector3(1,0,0))
				--	V:pushBack(p)
				--end


			end
		end
		return errorSum/(#samples*#markers)
	end
end
return smpl
