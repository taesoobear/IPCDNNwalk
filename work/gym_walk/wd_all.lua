
local function quatToVec(q)
	return CT.vec(q.w, q.x, q.y,q.z)
end
input={
	skel="../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T.wrl",
	mot="../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_wd2_all.dof",
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
	azuma={
		contacts=
		{
			--      
			{{247, 249, 265,270}, {286, 288, 304, 308},},  --L
			--      280                  
			{{228, 230, 246, 250}, {267,269,285,288}, }, -- R
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
		useMMoffsetQ=true,
	},
	leanedBack={
		contacts=
		{
			--      
			{{8203, 8204, 8223, 8230}, {8245,8247, 8268, 8274,}, }, -- L
			{{8224, 8227, 8247,8251, }, {8268,8271,8293, 8296},},  --R
			--      280                  
		},
		minHalfStride=0.6,  --
		speed=1.05,  -- looks better
		--speed=0.9, -- original speed
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
		--useMMoffsetQ=true,
	},
	soldier={
		contacts=
		{
			--      
			{{10387, 10388, 10409, 10412}, {10430,10431,10450,10454}, }, -- R
			{{10408, 10409, 10430,10433}, {10451, 10453, 10473, 10476},},  --L
		},
		minHalfStride=0.6,  --
		speed=0.76, -- original speed
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
		--useMMoffsetQ=true,
	},
	sneaky={
		contacts=
		{
			{{12151,12155,12190,12194}, {12213,12216,12247,12250}},
			{{12181,12185,12220,12224}, {12241,12245,12276,12280}},
		},
		desiredCOMheight=0.75*0.95,
		minHalfStride=0.6,  --
		speed=0.5, -- original speed
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
	},
	-- soldier 10401
	-- sneaky 12189
	-- tong 13048
	-- skip 14186
}

-- azuma setting
--input.walk=input.azuma input.run=input.leanedBack
-- other settings
input.walk=input.leanedBack input.run=input.azuma
--input.walk=input.soldier input.run=input.azuma
--input.walk=input.sneaky input.run=input.azuma
