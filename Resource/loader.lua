--dofile('../MainLib/WrapperLua/mylib.lua')
loader("addItem", {
	--	{"boxer motions", "boxer", "boxer_down"},
	--	{"dance motions", "dance", "dance2", "dance3","boa", "waltz", "waltz_w","dance4"},
	--	{"locomotions", "loco_onion", "loco_hyunwoo_long","loco_hyunwoo_long_binary","locomotion","locomotion_balance"},
	--	{"taekwondo motions", "taekwondo", "taekwondo_long","taekwondo_short_binary","taekwondo_short_upsample_binary","taekwondo_long_binary",	"taekwondo_all", "taekwondo_all_binary", "taekwondo_all_nd",	"kick_all",	"punch_all",	"shadow_all"},
	{"taekwondo motions", "taekwondo_all_nd", "taekwondo_in_meter"},
	--	{"muaythai motions", "MuayThai", "MuayThai_binary", "MuayThai_COM", "MuayThai_ndCOM_binary","Kickboxing", "Kickboxing_Balance"},
	{"muaythai motions", "MuayThai_ndCOM_binary"},
	--	{"Iguana motions", "Iguana Motion Set",	"Iguana Motion Set binary", "Iguana attack"},
	--	{"WD2", "wd2_new_office_desk_sit01", "pelvis"},
	{"RigidBody", "justin_straight_run", "gymnist", "jump", "jump5_reducedDOF", "jump5"},
	--	{"V files", "justin_1"},
	{"shortcuts", "Iguana Motion Set binary", "hyunwoo_lowdof_T", "Kickboxing_Balance"}
})

--loader("setDefault", "MuayThai_ndCOM_binary")
--loader("setDefault", "gymnist")
--loader("setDefault", "Iguana Motion Set binary")
loader("setDefault", "hyunwoo_lowdof_T")

function numCharacter(motionId)
	if string.sub(motionId, 1, 10)== "taekwondo_" or
		motionId=="dance2" or
		motionId=="waltz" or
		string.sub(motionId,1,8)=="MuayThai" or
		string.sub(motionId, 1,10)=="Kickboxing" then
		return 2
	end

	return 1
end

dofile("../Resource/motion.lua")
function os.filename(target)-- fullpath
	local lastSep
	local newSep=0
	local count=0
	repeat lastSep=newSep
		newSep=string.find(target, "/", lastSep+1) 	    
		count=count+1
	until newSep==nil 

	local path=string.sub(target, 0, lastSep-1)

	local filename
	if lastSep==0 then filename=string.sub(target,lastSep) path='' else filename=string.sub(target, lastSep+1) end

	return filename
end

wrlFiles={}
for i,v in ipairs(resource) do
	if type(v)=='table' then
		local fn=v[1]
		if string.sub(fn, -4)=='.wrl' then
			wrlFiles[string.sub(fn, 1, -5)]=v
		end
	end
end

function isMotionDOF(motionId)
	local fn=wrlFiles[motionId]
	if fn then
		return true, fn[1]
	end
	return false
end
function load1(motionId, mot)
	print("Loading ", motionId)
	if wrlFiles[motionId] then
		local info=wrlFiles[motionId]
		local skel=RE.motionLoader(info[1])
		mot:loadMotion(default_path..info[3])
		if info.initialHeight then
			local mot=mot.mot
			local h=info.initialHeight
			for i=0, mot:numFrames()-1 do
				mot:row(i):set(1, mot:row(i):get(1)+h);
			end
		end
	elseif motionId=="justin_1" then
		--      local skel=RE.motionLoader("walker.vsk")
		local skel=RE.motionLoader("test_v/subject.vsk")
		skel:loadAnimation(mot, "test_v/ROM.V")

	elseif motionId=="wd2_new_office_desk_sit01" then
		local skel=RE.motionLoader("wd2/wd2.asf")
		skel:printHierarchy()
		skel:loadAMCmotion("../Resource/motion/wd2/wd2_new_office_desk_sit01.amc", mot)
		mot:setIdentifier("wd2_new_office_desk_sit01")	
		mot:translate(vector3(0,-0.8,0))
		loader:calcInterCon()
	elseif motionId=="jump" then
		local skel=RE.motionLoader("../Resource/scripts/ui/rigidbodywin/02.asf")
		skel:printHierarchy()
		skel:loadAnimation(mot, "../Resource/scripts/ui/rigidbodywin/02_04.amc")
		mot:setIdentifier("jump")
		mot:scale(0.045*100)
	elseif motionId=="pelvis" then
		local skel=RE.motionLoader("../Resource/motion/pelvis.bvh")
		skel:printHierarchy()
		skel:loadAnimation(mot, "../Resource/motion/pelvis.bvh")
		mot:setIdentifier("pelvis")
		mot:scale(1.0)

	elseif motionId=="jump5_reducedDOF" then
		mot:initFromFile("../Resource/scripts/ui/rigidbodywin/jump4.mot")
		mot:setIdentifier("jump5_reducedDOF")
		mot:scale(100)
	elseif motionId=="jump5" then
		mot:initFromFile("../Resource/scripts/ui/rigidbodywin/jump_raw.mot")
		mot:setIdentifier("jump5")
		mot:scale(100)
	elseif motionId=="loco_hyunwoo_long_binary" then
		--mot:initFromFile("locomotion_hyunwoo/locomotion_hl.mot")
		--mot:setIdentifier("locomotion_hl")
	elseif motionId=="loco_hyunwoo_long" then
		mot:initFromFile("LOCOMOTION_HYUNWOO/Run_01.bvh")
		mot:setIdentifier("locomotion_hl")
	elseif motionId=="locomotion" then
		mot:initFromFile("locomotion_hyunwoo2/run_01.bvh")
		mot:concatFromFile("locomotion_hyunwoo2/run_02.bvh")
		mot:concatFromFile("locomotion_hyunwoo2/run_03.bvh")
		mot:concatFromFile("locomotion_hyunwoo2/run_04.bvh")
		mot:concatFromFile("locomotion_hyunwoo2/run_05.bvh")
		mot:concatFromFile("locomotion_hyunwoo2/run_06.bvh")
		mot:concatFromFile("locomotion_hyunwoo2/run_07.bvh")
		mot:setIdentifier("locomotion")
		loader:calcInterCon()
		-- cm 
		mot:scale(170/70.68)
		print("hihi")			
		mot:calcInterFrameDifference()
		print("hihi2")
	elseif motionId=="locomotion_balance" then
		local skel=RE.motionLoader("locomotion_hyunwoo2/locomotion.mot")
		skel:insertRootJoint({"Balance"})
		skel:printHierarchy()
		mot:init(skel);
		mot:calcInterFrameDifference()
		mot:setIdentifier("locomotion_balance")
	elseif motionId=="boxer" then
		mot:initFromFile("BOXER/BOXER.ASF");
		mot:setIdentifier("boxer");
		mot:skeleton():printHierarchy()
		loader:calcInterCon()
	elseif motionId=="Iguana Motion Set" then
		--mot:initFromFile("test1.trc")
		mot:initFromFile("Iguana.trc")	-- this will load "TRC/Day5_Session1_Take01_ddong_-_walk.trc". see motion.lua for detail.
		mot:concatFromFile("trc/Day6_Session1_Take06_-_attack.trc")
		mot:concatFromFile("trc/Day6_Session1_Take09_-_xrun.trc")
		mot:concatFromFile("trc/Day6_Session2_Take05_good_walk.trc")
		mot:concatFromFile("trc/Day6_Session2_Take08_good_walk.trc")
		mot:concatFromFile("trc/Day7_Session2_Take01_-_walk.trc")
		mot:concatFromFile("trc/Day7_Session2_Take02_-_slow_left_turn.trc")
		mot:concatFromFile("trc/Day7_Session2_Take02_-_slow_left_turn.trct")
		mot:concatFromFile("trc/Day7_Session2_Take05_-_run_for_a_while.trc")
		mot:concatFromFile("trc/Day7_Session2_Take06_-_jump.trc")
		mot:concatFromFile("trc/Day7_Session2_Take08_-_stop_walk_run_walk_stop.trc")
		mot:concatFromFile("trc/Day7_Session2_Take12_-_standing_left_turn.trc")
		mot:concatFromFile("trc/Day7_Session2_Take13_-_statnding_left_turn.trc")
		mot:concatFromFile("trc/Day7_Session2_Take13_-_statnding_right_turn.trc")
		mot:concatFromFile("trc/Day7_Session2_Take14_-_walk.trc")
		mot:setIdentifier("Iguana Motion Set")
	elseif motionId=="Iguana Motion Set binary" then			
		skel=RE.motionLoader("iguana.skl")
		skel:readJointIndex("../Resource/motion/trc/iguana_from248.ee")
		skel:loadAnimation(mot, "trc/iguana_motion_set.mot")
		mot:setIdentifier("Iguana Motion Set")			
	elseif motionId=="Iguana attack" then
		mot:initFromFile("trc/Day6_Session1_Take06_-_attack.trc")
		mot:setIdentifier("Iguana attack")
	end
	print("finished")
end

function load2(motionId, mot1, mot2)


	if motionId=="taekwondo_all_binary" then
		mot1:initFromFile("MAN1ALL.MOT");
		mot2:initFromFile("MAN2ALL.MOT");
		mot1:setIdentifier("man1all");
		mot2:setIdentifier("man2all");
	elseif motionId=="taekwondo_all_nd" then
		mot1:initFromFile("kickboxer/man1all_nd.mot");
		mot2:initFromFile("kickboxer/man2all_nd.mot");
		mot1:setIdentifier("man1all_nd");
		mot2:setIdentifier("man2all_nd");
	elseif motionId=="taekwondo_in_meter" then
		mot1:initFromFile("kickboxer/man1all_nd.mot");
		mot2:initFromFile("kickboxer/man2all_nd.mot");
		mot1:scale(0.01);
		mot2:scale(0.01);
		mot1:setIdentifier("man1all_m");
		mot2:setIdentifier("man2all_m");
	elseif motionId=="MuayThai_binary" then
		local skel1=RE.motionLoader("muaythai1.skl")
		local skel2=RE.motionLoader("muaythai2.skl")
		loader:changeFactory(skel1, "TwoPosture")
		loader:changeFactory(skel2, "TwoPosture")
		skel1:loadAnimation(mot1, "muaythai/muaythai1.mot")
		skel2:loadAnimation(mot2, "muaythai/muaythai2.mot")
		mot1:setIdentifier("MuayThai1")
		mot2:setIdentifier("MuayThai2")
		-- TwoPostureSpecific
		computeTargetPos(mot1, mot2)
		computeTargetPos(mot2, mot1)
	elseif motionId=="MuayThai_COM" then
		local skel1=RE.motionLoader("muaythai/muaythai1.mot");
		local skel2=RE.motionLoader("muaythai/muaythai2.mot");			
		skel1:insertRootJoint({"COM", 10.0, 3.0})
		mot1:init(skel1)
		mot1:setIdentifier("MuayThai1_COM")
		skel2:insertRootJoint({"COM", 10.0, 3.0})
		mot2:init(skel2)
		mot2:setIdentifier("MuayThai2_COM")
		loader:calcInterCon()
	elseif motionId=="Kickboxing" then
		mot1:initFromFile("muaythai/muaythai1.mot");
		mot2:initFromFile("muaythai/muaythai2.mot");
		-- cm ??위???? ??환?磯?.
		mot1:scale(175/162);
		mot2:scale(175/162);
		mot1:setIdentifier("Kickboxing1")
		mot2:setIdentifier("Kickboxing2")
	elseif motionId=="Kickboxing_Balance" then
		local skel1=RE.motionLoader("muaythai/muaythai1.mot")
		local skel2=RE.motionLoader("muaythai/muaythai2.mot")			
		-- cm ??위???? ??환?磯?.
		skel1:scale(175/162)
		skel2:scale(175/162)
		skel1:insertRootJoint({"Balance"})
		skel2:insertRootJoint({"Balance"})
		skel1:printHierarchy()
		mot1:init(skel1);
		mot2:init(skel2);
		mot1:setIdentifier("Kickboxing1_Balance")
		mot2:setIdentifier("Kickboxing2_Balance")
	elseif motionId=="MuayThai_ndCOM_binary" then
		local skel1=RE.motionLoader("muaythai1_com.skl")
		local skel2=RE.motionLoader("muaythai2_com.skl")

		if EXT then
			EXT.changeFactory(skel1, "TwoPosture")
			EXT.changeFactory(skel2, "TwoPosture")
		else
			print("warning! cannot use TwoPosture")
		end

		skel1:loadAnimation(mot1, "kickboxer/muaythai/muaythai1_ndCOM.mot")
		skel2:loadAnimation(mot2, "kickboxer/muaythai/muaythai2_ndCOM.mot")
		skel1:readJointIndex("../Resource/motion/kickboxer/muaythai1.ee")
		skel2:readJointIndex("../Resource/motion/kickboxer/muaythai2.ee")
		mot1:setIdentifier("MuayThai1_ndCOM")
		mot2:setIdentifier("MuayThai2_ndCOM")

		if EXT then
			EXT.computeTargetPos(mot1, mot2)
			EXT.computeTargetPos(mot2, mot1)
		end
		--[[
		-- TwoPostureSpecific : it can't be done here.
		loader:changeFactory(skel1, "TwoPosture")
		loader:changeFactory(skel2, "TwoPosture")
		computeTargetPos(mot1, mot2)
		computeTargetPos(mot2, mot1)
		]]
	elseif motionId=="MuayThai" then
		local skel1=RE.motionLoader("muaythai1.skl")
		local skel2=RE.motionLoader("muaythai2.skl")
		mot1:initSkeleton(skel1)
		mot2:initSkeleton(skel2)
		mot1:concatFromFile("muaythai/Fight_001_1_trim1_sm_A.bvh")
		mot2:concatFromFile("muaythai/Fight_001_1_trim1_sm_B.bvh")
		mot1:concatFromFile("muaythai/Fight_002_6_sm_A.bvh")
		mot2:concatFromFile("muaythai/Fight_002_6_sm_B.bvh")
		mot1:concatFromFile("muaythai/Fight_003_2_trim1_sm_A.bvh")
		mot2:concatFromFile("muaythai/Fight_003_2_trim1_sm_B.bvh")
		mot1:concatFromFile("muaythai/Fight_003_2_trim2_sm_A.bvh")
		mot2:concatFromFile("muaythai/Fight_003_2_trim2_sm_B.bvh")
		mot1:concatFromFile("muaythai/Fight_004_1_sm_A.bvh")
		mot2:concatFromFile("muaythai/Fight_004_1_sm_B.bvh")
		mot1:concatFromFile("muaythai/Fight_005_2_sm_A.bvh")
		mot2:concatFromFile("muaythai/Fight_005_2_sm_B.bvh")
		mot1:concatFromFile("muaythai/Fight_006_1_sm_A.bvh")
		mot2:concatFromFile("muaythai/Fight_006_1_sm_B.bvh")
		mot1:concatFromFile("muaythai/Fight_007_1_sm_A.bvh")
		mot2:concatFromFile("muaythai/Fight_007_1_sm_B.bvh")
		mot1:concatFromFile("muaythai/Fight_008_1_trim1_sm_A.bvh")
		mot2:concatFromFile("muaythai/Fight_008_1_trim1_sm_B.bvh")
		mot1:concatFromFile("muaythai/Fight_008_1_trim2_sm_A.bvh")
		mot2:concatFromFile("muaythai/Fight_008_1_trim2_sm_B.bvh")
		mot1:concatFromFile("muaythai/Fight_009_1_trim1_sm_A.bvh")
		mot2:concatFromFile("muaythai/Fight_009_1_trim1_sm_B.bvh")
		mot1:setIdentifier("MuayThai1")
		mot2:setIdentifier("MuayThai2")

		EXT.changeFactory(skel1, "TwoPosture")
		EXT.changeFactory(skel2, "TwoPosture")
		loader:calcInterCon()
		-- TwoPostureSpecific
		computeTargetPos(mot1, mot2)
		computeTargetPos(mot2, mot1)
	end
	print("finished")
end

