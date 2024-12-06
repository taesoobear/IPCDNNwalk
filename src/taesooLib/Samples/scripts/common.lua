
require("module")

if not Physics or not Physics.DynamicsSimulator then return end
function Physics.DynamicsSimulator:getPoseDPose(ichar)
	local pose=vectorn()
	local dpose=vectorn()
	self:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, pose)
	self:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dpose)
	return pose, dpose
end
function Physics.DynamicsSimulator:setPoseDPose(ichar, pose, dpose)
	self:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, pose)
	self:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dpose)
	self:initSimulation()
end

if true then

	-- for compatibility with gmbs simulator
	-- this version is deprecated.
	-- use the functions in qpservo.lua 
	-- (simply use "requre('qpservo')" to overwrite the deprecated definitions)
	Physics.DynamicsSimulator_TRL_QP.calcMassMatrix3=Physics.DynamicsSimulator_TRL_QP.calcMassMatrix
	function Physics.DynamicsSimulator_TRL_QP:calcBoneDotJacobian(ichar, ibone, localpos, J, DJ)
		local jacobian=matrixn()
		local dotjacobian=matrixn()
		self:calcJacobianAt(ichar, ibone, jacobian, localpos);
		J:assign(jacobian:sub(3,6,0,0))
		self:calcDotJacobianAt(ichar, ibone, dotjacobian, localpos);
		DJ:assign(dotjacobian:sub(3,6,0,0))
	end
	function Physics.DynamicsSimulator_TRL_QP:calcBoneDotJacobian2(ichar, ibone, localpos, jacobian, dotjacobian)
		self:calcJacobianAt(ichar, ibone, jacobian, localpos);
		self:calcDotJacobianAt(ichar, ibone, dotjacobian, localpos);
	end
	function Physics.DynamicsSimulator_TRL_QP:calcJacobian(ichar, ibone, J)
		self:calcJacobianAt(ichar, ibone, J, vector3(0,0,0));
	end
	function Physics.DynamicsSimulator_TRL_QP:setCollisionMargin(i1,i2)
		-- does nothing
	end
	function Physics.DynamicsSimulator_TRL_QP:calcDotJacobian(ichar, ibone, DJ)
		self:calcDotJacobianAt(ichar, ibone, DJ, vector3(0,0,0));
	end
	function Physics.DynamicsSimulator_TRL_QP:stepKinematic(ddq, tau, integrate)
		assert(integrate)
		self:stepKinematic2(0, ddq)
	end
end
defineDerived(Physics.DynamicsSimulator, 
{
	Physics.DynamicsSimulator_gmbs,
	Physics.DynamicsSimulator_gmbs_penalty,
	Physics.DynamicsSimulator_TRL_QP,
	Physics.DynamicsSimulator_TRL_LCP,
}, 
{
	"getPoseDPose", 
	"setPoseDPose",
}
)

if Physics and PLDPrimVRML then
	PLDPrimVRML.setPose=Physics.DynamicsSimulator.setPose
end
-- if MPI.rank()==0 then
--    dbg.startTrace2()
-- end

gFolder=package.resourcePath

-- taesooLib supports many simulators.
-- sample_bullet.git
-- sample_mujoco.git
-- sample_physX.git : nvidia physX
simulators={UT=0, AIST=1, SDFAST=2, VP=3, gmbs=4, gmbs_penalty=5, gmbs_QP=6, TRL=7, TRL_penalty=8, bullet=9, mujoco=10, Trbdl_penalty=11,Trbdl_LCP=12, Trbdl_QP=13, PhysX=14, mujoco=15, TRL_gjk=16, Trbdl_impulse=17}

model_files={}
model_files.default={}
model_files.default.start=0 -- Nm/rad
-- Default PD-servo settings
model_files.default.k_p=600 -- Nm/rad
model_files.default.k_d=3 --  Nms/rad. worked in range [0, 1]
model_files.default.k_pos=vector3(0.1,0.1,0.1)
model_files.default.k_p_slide=20000-- Nm/rad
model_files.default.k_d_slide=1000 --  Nms/rad. worked in range [0, 1]
model_files.default.k_scale_active={ default={1,4,0}, hip={1,1,1}, knee={1,1,1}, elbow= {0.3,2,0}, shoulder={0.3,2,1}, collar={0.3,2,0}, ankle={1,2,1}, toes={0.3,0.3,0}, }
model_files.default.k_scale_passive=deepCopyTable(model_files.default.k_scale_active)

-- when useInverseDynamics==true
model_files.default.k_scale_id={ default={1,1,1}, }
model_files.default.k_scale_active_pd={ default={1,1,1}, }
model_files.default.k_scale_passive_pd=deepCopyTable(model_files.default.k_scale_active_pd)


--model_files.default.k_d0=0 --  Nms/rad. worked in range [0, 1]
model_files.default.timestep=1/8000
model_files.default.frame_rate=120
model_files.default.initialHeight=0.01 -- meters
model_files.default.penaltyDepthMax={0.0005}
model_files.default.penaltyForceStiffness=10000
model_files.default.penaltyForceDamp=1000
model_files.default.bones={}
model_files.default.simulator=simulators.SDFAST
model_files.default.totalmass=77

model_files.cart_pole=deepCopyTable(model_files.default)
model_files.cart_pole.file_name=package.resourcePath.."cart_pole.wrl"
model_files.cart_pole.simulator=simulators.UT
model_files.cart_pole.totalmass=nil  -- use the one in the file.

model_files.lowerbody=deepCopyTable(model_files.default)
model_files.lowerbody.file_name=package.resourcePath.."lowerbody.wrl"
model_files.lowerbody.bones.left_knee="LLEG_KNEE"
model_files.lowerbody.initialHeight=0.1 -- meters

model_files.lowerbody_allDOF1=deepCopyTable(model_files.lowerbody)
model_files.lowerbody_allDOF1.file_name=package.resourcePath.."lowerbody_allDOF1.wrl"

model_files.lowerbody_foot=deepCopyTable(model_files.lowerbody)
model_files.lowerbody_foot.file_name="../Resource/mesh/left_leg.wrl"

model_files.lowerbody_elastic=deepCopyTable(model_files.lowerbody)
model_files.lowerbody_elastic.file_name=package.resourcePath.."lowerbody_elastic.wrl"
model_files.lowerbody_elastic.initialHeight=0.7 -- meters

model_files.hyunwoo_allDOF1=deepCopyTable(model_files.default)
model_files.hyunwoo_allDOF1.file_name=package.resourcePath.."hyunwoo__allDOF1.wrl"
model_files.hyunwoo_allDOF1.bones.left_knee="LeftKnee"
model_files.hyunwoo_allDOF1.bones.right_knee="RightKnee"
model_files.hyunwoo_allDOF1.bones.left_heel="LeftAnkle"
model_files.hyunwoo_allDOF1.bones.right_heel="RightAnkle"
model_files.hyunwoo_allDOF1.bones.left_hip="LeftHip"
model_files.hyunwoo_allDOF1.bones.right_hip="RightHip"
model_files.hyunwoo_allDOF1.bones.chest="Chest"
model_files.hyunwoo_allDOF1.bones.chest2="Chest2"
model_files.hyunwoo_allDOF1.bones.neck="Neck"
model_files.hyunwoo_allDOF1.bones.head="Head"
model_files.hyunwoo_allDOF1.bones.left_collar="LeftCollar"
model_files.hyunwoo_allDOF1.bones.right_collar="RightCollar"
model_files.hyunwoo_allDOF1.initialHeight=1 -- meters

model_files.hyunwoo_lowdof_T=deepCopyTable(model_files.hyunwoo_allDOF1)
model_files.hyunwoo_lowdof_T.file_name=package.resourcePath.."locomotion_hyunwoo/hyunwoo_lowdof_T.wrl"
model_files.hyunwoo_lowdof_T.mot_file=package.resourcePath.."locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof"

model_files.hanyang_lowdof_T=deepCopyTable(model_files.hyunwoo_lowdof_T)
model_files.hanyang_lowdof_T.file_name=package.resourcePath.."MOB1/hanyang_lowdof_T.wrl"
model_files.hanyang_lowdof_T.mot_file=package.resourcePath.."MOB1/hanyang_lowdof_T_MOB1_Run_F_Loop.fbx.dof"


model_files.hyunwoo_fourDOF3=deepCopyTable(model_files.hyunwoo_allDOF1)
model_files.hyunwoo_fourDOF3.file_name=package.resourcePath.."hyunwoo_fourDOF3.wrl"
model_files.hyunwoo_fourDOF3.frame_rate=30
model_files.hyunwoo_fourDOF3.mot_file=package.resourcePath.."hyunwoo_fourDOF3.mot"
model_files.hyunwoo_fourDOF3.start=11350 --107 --: stand  -- 715 : running --590 --121

model_files.hyunwoo_real=deepCopyTable(model_files.hyunwoo_fourDOF3)
model_files.hyunwoo_real.file_name=package.resourcePath.."hyunwoo_real.wrl"
model_files.hyunwoo_real.mot_file=package.resourcePath.."hyunwoo_real.dof"
model_files.hyunwoo_real.timestep=1/10000
model_files.hyunwoo_real.initialHeight=0.05 -- meters
model_files.hyunwoo_real.penaltyForceStiffness=40000
model_files.hyunwoo_real.penaltyForceDamp=4000

model_files.hyunwoo_real_cart=deepCopyTable(model_files.hyunwoo_real)
model_files.hyunwoo_real_cart.file_name=package.resourcePath.."hyunwoo_real_cart.wrl"
model_files.hyunwoo_real_cart.mot_file=package.resourcePath.."hyunwoo_real_cart.dof"
model_files.hyunwoo_real_cart.k_p_slide=30000-- Nm/rad
model_files.hyunwoo_real_cart.k_d_slide=2000 -- Nms/rad. worked in range [0, 1]
model_files.hyunwoo_real_cart.k_p=700 -- Nm/rad
model_files.hyunwoo_real_cart.k_d=30
model_files.hyunwoo_real_cart.clampTorque=300
model_files.hyunwoo_real_cart.clampForce=3000
model_files.hyunwoo_real_cart.frame_rate=120
--model_files.hyunwoo_real_cart.simulationFrameRate=8400
model_files.hyunwoo_real_cart.simulationFrameRate=12000
model_files.hyunwoo_real_cart.penaltyForceStiffness=20000
model_files.hyunwoo_real_cart.penaltyForceDamp=1000
model_files.hyunwoo_real_cart.start=360 --107 --: stand  -- 715 : running --590 --121
model_files.hyunwoo_real_cart.initialHeight=0

model_files.hyunwoo_real_cart.k_scale_passive.default={0.5,4,0}
model_files.hyunwoo_real_cart.k_scale_passive.hip={1.0,3,1}
model_files.hyunwoo_real_cart.k_scale_passive.knee={0.5,3,1}
model_files.hyunwoo_real_cart.k_scale_passive.ankle={0.5,3,0}

model_files.hyunwoo_full=deepCopyTable(model_files.hyunwoo_real)
model_files.hyunwoo_full.file_name=package.resourcePath.."locomotion_hyunwoo/hyunwoo_full.wrl"
model_files.hyunwoo_full.mot_file=package.resourcePath.."locomotion_hyunwoo/hyunwoo_full.dof"

model_files.hyunwoo_full_cart=deepCopyTable(model_files.hyunwoo_real_cart)
model_files.hyunwoo_full_cart.file_name=package.resourcePath.."hyunwoo_full_cart.wrl"
model_files.hyunwoo_full_cart.mot_file=package.resourcePath.."hyunwoo_full_cart.dof"
model_files.hyunwoo_full_cart.k_p=700 -- Nm/rad --1000 was better than 800
model_files.hyunwoo_full_cart.k_d=30
model_files.hyunwoo_full_cart.clampTorque=400
model_files.hyunwoo_full_cart.clampTorqueID=1400
--model_files.hyunwoo_full_cart.simulationFrameRate=3000
model_files.hyunwoo_full_cart.simulationFrameRate=12000
model_files.hyunwoo_full_cart.initialHeight=1 -- meters
model_files.hyunwoo_full_cart.k_p_ID=100
model_files.hyunwoo_full_cart.k_d_ID=30
model_files.hyunwoo_full_cart.k_p_PD=100
model_files.hyunwoo_full_cart.k_d_PD=10

do
   local k_scale_active=model_files.hyunwoo_full_cart.k_scale_active
   k_scale_active.default={1,2,0}
   k_scale_active.hip={1,1,1}
   k_scale_active.knee={1,1,1}
   k_scale_active.ankle={1,1,1}
   k_scale_active.shoulder={1,1,1}
   k_scale_active.elbow={1,1,1}

end


model_files.hyunwoo_ball_cart=deepCopyTable(model_files.hyunwoo_full_cart)
model_files.hyunwoo_ball_cart.file_name=package.resourcePath.."hyunwoo_ball_cart.wrl"
model_files.hyunwoo_ball_cart.mot_file=package.resourcePath.."hyunwoo_ball_cart.dof"

model_files.jump=deepCopyTable(model_files.default)
model_files.jump.file_name=package.resourcePath.."jump.wrl"
model_files.jump.bones.left_hip="lfemur"
model_files.jump.bones.right_hip="rfemur"
model_files.jump.bones.left_knee="ltibia"
model_files.jump.bones.right_knee="rtibia"
model_files.jump.bones.left_ball="ltoes"
model_files.jump.bones.right_ball="rtoes"
model_files.jump.bones.left_heel="lfoot"
model_files.jump.bones.right_heel="rfoot"
model_files.jump.bones.chest="lowerback"
model_files.jump.bones.chest2="thorax"
model_files.jump.bones.neck="upperneck"
model_files.jump.bones.head="head"

model_files.jump.penaltyForceStiffness=20000
model_files.jump.penaltyForceDamp=2000
model_files.jump.penaltyForceStiffness_toes=20000
model_files.jump.penaltyForceDamp_toes=2000
model_files.jump.initialHeight=0.01 -- meters
model_files.jump.frame_rate=120
model_files.jump.mot_file=package.resourcePath.."jump.mot"
model_files.jump.start=0
model_files.jump.endFrame=390

model_files.jump2=deepCopyTable(model_files.jump)
model_files.jump2.file_name=package.resourcePath.."jump2.wrl"

model_files.jump3=deepCopyTable(model_files.jump)
model_files.jump3.file_name=package.resourcePath.."jump3.wrl"

model_files.jump4=deepCopyTable(model_files.jump)
model_files.jump4.file_name=package.resourcePath.."jump4.wrl"
model_files.jump4.mot_file=package.resourcePath.."jump4.mot"
model_files.jump4.k_p=1000
model_files.jump4.k_d=10

model_files.jump5=deepCopyTable(model_files.jump4)
model_files.jump5.file_name=package.resourcePath.."jump5.wrl"
model_files.jump5.mot_file=package.resourcePath.."jump5.mot"
model_files.jump5.k_p=1000
model_files.jump5.k_d=20
model_files.jump5.k_d0=5
model_files.jump5.timestep=1/10000
model_files.jump5.penaltyForceStiffness=2000
model_files.jump5.penaltyForceDamp=800

model_files.jump5_cart=deepCopyTable(model_files.jump5)
model_files.jump5_cart.file_name=package.resourcePath.."jump5_cart.wrl"
model_files.jump5_cart.mot_file=package.resourcePath.."jump5_cart.mot"
model_files.jump5_cart.k_p_slide=30000
model_files.jump5_cart.k_d_slide=1000
model_files.jump5_cart.penaltyForceStiffness=30000
model_files.jump5_cart.penaltyForceDamp=600
model_files.jump5_cart.penaltyForceStiffness_toes=30000
model_files.jump5_cart.penaltyForceDamp_toes=600
model_files.jump5_cart.initialHeight=-0.02 -- meters
model_files.jump5_cart.timestep=1/8000


model_files.justin_jump=deepCopyTable(model_files.jump5)
model_files.justin_jump.bones.chest2="upperback"
model_files.justin_jump.bones.neck="LowerNeck"
model_files.justin_jump.bones.head="LowerNeck"
model_files.justin_jump.bones.left_ball=nil
model_files.justin_jump.bones.right_ball=nil
model_files.justin_jump.bones.left_collar="lclavicle"
model_files.justin_jump.bones.right_collar="rclavicle"
model_files.justin_jump.bones.left_heel="lfoot"
model_files.justin_jump.bones.right_heel="rfoot"
model_files.justin_jump.start=22220
model_files.justin_jump.file_name=package.resourcePath.."justin_jump.wrl"
model_files.justin_jump.mot_file=package.resourcePath.."justin_jump.dof"

model_files.woody=deepCopyTable(model_files.default)
model_files.woody.file_name=package.resourcePath.."woody/woody_tall.wrl"
model_files.woody.mot_file=package.resourcePath.."woody/woody.dof"
model_files.woody.bones.left_hip="LeftUpLeg"
model_files.woody.bones.right_hip="RightUpLeg"
model_files.woody.bones.left_knee="LeftLeg"
model_files.woody.bones.right_knee="RightLeg"
model_files.woody.bones.left_ball="LeftToes"
model_files.woody.bones.right_ball="RightToes"
model_files.woody.bones.left_heel="LeftFoot"
model_files.woody.bones.right_heel="RightFoot"
model_files.woody.bones.chest="Spine"
model_files.woody.bones.chest2="Spine1"
model_files.woody.bones.neck="Head"
model_files.woody.bones.head="Head"
model_files.woody.bones.left_collar="LeftShoulder1"
model_files.woody.bones.right_collar="RightShoulder"

model_files.justin_run=deepCopyTable(model_files.justin_jump)
model_files.justin_run.file_name=package.resourcePath.."justin_run.wrl"

model_files.justin_run.mot_file=package.resourcePath.."justin_run.dof"

model_files.justin_jump_cart=deepCopyTable(model_files.justin_jump)
model_files.justin_jump_cart.file_name=package.resourcePath.."justin_jump_cart.wrl"
model_files.justin_jump_cart.mot_file=package.resourcePath.."justin_jump_cart_smooth.dof"
--model_files.justin_jump_cart.mot_file=package.resourcePath.."justin_jump_cart.dof"
model_files.justin_jump_cart.k_p=1000
model_files.justin_jump_cart.k_d=30
model_files.justin_jump_cart.k_p_slide=20000
model_files.justin_jump_cart.k_d_slide=2000
model_files.justin_jump_cart.penaltyForceStiffness=20000
model_files.justin_jump_cart.penaltyForceDamp=2000
model_files.justin_jump_cart.initialHeight=0.025 -- meters
model_files.justin_jump_cart.timestep=1/8000

model_files.justin_runf3=deepCopyTable(model_files.justin_jump)
model_files.justin_runf3.file_name=package.resourcePath.."justin_runf3.wrl"
model_files.justin_runf3.mot_file=package.resourcePath.."justin_runf3.mot"



model_files.justin_straight_run_cart=deepCopyTable(model_files.justin_jump)
model_files.justin_straight_run_cart.file_name=package.resourcePath.."justin_straight_run_cart.wrl"
model_files.justin_straight_run_cart.mot_file=package.resourcePath.."justin_straight_run_cart.dof"
model_files.justin_straight_run_cart.initialHeight=0.03 -- meters
model_files.justin_straight_run_cart.k_p=700 -- Nm/rad --1000 was better than 800
model_files.justin_straight_run_cart.k_d=30
model_files.justin_straight_run_cart.bones.left_heel="lfoot"
model_files.justin_straight_run_cart.bones.left_ball="ltoes"
model_files.justin_straight_run_cart.bones.right_heel="rfoot"
model_files.justin_straight_run_cart.bones.right_ball="rtoes"

model_files.justin_straight_run_cart.clampTorque=800
model_files.justin_straight_run_cart.clampTorqueID=1200
model_files.justin_straight_run_cart.simulationFrameRate=3000
model_files.justin_straight_run_cart.penaltyForceStiffness=15000
model_files.justin_straight_run_cart.penaltyForceDamp=2000
model_files.justin_straight_run_cart.k_p_ID=200
model_files.justin_straight_run_cart.k_d_ID=30
model_files.justin_straight_run_cart.k_p_PD=100
model_files.justin_straight_run_cart.k_d_PD=10

do
   local k_scale_active=model_files.justin_straight_run_cart.k_scale_active
   k_scale_active.default={1,2,0}
   k_scale_active.hip={1,1,1}
   k_scale_active.knee={1,1,1}
   k_scale_active.ankle={1,1,1}
   k_scale_active.shoulder={1,1,1}
   k_scale_active.elbow={1,1,1}

end

model_files.justin_straight_run=deepCopyTable(model_files.justin_straight_run_cart)
model_files.justin_straight_run.file_name=package.resourcePath.."justin_straight_run/justin_straight_run.wrl"
model_files.justin_straight_run.mot_file=package.resourcePath.."justin_straight_run/justin_straight_run.dof"

model_files.gymnist=deepCopyTable(model_files.justin_straight_run)
model_files.gymnist.file_name=package.resourcePath.."gymnist/gymnist.wrl"
model_files.gymnist.mot_file=package.resourcePath.."gymnist/gymnist.dof"
model_files.gymnist.bones.chest="lowerback"
model_files.gymnist.bones.chest2="upperback"
model_files.gymnist.bones.head="Neck"
model_files.gymnist.totalmass=60
model_files.gymnist.start=0
model_files.gymnist.initialHeight=0.07
model_files.gymnist.tiltedGround=transf(quater(math.rad(0.25), vector3(0,0,1)), vector3(0,0,0))

model_files.justin_runf3_cart=deepCopyTable(model_files.justin_straight_run_cart)
model_files.justin_runf3_cart.file_name=package.resourcePath.."justin_runf3_cart.wrl"
model_files.justin_runf3_cart.mot_file=package.resourcePath.."justin_runf3_cart.dof"
model_files.justin_runf3_cart.bones.left_ball=nil
model_files.justin_runf3_cart.bones.right_ball=nil


model_files.justin_run_cart=deepCopyTable(model_files.justin_jump_cart)
model_files.justin_run_cart.file_name=package.resourcePath.."justin_run_cart.wrl"
model_files.justin_run_cart.mot_file=package.resourcePath.."justin_run_cart.dof"
model_files.justin_run_cart.bones.neck=nil
model_files.justin_run_cart.frame_rate=120
model_files.justin_run_cart.k_p=800
model_files.justin_run_cart.k_d=40
model_files.justin_run_cart.clampTorque=300
model_files.justin_run_cart.clampForce=3000
model_files.justin_run_cart.frame_rate=120
--model_files.hyunwoo_real_cart.simulationFrameRate=8400
model_files.justin_run_cart.simulationFrameRate=12000
model_files.justin_run_cart.penaltyForceStiffness=20000
model_files.justin_run_cart.penaltyForceDamp=2000

do
   local k_scale_active=model_files.justin_run_cart.k_scale_active
   k_scale_active.default={0.5,2,0.2}
   k_scale_active.hip={1,1,1}
   k_scale_active.knee={1,1,1}
   k_scale_active.ankle={1,1,1}
   k_scale_active.shoulder={1,1,1}
   k_scale_active.elbow={1,1,1}
end



function model_files.findModel(file_name)
   local model=nil

   for k,v in pairs(model_files) do

      if type(v)=="table" and v.file_name ~=nil then
	 local len=math.min(string.len(file_name),string.len(v.file_name))-3
	 local a= string.upper(str.right(v.file_name,len))
	 local b= string.upper(str.right(file_name,len)) 
	 print(a,b)
	 if a==b then
	    assert(model==nil)
	    model=v
	 end
      end
   end
   if model==nil then print( "Warning! no model for "..file_name.." is defined in common.lua") end
   return model
end

function createSimulator(simulator)
	local usePenaltyMethod=usePenaltyMethod
	if usePenaltyMethod==nil then
		usePenaltyMethod=true
	end
	Physics.setParameter("usePenaltyMethod", usePenaltyMethod )
   if simulator==simulators.UT and usePenaltyMethod==false then
      return Physics.DynamicsSimulator_UT() 
   elseif simulator==simulators.UT then
      return Physics.DynamicsSimulator_UT_penalty() 
   elseif simulator==simulators.AIST and usePenaltyMethod==false then
      return Physics.DynamicsSimulator_AIST() 
   elseif simulator==simulators.AIST then
      return Physics.DynamicsSimulator_AIST_penalty() 
   elseif simulator==simulators.VP then
      return Physics.DynamicsSimulator_VP_penalty() 
   elseif simulator==simulators.gmbs then
	   if usePenaltyMethod==true then
		   return Physics.DynamicsSimulator_gmbs_penalty() 
	   else
		   if false then
			   return Physics.DynamicsSimulator_gmbs()
		   end
		   if useCase then
			   --assert(not useCase.useBulletColdet)
			   --return Physics.DynamicsSimulator_gmbs(not useCase.useBulletColdet) 
			   return Physics.DynamicsSimulator_gmbs(not useCase.useBulletColdet) 
		   else
			   return Physics.DynamicsSimulator_gmbs(true) -- use simple collision detector
		   end
	   end
   elseif simulator==simulators.gmbs_penalty then
	   return Physics.DynamicsSimulator_gmbs_penalty()
   elseif simulator==simulators.gmbs_QP then
	   return Physics.DynamicsSimulator_gmbs(true) -- use simple collision detector
   elseif simulator==simulators.TRL_penalty then
	   return Physics.DynamicsSimulator_TRL_penalty("libccd")
   elseif simulator==simulators.TRL then
	   --return Physics.DynamicsSimulator_TRL_LCP("libccd")
	   return Physics.DynamicsSimulator_TRL_QP("simple")
	   --return Physics.DynamicsSimulator_TRL_QP('simple') -- use simple collision detector 
	elseif simulator==simulators.Trbdl_penalty then
	   return Physics.DynamicsSimulator_Trbdl_penalty("libccd")
	elseif simulator==simulators.Trbdl_LCP then
	   return Physics.DynamicsSimulator_Trbdl_LCP("libccd")
	elseif simulator==simulators.Trbdl_QP then
	   return Physics.DynamicsSimulator_Trbdl_QP("libccd")
   elseif simulator==simulators.Trbdl_impulse then
	   return Physics.DynamicsSimulator_Trbdl_impulse("libccd")
   else
      return Physics.DynamicsSimulator_SDFAST()
   end
end

function registerContactPairAll(model, loader, floor, simulator, param)
	if not param  then
		param=vectorn ()
		param:setValues(0.5,0.5, model.penaltyForceStiffness or 10000, model.penaltyForceDamp or 1000)
	end
   for i=1,loader:numBone()-1 do
	   for j=1, floor:numBone()-1 do
		   local bone_i=loader:VRMLbone(i)
		   simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(j):name(), param)
	   end
   end
end
function registerContactPairOnly(model, loader, floor, simulator, treeIndices, param)
	if not param  then
		param=vectorn ()
		param:setValues(0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp)
	end
	for ii=0, treeIndices:size()-1 do
		local i=treeIndices(ii)
		local bone_i=loader:VRMLbone(i)
		simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param)
   end
end

function registerContactPair(model, loader, floor, simulator)
   param=vectorn ()
   param:assign({0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp})
   for i=1,loader:numBone()-1 do
      local bone_i=loader:VRMLbone(i)
--      if not collisionTestOnlyAnkle or str_include(bone_i.Nameid, "Ankle") then
	 
      if str_include(bone_i:name(), "Ankle") then
	 simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param)
      end
      --[==[			for j=i+1,loader:numBone()-1 do
      local bone_j=loader:getBoneByTreeIndex(j)
      if bone_j:parent()~=bone_i then
	 simulator:registerCollisionCheckPair(
	    loader:name(),bone_i.NameId,
	    loader:name(),bone_j.NameId,0.5,0.5)
	 --[=[simulator:registerCollisionCheckPair(
	 loader:name(),bone_j.NameId,
	 loader:name(),bone_i.NameId,0.5,0.5)]=]--
end
end--]==]
end
end

function registerContactPairJump(model, loader, floor, simulator)
   param=vectorn ()
   param:assign({0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp})
   param_toes=vectorn ()
   param_toes:assign({0.5,0.5, model.penaltyForceStiffness_toes, model.penaltyForceDamp_toes})
   
   for i=1,loader:numBone()-1 do
      local bone_i=loader:VRMLbone(i)
      if str_include(bone_i:name(), "toes") then
	 print("registering toes")
	 simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param_toes)
      elseif str_include(bone_i:name(), "foot") then
--      else
	 print("registering ", bone_i:name())
	 simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param)
      end
   end
end


function calcDerivative_row(i, dmotionDOF, motionDOF)
   local dmotionDOF_i=dmotionDOF:row(i);
   dmotionDOF_i:sub(motionDOF:row(i+1), motionDOF:row(i)) -- forward difference

   local frameRate=120
   if model then frameRate=model.frame_rate end
   dmotionDOF_i:rmult(frameRate)
   
   assert(not motionDOF.dofInfo or motionDOF.dofInfo:numSphericalJoint()==1) 
   -- otherwise following code is incorrect
   local T=MotionDOF.rootTransformation(motionDOF:row(i))
   local V=T:twist( MotionDOF.rootTransformation(motionDOF:row(i+1)), 1/frameRate)
   dmotionDOF_i:setVec3(0, V.v)
   dmotionDOF_i:setVec3(4, V.w)
end

function calcDerivative_row_fd(i, dmotionDOF, motionDOF)
   local q=quater()
   local v=vector3()
   local dmotionDOF_i=dmotionDOF:row(i);
   dmotionDOF_i:sub(motionDOF:row(i+1), motionDOF:row(i)) -- forward 
   MainLib.VRMLloader.projectAngles(dmotionDOF_i) -- align angles
   dmotionDOF_i:rmult(model.frame_rate)

   local T=motionDOF:row(i):toTransf(0)
   local twist=T:twist(motionDOF:row(i+1):toTransf(0),1/model.frame_rate)
   -- incorrect, but previous results were optimized using this.
   dmotionDOF_i:setVec3(0, twist.v)
   dmotionDOF_i:setVec3(4, twist.w)

   if false then
	   local test=vectorn()
	   motionDOF:calcForwardDerivative(i, test, model.frame_rate)
	   dbg.console()
   end
   if false then
	   -- smoothing
	   if i>0 then
		   dmotionDOF_i:assign(dmotionDOF_i*0.25+dmotionDOF:row(i-1)*0.75)
	   end

	   q:difference(motionDOF:row(i):toQuater(3), motionDOF:row(i+1):toQuater(3))
	   v=q:rotationVector()
	   v:assign(v*(model.frame_rate))
	   dmotionDOF_i:set(3,0);
	   dmotionDOF_i:set(4,v.x);
	   dmotionDOF_i:set(5,v.y);
	   dmotionDOF_i:set(6,v.z);
   end
end

-- central difference
function calcDerivative(motionDOF, discontinuity)
	if discontinuity then
		local dmotionDOF=matrixn()
		dmotionDOF:setSize(motionDOF:numFrames(), motionDOF:numDOF())


		local segFinder=SegmentFinder(discontinuity)

		local cols=dmotionDOF:cols()
		for i=0, segFinder:numSegment()-1 do
			local s=segFinder:startFrame(i)
			local e=segFinder:endFrame(i)

			dmotionDOF:range(s,e,0, cols):assign(calcDerivative_sub(motionDOF:range(s,e)))
		end

		return dmotionDOF
   end
   return calcDerivative_sub(motionDOF)
end

-- works for a matrix of vector3, quater, transf, or motiondof
function calcDerivative_general(mat, discontinuity)
	if discontinuity then
		local dmotionDOF=matrixn()
		dmotionDOF:setSize(mat:rows(), mat:cols())

		local segFinder=SegmentFinder(discontinuity)

		local cols=dmotionDOF:cols()
		for i=0, segFinder:numSegment()-1 do
			local s=segFinder:startFrame(i)
			local e=segFinder:endFrame(i)

			dmotionDOF:range(s,e,0, cols):assign(calcDerivative_sub_general(mat:range(s,e,0,cols)))
		end

		return dmotionDOF
   end
   return calcDerivative_sub_general(motionDOF)
end

-- forward difference 를 사용한다는 점을 제외하면
-- calcDerivative와 동일함
function calcForwardDerivative(motionDOF, discontinuity)
	if discontinuity then
		local dmotionDOF=matrixn()
		dmotionDOF:setSize(motionDOF:numFrames(), motionDOF:numDOF())
		local segFinder=SegmentFinder(discontinuity)

		local cols=dmotionDOF:cols()
		for i=0, segFinder:numSegment()-1 do
			local s=segFinder:startFrame(i)
			local e=segFinder:endFrame(i)

			dmotionDOF:range(s,e,0, cols):assign(calcForwardDerivative(motionDOF:range(s,e)))
		end

		return dmotionDOF
	end
	assert(discontinuity==nil)
	assert(motionDOF~=nil)
	dmotionDOF=matrixn()

	dmotionDOF:setSize(motionDOF:numFrames(), motionDOF:numDOF())

	local frameRate=motionDOF.dofInfo:frameRate()
	assert(motionDOF.dofInfo:numSphericalJoint()==1) 
	for i=1, motionDOF:rows()-2 do
		local dmotionDOF_i=dmotionDOF:row(i);
		dmotionDOF_i:sub(motionDOF:row(i+1), motionDOF:row(i)) -- central difference
		dmotionDOF_i:rmult(frameRate)
		calcDerivative_row(i,dmotionDOF, motionDOF)
		local T1=MotionDOF.rootTransformation(motionDOF:row(i))
		local T2=MotionDOF.rootTransformation(motionDOF:row(i+1))
		local V=T1:twist( T2, 1/frameRate)
		dmotionDOF_i:setVec3(0, V.v)
		dmotionDOF_i:setVec3(4, V.w)
	end

	-- fill in empty rows
	dmotionDOF:row(0):assign(dmotionDOF:row(1))
	dmotionDOF:row(dmotionDOF:rows()-1):assign(dmotionDOF:row(dmotionDOF:rows()-2))
	return dmotionDOF
end

function calcCOMtrajectory_sub(skel, motionDOF)
   local out=vector3N(motionDOF:numFrames())
   for i=0, motionDOF:rows()-1 do
      skel:setPoseDOF(motionDOF:row(i))
      out(i):assign(skel:calcCOM())
   end

   math.filter(out:matView(), 31)
   return out
end

function calcCOMtrajectory(skel, motionDOF, discontinuity)

   if discontinuity then

      local out=vector3N(motionDOF:rows())
      local segFinder=SegmentFinder(discontinuity)

      for i=0, segFinder:numSegment()-1 do
	 local s=segFinder:startFrame(i)
	 local e=segFinder:endFrame(i)

	 out:matView():range(s,e,0,3):assign(calcCOMtrajectory_sub(skel, motionDOF:range(s,e)):matView())
      end

      return out
   end
   return calcCOMtrajectory_sub(skel, motionDOF)
end

function calcDerivative_sub(motionDOF)
   assert(motionDOF~=nil)
   local dmotionDOF=matrixn()
   
   dmotionDOF:setSize(motionDOF:numFrames(), motionDOF:numDOF())
   
   for i=1, motionDOF:rows()-2 do
      calcDerivative_row(i,dmotionDOF, motionDOF)
   end
   
   -- fill in empty rows
   dmotionDOF:row(0):assign(dmotionDOF:row(1))
   dmotionDOF:row(dmotionDOF:rows()-1):assign(dmotionDOF:row(dmotionDOF:rows()-2))
   
   if false then
	   -- check for discontinuities
	   --   motionDOF:drawSignals("motionDOF.bmp",false)
	   --   dmotionDOF:drawSignals("dmotionDOF.bmp",false)

	   for i=1, dmotionDOF:rows()-2 do
		   for j=0, dmotionDOF:cols()-1 do
			   if math.abs(dmotionDOF:get(i+1,j)-dmotionDOF:get(i,j))>20 then
				   print("Discontinuity at frame ", i, ", joint ", j)
			   end
		   end
	   end
   end
   return dmotionDOF
end

function integrate_general(vec, dotvec)
	local frameRate=120
	local timestep=1/frameRate
	local out=vectorn(vec:size())
	
	if vec:size()>=7 then
		if vec:size()>7 then
			out:range(7, vec:size()):add(vec:range(7, vec:size()), dotvec:range(7, dotvec:size())*timestep)
		end
		local T=MotionDOF.rootTransformation(vec)
		local v=liegroup.se3()
		v.v:assign(dotvec:toVector3(0))
		v.w:assign(dotvec:toVector3(4))
		T:integrate(v, timestep)
		MotionDOF.setRootTransformation(out, T)
	elseif vec:size()==4 then
		local q=vec:toQuater(0)
		local delta=quater()
		delta:setRotation(dotvec:toVector3(1)*timestep)
		q:leftMult(delta)
		out:setQuater(0, q)
	else
		out:add(vec, dotvec*timestep)
	end
	return out
end
function calcDerivative_sub_general(mat)
	local frameRate=120
	if mat:cols()>=7 then -- transf deriv
		local motionDOF=mat
		assert(motionDOF~=nil)
		local dmotionDOF=matrixn()

		dmotionDOF:setSize(motionDOF:rows(), motionDOF:cols())

		for i=1, motionDOF:rows()-2 do
			local dmotionDOF_i=dmotionDOF:row(i);
			dmotionDOF_i:sub(motionDOF:row(i+1), motionDOF:row(i-1)) -- central difference
			dmotionDOF_i:rmult(frameRate/2.0)
			local T=MotionDOF.rootTransformation(motionDOF:row(i))
			local V=T:twist( MotionDOF.rootTransformation(motionDOF:row(i+1)), 1/frameRate)
			dmotionDOF_i:setVec3(0, V.v)
			dmotionDOF_i:setVec3(4, V.w)
		end

		-- fill in empty rows
		dmotionDOF:row(0):assign(dmotionDOF:row(1))
		dmotionDOF:row(dmotionDOF:rows()-1):assign(dmotionDOF:row(dmotionDOF:rows()-2))
		return dmotionDOF
	elseif mat:cols()==4 then --quater deriv
		local dmotionDOF=matrixn(mat:rows(),4)
		local motionDOF=mat:quatViewCol(0):copy()
		motionDOF:align()
		local q=quater()
		for i=1, mat:rows()-2 do
			q:difference(motionDOF(i-1), motionDOF(i+1))
			local v=q:rotationVector()
			v:assign(v*(frameRate/2.0))
			dmotionDOF:row(i):set(0,0);
			dmotionDOF:row(i):setVec3(1,v);
		end
		-- fill in empty rows
		dmotionDOF:row(0):assign(dmotionDOF:row(1))
		dmotionDOF:row(dmotionDOF:rows()-1):assign(dmotionDOF:row(dmotionDOF:rows()-2))
		return dmotionDOF
	else -- linear deriv
		return mat:derivative(frameRate)
	end
end

function calcAcceleration(motionDOF)
	local dmot= calcDerivative(motionDOF)
	ddmotionDOF=dmot:derivative(120)
   return ddmotionDOF
end

function convertJointNameToIndex(bones, skel)

   for k,v in pairs(bones) do
      if type(v)=="string" then
	 bones[k]=skel:getBoneByName(v):treeIndex()
      end
   end
end

function convertDOFindexToJointName(dofInfo, dofIndex)
   for i=1, dofInfo:skeleton():numBone()-1 do
      local bone=dofInfo:skeleton():bone(i)
      local vbone=bone:treeIndex()
      local nJoint=dofInfo:numDOF(vbone)
      for j=0, nJoint-1 do
	 if dofIndex==dofInfo:DOFindex(vbone,j) then
	    print(bone:name(),j)
	    return
	 end
      end
   end
end

function convertJointNameToBone(bones, skel)

   for k,v in pairs(bones) do
      if type(v)=="string" then
--	 bones[k]=skel:getBoneByName(v)
	 bones[k]=skel:VRMLbone(skel:getBoneByName(v):treeIndex())
      end
   end
end


modelChooser={}
function modelChooser:createMenu(title)
	this:create("Choice", title)

	local ignore={default=true, cart_pole=true, lowerbody=true, lowerbody_allDOF1=true, lowerbody_foot=true,
	lowerbody_elastic=true,hyunwoo_allDOF1=true,hyunwoo_fourDOF3=true,
	jump=true, jump2=true, jump3=true, jump4=true, jump5=true, 
	justin_jump2=true, justin_run=true, justin_straight_run=false}
	function ignorePattern(k,v)
		if select(1,string.find(k, "_cart") )then return true
		elseif select(1,string.find(k, "justin_run") )then return true
		end
		return false
	end

	local shortcuts={gymnist='FL_CTRL+g', hanyang_lowdof_T='FL_CTRL+h'}
	local n=0
	local function isIgnore(k,v)
		if ignore[k]==true or type(v)=='function' then
			return true
		end
		return false
	end
	table.foreach(model_files, function(k,v)  
		if not isIgnore(k,v) and not ignorePattern(k,v) then 
			n=n+1 
		end 
	end)
	this:widget(0):menuSize(n+1)
	this:widget(0):menuItem(0, title)
	n=1
	self.models={}
	for k,v in pairs(model_files) do
		if not isIgnore(k,v) and not ignorePattern(k,v) then
			if shortcuts[k] then
				this:widget(0):menuItem(n, k, shortcuts[k])
			else
				this:widget(0):menuItem(n, k)
			end
			self.models[n]=v
			n=n+1
		end
	end
	this:widget(0):menuValue(0)

end
function modelChooser:createUseCaseMenu(title)
   if useCases then
      this:create("Choice", title)
      
      local n=0
      table.foreach(useCases, function(k,v) if type(v)=="table" then n=n+1 end end)

      this:widget(0):menuSize(n+1)
      this:widget(0):menuItem(0, title)
      local n=1
      self.useCases={}
      for k,v in pairs(useCases) do
	 if type(v)=="table" then
	    this:widget(0):menuItem(n, k)
	    self.useCases[n]=v
	    n=n+1
	 end
      end
   end
end


function calcFirstAndSecondDerivatives(pos,frameRate)
   local timing=vectorn()
   timing:linspace(0,(pos:rows()-1)/frameRate, pos:rows())
   local curveFit=math.NonuniformSpline(timing, pos)
   
   local vel=matrixn()
   local acc=matrixn()
   curveFit:getFirstDeriv(timing, vel)
   curveFit:getSecondDeriv(timing, acc)
   return vel,acc
   
end

function calcAngularVelocity(in_qarray, out_v3array, frame_rate)
   assert(in_qarray:size()==out_v3array:size())

   local q=quater()
   for i=0,in_qarray:size()-2 do
      q:difference(in_qarray(i),in_qarray(i+1))
      out_v3array(i):rotationVector(q)
      out_v3array(i):scale(frame_rate)
   end
   out_v3array(  in_qarray:size()-1):assign(out_v3array(in_qarray:size()-2))
end

function _exportBVH(chosenFile, identitypose, initialHeight, downSampleMethodNearest, _dofScale, _dofRot)
	if mMotionDOFcontainer~=nil and chosenFile~=nil then
		local dofScale=_dofScale or 0.01 -- millimeters to meters
		--local dofScale=1
		local dofRot=_dofRot or quater(math.rad(-90), vector3(0,1,0))
		--local dofRot=quater(math.rad(0), vector3(0,1,0))
		*quater(math.rad(-90), vector3(1,0,0)) -- change to Y_up

		local tempLoader1=MainLib.VRMLloader(model_path..model_file..".wrl")
		local tempLoader=MainLib.VRMLloader(model_path..model_file..".wrl")

		local mot=Motion(tempLoader)
		mMotionDOFcontainer.mot:get(mot)
		mot:initSkeleton(tempLoader)

		if initialHeight then
			print("initialHeight=",initialHeight)
			for i=0, mot:pose(0).translations:size()-1 do
				mot:pose(0).translations(i):radd(vector3(0,initialHeight,0))
			end
		end
		--tempLoader.mMotion:assign(mot) -- export at 120hz
		do
			-- copy downsampled to 60hz
		 	local nf=math.floor(mot:numFrames()/2)
			assert(nf*2<=mot:numFrames())

			tempLoader.mMotion:init(mot, 0, nf)
			if downSampleMethodNearest then
				print('exporting BVH using nearest sampling')
				for i=0, nf-1 do
					tempLoader.mMotion:pose(i):assign(mot:pose(i*2))
				end
			else
				for i=0, nf-1 do
					tempLoader.mMotion:pose(i):blend(mot:pose(i*2), mot:pose(i*2+1), 0.5)
				end
			end
		end

		rotateSkeleton(tempLoader, tempLoader1,dofRot:inverse())
		mot=tempLoader1.mMotion

		-- convert to rendering space (centimeters, Z_up)
		mot:scale(1/dofScale)

		for i=1, mLoader:numBone()-1 do
			local bone=mLoader:bone(i)
			local tgtbone=mot:skeleton():getBoneByName(bone:name())
			if bone:childHead()==nil and bone:numChannels()~=0 then
				local  com=mLoader:VRMLbone(i):localCOM()*(1/dofScale)

				com:rotate(dofRot:inverse())

				mot:skeleton():insertChildBone(tgtbone, "SITE", false)

				print(bone:name(), mLoader:numBone(), mot:skeleton():numBone(), tempLoader:numBone())
				tgtbone:childHead():getOffsetTransform().translation:assign(
				com)
			end
		end

		for i=1, mot:skeleton():numBone()-1 do
			local bone=mot:skeleton():bone(i)
			local trans=bone:getTranslationalChannels()
			local rot=bone:getRotationalChannels()
			if rot~=nil and string.len(rot)~=0 then
				rot="ZXY"
			end

			if trans~=nil and string.len(trans)~=0 then
				trans="XYZ"
			end

			if rot==nil then rot='' end
			if trans==nil then trans='' end

			bone:setChannels(trans, rot)
		end
		if identitypose then
			for i=0, mot:pose( 0).rotations:size()-1 do
				mot:pose(0).rotations(i):identity()
			end
			for i=0, mot:pose(0).translations:size()-1 do
				mot:pose(0).translations(i):zero()
			end
		end
		MotionUtil.exportBVH(mot, chosenFile, 0, mot:numFrames())

	end

end

function _exportBVH2(chosenFile, identitypose, initialHeight, downSampleMethodNearest)
	if mMotionDOFcontainer~=nil and chosenFile~=nil then
	--local dofScale=0.01 -- millimeters to meters
		local dofScale=1
--		local dofRot=quater(math.rad(-90), vector3(0,1,0))
		local dofRot=quater(math.rad(0), vector3(0,1,0))
--*quater(math.rad(-90), vector3(1,0,0)) -- change to Y_up

		local tempLoader1=MainLib.VRMLloader(model_path..model_file..".wrl")
		local tempLoader=MainLib.VRMLloader(model_path..model_file..".wrl")

		local mot=Motion(tempLoader)
		mMotionDOFcontainer.mot:get(mot)
		mot:initSkeleton(tempLoader)

		if initialHeight then
			print("initialHeight=",initialHeight)
			for i=0, mot:pose(0).translations:size()-1 do
				mot:pose(0).translations(i):radd(vector3(0,initialHeight,0))
			end
		end
		tempLoader.mMotion:assign(mot) -- export at 120hz
		--do
			-- copy downsampled to 60hz
		--	local nf=math.floor(mot:numFrames()/2)
		--	assert(nf*2<=mot:numFrames())
		--	local nf=mot:numFrames()

		--	tempLoader.mMotion:init(mot, 0, nf)
		--	if downSampleMethodNearest then
		--		print('exporting BVH using nearest sampling')
		--		for i=0, nf-1 do
		--			tempLoader.mMotion:pose(i):assign(mot:pose(i*2))
		--		end
		--	else
		--		for i=0, nf-1 do
		--			tempLoader.mMotion:pose(i):blend(mot:pose(i*2), mot:pose(i*2+1), 0.5)
		--		end
		--	end
		--end

		rotateSkeleton(tempLoader, tempLoader1,dofRot:inverse())
		mot=tempLoader1.mMotion

		-- convert to rendering space (centimeters, Z_up)
		mot:scale(1/dofScale)

		for i=1, mLoader:numBone()-1 do
			local bone=mLoader:bone(i)
			local tgtbone=mot:skeleton():getBoneByName(bone:name())
			if bone:childHead()==nil and bone:numChannels()~=0 then
				local com=mLoader:VRMLbone(i):localCOM()*(1/dofScale)

				com:rotate(dofRot:inverse())

				mot:skeleton():insertChildBone(tgtbone, "SITE", false)

				print(bone:name(), mLoader:numBone(), mot:skeleton():numBone(), tempLoader:numBone())
				tgtbone:childHead():getOffsetTransform().translation:assign(
				com)
			end
		end

		for i=1, mot:skeleton():numBone()-1 do
			local bone=mot:skeleton():bone(i)
			local trans=bone:getTranslationalChannels()
			local rot=bone:getRotationalChannels()
			if rot~=nil and string.len(rot)~=0 then
				rot="ZXY"
			end

			if trans~=nil and string.len(trans)~=0 then
				trans="XYZ"
			end

			if rot==nil then rot='' end
			if trans==nil then trans='' end

			bone:setChannels(trans, rot)
		end
		if identitypose then
			for i=0, mot:pose(0).rotations:size()-1 do
				mot:pose(0).rotations(i):identity()
			end
			for i=0, mot:pose(0).translations:size()-1 do
				mot:pose(0).translations(i):zero()
			end
		end
		MotionUtil.exportBVH(mot, chosenFile, 0, mot:numFrames())

	end

end

function _importBVH(chosenFile, initialheihgt)
	if mMotionDOFcontainer~=nil and chosenFile~=nil then
		local dofScale=0.01 -- millimeters to meters
		local dofRot=quater(math.rad(-90), vector3(0,1,0))
		*quater(math.rad(-90), vector3(1,0,0)) -- change to Y_up

		local bvhloader=RE.createMotionLoader(chosenFile, chosenFile)
		-- rotated skeleton
		local bvhloader1=RE.createMotionLoader(chosenFile, chosenFile)
		rotateSkeleton(bvhloader, bvhloader1,dofRot)

		local mot=bvhloader1.mMotion
		local mot2=Motion(bvhloader)
		-- convert to model space (meter, Y_up)
		mot:scale(dofScale)
		
		if initialHeight then
			print("initialHeight=",initialHeight)
			for i=0, mot:pose(0).translations:size()-1 do
				mot:pose(0).translations(i):radd(vector3(0,-initialHeight,0))
			end
		end
		MotionUtil.upsample(mot2, mot, 2)

	  	local motdof=convertMotionToMotDOF(bvhloader1, mot2, mLoader)

		return motdof
	end
end

function _createEmptyMotion(loader, fn)
	local motionDOFcontainer=MotionDOFcontainer(loader.dofInfo)
	motionDOFcontainer:resize(10)
	for i=0, 9 do
		loader:getPoseDOF(motionDOFcontainer.mot:row(i))
	end
	motionDOFcontainer.mot:matView():column(1):setAllValue(0.95)
	motionDOFcontainer:exportMot(fn)
end

function rotateSkeleton(skel, skel2, rotation)

	skel:updateInitialBone()
	local pose=Pose()
	skel:getPose(pose)
	pose.rotations(0):leftMult(rotation)
	pose.translations(0):rotate(rotation)
	skel:setPose(pose)

	for i=1, skel2:numBone()-1 do
		local offset=skel2:bone(i):getOffsetTransform()
		offset.translation:rotate(rotation)
		offset.rotation:assign(rotation*offset.rotation*rotation:inverse())
	end

	skel2:updateInitialBone()

	local poseTransfer=MotionUtil.PoseTransfer(skel, skel2, "", true)

	skel2.mMotion:initSkeleton(skel2)
	skel2.mMotion:resize(skel.mMotion:numFrames())
	for i=0, skel.mMotion:numFrames()-1 do
		local srcpose=skel.mMotion:pose(i)
		local tgtpose=skel2.mMotion:pose(i)
		srcpose.rotations(0):leftMult(rotation)
		srcpose.translations(0):rotate(rotation)

		poseTransfer:setTargetSkeletonBothRotAndTrans(srcpose)
		skel2:getPose(tgtpose)

		if i==0 then
			print(srcpose.rotations(0), tgtpose.rotations(0))
			for kk=0,tgtpose:numTransJoint()-1 do
				print(srcpose.translations(kk),tgtpose.translations(kk))
			end
		end
	end      
end

VRMLloaderViewHelper=LUAclass()
function VRMLloaderViewHelper:__init(src_simulator, bone_voca, local_pos, do_not_create_simulator)
	self.src_simulator=src_simulator
	self.src_skel=src_simulator:skeleton(0)
	if bone_voca==MotionLoader.HIPS then
		self.skel=MainLib.VRMLloaderView(self.src_skel, self.src_skel:bone(1), local_pos)
	else
		self.skel=MainLib.VRMLloaderView(self.src_skel, self.src_skel:getBoneByVoca(bone_voca), local_pos)
	end
	MotionLoader.setVoca(self.skel, model.bones)
	if not do_not_create_simulator then
		self.simulator=createSimulator(simulator)
		self.simulator:registerCharacter(self.skel)
		--self.simulator:registerCharacter(self.src_skel)
		self.simulator:setGVector(vector3(0,9.8,0)) 
		self.simulator:init(src_simulator:getTimestep(), Physics.DynamicsSimulator.EULER)
	end
	self.state=vectorn()
	self.dstate=vectorn()
	self.state2=vectorn()
	self.dstate2=vectorn()
end

function VRMLloaderViewHelper:copyStateToSource()
	local state=vectorn()
	local dstate=vectorn()
	local state2=vectorn()
	local dstate2=vectorn()
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, state)
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dstate)

	self.skel:convertDOFexceptRoot(state, state2)
	self.skel:convertDOFexceptRoot(dstate, dstate2)
	local T=self.simulator:getWorldState(0):globalFrame(self.skel._srcRootTransf)*transf(quater(1,0,0,0), self.skel._srcRootOffset)
	MotionDOF.setRootTransformation(state2, T)
	local invQ=T.rotation:inverse()

	dstate2:setVec3(0, rotate(self.simulator:getWorldVelocity(0, self.skel._srcRootTransf, self.skel._srcRootOffset), invQ))
	dstate2:set(3,1)
	dstate2:setVec3(4, rotate(self.simulator:getWorldAngVel(0, self.skel._srcRootTransf), invQ))

	self.src_simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, state2)
	self.src_simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dstate2)
	self.src_simulator:initSimulation()
end

function VRMLloaderViewHelper:_copyStateFromSource(state, dstate)
	local state2=vectorn()
	local dstate2=vectorn()
	if false then
		state:set(1, 1.6)
		self.src_simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, state)
		self.src_simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dstate)
		self.src_simulator:initSimulation()
	end

	self.skel:convertSourceDOFexceptRoot(state, state2)
	self.skel:convertSourceDOFexceptRoot(dstate, dstate2)
	local T=self.src_simulator:getWorldState(0):globalFrame(self.skel._newRootBone)
	MotionDOF.setRootTransformation(state2, T)
	local invQ=T.rotation:inverse()

	dstate2:setVec3(0, rotate(self.src_simulator:getWorldVelocity(0, self.skel._newRootBone, vector3(0,0,0)), invQ))
	dstate2:set(3,1)
	dstate2:setVec3(4, rotate(self.src_simulator:getWorldAngVel(0, self.skel._newRootBone), invQ))

	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, state2)
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dstate2)
	self.simulator:initSimulation()
	return state2, dstate2
end
function VRMLloaderViewHelper:copyStateFromSource()
	local state=vectorn()
	local dstate=vectorn()
	self.src_simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, state)
	self.src_simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dstate)

	self:_copyStateFromSource(state, dstate)
end
do
	Markers=LUAclass()

	-- data members: self.markers (table), self.globalMarkers(matrixn)
	function Markers:__init(skel, chain, markerFile, objectList)
		self.skel=skel

		if not markerFile then
			self.markers=array:new()
			for i=1, skel:numBone()-1 do
				local bone=skel:VRMLbone(i)
				self.markers:pushBack({bone:treeIndex(), bone:localCOM()})
			end
		elseif markerFile=="NONE" then
			self.markers=array:new()
			-- empty set
		else
			assert(os.isFileExist(markerFile))
			self:unpackMarkers(markerFile)
		end
		self.chain=chain
		self.objectList=objectList
		self.markerFile=markerFile
		self.currFrame=0
	end
	function Markers:unpackMarkers(markerfile, fromTextFile)
		if string.sub(markerfile, -4)==".lua" then
			fromTextFile=true
		end
		if fromTextFile then
			self.markers=util.convertFromLuaNativeTable(table.fromstring(util.readFile(markerfile)))
			for i=1, table.getn(self.markers) do
				if self.markers[i][1]=="__AVG" then
					self.markers[i][2]=self.skel:getBoneByName(self.markers[i][2]):treeIndex()
					self.markers[i][4]=self.skel:getBoneByName(self.markers[i][4]):treeIndex()
				else
					self.markers[i][1]=self.skel:getBoneByName(self.markers[i][1]):treeIndex()
				end
			end
		else
			--util.msgBox('binary mrk files are no longer supported')
			self.markers=util.loadTable(markerfile)
			setmetatable(self.markers, {__index=array})
		end
		setmetatable(self.markers, {__index=array})

	end
	function Markers:numMarkers()
		return table.getn(self.markers)
	end
	function Markers:numGlobalMarkers()
		if self.markers.globalMarkers then
			return self.markers.globalMarkers:cols() 
		end
		return 0
	end
	function Markers:findMarker(boneVoca)--returns markerIndex (1-indexing)
		local treeIndex=self.skel:getBoneByVoca(boneVoca):treeIndex()
		for i=1, table.getn(self.markers) do
			if self.markers[i][1]==treeIndex then
				return i
			end
		end
		return -1
	end
	function Markers:calcCenter(points)
		local center=vector3(0,0,0)

		for i=0, self:numMarkers()-1 do
			center:radd(points:toVector3(i*3))
		end
		center:scale(1/self:numMarkers())
		return center
	end
	function Markers:reconnect(skel, chain, objectList)
		self.skel=skel
		self.chain=chain
		self.objectList=objectList
	end
	function Markers:pack(markerFile, outputToTextFile)
		markerFile=markerFile or self.markerFile
		if markerFile then  
			if outputToTextFile then
				local tbl=util.convertToLuaNativeTable(self.markers)
				for i=1, table.getn(self.markers) do
					if tbl[1][1]=="__AVG" then
						tbl[i][2]=self.skel:bone(tbl[i][2]):name()
						tbl[i][4]=self.skel:bone(tbl[i][4]):name()
					else
						tbl[i][1]=self.skel:bone(tbl[i][1]):name()
					end
				end
				util.writeFile( markerFile, table.tostring(tbl))
			else
				util.msgBox("Warning! .mrk file is no longer supported")
			end
		end
	end
	function Markers:onFrameChanged(win, iframe)
		--print(iframe)
		mLoader:setPoseDOF(mMotionDOF:row(iframe))
		self.currFrame=iframe
		self:redraw()
	end
	function Markers:calcMarkerPos(chain, imarker)
		local v=self.markers[imarker]
		if v[1]=="__AVG" then
			local pos1=chain:globalFrame(v[2]):toGlobalPos(v[3])      
			local pos2=chain:globalFrame(v[4]):toGlobalPos(v[5])      
			return (pos1+pos2)*0.5
		else
			local pos=self.chain:globalFrame(v[1]):toGlobalPos(v[2])      
			return pos
		end
	end
	function Markers:redraw()
		if self.objectList then
			for i=1, self.skel:numBone()*2 do
				self.objectList:erase("markers"..tostring(i))
			end
			-- local markers
			for i,v in ipairs(self.markers) do
				dbg.drawSphere(self.objectList, self:calcMarkerPos(self.chain, i)*100, "markers"..tostring(i), nil, 2)

			end   
			-- global markers (such as mocap markers)
			local g=self.markers.globalMarkers
			if g then 
				local np=g:cols()/3
				for j=0,np-1 do
					self.objectList:erase("gmarkers"..tostring(j))
				end
				if self.currFrame<g:rows() then
					local i=self.currFrame
					for j=0,np-1 do
						local pos=vector3(g(i, j*3), g(i,j*3+1), g(i,j*3+2))
						if self.drawOffset then pos=pos+self.drawOffset end
						dbg.drawSphere(self.objectList, pos*100, "gmarkers"..tostring(j), nil, 2)
					end
				end
			end
		end
	end
	function Markers:translateMarker(markerIndex,t)
		local v=self.markers[markerIndex]
		local pos=self.chain:globalFrame(v[1]):toGlobalPos(v[2])
		v[2]:assign(self.chain:globalFrame(v[1]):toLocalPos(pos+t))
		self:redraw(self.objectList)
	end
end
if MotionUtil then
	function MotionUtil.getMergedMesh(tempSkel)
		local mesh=Mesh()
		for i=1, tempSkel:numBone()-1 do
			local bone=tempSkel:VRMLbone(i)
			if bone:hasShape() then
				local gmesh=bone:getMesh():copy()
				gmesh:transform(matrix4(bone:getFrame()))
				mesh:mergeMesh(mesh, gmesh)
			end
		end
		return mesh
	end
	function MotionUtil.calcSkeletonMesh(fn)
		local objFolder=string.sub(fn, 1, -5).."_sd"

		os.createDir(objFolder)
		local tempFile=objFolder.."/temp.wrl"
		local mot=Motion(mLoader)
		mot:resize(1)
		mLoader:getPose(mot:pose(0))
		MotionUtil.exportVRMLforRobotSimulation(mot, tempFile, mLoader:name(), 0.01)
		local tempSkel=MainLib.VRMLloader(tempFile)
		mLoader:printHierarchy()
		tempSkel:printHierarchy()
		tempSkel:setPose(mot:pose(0))
		local mesh=MotionUtil.getMergedMesh(tempSkel)
		return mesh
	end
	function MotionUtil.exportCurrentSurfaceMesh(loader, skeleton_file_name)
		local mesh1=MotionUtil.getMergedMesh(loader)
		local mesh2=MotionUtil.calcSkeletonMesh(skeleton_file_name)

		local dofScale=1.0/100.0 -- millimeters to meters
		local dofRot=quater(math.rad(-90), vector3(0,1,0))
		*quater(math.rad(-90), vector3(1,0,0)) -- change to Y_up

		local t=matrix4()
		t:identity()
		t:leftMultRotation(dofRot:inverse())
		t:leftMultScaling(1/dofScale,1/dofScale,1/dofScale)

		mesh1:transform(t)
		mesh2:transform(t)
		local objFolder=string.sub(skeleton_file_name, 1, -5).."_sd"
		mesh1:saveOBJ(objFolder.."/_surface100.obj",false,false)
		mesh2:saveOBJ(objFolder.."/_skel100.obj",false,false)
	end
end

-- sop.multiMap(value, {1,2,3}, {1,4,2})
function sop.multiMap(a, source, dest)
	-- linear extrapolation, spline interpolation
	if a<source[1] then
		return sop.map(a, source[1], source[2], dest[1], dest[2])
	end
	if a>source[#source] then
		return sop.map(a, source[#source-1], source[#source], dest[#dest-1], dest[#dest])
	end
	local keyTime=CT.vec(source)
	local controlPoints=CT.vec(dest)
	local spline=math.NonuniformSpline(keyTime, controlPoints:column())
	local out=matrixn(1,1)
	local inp=vectorn(1)
	inp:set(0, a)
	spline:getCurve(inp, out)
	return out(0,0)
end
