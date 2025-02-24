
VelocityFields=LUAclass()
function VelocityFields:__init(loader, motdofc, initialPose)
	self.skel=loader
	self.dmot=calcDerivative(motdofc.mot, motdofc.discontinuity)
	self.simulator=Physics.DynamicsSimulator_TRL_QP('libccd')
	self.simulator:registerCharacter(self.skel)
	-- timestep: 1/120
	self.simulator:init(1/120, Physics.DynamicsSimulator.EULER)
	if initialPose then
		self.pose=initialPose:copy()
	end
end

function VelocityFields:setInitPose(initialPose)
	self.pose=initialPose:copy();
end

function calcVelocity(p1, p2, frame_rate)
   local q=quater()
   local v=vector3()
   local dmotionDOF_i=p1:copy()
   dmotionDOF_i:sub(p2, p1)
   MainLib.VRMLloader.projectAngles(dmotionDOF_i) -- align angles
   dmotionDOF_i:rmult(frame_rate)

   local T=MotionDOF.rootTransformation(p1)
   local twist=T:twist(MotionDOF.rootTransformation(p2),1/frame_rate)
   -- incorrect, but previous results were optimized using this.
   dmotionDOF_i:setVec3(0, twist.v)
   dmotionDOF_i:setVec3(4, twist.w)
   return dmotionDOF_i
end

function VelocityFields:singleStep(dpose)
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.pose)
	local zero=dpose:copy()
	zero:setAllValue(0)
	local ddq=zero:copy()
	ddq:resize(ddq:size()-1)
	local tau=zero
	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dpose)
	self.simulator:initSimulation()
	self.simulator:stepKinematic(ddq, tau, true)
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.pose)
end
function VelocityFields:stepKinematic(dpose, nextpose, weight)
	self:singleStep(dpose)

	if nextpose then
		local out=zero
		local v=calcVelocity(self.pose, nextpose, 120)
		v:rmult(weight)
		v:clamp(10)
		self:singleStep(v)
		--self.skel.dofInfo:blend(out, self.pose, nextpose, weight)
		--self.pose=out
	end
end
