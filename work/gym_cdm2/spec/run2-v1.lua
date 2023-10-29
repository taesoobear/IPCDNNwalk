
require("config")
require("common")
useQPservo_v3=true
require("gym_cdm2/deepmimiccdm-v1")

model=require('gym_cdm2/defaultRLsetting')('run4')
function model:filterContact(li, rotY, mode)
	assert(false)
end

model.cf_option.usefilteredPosForSpportPhaseToo = false

model.maxTurning=0.4
model.facingWeight=10
model.maxLegLen=1.5 --  leg length doesn't work well.
model.unlimitedLeglen=false -- use y instead
model.contactThr=0.1
healthy_y_range={0.8, 1.1} -- actually y (height)
model.IKconfig={
	IKpostprocess=true,
	removeRemainingSliding=false,  -- todo: terrain adaptation
	doNotUseKneeDamping=false,
	wVelRoot=1,
	wMM=0.2,
	wCOMy=0,
	wCOMxz=0,
}
model.terrainHeight=1.0
-- testing...
--disable_IK_unfiltered =true -- causes feet sliding
disable_fix_floating_contactFoot =true -- showHuman.lua: fix contact feet floating above ground (using vertical translation which is dissipated over time)
disable_IK_filtered =true
