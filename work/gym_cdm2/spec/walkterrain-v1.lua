
require("config")
require("common")
useQPservo_v3=true
--require("gym_cdm2/deepmimiccdm-v1")
require("gym_cdm2/deepmimiccdmterrain-v1")

--model=require('gym_cdm2/defaultRLsetting')('walk3')
model=require('gym_cdm2/defaultRLsetting')('walk4')


function model:filterContact(li, rotY, mode)
	assert(false)
end

draw_option.contactpos=false
draw_option.swingpos=false
model.cf_option.usefilteredPosForSpportPhaseToo = false

healthy_y_range={0.7, 2.0} -- actually y (height)
model.maxLegLen=1.10
model.unlimitedLeglen=false
model.touchDownHeight=0.83 
model.doNotUseConWeight=true
model.EFdif_scale=1.0
QPparam.k_p_ID=120
model.IKconfig={
	IKpostprocess=true,
	removeRemainingSliding=false,  -- todo: terrain adaptation
	doNotUseKneeDamping=false,
	wVelRoot=0.1,
	wMM=0.1,
}
if param and param.collectRefTraj then
	-- for analysis
	model.terrainHeight=0.0
	model.terrainMinHeight=0.0
else
	model.terrainHeight=1.0
	model.terrainMinHeight=0.1
end

model.useTerrain=true
model.randomInitialOri=false
-- testing...
disable_IK_unfiltered =true -- causes feet sliding
disable_fix_floating_contactFoot =true -- showHuman.lua: fix contact feet floating above ground (using vertical translation which is dissipated over time)
disable_IK_filtered =false

if not reset_noise_scale==5e-2 then
	reset_noise_scale_velxz=1
end
--model.delayedVis=1 -- to see raw output (unfiltered).

draw_constraints=false
-- todo 
-- draw_constraints (draw conpos, conori)


--placeObstacle=true
--reset_noise_scale=5e-3
--reset_noise_scale_pos=5e-3
--reset_noise_scale_velxz=5e-3

if model.showHuman then
	require("gym_cdm2/module/showHuman") 
end
