
meshFile=motionId..".mesh"
mappingFile="" -- for maya exported files
bindPoseFile="" -- use the first frame 

function str_left(str, n)
	return string.sub(str, 1,n)
end
function str_include(str, pattern)
	local ss= string.find(str, pattern)

	if ss then return true end
	return false 
end
-- override default options
-- meshFile
if str_left(motionId, 4)=="man1" then 
	meshFile="man1.mesh"
elseif str_left(motionId, 4)=="man2" then 
	meshFile="man2.mesh"
elseif str_left(motionId, 9)=="MuayThai1" then
	meshFile="muaythai1.mesh"
elseif str_left(motionId, 9)=="MuayThai2" then
	meshFile="muaythai2.mesh"
elseif str_left(motionId, 10)=="locomotion" then
	meshFile="hyunwoo.mesh"
elseif string.lower(string.sub(motionId, 1, 6))=="iguana" then
	meshFile="iguana_physics.mesh"
end

-- mappingFile
if str_left(motionId, 3)=="man" or str_left(motionId,10)=="locomotion" then	
	--for max exported files
	mappingFile="../Resource/motion/locomotion_hyunwoo/runhyunwoo.txt"
elseif str_left(motionId, 8)=="MuayThai" then	
	--for max exported files
	mappingFile="../Resource/motion/kickboxer/muaythai.txt"
end

-- bindPoseFile
if str_left(motionId, 9)=="MuayThai1" then
	if str_include(motionId,"COM")==true then
		print("com included")
		bindPoseFile="../Resource/motion/kickboxer/muaythai1com_bind.pose"
	else
		print("com not included")
		bindPoseFile="../Resource/motion/kickboxer/muaythai1_bind.pose"
	end
elseif str_left(motionId, 9)=="MuayThai2" then
	if str_include(motionId,"COM")==true then
		bindPoseFile="../Resource/motion/kickboxer/muaythai2com_bind.pose"
	else
		bindPoseFile="../Resource/motion/kickboxer/muaythai2_bind.pose"
	end
elseif str_left(motionId, 4)=="man1" then
	bindPoseFile="../Resource/motion/kickboxer/man1_bind.pose"
elseif str_left(motionId, 4)=="man2" then
	bindPoseFile="../Resource/motion/kickboxer/man2_bind.pose"
elseif str_left(motionId, 10)=="locomotion" then
	bindPoseFile="../Resource/motion/locomotion_h_bind.pose"
elseif string.lower(str_left(motionId, 6))=="iguana" then
	bindPoseFile="../Resource/motion/iguana_bind.pose"
end

