print(motionId)

toe_height_thr=3
toe_speed_thr=16
heel_height_thr=5
heel_speed_thr=16
cleanup=0
fillGap=1
checkIKPossible=false
-- remove detected constraints shorter than minConDuration
minConDuration=1

-- shorten detected constraints by reducedcon frames

reduceCon=0

-- gravity=980* model_height/actual_height
actualActorHeight=170 -- in unit length
modelHeight=170 -- in centimeters

-- CHOOSE_AND=0, CHOOSE_OR, CHOOSE_TOE, CHOOSE_HEEL
howToChooseConstraint=2

desired_min_height=0.0 -- desired max height thr???? ?۰ų? ???ƾ? ??.
desired_max_height=1.8
-- ???? ?????? ??????(1.0) ??Ÿ???? ????. 
length_thr=1.1
--??ǥ��ġ???? ?Ÿ??? ��?Ƹ? ?????? x?躸?? ?ָ? ??Ÿ???? ????.
dist_thr=1.0

function str_left(str, n)
	return string.sub(str, 1, n)
end

if str_left(motionId, 3)=="man" then -- taekwondo
	toe_height_thr=1.4
	toe_speed_thr=2

elseif str_left(motionId,9)=="MuayThai1" then
	toe_height_thr=5
	toe_speed_thr=1.5
	desired_min_height=3.8
	desired_max_height=5.8

elseif str_left(motionId,9)=="MuayThai2" then
	toe_height_thr=12
	toe_speed_thr=0.5
	desired_min_height=5.8
	desired_max_height=7.8

elseif str_left(motionId,10)=="locomotion" then
	toe_height_thr=1.8
	toe_speed_thr=0.35
	dist_thr=1.0

end


estimateKernelSize=1
filteringWindowSize=1.0

if str_left(motionId, 12)=="locomotion_h" then
	cleanup=1
end

if str_left(motionId, 8)=="MuayThai" then
	fillGap=0
	minConDuration=2
	reduceCon=2
	actualActorHeight=160
	modelHeight=175
	estimateKernelSize=0
	filteringWindowSize=4.5 -- 1445, 3460
	checkIKPossible=false
	checkIKPossible_lengthGoal=0.95
	checkIKPossible_distGoal=1.0
elseif str_left(motionId, 3)=="man" then
	actualActorHeight=64
	modelHeight=184

elseif str_left(motionId, 7)=="newmana" then
	actualActorHeight=147
	modelHeight=175 
end


