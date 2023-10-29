# m1 macbook air 기준으로 두시간이면 대략 Updates 900 (run3). 완전 충분. 1시간이면 대충 안정화 됨.
# timeout은mac에서는 "brew install coreutils"
# 학습 강제종료는 pkill timeout으로!
# use bash on ubuntu!!!
echo "---------------------------">> log.txt
# backup training history
mkdir -p runs2
#mv runs/* runs2/ 
function  train2 {
	echo "$1 $2 $3"
date >>log.txt & echo "$3 started">> log.txt
	timeout "$1" python3 gym_cdm2/train_gym.py --env-name "$2" --num-processes 16
	date >>log.txt & echo "$3 finished">> log.txt
}

#train 2h walk2-v6 walk2
#train 2h walkpush-v6 walk3
#train 2h walk3-v8 walk3
#train 1h walk3-v5 walk3
#train 8h walkslope-v7 walk3
#train 8h walk3-v8 walk3
#train 1h walk3-v9 walk3
train2 3h walk2-v1 walk2
train2 2h run2-v1 run2
train2 6h walkterrain-v1 walkterrain
#train 8h walk3-v7 walk3
#train 2h run3-v5 run3
#train 2h jog-v5 jog
#train 2h jog-v6 jog
#train 1h runjump-v5 runjump
#train2 2h run180_1-v1 run180_1
#train2 2h run180_2-v1 run180_2
#train2 2h run180_3-v1 run180_3
#train 2h run90_1-v5 run90_1
#train 2h run90_2-v5 run90_2
#train 2h run90_3-v5 run90_3
#train 2h run90l2-v5 run90l2
#train 2h run90R_1-v5 run90R_1
#train 2h run90R_2-v5 run90R_2
#train 2h run90R_3-v5 run90R_3
#train 2h run90Rl2-v5 run90Rl2
#train 2h run180l2-v5 run180
#train 4h run90-v5 run90
#train 10h multi-v1 multi
#train 10h multi2-v1 multi2
#train 2h fastwalk-v5 fastwalk
#train 2h fastwalk-v6 fastwalk


#make train_jog_deepmimic
