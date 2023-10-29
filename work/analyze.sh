date >>log.txt & echo "$1 started">> log.txt
python3 test_sample_python.py --dostring="spec_id='$2';param={collectRefTraj=true,exitAt=$3}" gym_cdm2/test_cdm_deepmimic.lua
cp refTraj.dat gym_cdm2/spec/refTraj_$2.dat
#python3 test_sample_python.py --dostring="param={motionId='$1',envName='$2',exitAt=0}" gym_cdm2/testPoseSampler.lua
python3 test_sample_python.py --dostring="param={motionId='$1',envName='$2',}" gym_cdm2/testPoseSampler.lua
date >>log.txt & echo "$1 finished">> log.txt
