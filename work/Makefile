# siggraph asia 2023: Adaptive Tracking of a Single-Rigid-Body Character in Various Environments
random:
	python3 test_sample_python.py --dostring='spec_id="walk2-v1";random_walk=true' gym_cdm2/test_cdm_deepmimic.lua
	#python3 test_sample_python.py --dostring='spec_id="run2-v1";random_walk=true' gym_cdm2/test_cdm_deepmimic.lua
walk: # does not use gym
	python3 test_sample_python.py --dostring='spec_id="walk2-v1"' gym_cdm2/test_cdm_deepmimic.lua
walk_gym: # uses gym. Other than that, same as the "walk" above
	python3 gym_cdm2/test_gym.py --load-dir trained_models/ppo --env-name "walk2-v1" --device "cpu"
run:
	python3 test_sample_python.py --dostring='spec_id="run2-v1"' gym_cdm2/test_cdm_deepmimic.lua
walkterrain:
	python3 test_sample_python.py --dostring='spec_id="walkterrain-v1"' gym_cdm2/test_cdm_deepmimic.lua
train_walk:
	python3 gym_cdm2/train_gym.py --env-name "walk2-v1" --num-processes 16
train_run:
	python3 gym_cdm2/train_gym.py --env-name "run2-v1" --num-processes 16
postprocess_walk: # necessary once after training
	sh analyze.sh walk4 walk2-v1 60
postprocess_run: # necessary once after training
	sh analyze.sh run4 run2-v1 60
test:
	python3 test_sample_python.py gym_cdm2/testShowRefMotion.lua
urdf:
	python3 test_sample_python.py showRobotFile.lua
# siggraph 2020: Fast and flexible multilegged locomotion using learned centroidal dynamics
walknn:
	python3 test_walk2.py gym_walkCDM/testWalkV2_FA.lua
pushnn:
	python3 test_walk2.py gym_walkCDM/testPush_FA.lua

install_dependencies:
	pip3 install torch python3-tk gym
example_ragdoll:
	python3 test_sample_python.py examples/RagdollFallLCP_selfContained.lua
example_ballfall:
	python3 test_sample_python.py examples/BallFall.lua
example_pendulum:
	python3 test_sample_python.py examples/BallFall2.lua
example_boxfall:
	python3 test_sample_python.py examples/BoxFall.lua

setSeperateWin: # useful when creating high-quality captures for a demo video. 
	cp ../Resource/ogreconfig_linux_sepwin.txt ../Resource/ogreconfig_personal.txt
unsetSeperateWin: # go back to the default layout
	rm -f ../Resource/ogreconfig_personal.txt
