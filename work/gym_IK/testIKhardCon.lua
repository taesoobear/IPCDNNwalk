require("config")
require("module")
require("common")
require("RigidBodyWin/subRoutines/Constraints")



config_hyunwoo={
	"../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T.wrl",
	"../Resource/motion/locomotion_hyunwoo/hyunwoo_lowdof_T_locomotion_hl.dof",
	{
		{'LeftKnee', 'LeftAnkle', vector3(0, -0.06, 0.08), reversed=false},
		{'RightKnee', 'RightAnkle', vector3(0, -0.06, 0.08), reversed=false},
	},
	skinScale=100,
}


config=config_hyunwoo


function eventFunction(ev)
	if ev=='drag_finished' then
		limbik()
	end
end
function createIKinfo(loader, config)
	local out={}
	local mEffectors=MotionUtil.Effectors()
	local numCon=#config
	mEffectors:resize(numCon);
	out.effectors=mEffectors
	out.numCon=numCon

	for i=0, numCon-1 do
		local conInfo=config[i+1]
		local kneeInfo=1
		local lknee=loader:getBoneByName(conInfo[kneeInfo])
		mEffectors(i):init(loader:getBoneByName(conInfo[kneeInfo+1]), conInfo[kneeInfo+2])
	end
	return out
end

function loadMotion(skel, motion, skinScale)
	local mot={}
	mot.loader=MainLib.VRMLloader (skel)
	mot.motionDOFcontainer=MotionDOFcontainer(mot.loader.dofInfo, motion)
	if skinScale then
		mot.skin=createSkin(skel, mot.loader, skinScale)
		mot.skin:applyMotionDOF(mot.motionDOFcontainer.mot)
		mot.skin:setMaterial('lightgrey_transparent')
	end
	return mot
end

function ctor()
	this:create("Button", "rotate light", "rotate light",1)
	this:create("Value_Slider", "arm ori y", "arm ori y",1)
	this:widget(0):sliderRange(-math.rad(90),math.rad(90))
	this:create("Button", "solve", "solve",1)
	this:widget(0):buttonShortcut('FL_CTRL+s')


	this:updateLayout();

	--dbg.console();
	mMot=loadMotion(config[1], config[2])
	mLoader=mMot.loader

	--local max_iter=mLoader:numBone()
	for i=1, mLoader:numBone()-1 do
		if mLoader:VRMLbone(i):numChannels()==0 then
			mLoader:removeAllRedundantBones()
			--mLoader:removeBone(mLoader:VRMLbone(i))
			--mLoader:export(config[1]..'_removed_fixed.wrl')
			break
		end
	end
	mLoader:_initDOFinfo()


	mMotionDOFcontainer=mMot.motionDOFcontainer
	mMotionDOF=mMotionDOFcontainer.mot

	-- in meter scale
	for i=0, mMotionDOF:rows()-1 do
		mMotionDOF:matView():set(i, 1, mMotionDOF:matView()(i,1)+(config.initialHeight or 0))
	end

	-- rendering is done in cm scale
	mSkin= RE.createVRMLskin(mLoader, true);
	mSkin:setMaterial("lightgrey_transparent")
	local s=config.skinScale
	mSkin:scale(s,s,s); -- motion data often is in meter unit while visualization uses cm unit.
	mSkin:setThickness(3/s)
	mPose=vectorn()
	mPose:assign(mMotionDOF:row(0));
	mSkin:setPoseDOF(mPose);


	mSolverInfo=createIKinfo(mLoader, config[3])
	local numCon=mSolverInfo.numCon
	do 
		-- for python solve
		local useEulerRoot=true
		local useFixedRootPos=false
		--mSolverInfo.tree=MotionUtil.LoaderToTree(mLoader, mEffectors, g_con, useEulerRoot, useFixedRootPos)
		mSolverInfo.tree=MotionUtil.LoaderToTree(mLoader, useEulerRoot, useFixedRootPos) -- effector independent tree.
	end

	footPos=vector3N (numCon);

	mLoader:setPoseDOF(mPose)
	local originalPos={}
	local mEffectors=mSolverInfo.effectors
	for i=0,numCon-1 do
		local opos=mEffectors(i).bone:getFrame():toGlobalPos(mEffectors(i).localpos)
		originalPos[i+1]=opos*config.skinScale
	end
	table.insert(originalPos, mLoader:bone(1):getFrame().translation*config.skinScale) -- desired COM initially unused.
	mCON=Constraints(unpack(originalPos))
	--mCON:setOption(1*config.skinScale)
	mCON:connect(eventFunction)
end

function getConstraintJac_ik(name, varset, jac)
	local solverInfo=mSolverInfo
	local x=mSolverInfo.v1:getCurrValues()
	local tree=solverInfo.tree
	local mEffectors=solverInfo.effectors
	local nEffector=mEffectors:size()
	tree:setTheta(x)
	tree:computeTree()

	local x=mSolverInfo.v1:getCurrValues()
	local tree=solverInfo.tree
	local JT=matrixn(x:size(), 3)
	for i=0, nEffector-1 do
		local targetPos=solverInfo.target(i)
		local bone1=mEffectors(i).bone
		local lpos=mEffectors(i).localpos
		tree:calcJacobianTransposeAt(JT, bone1:treeIndex(), lpos)
		jac:sub(0,0, i*3, (i+1)*3):assign(JT)
	end
	return true
end
function getConstraint_ik(name, out)
	local solverInfo=mSolverInfo
	local x=mSolverInfo.v1:getCurrValues()
	local tree=solverInfo.tree
	local mEffectors=solverInfo.effectors
	local nEffector=mEffectors:size()
	tree:setTheta(x)
	tree:computeTree()

	local x=mSolverInfo.v1:getCurrValues()
	local tree=solverInfo.tree
	for i=0, nEffector-1 do
		local targetPos=solverInfo.target(i)
		local bone1=mEffectors(i).bone
		local lpos=mEffectors(i).localpos
		local tf=tree:getLastNode(bone1:treeIndex()):globalFrame();
		local deltaS=tf*lpos-targetPos
		out:setVec3(i*3, deltaS)
	end
end

function ik_gradient(x, grad)
	local solverInfo=mSolverInfo
	local tree=solverInfo.tree
	local m_x0=solverInfo.x0
	tree:setTheta(x)
	tree:computeTree()

	local mEffectors=solverInfo.effectors
	local nEffector=mEffectors:size()
	local nCol=x:size()
	local nRow=3*nEffector

	--nEffector=0

	local JT=matrixn(nCol, 3)
	JT:setAllValue(0.0)

	local fx = 0.0;
	
	local N=x:size()
	local Ng=0
	if grad then
		Ng=grad:size()
	end
	if Ng>0 then
		assert(grad:size()==x:size())
		grad:setAllValue(0)
	end


	if true then
		-- COM constraint
		local loader=mLoader
		local deltaS=solverInfo.COM-tree:calcCOM(loader)
		fx=fx+deltaS:dotProduct(deltaS)
		if Ng>0 then
			tree:calcCOMjacobianTranspose(loader, JT)
			assert(Ng==JT:rows());
			for i=0, Ng-1 do
				grad:set(i, grad(i)- 2.0*deltaS:dotProduct(JT:row(i):toVector3(0)));
			end
		end
	end

	-- damping terms
	local w_root=0.01
	local w_other=0.01

	local w

	local SQR=function(x) return x*x end 
	for i=0, N-1 do
		if (i<6) then 
			w=w_root; 
		else 
			w=w_other; 
		end
		fx=fx+w*SQR(x(i)-m_x0(i));
		if Ng>0 then
			grad:set(i, grad(i)+ w* 2.0*(x(i)-m_x0(i)))
		end
	end
	--print(x, grad, fx)
	return fx
end

function getCost_ik(name )

	local x=mSolverInfo.v1:getCurrValues()
	return ik_gradient(x, nil)
end

function getGradient_ik(name, var, grad)
	local x=mSolverInfo.v1:getCurrValues()
	assert(grad:size()==x:size())

	ik_gradient(x, grad)
	return true
end
function valueUpdated_ik(name )
end




function limbik()
	mPose:assign(mMotionDOF:row(0));
	mLoader:setPoseDOF(mPose);

	local tree=mSolverInfo.tree
	tree:setPoseDOF(mLoader.dofInfo, mPose)
	tree:computeTree()
	local N=tree:nDOFinTree()
	local x=vectorn(N)
	tree:getTheta(x)

	local COM=mCON.conPos(2)/config.skinScale

	local numCon=mSolverInfo.numCon
	local footPos=vector3N(numCon)
	-- local pos to global pos
	for i=0,numCon-1 do
		local originalPos=mCON.conPos(i)/config.skinScale
		footPos(i):assign(originalPos);
	end
	local x0=x:copy()
	mSolverInfo.x0=x0
	mSolverInfo.target=footPos
	mSolverInfo.COM=COM

	-- IPOPT solve

	local solver=IPOPTsolver()
	solver("initNLP")
	local v1=solver:addVariableSet(x:size(), 'v1')
	v1:setInitialValues(x)
	local con=solver:addConstraintSet(numCon*3, 'ik', 'ik')
	local c1=solver:addCostTerm('ik','ik') -- will be connected to the following lua functions : getCost_ik, getGradient_ik

	mSolverInfo.v1=v1
	local opt=TStrings() local optval=vectorn()
	--opt:pushBack('I:print_level') optval:pushBack(5)
	--opt:pushBack('S:linear_solver') opt:pushBack('ma27')
	--opt:pushBack('S:print_user_options') opt:pushBack('no')
	--opt:pushBack("S:derivative_test") opt:pushBack("first-order") opt:pushBack("derivative_test_tol") optval:pushBack(1e-3);
	--opt:pushBack('tol') optval:pushBack(0.003)
	--opt:pushBack('I:max_iter') optval:pushBack(50000)
	--opt:pushBack('max_cpu_time') optval:pushBack(400)
	local res=solver:solve(opt, optval)
	x:assign(v1:getCurrValues())
	

	tree:setTheta(x)
	tree:computeTree()
	tree:getPoseDOF(mLoader.dofInfo, mPose)

	print(x- mSolverInfo.x0)

	mSkin:setPoseDOF(mPose);

end

function onCallback(w, userData)  

	if w:id()=="button1" then
		print("button1\n");
	elseif w:id()=='rotate light' then
		local osm=RE.ogreSceneManager()
		if osm:hasSceneNode("LightNode") then
			local lightnode=osm:getSceneNode("LightNode")
			lightnode:rotate(quater(math.rad(30), vector3(0,1,0)))
		end
	elseif w:id()=='solve' then
		limbik()
	end
end

function dtor()
end

function frameMove(fElapsedTime)
end

function handleRendererEvent(ev, button, x,y) 
	if mCON then
		return mCON:handleRendererEvent(ev, button, x,y)
	end
	return 0
end
