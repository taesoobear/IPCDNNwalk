require("config")
require("module")
require("common")
require("RigidBodyWin/subRoutines/Constraints")
require("RigidBodyWin/subRoutines/Timeline")



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


function eventFunction()
	--limbik()
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

if EventReceiver then
	--class 'EVR'(EventReceiver)
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		--EventReceiver.__init(self)
		self.currFrame=0
		self.cameraInfo={}
	end

end
function ctor()
	mEventReceiver=EVR()
	mTimeline=Timeline("Timeline", 10000)
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

function getVar(varset)
	return mSolverInfo.var[tonumber(varset)+1]
end

function getConstraintJac_ik(name, varset, jac)
	if name~=varset then
		return false
	end
	local con_frame=tonumber(name)

	local solverInfo=mSolverInfo
	local var=getVar(varset)
	local x=var:getCurrValues()
	local tree=solverInfo.tree
	local mEffectors=solverInfo.effectors
	local nEffector=mEffectors:size()
	tree:setTheta(x)
	tree:computeTree()

	local tree=solverInfo.tree
	local JT=matrixn(x:size(), 3)
	for i=0, nEffector-1 do
		local bone1=mEffectors(i).bone
		local lpos=mEffectors(i).localpos
		tree:calcJacobianTransposeAt(JT, bone1:treeIndex(), lpos)
		jac:sub(0,0, i*3, (i+1)*3):assign(JT)
	end
	return true
end
function getConstraint_ik(name, out)
	local solverInfo=mSolverInfo

	local iframe=tonumber(name)

	local var=getVar(iframe)
	local x=var:getCurrValues()
	local tree=solverInfo.tree
	local mEffectors=solverInfo.effectors
	local nEffector=mEffectors:size()
	tree:setTheta(x)
	tree:computeTree()

	local tree=solverInfo.tree
	for i=0, nEffector-1 do
		local targetPos=solverInfo.target(i):copy()
		if i==0 then
			targetPos.z=targetPos.z+iframe*0.02
		end

		local bone1=mEffectors(i).bone
		local lpos=mEffectors(i).localpos
		local tf=tree:getLastNode(bone1:treeIndex()):globalFrame();
		local deltaS=tf*lpos-targetPos
		out:setVec3(i*3, deltaS)
	end
end

function ik_gradient(iframe, x, grad)
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
		--grad:setAllValue(0)
	end


	if true then
		-- COM constraint
		local loader=mLoader
		local deltaS=solverInfo.COM-tree:calcCOM(loader)
		local w_COM=100
		fx=fx+w_COM*deltaS:dotProduct(deltaS)
		if Ng>0 then
			tree:calcCOMjacobianTranspose(loader, JT)
			assert(Ng==JT:rows());
			for i=0, Ng-1 do
				grad:set(i, grad(i)- 2.0*w_COM*deltaS:dotProduct(JT:row(i):toVector3(0)));
			end
		end
	end

	-- damping terms
	local w_root=1e-2
	local w_other=1e-2

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

function getCost_ik(name)
	local var=getVar(name)
	local x=var:getCurrValues()
	return ik_gradient(tonumber(name), x, nil)
end

function getGradient_ik(name, varname, grad)
	if name==varname then
	else
		return false
	end

	local var=getVar(varname)
	local x=var:getCurrValues()
	assert(grad:size()==x:size() )

	ik_gradient(tonumber(varname), x, grad)
	return true
end
function getCost_acc(name)
	local iframe=tonumber(name)
	local solverInfo=mSolverInfo
	local varp=getVar(iframe-1)
	local var=getVar(iframe)
	local varn=getVar(iframe+1)

	local w_term=0.01
	local invdt=30
	local velp=(var:getCurrValues()-varp:getCurrValues())*invdt
	local veln=(varn:getCurrValues()-var:getCurrValues())*invdt
	local acc= (veln-velp)*invdt
	local cost=(acc:row()*acc:column())(0,0)*w_term
	return cost
end

function getGradient_acc(name, var, grad)

	-- cost=
	-- |   ((x2-x1)*invdt-(x1-x0)*invdt)*invdt   |^2
	-- = |    (x2-2*x1+x0)*invdt^2    |^2
	--
	-- grad=d(f)^2/dx1= d f^2 /df * df/dx1
	--                = 2f * df/dx1

	local iframe=tonumber(name)
	local iframe2=tonumber(var)

	local w_=2.0*30*30   -- 2dfdx
	if iframe2==iframe then
		w_=-2*w_
	elseif iframe2==iframe-1 then
	elseif iframe2==iframe+1 then
	else
		return false
	end
	local solverInfo=mSolverInfo
	local varp=getVar(iframe-1)
	local var=getVar(iframe)
	local varn=getVar(iframe+1)

	local invdt=30
	local w_term=0.01
	local velp=(var:getCurrValues()-varp:getCurrValues())*invdt
	local veln=(varn:getCurrValues()-var:getCurrValues())*invdt
	local acc= (veln-velp)*invdt

	grad:radd((w_*w_term)*acc)
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

	local varsets={}
	local consets={}
	local costsets={}
	local acccostsets={}
	local nf=10

	for i=0, nf-1 do
		varsets[i+1]=solver:addVariableSet(x:size(), tostring(i))
		varsets[i+1]:setInitialValues(x)
		if i==0 or i== nf-1 then
			consets[i+1]=solver:addConstraintSet(numCon*3, tostring(i), 'ik')
		end
		costsets[i+1]=solver:addCostTerm(tostring(i), 'ik')
	end
	for i=1, nf-2 do
		acccostsets[i+1]=solver:addCostTerm(tostring(i), 'acc')
	end

	mSolverInfo.var=varsets
	mSolverInfo.con=consets
	mSolverInfo.cost=costsets
	local opt=TStrings() local optval=vectorn()
	--opt:pushBack('I:print_level') optval:pushBack(5)
	--opt:pushBack('S:linear_solver') opt:pushBack('ma27')
	--opt:pushBack('S:print_user_options') opt:pushBack('no')
	--opt:pushBack("S:derivative_test") opt:pushBack("first-order") opt:pushBack("derivative_test_tol") optval:pushBack(1e-3);
	--opt:pushBack('tol') optval:pushBack(0.003)
	--opt:pushBack('I:max_iter') optval:pushBack(50000)
	--opt:pushBack('max_cpu_time') optval:pushBack(400)
	local res=solver:solve(opt, optval)
	g_mot=matrixn(nf, mPose:size())

	for i=1,nf do

		x:assign(varsets[i]:getCurrValues())
		tree:setTheta(x)
		tree:computeTree()
		tree:getPoseDOF(mLoader.dofInfo, g_mot:row(i-1))
	end

	mPose:assign(g_mot:row(0))
	--print(x- mSolverInfo.x0)

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
	RE.motionPanel():motionWin():changeCurrFrame(0)
	RE.renderer():removeFrameMoveObject(self)
	RE.motionPanel():motionWin():detachSkin(self)
end

function EVR:onFrameChanged(win, iframe)
	if g_mot then
		local iframe=math.fmod(iframe, g_mot:rows())
		mSkin:setPoseDOF(g_mot:row(iframe))
	end
end

function frameMove(fElapsedTime)
end

function handleRendererEvent(ev, button, x,y) 
	if mCON then
		return mCON:handleRendererEvent(ev, button, x,y)
	end
	return 0
end
