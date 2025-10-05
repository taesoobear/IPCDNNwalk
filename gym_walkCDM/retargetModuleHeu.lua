
useCache=true
useVelMap=true
function boolN:bitOr( b)
	local c=boolN()
	c:_or(self,b)
	return c
end
function boolN:bitAnd( b)
	local c=boolN()
	c:_and(self,b)
	return c
end
function MotionDOFinfo:DOFtoPose(dof)
	local pose=Pose()
	--self:setDOF(dof, pose)
	self:skeleton():setPoseDOF(dof)
	self:skeleton():getPose(pose)
	return pose
end
function MotionDOFinfo:blendQ(out, dof1, dof2, t)
	local pose1=self:DOFtoPose(dof1)
	local pose2=self:DOFtoPose(dof2)
	local outp=Pose()
	outp:blend(pose1, pose2, t)
	self:getDOF(outp, out)
end

function getVar(varset)
	local varset=mSolverInfo.var[tonumber(varset)+1]
	if varset==nil then
		assert(false)
	end
	if varset[1] then
		return varset[2], true
	else
		return varset[2]:getCurrValues(), false
	end
end

function getVarset(varset)
	local varset=mSolverInfo.var[tonumber(varset)+1]
	if varset==nil then
		assert(false)
	end
	if varset[1] then
		return nil
	else
		return varset[2]
	end
end

function _setVarSet(varsets, iframe, tbl)
	varsets[iframe+1]=tbl
end
function _getVarSet(varset, iframe)
	return varset[iframe+1]
end


function computeTree(solverInfo, tree, con_frame, x, dbgInfo)
	tree:setTheta(x)
	tree:computeTree()
end

function getConstraintJac_ik(name, varset, jac)
	if name~=varset then
		return false
	end
	local con_frame=tonumber(name)

	local solverInfo=mSolverInfo
	local x=getVar(varset)
	local mEffectors=solverInfo.effectors
	local nEffector=mEffectors:size()
	local tree=solverInfo.tree

	computeTree(solverInfo, tree, con_frame, x,'jac_ik')
	local JT=matrixn(x:size(), 3)

	local c=0
	local con=solverInfo.con
	for i=0, nEffector-1 do
		if con[i+1](con_frame) then
			local bone1=mEffectors(i).bone
			local lpos=mEffectors(i).localpos
			tree:calcJacobianTransposeAt(JT, bone1:treeIndex(), lpos)
			local isToe=math.fmod(i,2)==0
			if isToe or (not con[i](con_frame))  then
				-- toe or heel-only 
				jac:sub(0,0, c, c+3):assign(JT)
				c=c+3
			else
				-- heel x,y 
				jac:sub(0,0,c, c+2):assign(JT:sub(0,0,0,2)) 
				c=c+2
			end
		end
	end
	return true
end
function alignAngles(x)
	local delta=0
	local y=x:copy()
	for i=0, x:size() -2 do
		y:set(i, x(i)+delta)
		if x(i)<-math.pi*0.5  and x(i+1)>math.pi*0.5 then
			-- negative clip
			delta=delta-math.pi*2
		elseif x(i)>math.pi*0.5  and x(i+1)<-math.pi*0.5 then
			delta=delta+math.pi*2
		end
	end
	local i=x:size()-1
	y:set(i, x(i)+delta)

	if y:avg()<-math.pi then
		y:radd(2*math.pi)
	end
	if y:avg()>math.pi then
		y:rsub(2*math.pi)
	end
	x:assign(y)

end
function getConstraint_ik(name, out)
	local solverInfo=mSolverInfo

	local iframe=tonumber(name)

	local x=getVar(iframe)
	local tree=solverInfo.tree
	local mEffectors=solverInfo.effectors
	local nEffector=mEffectors:size()
	computeTree(solverInfo, tree, iframe, x, 'con_ik')

	local tree=solverInfo.tree

	local conPos=solverInfo.conPos

	local c=0
	local con=solverInfo.con
	for i=0, nEffector-1 do
		if con[i+1](iframe) then
			assert(conPos[i+1]:rows()>iframe)
			local targetPos=conPos[i+1](iframe):copy()
			local bone1=mEffectors(i).bone
			local lpos=mEffectors(i).localpos
			local tf=tree:getLastNode(bone1:treeIndex()):globalFrame();
			local deltaS=tf*lpos-targetPos
			local isToe=math.fmod(i,2)==0
			if isToe or (not con[i](iframe))  then
				-- toe or heel-only
				out:setVec3(c, deltaS)
				c=c+3
			else
				-- heel x,y
				out:set(c, deltaS.x)
				out:set(c+1, deltaS.y)
				c=c+2
			end
		end
	end
end
function getConstraintJac_relCon(name, varset, jac)
	if name~=varset then
		return false
	end
	local con_frame=tonumber(name)

	local solverInfo=mSolverInfo
	local var=getVar(varset)
	local x=var:getCurrValues()
	if solverInfo.relCon then
		local mEffectors=solverInfo.relCon[1]
		local distX=solverInfo.relCon[2]
		local nEffector=distX:cols()
		local tree=solverInfo.tree

		computeTree(solverInfo, tree, con_frame, x)
		local JT=matrixn(x:size(), 3)
		local JT2=matrixn(x:size(), 3)

		for i=0, nEffector-1 do
			local bone1=mEffectors(i*2).bone
			local lpos=mEffectors(i*2).localpos
			tree:calcJacobianTransposeAt(JT, bone1:treeIndex(), lpos)
			local bone2=mEffectors(i*2+1).bone
			local lpos2=mEffectors(i*2+1).localpos
			tree:calcJacobianTransposeAt(JT2, bone2:treeIndex(), lpos2)

			jac:column(i):assign(JT:column(0)-JT2:column(0))
		end
	else
		local mEffectors=solverInfo.relConZ[1]
		local distZ=solverInfo.relConZ[2]
		local tree=solverInfo.tree

		computeTree(solverInfo, tree, con_frame, x)
		local JT=matrixn(x:size(), 3)
		local JT2=matrixn(x:size(), 3)

		local bone1=mEffectors(0).bone
		local lpos=mEffectors(0).localpos
		tree:calcJacobianTransposeAt(JT, bone1:treeIndex(), lpos)
		local bone2=mEffectors(1).bone
		local lpos2=mEffectors(1).localpos
		tree:calcJacobianTransposeAt(JT2, bone2:treeIndex(), lpos2)

		local JT12=JT:column(2)+JT2:column(2)

		bone1=mEffectors(2).bone
		lpos=mEffectors(2).localpos
		tree:calcJacobianTransposeAt(JT, bone1:treeIndex(), lpos)
		bone2=mEffectors(3).bone
		lpos2=mEffectors(3).localpos
		tree:calcJacobianTransposeAt(JT2, bone2:treeIndex(), lpos2)
		local JT34=JT:column(2)+JT2:column(2)

		jac:column(0):assign(JT12-JT34)
	end
	return true
end
function getConstraint_relCon(name, out)
	local solverInfo=mSolverInfo

	local iframe=tonumber(name)

	local var=getVar(iframe)
	local x=var:getCurrValues()
	local tree=solverInfo.tree
	if solverInfo.relCon then
		local mEffectors=solverInfo.relCon[1]
		local distX=solverInfo.relCon[2]
		local nEffector=distX:cols()
		local tree=solverInfo.tree

		computeTree(solverInfo, tree, iframe, x)

		local tree=solverInfo.tree

		local c=0
		local con=solverInfo.con
		for i=0, nEffector-1 do
			local bone1=mEffectors(i*2).bone
			local lpos=mEffectors(i*2).localpos
			local x1=(tree:getLastNode(bone1:treeIndex()):globalFrame()*lpos).x
			local bone2=mEffectors(i*2+1).bone
			local lpos2=mEffectors(i*2+1).localpos
			local x2=(tree:getLastNode(bone2:treeIndex()):globalFrame()*lpos2).x
			out:set(i, x1-x2-distX(iframe, i))
		end
	else
		local mEffectors=solverInfo.relConZ[1]
		local distZ=solverInfo.relConZ[2]
		local tree=solverInfo.tree

		computeTree(solverInfo, tree, iframe, x)

		local tree=solverInfo.tree

		local c=0
		local con=solverInfo.con
		local bone1=mEffectors(0).bone
		local lpos=mEffectors(0).localpos
		local z1=(tree:getLastNode(bone1:treeIndex()):globalFrame()*lpos).z

		local bone2=mEffectors(1).bone
		local lpos2=mEffectors(1).localpos
		local z2=(tree:getLastNode(bone2:treeIndex()):globalFrame()*lpos2).z

		bone1=mEffectors(2).bone
		lpos=mEffectors(2).localpos
		local z3=(tree:getLastNode(bone1:treeIndex()):globalFrame()*lpos).z

		bone2=mEffectors(3).bone
		lpos2=mEffectors(3).localpos
		local z4=(tree:getLastNode(bone2:treeIndex()):globalFrame()*lpos2).z

		out:set(0, z1+z2-(z3+z4)-distZ(iframe))
	end
end

function getConstraint_com(name, out)
	local solverInfo=mSolverInfo
	local iframe=tonumber(name)
	local x=getVar(iframe)

	local tree=solverInfo.tree
	local mEffectors=solverInfo.effectors
	local nEffector=mEffectors:size()
	computeTree(solverInfo, tree, iframe, x, 'con_com')
	
	-- COM constraint
	local loader=mMot.loader
	local deltaS=tree:calcCOM(loader)-solverInfo.COM:row(iframe):toVector3(0)
	assert(out:size()==3)
	out:setVec3(0, deltaS)
end
function getConstraintJac_com(name, varset, jac)
	if name~=varset then
		return false
	end

	local solverInfo=mSolverInfo
	local iframe=tonumber(name)
	local x=getVar(iframe)

	local tree=solverInfo.tree
	computeTree(solverInfo, tree, iframe, x, 'jac_com')
	
	-- COM constraint
	local loader=mMot.loader
	tree:calcCOMjacobianTranspose(loader, jac)
	return true
end

function ik_gradient(iframe, x, grad)
	local solverInfo=mSolverInfo
	local tree=solverInfo.tree
	local m_x0=solverInfo.x0:row(iframe)
	computeTree(solverInfo, tree, iframe, x, 'ik_gradient')

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


	if solverInfo.options.useCOMcost then
		-- COM constraint
		local loader=mMot.loader
		local deltaS=solverInfo.COM:row(iframe):toVector3(0)-tree:calcCOM(loader)
		local w_COM=solverInfo.w_COM
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
	local w=solverInfo.w_joint

	local SQR=function(x) return x*x end 
	for i=0, N-1 do
		fx=fx+w(i)*SQR(x(i)-m_x0(i));
		if Ng>0 then
			grad:set(i, grad(i)+ w(i)* 2.0*(x(i)-m_x0(i)))
		end
	end
	--print(x, grad, fx)
	return fx
end

function getCost_ik(name)
	local x=getVar(name)
	return ik_gradient(tonumber(name), x, nil)
end

function getGradient_ik(name, varname, grad)
	if name==varname then
	else
		return false
	end

	local x=getVar(varname)
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

	local w_term=solverInfo.w_acc
	local invdt=30
	local velp=(var:getCurrValues()-varp:getCurrValues())*invdt
	local veln=(varn:getCurrValues()-var:getCurrValues())*invdt
	local acc= (veln-velp)*invdt - solverInfo.ddx0:row(iframe)
	acc:range(0,3):rmult(0.001)
	acc:range(3,6):rmult(0.01)
	local cost=(acc:row()*acc:column())(0,0)*w_term
	return cost
end

function getGradient_acc(name, var, grad)

	-- cost=
	-- |   ((x2-x1)*invdt-(x1-x0)*invdt)*invdt   |^2
	-- = |    (x2-2*x1+x0)*invdt^2   -ddx0 |^2
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
	local w_term=solverInfo.w_acc
	local velp=(var:getCurrValues()-varp:getCurrValues())*invdt
	local veln=(varn:getCurrValues()-var:getCurrValues())*invdt
	local acc= (veln-velp)*invdt- solverInfo.ddx0:row(iframe)
	acc:range(0,3):rmult(0.001)
	acc:range(3,6):rmult(0.01)
	grad:radd((w_*w_term)*acc)
	return true
end

function _calcMomentumJT(solverInfo, tree, iframe, x, JT2)
	computeTree(solverInfo, tree, iframe, x)
	local loader=mMot.loader

	if useVelMap then
		-- generalized velocity mapping
		local I=vectorn ();
		local m=tree:calcInertia(loader, I);
		local r=vector3(I(7), I(8), I(9));
		local I6=CT.mat(6,6, 
		I(0),I(3), I(4), 0.0, -r.z, r.y,
		I(3),I(1), I(5), r.z, 0.0,  -r.x,
		I(4),I(5), I(2), -r.y, r.x, 0.0 ,
		0.0, r.z, -r.y, m,   0.0 , 0.0,
		-r.z,0.0,  r.x, 0.0,  m,  0.0,
		r.y, -r.x, 0.0, 0.0,  0.0,  m);


		local invI=matrixn()
		invI:inverse(I6)

		local JT=matrixn()
		tree:calcMomentumJacobianTranspose(loader, JT);

		--local COMJT=matrixn()
		--tree:calcCOMjacobianTranspose(loader, COMJT);
		--dbg.console()

		-- invI*j*dq 
		-- J2 = invI*j
		-- JT2 = jt*invI'

		JT2:multABt(JT, invI);
	else
		-- momentum mapping
		tree:calcMomentumJacobianTranspose(loader, JT2);
		--local x_axes=solverInfo.x_axes
		--for i=0, x_axes:size()-1 do
		--	if not x_axes(i) then
		--		JT2:row(i):zero()
		--	end
		--end
	end

end
if useCache then
	function updateCache(solverInfo, tree, iframe, x0)
		local JT=matrixn()
		_calcMomentumJT(solverInfo, tree, iframe, x0, JT)
		g_cache={iframe, JT}
	end

	function calcMomentumJT(solverInfo, tree, iframe, x, JT)
		local iframeJ, JT2=unpack(g_cache)
		--assert(math.abs(iframeJ-iframe)<=1.1)
		assert(iframe==iframeJ or iframe==iframeJ-1)
		JT:assign(JT2)
	end
else
	function updateCache(solverInfo, tree, iframe, x0)
	end
	calcMomentumJT=_calcMomentumJT
end
function getCost_vel(name)
	local iframe=tonumber(name)
	local solverInfo=mSolverInfo
	local var=getVar(iframe)
	local varn=getVar(iframe+1)

	local invdt=30
	local JT2=matrixn()
	calcMomentumJT(solverInfo, solverInfo.tree, iframe, var, JT2)

	local cost=0
	do
		local w_term=solverInfo.options.w_vel
		local vel=(varn-var)*invdt 

		local deltaS=vectorn(6);
		deltaS:column():multAtB(JT2, vel:column())
		local _vel=solverInfo.bar_comvel:row(iframe)

		deltaS:rsub(_vel) -- invI*Hcom - desired_vel


		cost=cost+(deltaS:row()*deltaS:column())(0,0)*w_term;
	end
	local w_term=solverInfo.options.w_jointvel
	if  w_term>0 then
		local vel=(varn-var)*invdt  -solverInfo.dx0:row(iframe)
		--local vel=(varn-var)  -solverInfo.dx0:row(iframe)*(1.0/invdt)

		cost=cost+(vel:row()*vel:column())(0,0)*w_term;
		--local deltaS=vectorn(6);
		--deltaS:column():multAtB(JT2, vel:column())

		--cost=cost+(deltaS:row()*deltaS:column())(0,0)*w_term;
	end
	return cost
end

function getGradient_vel(name, var, grad)

	-- cost=
	-- |   ((x2-x1)*invdt -dx0 |^2
	--
	-- grad=d(f)^2/dx1= d f^2 /df * df/dx1
	--                = 2f * df/dx1

	local iframe=tonumber(name)
	local iframe2=tonumber(var)

	local w_=2.0*30 -- 2dfdx
	if iframe2==iframe then
		w_=-1*w_
	elseif iframe2==iframe+1 then
	else
		return false
	end
	local solverInfo=mSolverInfo
	local var=getVar(iframe)
	local varn=getVar(iframe+1)

	local invdt=30
	
	local JT2=matrixn()
	calcMomentumJT(solverInfo, solverInfo.tree, iframe, var, JT2)

	do 
		local w_term=solverInfo.options.w_vel
		local vel=(varn-var)*invdt -- - solverInfo.dx0[iframe]
		local deltaS=vectorn(6);
		deltaS:column():multAtB(JT2, vel:column())
		local _vel=solverInfo.bar_comvel:row(iframe)
		deltaS:rsub(_vel) -- invI*Hcom - desired_vel
		--deltaS:range(3,6):setAllValue(0) -- turn off lin

		local N=grad:size();

		for i=0, N-1 do
			grad:set(i, grad(i)+ (w_*w_term)*(deltaS:dotProduct(JT2:row(i))));
		end
	end
	local w_term=solverInfo.options.w_jointvel
	if  w_term>0 then
		local vel=(varn-var)*invdt - solverInfo.dx0:row(iframe)
		--local vel=(varn-var)  -solverInfo.dx0:row(iframe)*(1.0/invdt)
		local N=grad:size();
		for i=0, N-1 do
			grad:set(i, grad(i)+ w_*w_term* vel(i))
		end

		--local deltaS=vectorn(6);
		--deltaS:column():multAtB(JT2, vel:column())

		--local N=grad:size();

		--for i=0, N-1 do
		--	grad:set(i, grad(i)+ (w_*w_term)*(deltaS:dotProduct(JT2:row(i))));
		--end
	end


	return true
end
function getCost_mm(name)
	local iframe=tonumber(name)
	local solverInfo=mSolverInfo
	local var=getVar(iframe)

	local invdt=30
	local JT2=matrixn()
	calcMomentumJT(solverInfo, solverInfo.tree, iframe, var, JT2)

	local cost=0
	local w_term=solverInfo.options.w_mm
	if  w_term>0 then
		local vel=(var-solverInfo.x0:row(iframe))*invdt  
		local deltaS=vectorn(6);
		deltaS:column():multAtB(JT2, vel:column())

		cost=cost+(deltaS:row()*deltaS:column())(0,0)*w_term;
	end
	return cost
end

function getGradient_mm(name, var, grad)

	-- cost=
	-- |   ((x2-x1)*invdt -dx0 |^2
	--
	-- grad=d(f)^2/dx1= d f^2 /df * df/dx1
	--                = 2f * df/dx1

	local iframe=tonumber(name)
	local iframe2=tonumber(var)

	local w_=2.0*30 -- 2dfdx
	if iframe2==iframe then
	else
		return false
	end

	local solverInfo=mSolverInfo
	local var=getVar(iframe)

	local invdt=30
	
	local JT2=matrixn()
	calcMomentumJT(solverInfo, solverInfo.tree, iframe, var, JT2)

	local w_term=solverInfo.options.w_mm
	if  w_term>0 then
		local vel=(var-solverInfo.x0:row(iframe))*invdt 
		--local vel=(varn-var)  -solverInfo.dx0:row(iframe)*(1.0/invdt)
		local deltaS=vectorn(6);
		deltaS:column():multAtB(JT2, vel:column())

		local N=grad:size();

		for i=0, N-1 do
			grad:set(i, grad(i)+ (w_*w_term)*(deltaS:dotProduct(JT2:row(i))));
		end
	end


	return true
end


function valueUpdated_ik(name )
end
function valueUpdated_com(name )
end

function retarget(options, mot, con, conPos, cdm,sf)

	if useCache then
		g_cache={}
	end
	-- retarget
	local mLoader=mMot.loader
	--mLoader:setPoseDOF(mPose);

	local numCon=mSolverInfo.numCon
	local nf=mot:rows()
	local tree=mSolverInfo.tree
	local solverInfo=mSolverInfo
	local N=tree:nDOFinTree()
	mSolverInfo.x0=matrixn(nf, N)
	mSolverInfo.dx0=matrixn(nf, N)
	mSolverInfo.bar_comvel=matrixn(nf-1, 6)
	mSolverInfo.ddx0=matrixn(nf, N)

	if true then
		local function thinCon(v)
			local a=intIntervals()
			a:runLengthEncode( v)
			for j=0, a:size()-1 do
				local sj=a:startI(j)
				local ej=a:endI(j)
				if sj~=0 then
					v:set(sj, false)
				end
				if ej~=v:size() then
					v:set(ej-1, false)
					v:set(ej-2, false)
				end
			end
		end
		-- con thinning
		for i, v in ipairs(con) do
			thinCon(v)
		end
	end
	mSolverInfo.con=con
	mSolverInfo.conPos=conPos
	mSolverInfo.COM=cdm
	mSolverInfo.w_acc=options.w_acc
	mSolverInfo.w_COM=options.w_COM
	mSolverInfo.w_joint=vectorn(N)
	mSolverInfo.w_joint:range(0,6):setAllValue(options.w_root)
	mSolverInfo.w_joint:slice(6,0):setAllValue(options.w_other)
	mSolverInfo.options=options



	--local nf=math.floor(mot:rows()/2)
	-- add variables
	for i=0, nf-1 do
		-- convert quaternion to euler angles
		local pose=mot:row(i)

		tree:setPoseDOF(mLoader.dofInfo, pose)
		tree:computeTree()
		local x=vectorn(N)
		tree:getTheta(x)
		solverInfo.x0:row(i):assign(x)
	end
	if false then
		tree:setPoseDOF(mLoader.dofInfo, mot:row(0))
		local test=boolN()
		tree:findAxes(test, vector3(1,0,0))
		test:range(0,6):setAllValue(true)
		local w=mSolverInfo.w_joint
		for i=0, test:size()-1 do
			if test(i)==false then
				w:set(i, w(i)*100)
			end
		end
		solverInfo.x_axes=test
	end

	if true then
		-- align euler angles
		local x0=solverInfo.x0
		local x=x0:column(3)
		local y=x0:column(4)
		local z=x0:column(5)
		alignAngles(x)
		alignAngles(y)
		alignAngles(z)
		math.changeChartPrecision(30) -- height of a panel
		RE.motionPanel():scrollPanel():addPanel(x)
		RE.motionPanel():scrollPanel():setLastPanelXOffset(sf)
		RE.motionPanel():scrollPanel():addPanel(y)
		RE.motionPanel():scrollPanel():setLastPanelXOffset(sf)
		RE.motionPanel():scrollPanel():addPanel(z)
		RE.motionPanel():scrollPanel():setLastPanelXOffset(sf)

		--util.saveTableToLua({x:copy(),y:copy(),z:copy()}, 'jumps.lua')
		--util.saveTableToLua({mot:sub(0,0,3,7):copy()}, 'jumpsQuat.lua')
		--return nil
	end


	local angleBound={vectorn(), vectorn()}
	angleBound[1]:minimum(solverInfo.x0)
	angleBound[2]:maximum(solverInfo.x0)
	local sf=1.5
	local zero=CT.zeros(solverInfo.x0:cols())
	angleBound[1]:minimum(CT.matrixn( angleBound[1], zero))
	angleBound[2]:maximum(CT.matrixn( angleBound[2], zero))

	local indices={}
	local indices2={1,1,2,1, 1,2,1,2}
	local angleOffset={0,0,10,-10, 0,0,0,0}
	table.insert(indices, tree:getVarIndexByAxis(mLoader:getTreeIndexByName('LeftKnee'), 'X'))
	table.insert(indices, tree:getVarIndexByAxis(mLoader:getTreeIndexByName('RightKnee'), 'X'))
	table.insert(indices, tree:getVarIndexByAxis(mLoader:getTreeIndexByName('LeftHip'), 'Z'))
	table.insert(indices, tree:getVarIndexByAxis(mLoader:getTreeIndexByName('RightHip'), 'Z'))
	table.insert(indices, tree:getVarIndexByAxis(mLoader:getTreeIndexByName('LeftHip'), 'Y'))
	table.insert(indices, tree:getVarIndexByAxis(mLoader:getTreeIndexByName('LeftHip'), 'Y'))
	table.insert(indices, tree:getVarIndexByAxis(mLoader:getTreeIndexByName('RightHip'), 'Y'))
	table.insert(indices, tree:getVarIndexByAxis(mLoader:getTreeIndexByName('RightHip'), 'Y'))

	local angles={}
	for i=1,#indices do
		angles[i]= angleBound[indices2[i]](indices[i])+math.rad(angleOffset[i])
	end

	angleBound[1]:rsub(math.rad(30)/sf)
	angleBound[2]:radd(math.rad(30)/sf)

	angleBound[1]:rmult(sf)
	angleBound[2]:rmult(sf)

	local chestJoint=tree:getVarIndexByAxis(mLoader:getTreeIndexByName('Chest'), 'X')
	angleBound[1]:set(chestJoint, 0)
	for i=1,#indices do
		angleBound[indices2[i]]:set(indices[i],angles[i])
	end

	for i=0, nf-2 do
		local x0=solverInfo.x0
		local invdt=30
		local veln=(x0:row(i+1)-x0:row(i))*invdt
		mSolverInfo.dx0:row(i):assign(veln)
	end
	mSolverInfo.dx0:row(nf-1):assign(mSolverInfo.dx0:row(nf-2))
	for i=1, nf-2 do
		local x0=solverInfo.x0
		local invdt=30
		local velp=(x0:row(i)-x0:row(i-1))*invdt
		local veln=(x0:row(i+1)-x0:row(i))*invdt
		local acc=(veln-velp)*invdt
		local acc_scale=1
		mSolverInfo.ddx0:row(i):assign(acc*acc_scale)
	end
	mSolverInfo.ddx0:row(0):assign(mSolverInfo.ddx0:row(1))
	mSolverInfo.ddx0:row(nf-1):assign(mSolverInfo.ddx0:row(nf-2))

	for i=0, nf-2 do

		local COM=mSolverInfo.COM
		local iframe=i
		local _vel=COM:row(iframe):range(7,COM:cols())*0.5+COM:row(iframe+1):range(7,COM:cols())*0.5

		local _inertia=toYUP0(vector3(10.209,9.683,2.056))
		local _mass=60

		if not useVelMap then
			-- momentum mapping
			local I=matrix3()
			I:setValue(_inertia.x, 0, 0, 0, _inertia.y, 0, 0 ,0,_inertia.z)
			local R=matrix3()
			R:setFromQuaternion(COM:row(iframe):toQuater(3))
			local Is=R*I*R:Transpose()
			_vel:setVec3(0, Is*_vel:toVector3(0))
			_vel:setVec3(3, _mass*_vel:toVector3(3))
		end

		mSolverInfo.bar_comvel:row(i):assign(_vel)
	end

	if options.constrainShoulderDistX or options.constrainHipDistX then
		assert(not options.constrainAnkleShoulderDistZ )
		local bones=MotionUtil.Effectors()
		local c=0
		if options.constrainShoulderDistX then
			c=c+2
		end
		if options.constrainHipDistX then
			c=c+2
		end
		bones:resize(c)
		local c=0
		if options.constrainShoulderDistX then
			bones(c):init(mMot.loader:getBoneByName(input.bones.left_shoulder), vector3(0,0,0))
			bones(c+1):init(mMot.loader:getBoneByName(input.bones.right_shoulder), vector3(0,0,0))
			c=c+2
		end
		if options.constrainHipDistX then
			bones(c):init(mMot.loader:getBoneByName(input.bones.left_hip), vector3(0,0,0))
			bones(c+1):init(mMot.loader:getBoneByName(input.bones.right_hip), vector3(0,0,0))
			c=c+2
		end

		local distX=matrixn(nf, c/2)
		local loader=mMot.loader
		for i=0, nf-1 do
			for j=0, distX:cols()-1 do
				loader:setPoseDOF(mot:row(i))
				distX:set(i, j,  bones(j*2).bone:getFrame().translation.x-bones(j*2+1).bone:getFrame().translation.x)
			end
		end

		mSolverInfo.relCon={bones, distX}
	end
	if options.constrainAnkleShoulderDistZ then
		assert(mSolverInfo.relCon==nil)
		local bones=MotionUtil.Effectors()
		local c=4
		bones:resize(c)

		bones(0):init(mMot.loader:getBoneByName(input.bones.left_shoulder), vector3(0,0,0))
		bones(1):init(mMot.loader:getBoneByName(input.bones.right_shoulder), vector3(0,0,0))
		bones(2):init(mMot.loader:getBoneByName(input.bones.left_ankle), vector3(0,0,0))
		bones(3):init(mMot.loader:getBoneByName(input.bones.right_ankle), vector3(0,0,0))

		local distZ=vectorn(nf)
		local loader=mMot.loader
		for i=0, nf-1 do
			loader:setPoseDOF(mot:row(i))
			distZ:set(i, 
			(bones(0).bone:getFrame().translation.z+ bones(1).bone:getFrame().translation.z)
			-
			(bones(2).bone:getFrame().translation.z+ bones(3).bone:getFrame().translation.z))
		end

		mSolverInfo.relConZ={bones, distZ}
	end
	local mEffectors=mSolverInfo.effectors
	local sparsity
	-- contains which joints to constrain for bound constraint
	for i=0, mEffectors:size()-1 do

		local s=boolN()
		tree:getJacobianSparsity(s, mEffectors(i).bone:treeIndex())

		if not sparsity then
			sparsity=s
		else
			sparsity:_or(sparsity, s)
		end
	end

	RE.motionPanel():scrollPanel():addPanel(solverInfo.con[1], CPixelRGB8(255,255,255))
	RE.motionPanel():scrollPanel():addPanel(solverInfo.con[2], CPixelRGB8(255,255,255))
	--solverInfo.con[2]:setAllValue(false)
	--solverInfo.con[4]:setAllValue(false)
	--local g_mot
	--if true then
	--	g_mot=performForwardPass(mot, angleBound, options, sparsity)
	--else
	--	g_mot=performBackwardPass(mot, angleBound, options, sparsity)
	--end

	local motA=performForwardPass(mot, angleBound, options, sparsity)
	local motB=performBackwardPass(mot, angleBound, options, sparsity)
	local conAll=boolN()
	conAll:_or(con[1], con[2])
	conAll=conAll:bitOr(con[3])
	conAll=conAll:bitOr(con[4])

	local a=intIntervals()
	a:runLengthEncode( conAll)
	assert(a:size()==2)
	local fs=a:endI(0)
	local fe=a:startI(1)

	local g_mot=motA:copy()
	g_mot:sub(fe, g_mot:rows()):assign(motB:sub(fe,g_mot:rows(),0,0))

	local dofInfo=mLoader.dofInfo
	for i=fs, fe do
		local w=sop.map(i, fs-1, fe, 0, 1)
		--dofInfo:blend(g_mot:row(i), motA:row(i), motB:row(i), w)
		dofInfo:blendQ(g_mot:row(i), motA:row(i), motB:row(i), w)
	end

	return g_mot
end

function performForwardPass(mot, angleBound, options, sparsity)
	local mLoader=mMot.loader
	local numCon=mSolverInfo.numCon
	local nf=mot:rows()
	local tree=mSolverInfo.tree
	local solverInfo=mSolverInfo
	local N=tree:nDOFinTree()
	local function itob(b)
		if b then return 1 end
		return 0 
	end

	local g_mot=matrixn(nf, mot:cols())
	local varsets={}
	local consets={}
	local rel_consets={}
	local COMconsets={}
	local costsets={}
	local acccostsets={}
	local velcostsets={}
	do
		-- forward pass

		_setVarSet(varsets, 0, {true, solverInfo.x0:row(0):copy()})-- fixed first frame
		for i=1, nf-1 do
			updateCache(solverInfo, tree, i-1, _getVarSet(varsets, i-1)[2])

			local solver=IPOPTsolver()
			solver("initNLP")

			-- set initial values
			_setVarSet(varsets, i, {false, solver:addVariableSet(N, tostring(i))})
			local x=solverInfo.x0:row(i)
			_getVarSet(varsets, i)[2]:setInitialValues(x)
			_getVarSet(varsets, i)[2]:addBound(angleBound[1], angleBound[2])

			local nc1= itob(solverInfo.con[1](i))*3+ itob(solverInfo.con[2](i))*2
			local nc2= itob(solverInfo.con[3](i))*3+ itob(solverInfo.con[4](i))*2

			if nc1==2 then nc1=3 end
			if nc2==2 then nc2=3 end
			local nc=nc1+nc2

			if nc >0 then
				--if i==0 or i==nf-1 then
				consets[i+1]=solver:addConstraintSet(nc, tostring(i), 'ik')
				--for ii=0, numCon*3-1 do
				--	consets[i+1]:addBound(ii, -0.01, 0.01)
				--end
			end

			if solverInfo.relConZ and nc>0 then
				rel_consets[i+1]=solver:addConstraintSet(1, tostring(i), 'relCon')
				local thr=0.01
				rel_consets[i+1]:addBound(0, -thr, thr)
			end

			if solverInfo.relCon and math.fmod(i,10)==0 then
				local nc=solverInfo.relCon[2]:cols()
				rel_consets[i+1]=solver:addConstraintSet(nc, tostring(i), 'relCon')
				local thr=0.01
				for c=0, nc-1 do
					rel_consets[i+1]:addBound(c, -thr, thr)
				end
			end
			costsets[i+1]=solver:addCostTerm(tostring(i), 'ik')
			if options.useCOMcon then
				COMconsets[i+1]=solver:addConstraintSet(3, tostring(i), 'com')
				local thr=0.05
				COMconsets[i+1]:addBound(0, -thr, thr)
				COMconsets[i+1]:addBound(1, -thr, thr)
				COMconsets[i+1]:addBound(2, -thr, thr)
			end
			if options.useCOMVELcost and i<nf-1 then
				velcostsets[i+1]=solver:addCostTerm(tostring(i-1), 'vel')
			end
			if options.w_mm and options.w_mm>0 then
				velcostsets[i+1]=solver:addCostTerm(tostring(i), 'mm')
			end

			mSolverInfo.var=varsets
			mSolverInfo.consets=consets
			mSolverInfo.costsets=costsets


			--sparsity:set(5, false)

			local function boundFrame(i)
				local nc= itob(solverInfo.con[1](i))+ itob(solverInfo.con[2](i))+ itob(solverInfo.con[3](i))+ itob(solverInfo.con[4](i))
				if nc==0 then
					local v0=getVarset(i)
					local x0=solverInfo.x0:row(i)
					for i=0, sparsity:size()-1 do
						v0:addBound(i, x0(i))
					end
				else
					local v0=getVarset(i)
					local x0=solverInfo.x0:row(i)
					for i=0, sparsity:size()-1 do
						if not sparsity(i) then
							v0:addBound(i, x0(i))
						end
					end
				end
			end
			sparsity:range(3,6):setAllValue(false) -- also prevents rotation drift
			if i==0 and options.boundFirstFrame then
				boundFrame(0)
				--boundFrame(1)
			end
			if i==nf-1 and options.boundLastFrame then
				--boundFrame(nf-2)
				boundFrame(nf-1)
			end

			if options.boundRootX then
				local x0=solverInfo.x0:row(i)
				local v=getVarset(i)
				v:addBound(0, x0(0))
			end

			local opt=TStrings() local optval=vectorn()
			--opt:pushBack('I:print_level') optval:pushBack(5)
			--opt:pushBack('S:linear_solver') opt:pushBack('ma27')
			--opt:pushBack('S:print_timing_statistics') opt:pushBack('yes')
			--opt:pushBack('S:print_user_options') opt:pushBack('no')
			--opt:pushBack("S:derivative_test") opt:pushBack("first-order") opt:pushBack("derivative_test_tol") optval:pushBack(1e-3);
			--opt:pushBack('tol') optval:pushBack(0.003)
			--opt:pushBack('I:max_iter') optval:pushBack(50000)
			opt:pushBack('max_cpu_time') optval:pushBack(400)


			local res=solver:solve(opt, optval)

			_setVarSet(varsets, i,{true, varsets[i+1][2]:getCurrValues():copy()})
		end
		for i=0, nf-1 do
			local x=vectorn(N)
			local vs=_getVarSet(varsets, i)[2]
			assert(vs)
			x:assign(vs)
			tree:setTheta(x)
			tree:computeTree()
			tree:getPoseDOF(mLoader.dofInfo, g_mot:row(i))
		end

	end
	return g_mot
end
function performBackwardPass(mot, angleBound, options, sparsity)
	local mLoader=mMot.loader
	local numCon=mSolverInfo.numCon
	local nf=mot:rows()
	local tree=mSolverInfo.tree
	local solverInfo=mSolverInfo
	local N=tree:nDOFinTree()
	local function itob(b)
		if b then return 1 end
		return 0 
	end

	local g_mot=matrixn(nf, mot:cols())
	local varsets={}
	local consets={}
	local rel_consets={}
	local COMconsets={}
	local costsets={}
	local acccostsets={}
	local velcostsets={}
	do
		-- backward pass
		_setVarSet(varsets,nf-1,{true, solverInfo.x0:row(nf-1):copy()})-- fixed first frame
		for i=nf-2, 0, -1 do
			updateCache(solverInfo, tree, i+1, _getVarSet(varsets, i+1)[2])

			local solver=IPOPTsolver()
			solver("initNLP")

			-- set initial values
			varsets[i+1]={false, solver:addVariableSet(N, tostring(i))}
			local x=solverInfo.x0:row(i)
			_getVarSet(varsets, i)[2]:setInitialValues(x)
			_getVarSet(varsets, i)[2]:addBound(angleBound[1], angleBound[2])

			local nc= itob(solverInfo.con[1](i))+ itob(solverInfo.con[2](i))+ itob(solverInfo.con[3](i))+ itob(solverInfo.con[4](i))
			if nc >0 then
				--if i==0 or i==nf-1 then
				consets[i+1]=solver:addConstraintSet(nc*3, tostring(i), 'ik')
				--for ii=0, numCon*3-1 do
				--	consets[i+1]:addBound(ii, -0.01, 0.01)
				--end
			end

			if solverInfo.relConZ and nc>0 then
				rel_consets[i+1]=solver:addConstraintSet(1, tostring(i), 'relCon')
				local thr=0.01
				rel_consets[i+1]:addBound(0, -thr, thr)
			end

			if solverInfo.relCon and math.fmod(i,10)==0 then
				local nc=solverInfo.relCon[2]:cols()
				rel_consets[i+1]=solver:addConstraintSet(nc, tostring(i), 'relCon')
				local thr=0.01
				for c=0, nc-1 do
					rel_consets[i+1]:addBound(c, -thr, thr)
				end
			end
			costsets[i+1]=solver:addCostTerm(tostring(i), 'ik')
			if options.useCOMcon then
				COMconsets[i+1]=solver:addConstraintSet(3, tostring(i), 'com')
				local thr=0.05
				COMconsets[i+1]:addBound(0, -thr, thr)
				COMconsets[i+1]:addBound(1, -thr, thr)
				COMconsets[i+1]:addBound(2, -thr, thr)
			end
			if options.useCOMVELcost and i<nf-1 then
				velcostsets[i+1]=solver:addCostTerm(tostring(i), 'vel')
			end

			if i>0 and i<nf-1 then
				--varsets[1]:addBound(
				--	acccostsets[i+1]=solver:addCostTerm(tostring(i), 'acc')

				--print(i, acc:length())
			end
			mSolverInfo.var=varsets
			mSolverInfo.consets=consets
			mSolverInfo.costsets=costsets


			--sparsity:set(5, false)

			local function boundFrame(i)
				local nc= itob(solverInfo.con[1](i))+ itob(solverInfo.con[2](i))+ itob(solverInfo.con[3](i))+ itob(solverInfo.con[4](i))
				if nc==0 then
					local v0=getVarset(i)
					local x0=solverInfo.x0:row(i)
					for i=0, sparsity:size()-1 do
						v0:addBound(i, x0(i))
					end
				else
					local v0=getVarset(i)
					local x0=solverInfo.x0:row(i)
					for i=0, sparsity:size()-1 do
						if not sparsity(i) then
							v0:addBound(i, x0(i))
						end
					end
				end
			end
			sparsity:range(3,6):setAllValue(false) -- also prevents rotation drift
			if i==0 and options.boundFirstFrame then
				boundFrame(0)
				--boundFrame(1)
			end
			if i==nf-1 and options.boundLastFrame then
				--boundFrame(nf-2)
				boundFrame(nf-1)
			end

			if options.boundRootX then
				local x0=solverInfo.x0:row(i)
				local v=getVarset(i)
				v:addBound(0, x0(0))
			end

			local opt=TStrings() local optval=vectorn()
			--opt:pushBack('I:print_level') optval:pushBack(5)
			--opt:pushBack('S:linear_solver') opt:pushBack('ma27')
			--opt:pushBack('S:print_timing_statistics') opt:pushBack('yes')
			--opt:pushBack('S:print_user_options') opt:pushBack('no')
			--opt:pushBack("S:derivative_test") opt:pushBack("first-order") opt:pushBack("derivative_test_tol") optval:pushBack(1e-3);
			--opt:pushBack('tol') optval:pushBack(0.003)
			--opt:pushBack('I:max_iter') optval:pushBack(50000)
			--opt:pushBack('max_cpu_time') optval:pushBack(40)


			local res=solver:solve(opt, optval)

			varsets[i+1]={true, varsets[i+1][2]:getCurrValues():copy()}
		end
		for i=0, nf-1 do
			local x=vectorn(N)
			assert(varsets[i+1][2])
			x:assign(varsets[i+1][2])
			tree:setTheta(x)
			tree:computeTree()
			tree:getPoseDOF(mLoader.dofInfo, g_mot:row(i))
		end
	end
	return g_mot
end
