
CollisionChecker=LUAclass()
function CollisionChecker:__init(...)
	local obj={...}
	local colType='libccd'
	--local colType='fcl'
	--local colType='gjk'
	if type(obj[1])=='string' then
		colType=obj[1]
		obj=obj[2] or {}
	end
	self.nbObj=#obj;
	-- in cpp, use OpenHRP::createCollisionDetector_libccd() or ..._bullet()
	if colType=='libccd' then
		self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_libccd()
	elseif colType=='fcl' then
		self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_fcl()
	elseif colType=='gjk' then
		self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_gjk()
	else
		self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_libccd_merged()
	end
	--self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_libccd_LBS()
	--self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_bullet()
	--self.collisionDetector=Physics.CollisionSequence.createCollisionDetector_gjk() -- experimental
	self.collisionSequence=Physics.CollisionSequence()

	self.pose={}
	for i,v in ipairs(obj) do
		local ltype=getmetatable(v).luna_class
		if ltype=='MainLib.VRMLloader' then
			self.collisionDetector:addModel(v)
		else
			self.collisionDetector:addObstacle(v)
		end
	end
end

-- s: from , t: to in cm scale.
function CollisionChecker:checkRayIntersection(s, t)
	local cd=self.collisionDetector
	local r=Physics.RayTestResult ()
	s=s/config.skinScale
	t=t/config.skinScale

	local clostest_T=1.0
	local hitPos
	local hitNormal

	for j=1,self.nbObj do
		local nb=cd:getModel(j-1):numBone()
		for i=1, nb -1 do
			cd:rayTest(j-1, i,s ,t , r)
			--cd:rayTestBackside(j-1, i,s ,t , r)

			if r:hasHit() and r.m_closestHitFraction<clostest_T then
				clostest_T=r.m_closestHitFraction
				local o=vector3()
				o:interpolate(r.m_closestHitFraction, s,t)
				hitPos=o
				hitNormal=r.m_hitNormalWorld:copy()
			end
		end
	end
	return clostest_T~=1.0, hitPos, hitNormal
end
function CollisionChecker:setPoseDOF(iloader, dof)
	local loader=self.collisionDetector:getModel(iloader)
	assert(dof:size()==loader.dofInfo:numDOF())
	loader:setPoseDOF(dof)
	self.collisionDetector:setWorldTransformations(iloader, loader:fkSolver())
	self.pose[iloader]=dof:copy()
end


function CollisionChecker:registerSelfPairs(iloader1, collpairs)
	local loader1=self.collisionDetector:getModel(iloader1)
	for ii, v in ipairs(collpairs) do
		local i, j
		if type(v[1])=='string' then
			i=loader1:getTreeIndexByName(v[1])
			j=loader1:getTreeIndexByName(v[2])
		else
			i=v[1]
			j=v[2]
		end
		local bone_i=loader1:VRMLbone(i)
		local bone_j=loader1:VRMLbone(j)
		self.collisionDetector:addCollisionPair(loader1, bone_i:treeIndex(), loader1, bone_j:treeIndex())
	end
end

function CollisionChecker:registerPair(iloader1, iloader2)
	local loader1=self.collisionDetector:getModel(iloader1)
	local loader2=self.collisionDetector:getModel(iloader2)
	for i=1,loader1:numBone()-1 do
		for j=1,loader2:numBone()-1 do
			local bone_i=loader1:VRMLbone(i)
			local bone_j=loader2:VRMLbone(j)
			self.collisionDetector:addCollisionPair(loader1, bone_i:treeIndex(), loader2, bone_j:treeIndex())
		end
	end
end
function CollisionChecker:checkCollision()
	self.collisionDetector:testIntersectionsForDefinedPairs(self.collisionSequence)
	return self.collisionSequence
end

do 

	CollisionCheckerUsingSimulator=LUAclass()
	function CollisionCheckerUsingSimulator:__init(...)
		local obj={...}
		self.nbObj=#obj;
		--self.simulator=Physics.DynamicsSimulator_gmbs("bullet") -- works well
		--self.simulator=Physics.DynamicsSimulator_gmbs("opcode") -- works best, but extremely slow.
		self.simulator=Physics.DynamicsSimulator_gmbs("libccd") -- currently developing. works only for simple geometries now.


		for i,v in ipairs(obj) do
			local ltype=getmetatable(v).luna_class
			if ltype=='MainLib.VRMLloader' then
				self.simulator:registerCharacter(v);
			else
				--self.simulator:createObstacle(v)
				self.simulator:createFreeBody(v);
			end

			local v=self.simulator:skeleton(i-1)

			print(v:name())
			local zero=CT.zeros(v.dofInfo:numDOF())
			if zero:size()>0 then
				zero:set(3, 1) -- quater(1,0,0,0)
				self.simulator:setLinkData(i-1, Physics.DynamicsSimulator.JOINT_VALUE, zero)
			end
		end

		local timestep=1
		--local timestep=1/8000
		self.timestep=timestep
		self.simulator:init(timestep, Physics.DynamicsSimulator.EULER)
	end

	function registerContactPairAll(model, loader, floor, simulator)
		local param=vectorn()
		param:setValues(0.5,0.5, model.penaltyForceStiffness, model.penaltyForceDamp)
		for i=1,loader:numBone()-1 do
			local bone_i=loader:VRMLbone(i)
			simulator:registerCollisionCheckPair(loader:name(),bone_i.NameId, floor:name(), floor:bone(1):name(), param)
		end
	end
	function CollisionCheckerUsingSimulator:setPoseDOF(iloader, dof)
		self.simulator:setLinkData(iloader, Physics.DynamicsSimulator.JOINT_VALUE, dof)
	end



	function CollisionCheckerUsingSimulator:registerPair(iloader1, iloader2)
		local param=vectorn ()
		--param:setValues(0.5,0.5, 10000,1000)
		param:setValues(0.6,0.6, 10000,1000)
		local loader1=self.simulator:skeleton(iloader1)
		local loader2=self.simulator:skeleton(iloader2)
		--print(loader1:name(),loader2:name())
		self.linkpairs=self.linkpairs or {}
		for i=1,loader1:numBone()-1 do
			for j=1,loader2:numBone()-1 do
				local bone_i=loader1:VRMLbone(i)
				local bone_j=loader2:VRMLbone(j)
				self.simulator:registerCollisionCheckPair(loader1:name(),bone_i.NameId, loader2:name(), bone_j.NameId, param)
				--print('registerPair', bone_i.NameId, bone_j.NameId);
				table.insert(self.linkpairs, {iloader1, i,iloader2, j})
			end
		end
	end
	function CollisionCheckerUsingSimulator:checkCollision()
		self.simulator:initSimulation()
		local bases=Physics.Vec_ContactBasis()
		self.simulator:getContactBases(bases,0.5)
		--print(bases:size(), #self.linkpairs)
		--
		return bases
	end
	function CollisionCheckerUsingSimulator:checkRayIntersection(iloaders, startPos, direction)
		local sim=self.simulator 
		local cd=sim:getCollisionDetector()
		for i=0, startPos:size()-1 do
			local r=Physics.RayTestResult ()
			local s=startPos(i)/config.skinScale
			local t=s+direction*100

			for j=1,#iloaders do
				local ibone=1
				cd:rayTest(iloaders[j], ibone,s ,t , r)

				if r:hasHit() then
					local o=vector3()
					o:interpolate(r.m_closestHitFraction, s,t)
					--dbg.draw("Sphere", o*config.skinScale, "x0")
					dbg.draw('Arrow', startPos(i), o*config.skinScale, 'arraow'..i..'_'..j, 5, 'red_transparent')
				else
					dbg.erase('Arrow', 'arraow'..i..'_'..j)
				end
			end
		end
	end
end

do 
	EVR=LUAclass(EventReceiver)
	function EVR:__init()
	end
	function EVR:onFrameChanged(win, iframe)
		self:_onFrameChanged(win, iframe)
	end

	EVRecoder=LUAclass(EventReceiver)
	-- mode: "save", "load", "none",
	function EVRecoder:__init(mode, filename)
		self.drawCursor=true
		self.currFrame=0
		self.crow=0
		self.filename=filename
		self:changeMode(mode)

		self.invEvMap={"DRAG", "RELEASE", "KEYUP","KEYDOWN", "MOVE", "PUSH", "onFrameChanged","FrameMove"}
		self.evMap={}
		for i,v in ipairs(self.invEvMap) do
			self.evMap[v]=i-1
		end
	end

	function EVRecoder:changeMode(mode)
		self.data=matrixn()
		self.mode=mode
		if mode=="load" then
			self.tbl=util.loadTable(self.filename)
			self.data=self.tbl[1]
		end
	end
	-- reimplment these functions (whose name start with _)
	function EVRecoder:_onFrameChanged(win, iframe)
	end
	function EVRecoder:_handleRendererEvent(ev, button, x,y) 
	end
	function EVRecoder:_frameMove(fElapsedTime)
	end

	function handleRendererEvent(ev, button, x, y)
		return mEventReceiver:handleRendererEvent(ev, button, x,y)
	end
	function frameMove(fElapsedTime)
		mEventReceiver:frameMove(fElapsedTime)
	end
	function EVRecoder:frameMove(fElapsedTime)
		if self.mode=="load" then
			local crow=self.data:row(self.crow)
			self.crow=math.min(self.crow+1, self.data:rows()-1)
			
			ev=self.invEvMap[crow(0)+1]

			if ev=="onFrameChanged" then
				local iframe=crow(1)
				self:_onFrameChanged(win, iframe)
			elseif ev=="FrameMove" then
				local fElapsedTime=crow(1)
				self:_frameMove(fElapsedTime, fElapsedTime)
			else
				local button=crow(1)
				if ev=='KEYDOWN' or ev=='KEYUP' then
					print("button", button)
					button=tostring(button)
				end
				local x=crow(2)
				local y=crow(3)

				if self.drawCursor then
					local overlay=RE.getOverlayByName("CursorOverlay")
					if not overlay:isVisible() then
						overlay:show()
					end
					overlay:setChildPosition( "TCursor", x, y)
				end
				self:_handleRendererEvent(ev, button, x,y)
			end
		elseif self.mode=="save" then
			self.data:pushBack(CT.vec(self.evMap.FrameMove, fElapsedTime, 0,0))
			self:save()
			self:_frameMove(fElapsedTime)
		else
			self:_frameMove(fElapsedTime)
		end
	end
	function EVRecoder:save()
		if math.mod(self.data:rows(), 50)==0 then
			util.saveTable( {self.data}, self.filename)
		end
	end
	function EVRecoder:saveData()
	end
	function EVRecoder:handleRendererEvent(ev, button, x,y)
		print(ev, button, x,y)
		if self.mode=="save" then
			self.data:pushBack(CT.vec(self.evMap[ev], tonumber(button), x or 0, y or 0))
			self:save()
		elseif self.mode=="load" then
			return 0 -- ignore events
		end
		return self:_handleRendererEvent(ev, button, x,y)
	end

	function EVRecoder:onFrameChanged(win, iframe)
		if self.mode=="save" then
			self.data:pushBack(CT.vec(self.evMap.onFrameChanged, iframe, 0,0))
			self:save()
		elseif self.mode=="load" then
			return -- all events are played back in framemove
		end
		self:_onFrameChanged(win, iframe)
	end

end
