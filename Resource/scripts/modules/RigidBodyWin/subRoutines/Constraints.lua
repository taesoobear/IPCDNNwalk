
Constraints=LUAclass()
function Constraints:setBasePosition(v)

	self.objectList:getCurrRootSceneNode():setPosition(v.x, v.y, v.z)
	self.basePos:assign(v)
end

function Constraints:__init(...)
	local points={...}
	self.basePos=vector3(0,0,0)
	self.prefix="con"..RE.generateUniqueName()
	self.matSelected='red'
	self.matUnselected='blue_transparent'
	self.objectList=Ogre.ObjectList()
	if type(points[1])=='table' then
		points=points[1]
		self.prefix=points.prefix or "con"
	end
	if dbg.lunaType(points[1])=='vector3' then
		self.conPos=vector3N(#points)
		for i=1, #points do
			self.conPos(i-1):assign(points[i])
		end
	else 
		assert(dbg.lunaType(points[1])=='vector3N')
		self.conPos=points[1]:copy()
	end
	self.selectedVertex = -1
	self.planeNormal= self:_calcPlaneNormal()
	self:drawConstraints()
	self.size=5
end

function Constraints:addCON(pos)
	self.conPos:pushBack(pos)
	self.selectedVertex = -1
	self:drawConstraints()
end
function Constraints:_calcPlaneNormal()

	if self.isCtrl then
		local vdir=RE.viewpoint().vat-RE.viewpoint().vpos
		vdir.y=0
		vdir:normalize()
		return vdir*-1
	elseif self.isAlt then
		return vector3(0,1,0)
	end
	local vdir=RE.viewpoint().vat-RE.viewpoint().vpos
	vdir:normalize()
	return vdir*-1
end
function Constraints:setPos(points)
	for i=1, #points do
		self.conPos(i-1):assign(points[i])
	end
	self:drawConstraints()
end
function Constraints:getPos(ipoint)
	return self.conPos(ipoint-1)
end
function Constraints:setVec(i)
	self.selectedVertex = i

	--print("loadselected",self.selectedVertex,i)
end
function Constraints:setOption(param1, param2)
	if param2~=nil then
		if param1=='draggingDisabled' then
			self.draggingDisabled=param2
		end
	else
		local sphere_size=param1
		self.size=sphere_size
		self:drawConstraints()
	end
end
function Constraints:drawConstraints(ev)
	local conPos=self.conPos
	for i=0 ,conPos:size()-1 do
		self.objectList:erase(self.prefix..i)
	end
	self.objectList:erase(self.prefix..'arrow')
	
	local function dbgDraw(name, pos, color)
		color = color or 'blue_transparent'
		dbg.drawSphere(self.objectList, pos, name, color, self.size)
	end
	for i=0, conPos:size()-1 do
		if self.selectedVertex==i then
			dbgDraw(self.prefix..i,conPos(i), self.matSelected)

			dbg.drawArrow2D(self.objectList, conPos(i), self.planeNormal, self.prefix..'arrow')
		else
			dbgDraw(self.prefix..i,conPos(i), self.matUnselected)
		end
	end
	
	for i=conPos:size() , conPos:size()+2 do
		self.objectList:erase( self.prefix..i)
	end
	if self.selectedVertex~=-1 then
		dbg.drawSphere(self.objectList, conPos(self.selectedVertex), self.prefix.."selected",'red_transparent',self.size*5/3);
	else
		self.objectList:erase( self.prefix.."selected")
	end
end
Constraints.redraw=Constraints.drawConstraints
function Constraints:connect(fcn, arg)
	self.eventFunction=fcn
	self.eventFunctionArg=arg
end
function Constraints:handleRendererEvent(ev, button, x, y)
	local conPos=self.conPos
	if ev =="PUSH" or 
		ev=="MOVE" or
		ev=="DRAG" or
		ev=="RELEASE" then
	
		RE.output("event", ev, x,y, button);
		local ray = Ray()
			
		RE.FltkRenderer():screenToWorldRay(x,y,ray)
		if (ev =='PUSH') then
			-- 좌표와 점사이의 거리를 체크하여 self.selectedVertex 값 결정  
			-- self.selectedVertex 는 vertex의 index 값을 의미
			if self:updateSelectedVertex(ray)  then
				local normal=self:_calcPlaneNormal()
				self.planeNormal=normal
				self:drawConstraints()
				return 1
			end
		elseif (ev == 'MOVE') then
			if self:updateSelectedVertex(ray)  then
				local normal=self:_calcPlaneNormal()
				self.planeNormal=normal
				self:drawConstraints()
				return 1;
			end		
		elseif (ev =='DRAG') then
			if self.draggingDisabled then
				return 1
			end
			for i=0, conPos:size()-1 do 
				if i==self.selectedVertex then
					--	print('dragging')
					--	플레인 만들어서 충돌체크 후 좌표얻기
					local t= self.planeNormal
					local p = Plane(t,conPos(i)+self.basePos);     
					local PlaneRay_intersectsCheck = ray:intersects(p) 
					--  좌표 
					--  ray:getPoint(PlaneRay_intersectsCheck(1))/10
					--  스피어 충돌체크 	

					-- 스피어와 레이가 충돌하면 좌표값을 Constraints 에 대입
					-- 스피어와 레이가 충돌한 이후(선) 에 플레인과 레이의 충돌(후)에서 얻은 좌표값이 필요한것
					self.drag=true;
					conPos(i):assign(ray:getPoint(PlaneRay_intersectsCheck(1))-self.basePos)
					--print(conPos)
					self:drawConstraints()
					if self.eventFunction then
						self.eventFunction ('drag', i, self.eventFunctionArg)
					end
					return 1;
				end
			end
		elseif (ev =='RELEASE'and self.selectedVertex~=-1 ) then

			if(self.drag) then
				if self.eventFunction then
					self.eventFunction ('drag_finished', self.selectedVertex, self.eventFunctionArg) -- TODO: change to drag_finished -.-
					-- for compatibility with old codes by students
					self.eventFunction ('drag_fineded', self.selectedVertex, self.eventFunctionArg) -- TODO: change to drag_finished -.-
				end
				--setup()
				self.drag=false;
				self.selectedVertex = -1
				return 1
			else
				local index=self.selectedVertex;
				if self.eventFunction then
					self.eventFunction ('clicked', index, self.eventFunctionArg)
				end
				self.selectedVertex=-1
				self:drawConstraints()
				return 1
			end
		end
		return 0;
	elseif ev=='KEYUP' or ev=='KEYDOWN' then
			
		RE.checkCtrlAndAlt(self, ev, button)
		if self.selectedVertex~=-1 and not self.isDragging then
			local normal=self:_calcPlaneNormal()
			self.planeNormal=normal
			self:drawConstraints()
		end
	end
	return 0;
end

function Constraints:updateSelectedVertex(ray)
	local conPos=self.conPos
	local prevSel=self.selectedVertex	

	for i=0,conPos:size()-1 do
		local ConstraintsSphere = Sphere(conPos(i)+self.basePos,self.size*2)
		local SphereRay_intersectsCheck = ray:intersects(ConstraintsSphere)
		local cursorPos=ray:getPoint(SphereRay_intersectsCheck(1))-self.basePos
		-- cursorPos 는 Sphere 와 Ray 의 충돌 값에서 얻어온 좌표 
		-- Plane 과 Ray 의 값을 안쓰는 이유는 화면돌림에 따라 거리값이 변해서 안씀(보는 시점에 따라 플레인과 레이의 거리가 달라지기때문)
		if(conPos(i):distance(cursorPos)< 11) then
			RE.output("selected" , i);
			self.selectedVertex=i
			if self.eventFunction and prevSel~=self.selectedVertex then
				self.eventFunction('selected', i, self.eventFunctionArg)
			end
			return true;
		end
	end

	if self.unselectRadius then

		if prevSel~=-1 then
			local i=prevSel
			RE.output2('prevsel2', prevSel~=-1)
			local ConstraintsSphere = Sphere(conPos(i)+self.basePos,self.unselectRadius)
			local SphereRay_intersectsCheck = ray:intersects(ConstraintsSphere)
			local cursorPos=ray:getPoint(SphereRay_intersectsCheck(1))-self.basePos
			-- cursorPos 는 Sphere 와 Ray 의 충돌 값에서 얻어온 좌표 
			-- Plane 과 Ray 의 값을 안쓰는 이유는 화면돌림에 따라 거리값이 변해서 안씀(보는 시점에 따라 플레인과 레이의 거리가 달라지기때문)
			if(conPos(i):distance(cursorPos)< self.unselectRadius*3) then
				return true;
			else
				if self.eventFunction then
					self.eventFunction('unselected', prevSel, self.eventFunctionArg)
				end
				self.selectedVertex=-1
				return true;
			end
		end
		do return false end
	end

	if prevSel~=-1 then
		if self.eventFunction then
			self.eventFunction('unselected', prevSel, self.eventFunctionArg)
		end
		self.selectedVertex=-1
		return true
	end
	return false
end
