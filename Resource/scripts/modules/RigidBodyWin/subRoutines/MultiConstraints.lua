
MultiConstraints=LUAclass()
function MultiConstraints:__init(...)
	local points={...}
	self.prefix = "con"
	self.objList=Ogre.ObjectList()
	if lunaType(points[1])=='vector3' then
		self.conPos=vector3N(#points)
		for i=1, #points do
			self.conPos(i-1):assign(points[i])
		end
	else 
		assert(lunaType(points[1])=='vector3N')
		self.conPos=points[1]:copy()
	end
	self.conIndex = intvectorn()
	self.selectedVertex=-1
	self:drawConstraints()
	self.size=5
end
function MultiConstraints:setPrefix(prefix)
	self.prefix=prefix
	self:drawConstraints()
end
function MultiConstraints:_calcPlaneNormal()

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
function MultiConstraints:setPos(points)
	for i=1, #points do
		self.conPos(i-1):assign(points[i])
	end
	self:drawConstraints()
end
function MultiConstraints:getPos(ipoint)
	return self.conPos(ipoint-1)
end
function MultiConstraints:setOption(param1, param2)
	local function changeSphereSize(param1)
		local sphere_size=param1
		self.size=sphere_size
		self:drawConstraints()
	end
	if param2 then
		if param1=='draggingDisabled' then
			self.draggingDisabled=param2
		elseif param1=='changeSphereSize' then
			changeSphereSize(param2)
		end
	else
		changeSphereSize(param1)
	end
end
function MultiConstraints:drawConstraints()
	local conPos=self.conPos
	self.objList:clear()
	--for i=0 ,conPos:size()-1 do
	--	self.objList:erase(self.prefix..i)
	--end
	
	self.objList:erase(self.prefix..'arrow')

	local function dbgDraw(name, pos, color)
		color = color or 'blue_transparent'
		self.objList:drawSphere(pos, name, color, self.size)
	end
	for i=0, conPos:size()-1 do
		--print(self.prefix..'_'..i)
		if self.selectedVertex==i then
			dbgDraw(self.prefix..'_'..i,conPos(i), 'red');
		else
			dbgDraw(self.prefix..'_'..i,conPos(i), 'blue_transparent');
		end
	end
	local conIndex=self.conIndex

	for j=0, conIndex:size()-1 do
		local i=conIndex(j)
		dbgDraw(self.prefix..'_'..i, conPos(i), 'green_transparent')

		if self.selectedVertex==i then
			dbg.drawArrow2D(self.objList, conPos(i), self.planeNormal, self.prefix..'arrow')
		end
	end
	
	for i=conPos:size() , conPos:size()+2 do
		self.objList:erase(self.prefix..'_'..i)
	end
	if self.selectedVertex~=-1 then
		self.objList:drawSphere(conPos(self.selectedVertex),self.prefix..'_'.."selected",'red_transparent',self.size*5/3);
	else
		self.objList:erase(self.prefix..'_'.."selected")
	end
end

function MultiConstraints:connect(fcn)
	self.eventFunction=fcn
end
function MultiConstraints:handleRendererEvent(ev, button, x, y)
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
			local conIndex=self.conIndex
			for j=0, conIndex:size()-1 do 
				local i=conIndex(j)
				if i==self.selectedVertex then
					--	print('dragging')
					--	플레인 만들어서 충돌체크 후 좌표얻기
					--local t= ray:direction() * -1	
					local t= self.planeNormal
					local p = Plane(t,conPos(i));     
					local PlaneRay_intersectsCheck = ray:intersects(p) 
					--  좌표 
					--  ray:getPoint(PlaneRay_intersectsCheck(1))/10
					--  스피어 충돌체크 	

					-- 스피어와 레이가 충돌하면 좌표값을 Constraints 에 대입
					-- 스피어와 레이가 충돌한 이후(선) 에 플레인과 레이의 충돌(후)에서 얻은 좌표값이 필요한것
					self.drag=true;
					conPos(i):assign(ray:getPoint(PlaneRay_intersectsCheck(1)))
					--print(conPos)
					self:drawConstraints()
					if self.eventFunction then
						self.eventFunction ('drag', i)
					end
					return 1;
				end
			end
		elseif (ev =='RELEASE'and self.selectedVertex~=-1 ) then
			if(self.drag) then
				if self.eventFunction then
					self.eventFunction ('drag_finished') 
				end
				--setup()
			else
				local index=self.selectedVertex;
				if self.eventFunction then
					self.eventFunction ('clicked', index)
				end
				local index=self.conIndex:findFirstIndex(self.selectedVertex);
				if index==-1 then
					assert(self.eventFunction)

					local conIndex=self.conIndex
					if self.eventFunction ('add_to_conIndex', self.selectedVertex) then
						conIndex:pushBack(self.selectedVertex);
					end
					self.eventFunction ('con_added', self.selectedVertex) 
				else
					assert(self.eventFunction)
					local conIndex=self.conIndex
					if self.eventFunction('remove_from_conIndex', self.selectedVertex) then
					 --	conIndexAll와 conPosAll의 size감소
						self.conIndex:bubbleOut(index, index+1);
						self.eventFunction ('con_removed', self.selectedVertex) 
					end
				end
				self.selectedVertex=-1
				self:drawConstraints()
			end
		end
		self.drag=false;
		self.selectedVertex = -1
		return 1;
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

function MultiConstraints:updateSelectedVertex(ray)
	local conPos=self.conPos
	local conIndex=self.conIndex
	local prevSel=self.selectedVertex	
	-- constraint vertex (high priority)
	for i=0,conIndex:size()-1 do
		local ConstraintsSphere = Sphere(conPos(i),self.size*2)
		local SphereRay_intersectsCheck = ray:intersects(ConstraintsSphere)
		local cursorPos=ray:getPoint(SphereRay_intersectsCheck(1))
		-- Plane 과 Ray 의 값을 안쓰는 이유는 화면돌림에 따라 거리값이 변해서 안씀(보는 시점에 따라 플레인과 레이의 거리가 달라지기때문)
		if(conPos(conIndex(i)):distance(cursorPos)< 11) then
			RE.output(self.prefix..'_'.."selected" , conIndex(i));
			self.selectedVertex=conIndex(i);
			if self.eventFunction and prevSel~=self.selectedVertex then
				self.eventFunction(self.prefix..'_'..'selected', conIndex(i))
			end
		end
	end
	-- other vertices (low priority)
	for i=0,conPos:size()-1 do
		local ConstraintsSphere = Sphere(conPos(i),self.size*2)
		local SphereRay_intersectsCheck = ray:intersects(ConstraintsSphere)
		local cursorPos=ray:getPoint(SphereRay_intersectsCheck(1))
		-- cursorPos 는 Sphere 와 Ray 의 충돌 값에서 얻어온 좌표 
		-- Plane 과 Ray 의 값을 안쓰는 이유는 화면돌림에 따라 거리값이 변해서 안씀(보는 시점에 따라 플레인과 레이의 거리가 달라지기때문)
		if(conPos(i):distance(cursorPos)< 11) then
			RE.output(self.prefix..'_'.."selected" , i);
			self.selectedVertex=i
			if self.eventFunction and prevSel~=self.selectedVertex then
				self.eventFunction(self.prefix..'_'..'selected', i)
			end
			return true;
		end
	end
	RE.output(self.prefix..'_'.."selected" , -1);
	if prevSel~=-1 then
		if self.eventFunction then
			self.eventFunction('unselected', prevSel)
		end
		self.selectedVertex=-1
		return true
	end
	return false
end
