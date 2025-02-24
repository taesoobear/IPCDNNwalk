require("RigidBodyWin/subRoutines/Constraints")
MultiSelectConstraints=LUAclass()


-- for usage, see testChain.lua
function MultiSelectConstraints:__init(positions, initialSelect, con_size)
	con_size=con_size or 5
	self.constrained=boolN()
	self.constrained:resize(positions:size())
	self.constrained:setAllValue(false)

	if initialSelect then
		for i=0, initialSelect:size()-1 do
			self.constrained:set(initialSelect(i), true)
		end
	end

	self.positions=positions:copy()
	local pos_c, pos_u=self:_getPositions()
	self.mCON=Constraints(pos_c)
	self.mCON.size=con_size
	self.mCON:connect(self._dragEventFunction, self)
	self.mCON:redraw()
	self.mSelectTool=Constraints(pos_u)
	self.mSelectTool.draggingDisabled=true
	self.mSelectTool.size=con_size/2
	self.mSelectTool:redraw()
	self.mSelectTool:connect(self._selectEventFunction, self)
end
function MultiSelectConstraints:handleRendererEvent(ev, button, x, y)
	local res= self.mCON:handleRendererEvent(ev, button, x,y)
	if res==0 then
		res=self.mSelectTool:handleRendererEvent(ev, button, x,y)
	else
		self.mSelectTool.selectedVertex=-1
		self.mSelectTool:redraw()
	end
	return res
end

function MultiSelectConstraints:selectSphere(i)
	self.constrained:set(i, true)
end


-- does not change select state, nor redraw
function MultiSelectConstraints:updatePositions(positions)
	self.positions=positions:copy()
end


function MultiSelectConstraints:redraw(unselectedOnly)

	local pos_c, pos_u=self:_getPositions()
	if not unselectedOnly then
		self.mCON:setCON(pos_c)
		self.mCON:redraw()
	end
	self.mSelectTool:setCON(pos_u)
	self.mSelectTool:redraw()
end

function MultiSelectConstraints:findSelected()
	return self.constrained:findAll(true), self.mCON.conPos
end

function MultiSelectConstraints:connect(eventFunction, eventFunctionArg)
	self.eventFunction=eventFunction
	self.eventFunctionArg=eventFunctionArg
end


function MultiSelectConstraints:_getPositions()

	local positions=self.positions
	local constrained=self.constrained
	local pos_c=vector3N()
	local pos_u=vector3N()
	for i=0, constrained:size()-1 do
		local bpos=positions(i)
		if constrained(i) then
			pos_c:pushBack(bpos)
		else
			pos_u:pushBack(bpos)
		end
	end
	return pos_c, pos_u
end

function MultiSelectConstraints._dragEventFunction(ev, index, self)

	if ev=='drag' or ev=='drag_finished' then
		if self.eventFunction then
			self.eventFunction(ev, index, self,self.eventFunctionArg)
		end
	elseif ev=='clicked' then
		local i=self.constrained:findNth(true, index)
		if i~=-1 then
			if not self.eventFunction('conRemoveNotPossible', i) then
				self.constrained:set(i, false)
				self:redraw()
				self.eventFunction('conRemoved', i)
			end
		end
	end
end

function MultiSelectConstraints._selectEventFunction(ev, index, self)
	if ev=='clicked' then
		local i=self.constrained:findNth(false, index)
		if i~=-1 then
			self.constrained:set(i, true)
			self:redraw()
			self.eventFunction('conAdded', i)
		end
	end
end
