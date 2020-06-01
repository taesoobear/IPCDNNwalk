--	example usage:
--	function ctor()
--	mEventReceiver=EVR()
--	mTimeline=Timeline("Timeline", 10000)
--	end
--  function EVR:onFrameChanged(win, iframe)
--
Timeline=LUAclass(LuaAnimationObject)
function Timeline:__init(label, totalTime, frametime)
	self.totalTime=totalTime
	self:attachTimer(frametime or 1/60, totalTime)		
	RE.renderer():addFrameMoveObject(self)
	RE.motionPanel():motionWin():addSkin(self)
end
-- has to be manually called
-- mTimeline:dtor() mTimeline=nil
function Timeline:dtor()
	RE.motionPanel():motionWin():changeCurrFrame(0)
	RE.renderer():removeFrameMoveObject(self)
	RE.motionPanel():motionWin():detachSkin(self)
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
