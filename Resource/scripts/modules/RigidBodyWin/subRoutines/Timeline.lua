--	example usage:
--	require('subRoutines/Timeline')
--	function ctor()
--	mEventReceiver=EVR()
--	mTimeline=Timeline("Timeline", 10000)
--	end
--  function EVR:onFrameChanged(win, iframe)
--
--
-- an example is in testScrollPanel.lua
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

	function EVR:attachCameraToMotion(mLoader, mMotionDOF, discontinuity)
		if mLoader~=nill then

			self.trajectory=matrixn(mMotionDOF:numFrames(),3)

			self.trajectoryOri=matrixn(mMotionDOF:numFrames(),4)
			if discontinuity then
				local discont=discontinuity
				local segFinder=SegmentFinder(discont)

				for i=0, segFinder:numSegment()-1 do
					local s=segFinder:startFrame(i)
					local e=segFinder:endFrame(i)

					for f=s,e-1 do
						self.trajectory:row(f):setVec3(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).translation)
						self.trajectory:row(f):set(1,0)
						self.trajectoryOri:row(f):setQuater(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).rotation:rotationY())
					end
					print("filtering",s,e)
					math.filter(self.trajectory:range(s,e,0, 3), 63)
					math.filter(self.trajectoryOri:range(s,e,0, 4), 63)
				end
			else
				if dbg.lunaType(mMotionDOF)=='Motion' then
					mMotionDOF=MotionDOF(mLoader.dofInfo, mMotionDOF)
				end

				local s=0
				local e=mMotionDOF:numFrames()

				for f=s,e-1 do
					self.trajectory:row(f):setVec3(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).translation)
					self.trajectory:row(f):set(1,0)
					self.trajectoryOri:row(f):setQuater(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).rotation:rotationY())
				end
				print("filtering",s,e)
				math.filter(self.trajectory:range(s,e,0, 3), 63)
				math.filter(self.trajectoryOri:range(s,e,0, 4), 63)
			end

			local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
			self.cameraInfo.vpos=RE.viewpoint().vpos-curPos
			self.cameraInfo.vat=RE.viewpoint().vat-curPos
			self.cameraInfo.dist=RE.viewpoint().vpos:distance(curPos)
			self.cameraInfo.refRot=self.trajectoryOri:row(self.currFrame):toQuater(0):rotationY()
		end
	end

	function EVR:moveCamera(iframe)

		if mEventReceiver.trajectory and iframe< mEventReceiver.trajectory:rows() then
			local curPos=mEventReceiver.trajectory:row(iframe):toVector3(0)*100
			RE.viewpoint().vpos:assign(mEventReceiver.cameraInfo.vpos+curPos)
			RE.viewpoint().vat:assign(mEventReceiver.cameraInfo.vat+curPos)
			RE.viewpoint():update()     
		end
	end

end
