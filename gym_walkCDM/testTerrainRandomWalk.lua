require("config")
require("module")
require("common")

package.projectPath='./gym_walkCDM'
package.path=package.path..";../../taesooLib/Samples/scripts/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/RigidBodyWin/subRoutines/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/?.lua" --;"..package.path
require("gym_walkCDM/testRandomWalk")


function ctor()

	init_planner()
	planner:changeTerrain(true)

	b_eval=true
	if b_eval then
		math.randomseed(10)
	end
	-- cannot run fast on terrain
	input.states[2].mSeq.defaultSeq.input.maxspeed=5

	planner:replan( true)
	planner:drawTrajectory()

	local numRun=0
	local prefix='_terrain'
	if b_eval then
		prefix='_terrainRandom_eval_'
	end
	g_history={filename=prefix..numRun..'.dat'} numRun=numRun+1
	for i=0, 10000 do
		print(i)
		local switchFrame=planner.switchTime*framerate
		local switched=false
		local failed=false
		if g_iframe>switchFrame then
			switched=true
		end
		EVR.onFrameChanged_old(nil, nil , i)

		if switched and g_failed then
			g_failed=nil
			failed=true
		end

		local rootpos=g_prevPose:toVector3(0)
		if math.abs(rootpos.x )>9 or math.abs(rootpos.z)>9 then
			failed =true
		end

		if not failed then
			if math.fmod(i, 2)==0 then
				RE.renderOneFrame(true)
			end
		else
			g_history={filename=prefix..numRun..'.dat'} numRun=numRun+1
			local self=planner
			g_iframe=0
			g_global_time=0
			g_COMoffset=vector3(0.006,0,0) -- YUP
			local fo=0
			g_footOffset=vector3(0,fo,0) -- ZUP

			local height=this:findWidget('height'):sliderValue()
			local I=quater(1,0,0,0)

			local fo=0.05
			local height=0.95
			local offsets={
				vector3(fo,0,0),
				vector3(-fo,0,0)
			}
			self.offsets=offsets
			local initialState={
				{
					lin={
						vector3(0,height,0), -- pos
						offsets[1],
						offsets[2],
					},
					quat={
						quater(1,0,0,0), --ori
						quater(1,0,0,0), --ori
						quater(1,0,0,0), --ori
					},
					vector3(0,0,0), -- vel
					vector3(0,0,0), -- angvel
				},
				-- (unused)
				{ },
				{
					--customState
					spprtCoord={
						{
							transf(quater(1,0,0,0), offsets[1]),
							transf(quater(1,0,0,0), offsets[2]),
						},
						{
							transf(quater(1,0,0,0), offsets[1]),
							transf(quater(1,0,0,0), offsets[2]),
						},
					},
					lastContactTime=
					{
						CT.vec(0,-18)/30.0 -36/30.0,  -- last contact timing
						CT.vec(0,-18)/30.0,  -- last contact timing
					},
				},
			}
			self:setInitialState(initialState)
			self.refPendState=nil
			local initialPose=input.states[g_currState].mSampler:samplePose(1.5,3.5)
			initialPose:set(1, 1.0)
			g_prevPose=initialPose:copy()
			planner:replan( true) 
			planner:drawTrajectory()
		end
	end
	this('exit',0)

end

EVR.onFrameChanged_old=EVR.onFrameChanged
function EVR:onFrameChanged(win, iframe)
end

