
require("config")
require("module")
require("common")

package.projectPath='./gym_walkCDM'
package.path=package.path..";../../taesooLib/Samples/scripts/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/RigidBodyWin/subRoutines/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/?.lua" --;"..package.path
require("gym_walkCDM/testWalkV2_FA")

function randomSamplingNormal(sigma)
	local x, y, r2;

	repeat
		x = -1 + 2 * getRandom()
		y = -1 + 2 * getRandom()

		r2 = x * x + y * y;
	until not (r2 > 1.0 or r2 == 0)

	-- Box-Muller transform 
	return sigma * y * math.sqrt (-2.0 * math.log (r2) / r2);
end

g_step=0

g_randomQueue=vectorn()

function getRandom()
	if g_randomQueue:size()==0 then
		math.randomseed(os.time()) -- to work around a strange bug where random seed is initialized somewhere unknown.
		for i=0,1000 do
			local r=math.random()
			g_randomQueue:pushBack(r)
		end
	end
	local r=g_randomQueue(g_randomQueue:size()-1)
	g_randomQueue:resize(g_randomQueue:size()-1)
	return r
end
function EVR:onFrameChanged(win, iframe)
	local switchFrame=planner.switchTime*framerate
	RE.output('switchFrame', switchFrame, g_iframe, g_global_time)
	local totalTime=planner.totalTime
	if g_iframe>switchFrame then
		planner:setInitialState(planner:getState(planner.switchTime))
		local r=getRandom()
		RE.output2('random', r)
		print('random', r, g_global_time)
		if r<0.3 then
			local defaultSeq=input.states[g_currState].mSeq.defaultSeq

			local v=this:findWidget("speed"):sliderValue()+randomSamplingNormal(1)

			local desiredspeed=math.max(defaultSeq.input.speed, defaultSeq.input.maxspeed*0.5+ defaultSeq.input.minspeed*0.5)
			if desiredspeed>v then
				v=v+randomSamplingNormal(1)
			else
				v=v-randomSamplingNormal(1)
			end

			v=math.min(v, defaultSeq.input.maxspeed)
			v=math.max(v, defaultSeq.input.minspeed)
			this:findWidget("speed"):sliderValue(v)
			
			local roty=this:findWidget("finalrotY"):sliderValue()
			roty=roty+randomSamplingNormal(0.5)
			while roty>3.14 do roty=roty-2*3.14 end
			while roty<-3.14 do roty=roty+2*3.14 end

			this:findWidget("finalrotY"):sliderValue(roty)
			if getRandom()<0.05 then
				initiateChangeMotionType()
			end
			this:updateLayout()
		end
		planner:replan()
		g_iframe=g_iframe-switchFrame
	end
	local iframe=g_iframe -- override. fixed frame rate
	planner:drawFrame(g_iframe/framerate)
	g_iframe=g_iframe+1
	g_global_time=g_global_time+1.0/framerate
end
ctor_FA=ctor
function ctor()
	_createTerrain()
	ctor_FA()
end
