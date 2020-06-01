require("config")

package.projectPath='../Samples/classification/'
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

require("module")
arc=require('gym_walk/arc')
require("gym_walk/IPC3d_approx")
require("RigidBodyWin/subRoutines/Constraints")

package.path=package.path..";../Samples/sample_luatorch/lua/?.lua" --;"..package.path
package.path=package.path..";../../taesooLib/Samples/scripts/RigidBodyWin/subRoutines/?.lua" --;"..package.path

useMonopod=false
useQuadruped=false

require("gym_walk/pendulumOnlineControl")
fc=require("gym_walk/footCoord")
require("gym_walk/PlannerInfo")
require("gym_walk/ConvexHull2D")
TE=require("gym_walk/COMtrajEdit")
require('gym_walk/trajOptObstacle')
require('gym_walk/pendulumOnlineWalk')
function handleRendererEvent()
	return 0
end
function frameMove() 
	return 0
end
package.path=package.path..";../Samples/QP_controller/ILIPM/?.lua" --;"..package.path
require("gym_walk/collisionAvoid")

function drawBoxes(boxes, name, y, skinscale, margin)

	y= y or 0
	name=name or 'boxes'
	margin=margin or 0
	local lines=vector3N()
	for i, b in ipairs(boxes) do
		if type(b)=='table' then
			b=b[1]
		end
		local min=b.min:toVector3()
		local max=b.max:toVector3()
		min.x=min.x-margin
		min.z=min.z-margin
		max.x=max.x+margin
		max.z=max.z+margin

		min.y=y
		max.y=y
		lines:pushBack(min)
		lines:pushBack(vector3(min.x,y, max.z))
		lines:pushBack(vector3(min.x,y, max.z))
		lines:pushBack(max)
		lines:pushBack(max)
		lines:pushBack(vector3(max.x,y, min.z))
		lines:pushBack(vector3(max.x,y, min.z))
		lines:pushBack(min)
	end

	--dbg.draw('Traj', lines:matView()*skinscale, name, 'solidgreen', 5, 'BillboardLineList')
	dbg.draw('Traj', lines:matView()*skinscale, name, 'solidgreen', 5, 'LineList')
end
function createBoxes(planner1)
	local function createBox(x, z, maxHalfGapSize, minHalfGapSize)

		minHalfGapSize=minHalfGapSize or 0.1
		local t=sop.map(math.random(),0,1,minHalfGapSize,maxHalfGapSize)
		local p=(math.random()-0.5)*0.3
		return Box2D(vector2(x-0.2, z-t+p), vector2(x+0.2, z+t+p))
	end

	local z=0.5 
	local maxHalfGapSize=0.2
	boxes={}
	for i=0, 40 do
		table.insert(boxes, createBox(math.random(), z, maxHalfGapSize))
		z=z+sop.map(math.random(), 0, 1, 0.8, 1.0)
	end
end
function createAndDrawBoxes(planner1)
	createBoxes(planner1)
	_drawBoxes(planner1)
end
function _drawBoxes(planner1)
	drawBoxes(boxes,'boxes', 0.01, 100)
end
function PendPlanner:createUI()
	createUI()
	local defaultInput=input[input.defaultMotionType]
	this:findWidget("speed"):sliderRange(0.01, defaultInput.maxspeed or 10)
	this:findWidget("speed"):sliderValue(defaultInput.speed)
	this:findWidget('draw pend traj'):checkButtonValue(false)
	this:create('Check_Button', 'solve ik', 'solve ik')
	this:widget(0):checkButtonValue(true)
	this:create("Check_Button", "use terrain", "use terrain", 1)
	this:widget(0):checkButtonValue(false)
	this:create("Button", "change motion type", "change motion type", 1)
	this:updateLayout()
end

function ctor()

	createAndDrawBoxes(planner)
	--RE.FltkRenderer():onCallback('OgreTraceManager')

	planner=PendPlanner(totalTime) 
	planner:createUI()

	this:create("Value_Slider", 'action 0', 'action 0')
	this:widget(0):sliderRange(-3, 3)
	this:widget(0):sliderValue(0)

	this:create("Value_Slider", 'action 1', 'action 1')
	this:widget(0):sliderRange(-3, 3)
	this:widget(0):sliderValue(0)

	this:create("Button", 'save state', 'save state')
	this:create("Button", 'load state', 'load state')

	this:redraw()


	m_pendulum=IPC3d("gym_walk/cart_pole_ball.wrl",0, 9.8,1.0/30, 30000)
	m_pendulum.skin:setVisible(false)

	planner:initializeGlobal()

	planner.refPendState={
		vector3(0,0.948, 0), -- com
		vector3(0,0,0), -- dotcom
		quater(1,0,0,0), -- ang
		vector3(0,0,0), -- angvel
		vector3(0,0,0), -- zmp
		vector3(0,0,0), -- zmpvel
	}
	planner:replan(getAction())

end

function getAction()
	local action=CT.vec(0,0)
	action:set(0, this:findWidget('action 0'):sliderValue())
	action:set(1, this:findWidget('action 1'):sliderValue())
	return action
end

onCallback_in_pendulumOnlineControl=onCallback
function onCallback(w, userData)
	if w:id()=='save state' then
		if g_lastState then
			g_savedState=g_lastState
		end
	elseif w:id()=="change motion type" then
		g_prevState=g_currState
		g_currState=math.fmod(g_currState, #input.states)+1
		RE.output2('currstate', g_currState)
	elseif w:id()=='load state' and g_savedState then
		planner.refPendState=g_savedState[2]
		planner:setInitialState(g_savedState[1])
		planner:replan(getAction())
		g_iframe=0
		g_global_time=0
	else
		return onCallback_in_pendulumOnlineControl(w, userData)
	end
end





if EventReceiver then
	--class 'EVR'(EventReceiver)
	EVR=LUAclass(EventReceiver)
	function EVR:__init()
	end
	function EVR:onFrameChanged(win, iframe)
		local switchFrame=planner.switchTime*framerate
		RE.output('switchFrame', switchFrame, g_iframe, g_global_time)
		local totalTime=planner.totalTime
		local lastState
		if g_iframe>switchFrame then

			lastState={
				planner:getState(planner.switchTime), 
				planner.refPendState
			}

			planner:setInitialState(lastState[1])
			planner:replan(getAction())
			g_iframe=g_iframe-switchFrame
		end

		if lastState then
			g_lastState=lastState
		end
		local iframe=g_iframe -- override. fixed frame rate
		planner:drawFrame(g_iframe/framerate)
		g_iframe=g_iframe+1
		g_global_time=g_global_time+1.0/framerate
	end
end
