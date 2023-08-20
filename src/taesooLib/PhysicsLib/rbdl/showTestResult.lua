
require("config")

package.projectPath='../Samples/classification/'
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

require("module")

require('subRoutines/Timeline')

local currentPath=select(2, os.processFileName(debug.getinfo(1,'S').source:sub(2)))

function unpackPrefix(file)
	local sphQ=file:unpackAny()
	local sphQDot=file:unpackAny()
	niter=file:unpackInt()
end

hasCompareTarget=false
function ctor()
	file=util.BinaryFile()
	local compareTargetPath=os.joinPath(currentPath, 'test.dat')
	if os.isFileExist(compareTargetPath) then
		file:openRead(compareTargetPath)
		unpackPrefix(file)
		hasCompareTarget=true
	end

	file2=util.BinaryFile()
	file2:openRead(os.joinPath(currentPath, 'test_quat.dat'))
	unpackPrefix(file2)


	iter=0
	mEventReceiver=EVR()
	mTimeline=Timeline("Timeline", 10000)

end

function dtor()
end

function onCallback(w, userData)
   
end

function vectorn:getEigenQuater(i)
	return quater(self(i+3), self(i), self(i+1), self(i+2))
end
			
function EVR:onFrameChanged(win, i)
	print(i)
	for iii=0, 10 do -- skip rendering
		if iter>=niter-1 then return end

		local offset=vector3(0,1.5,0)
		if hasCompareTarget then
			draw('1',file, offset)
		end
		draw('2',file2, offset+vector3(1,0,0))

		iter=iter+1
	end
	
end

			
function toYUP(v)
	assert(v.w==nil)
	return vector3(v.y, v.z, v.x);
end
function toYUP_q(q)
	return quater(q.w, q.y,  q.z, q.x);
end

function draw(prefix, file, offset)
	local _iter=file:unpackInt()
	local sphQ=file:unpackAny()
	local sphQDot=file:unpackAny()
	assert(iter== _iter)


	local p0=vector3()
	file:unpack(p0)
	local p1=vector3()
	file:unpack(p1)
	local p2=vector3()
	file:unpack(p2)

	dbg.namedDraw('Sphere', toYUP(p0)*100+offset*100, prefix..'p00', 'green_transparent')
	dbg.namedDraw('Sphere', toYUP(p1)*100+offset*100, prefix..'p10', 'green_transparent')
	dbg.namedDraw('Sphere', toYUP(p2)*100+offset*100, prefix..'p20', 'green_transparent')

	file:unpack(p0)
	file:unpack(p1)
	file:unpack(p2)

	dbg.namedDraw('Sphere', toYUP(p0)*100+offset*100, prefix..'p0')
	dbg.namedDraw('Sphere', toYUP(p1)*100+offset*100, prefix..'p1')
	dbg.namedDraw('Sphere', toYUP(p2)*100+offset*100, prefix..'p2')

	for j=0, 2 do
		local qq=file:unpackAny()
		local v=vector3()
		file:unpack(v)

		local q=qq:getEigenQuater(0)

		dbg.namedDraw("Axes", transf(toYUP_q(q), toYUP(v)+offset),prefix.. 'a'..j,100)
	end
end
function frameMove(fElapsedTime)

end
function handleRendererEvent(ev, x, y)
	return 0;
end
