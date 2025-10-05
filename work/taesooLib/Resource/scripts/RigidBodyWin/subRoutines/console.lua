require("config")

require("module")
M=require("scilua")

function d(fn)
	dofile(fn..'.lua')
	d=nil
end
-- usage1: l console "d'test_tt.lua'"
-----> runs test_tt.lua
-- usage2: l console
function ctor()
	if d==nil then 
		this("exit",{}) 
	else
		dbg.console()
		this("exit",{})
	end
end

function dtor()
end

function onCallback(w, userData)
end

function frameMove(fElapsedTime)
end
