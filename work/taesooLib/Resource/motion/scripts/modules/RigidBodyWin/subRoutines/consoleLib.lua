require("module")

--dbg.startTrace()
function ctor()
	main()
	this("exit",0)
end

function dtor()
end

function onCallback(w, userData)
end

function frameMove(fElapsedTime)
end
