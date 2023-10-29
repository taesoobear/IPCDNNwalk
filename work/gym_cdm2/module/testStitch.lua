require("config")
require("module")
require("common")
require("subRoutines/MatplotLib")
require("gym_cdm/module/DiscontinuityRemover")

function ctor()
	print('ctor')
	plotter=MatplotLib()
	local numSample=30
	local xfn=vectorn(numSample)
	local yfn=vectorn(numSample)
	local yfn2=vectorn(numSample)
	for i=0, numSample-1 do
		xfn:set(i, i)
		--		xfn:set(i, pendPos(i):x())
		yfn:set(i, i)
		--		yfn:set(i, pendPos(i):y())
		yfn2:set(i, i*i/numSample)
	end

	function plot(xfn, yfn2, stitched, fn)
		plotter:figure{1, nsubfig={1,1}, subfig_size={3,3}} -- two by one plotting area.
		plotter:add('grid(True)')
		plotter:subplot(0,0)
		plotter:plot(xfn, yfn)
		--plotter:plot(yfn2, xfn)
		plotter:plot(xfn+stitched:rows()-xfn:size(), yfn2)
		plotter:plot(CT.xrange(stitched:rows()), stitched:column(0))
		plotter:xlabel('x')
		plotter:ylabel('t')
		--plotter:legends('x(t)', 'z(t)')
		plotter:savefig(fn)
		plotter:close()
	end
	do
		local stitched=vectorn(yfn:size()+yfn2:size()-1)
		local s=yfn:size()-1
		stitched:range(0, s+1):assign(yfn)

		local delta=DiscontinuityRemoverLinear(25)
		delta:startBlending(CT.vec(yfn(s)), CT.vec(yfn2(0)) )
		local state=CT.vec(yfn(s))
		for i=0, yfn2:size()-1 do
			state:set(0, yfn2(i))
			delta:update(state)
			stitched:set(i+s, state(0))
		end
		plot(xfn, yfn2, stitched:column(), 'linearstich.png')
	end
	do
		local stitched=vectorn(yfn:size()+yfn2:size()-1)
		local s=yfn:size()-1
		stitched:range(0, s+1):assign(yfn)

		local delta=DiscontinuityRemoverQuintic(25)
		-- pos1, pos2, vel1, vel2
		delta:startBlending(CT.vec(yfn(s)), CT.vec(yfn2(0)) , CT.vec(yfn(s)-yfn(s-1)), CT.vec(yfn2(1)-yfn2(0)))
		local state=CT.vec(yfn(s))
		for i=0, yfn2:size()-1 do
			state:set(0, yfn2(i))
			delta:update(state)
			stitched:set(i+s, state(0))
		end
		plot(xfn, yfn2, stitched:column(), 'quinticstitch.png')
	end
	do
		local stitched=vectorn(yfn:size()+yfn2:size()-1)
		local s=yfn:size()-1
		stitched:range(0, s+1):assign(yfn)

		local delta=DiscontinuityRemoverQuinticHybrid(15, 25)
		-- pos1, pos2, vel1, vel2
		delta:startBlending(CT.vec(yfn(s)), CT.vec(yfn2(0)) , CT.vec(yfn(s)-yfn(s-1)), CT.vec(yfn2(1)-yfn2(0)))
		local state=CT.vec(yfn(s))
		for i=0, yfn2:size()-1 do
			state:set(0, yfn2(i))
			delta:update(state)
			stitched:set(i+s, state(0))
		end
		plot(xfn, yfn2, stitched:column(), 'quinticHybridStitch.png')
	end

end
function frameMove(fElapsedTime)
end

function onCallback(w, userData)
end

function dtor()
	print('dtor')
end

