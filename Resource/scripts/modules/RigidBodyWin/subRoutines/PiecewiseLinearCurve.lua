math.PiecewiseLinearCurve=LUAclass()

function math.PiecewiseLinearCurve:__init(timing, value)
	self.timing=math.__checkVec(timing):copy()
	self.value=math.__checkVec(value):copy()
	assert(self.timing(0)<=0)
end

function math.PiecewiseLinearCurve:sample(time)
	local timing=self.timing
	local value=self.value
	local iint=0
	for i=1, timing:size()-1 do
		if time<timing(i) then
			break
		end
		iint=i
	end
	if iint>timing:size()-2 then
		iint=timing:size()-2
	end
	return sop.map(time, timing(iint), timing(iint+1), value(iint), value(iint+1))
end
function math.PiecewiseLinearCurve.test()
	a=math.PiecewiseLinearCurve:new({0,0.15,0.5,1},{0,1,2,3})
	print('timing',a.timing)
	print('value', a.value)
	local function t(...)
		for i,v in ipairs({...}) do
			print(v, a:sample(v))
		end
	end
	t(0,0.15,0.5,1, 0.6, 0.7, 0.8, 0.9, 1)
end
