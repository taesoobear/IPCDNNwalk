
require("config")
require("module")
require("common")
require("gym_walk/getSeq_common")


function calcHalfCycle(contact1, contact2)
	local mid1=contact1[3]*0.5+contact1[2]*0.5
	local mid2=contact2[3]*0.5+contact2[2]*0.5
	return math.abs(mid2-mid1)
end
function getTouchOffDur(contact, halfCycle)
	local mid=contact[3]*0.5+contact[2]*0.5

	local d1=contact[3]-mid
	local d2=contact[4]-contact[3]
	return CT.vec(d1, d2, halfCycle-d1-d2)
end
function getTouchDownDur(contact, halfCycle)
	local mid=contact[3]*0.5+contact[2]*0.5

	local d1=contact[2]-contact[1]
	local d2=mid-contact[2]
	return CT.vec(halfCycle-d1-d2, d1, d2)
end


-- only for biped
function getSeq(input)

	local Lcontact, Rcontact
	if input.LRcontact then
		Lcontact=input.LRcontact
		Rcontact=input.LRcontact
	end

	if input.contacts then
		Lcontact=input.contacts[1]
		Rcontact=input.contacts[2]
	end

	local function totable(contact,i)
		if type(contact[i])~='table' then
			contact[i]=contact[i]:toTable()[3]
		end
	end
	totable(Lcontact,1)
	totable(Lcontact,2)
	totable(Rcontact,1)
	totable(Rcontact,2)

	local hL
	local hR

	if Lcontact[1][1]>Rcontact[1][1] then
		hL=calcHalfCycle(Rcontact[1], Lcontact[1])*0.5+calcHalfCycle(Rcontact[2], Lcontact[2])*0.5
		hR=calcHalfCycle(Lcontact[1], Rcontact[2])
	else
		hL=calcHalfCycle(Lcontact[1], Rcontact[1])*0.5+calcHalfCycle(Lcontact[2], Rcontact[2])*0.5
		hR=calcHalfCycle(Rcontact[1], Lcontact[2])
	end

	local h=hL*2.0/3.0+hR*1.0/3.0
	local seq={
		touchOff=getTouchOffDur(Lcontact[1],h)*0.5+getTouchOffDur(Lcontact[2],h)*0.5,
		touchDown=getTouchDownDur(Lcontact[1],h)*0.5+getTouchOffDur(Lcontact[2],h)*0.5,
	}
	seq.touchOff=seq.touchOff*(1.0/30.0)
	seq.touchDown=seq.touchDown*(1.0/30.0)

	printTable(seq)
	print(seq.touchDown)
	print(seq.touchOff)
	print(seq.touchDown:sum())
	--print(seq.touchOff:sum()) -- same
	seq.input=input
	return seq
end


function getScaledSeq(defaultSeq, halfCycle, sp_per_sw)
			local _dur, _phaseStart, _phaseEnd, _toe, _heel=unpack({0,1,2,3,4})

				--local flight=(halfCycle-halfStance)*2
				--assert(flight>0.2-1e-3)
				local seq={
					touchOff=
					CT.mat(5,3, -- 5 by 3 matrix
					0,0,0, --duration
					1.5,                  2,         3,    --phaseStart
					2,                    3,         3.5,  -- phaseEnd
					1 ,               1,     0,  -- toe
					1,               0,     0),  -- heel
					touchDown=
					CT.mat(5,3, -- 5 by 3 matrix
					0,0,0,
					3.5,      0    ,         1,
					4,        1    ,         1.5,
					0 ,            0,     1,
					0,            1,     1),
				}

				local halfCycleOrig=defaultSeq.touchOff:sum()

				local stanceDuration_orig=0
				local stanceDuration_new=0
				if true then 
					-- adjust only stance phases
					local flightOrig=defaultSeq.touchOff(2)
					local stanceOrig=halfCycleOrig-flightOrig

					local stanceNew
					if not sp_per_sw then
						stanceNew=stanceOrig
					else
						stanceNew=flightOrig*sp_per_sw
						stanceNew=math.max(stanceNew, 0.025)
					end
					seq.touchOff:row(0):range(0,2):assign(defaultSeq.touchOff:range(0,2)*(stanceNew/stanceOrig))
					seq.touchOff:set(0,2, flightOrig)
					seq.touchDown:row(0):range(1,3):assign(defaultSeq.touchDown:range(1,3)*(stanceNew/stanceOrig))
					seq.touchDown:set(0,0, flightOrig)

					RE.output("sp/sw ratio", sp_per_sw)
					RE.output("sp/sw ratio2", stanceNew/flightOrig)

					stanceDuration_orig=stanceDuration_orig+stanceOrig
					stanceDuration_new=stanceDuration_new+stanceNew

					local hcN=seq.touchOff:row(0):sum()
					seq.touchOff:row(0):assign(seq.touchOff:row(0)*(halfCycle/hcN))
					hcN=seq.touchDown:row(0):sum()
					seq.touchDown:row(0):assign(seq.touchDown:row(0)*(halfCycle/hcN))
				else
					seq.touchOff:row(0):assign(defaultSeq.touchOff*(halfCycle/halfCycleOrig))
					seq.touchDown:row(0):assign(defaultSeq.touchDown*(halfCycle/halfCycleOrig))
				end
				return seq, stanceDuration_new, stanceDuration_orig
			end

function makeSameLength(L,R)
	local diff=R:row(0):sum()-L:row(0):sum()
	if diff>0 then
		-- R sequence is longer than L sequence, so make them be same length.
		R:set(0,R:cols()-1, R(0, R:cols()-1)-diff)
	else
		L:set(0, L:cols()-1, L(0, L:cols()-1)+diff)
	end
end

local getSeq={}
function getSeq.extract(mat, check)
	local _dur, _phaseStart, _phaseEnd=unpack({0,1,2})
	local function mergeLast(mat, col)
		local last=mat:column(mat:cols()-1):copy()
		last:set(_dur, last(_dur)+col(_dur))
		last:set(_phaseEnd, col(_phaseEnd))
		mat:column(mat:cols()-1):assign(last)
	end
	local out=mat:sub(0,0,0,1):copy()
	local currVal=check(mat:column(0))
	for i=1, mat:cols()-1 do
		local col=mat:column(i)
		local v=check(col)
		if v==currVal then
			mergeLast(out,  col)
		else
			out=out..col:column()
			currVal=v
		end
	end
	return out
end

if false then
	-- for testing
	input_walk={
		--run
		Lcontact={{252, 254, 270,274},  {288, 291, 305, 310},},
		Rcontact={{270,273,287,291}, {306, 309, 324, 327},}
	}
	input_run={
		--run
		Lcontact={{754,756,761,765}, {778,780,784,789},   },
		Rcontact={{766,768,773,777}, {790,792,796,801},   },
	}
	getSeq(input_walk)
	getSeq(input_run)
end

return getSeq
