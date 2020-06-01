
function findPhase(seq, phase)
	local T=0
	local _phaseStart=1
	local _phaseEnd=2
	for i=0, seq:cols()-1 do
		if seq(_phaseStart, i)<=phase and phase<=seq(_phaseEnd,i) then
			local t=sop.map(phase,seq(_phaseStart, i), seq(_phaseEnd,i), 0, seq(0,i))
			return T+t
		end
		T=T+seq(0,i)
	end
	return nil
end
function countPhase(seq, phase)

	local count=0
	local _phaseStart=1
	local _phaseEnd=2
	for i=0, seq:cols()-1 do
		if seq(_phaseStart, i)<phase+1e-3 and phase<seq(_phaseEnd,i) then
			count=count+1
		end
	end
	return count
end
function changeLength(seq, newLength)
	local out=seq:copy()
	out:row(0):assign(seq:row(0)*(newLength/seq:row(0):sum()))
	return out
end
function mergeLast(mat, col)
	local _dur, _phaseStart, _phaseEnd, _toe, _heel=unpack({0,1,2,3,4})
	local last=mat:column(mat:cols()-1):copy()
	last:set(_dur, last(_dur)+col(_dur))
	last:set(_phaseEnd, col(_phaseEnd))
	mat:column(mat:cols()-1):assign(last)
end

function clipLeft(mat, dur)
	assert(mat)
	local t=0
	local out
	for i=0, mat:cols()-1 do
		t=t+mat(0,i)
		if t>dur then
			out=mat:sub(0,0, i,0):copy()

			local orig=out(0,0)
			out:set(0, 0, t-dur)
			out:set(1, 0, sop.map(orig-(t-dur), 0, orig, mat(1,i), mat(2,i)))
			break
		end
	end
	return out
end
function clipBoth(mat, left, right)
	local out=clipRight(mat, right)
	out=clipLeft(out, left)
	return out
end
function clipRight(mat, dur)
	local t=0
	local out
	for i=0, mat:cols()-1 do
		t=t+mat(0,i)
		if t>dur then
			local orig=mat(0,i)
			if(orig-(t-dur)<0.0001) then
				out=mat:sub(0,0,0,i):copy()
				out:set(0,out:cols()-1, out(0,-1)+orig-(t-dur))
				return out
			end

			out=mat:sub(0,0, 0,i+1):copy()

			out:set(0, i, orig-(t-dur))
			if out:rows()>2 then
				out:set(2, i, sop.map(orig-(t-dur), 0, orig, mat(1,i), mat(2,i)))
			end
			break
		end
	end
	return out
end
function extract(mat, check)
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
function convertClipToDur(L)
	local _dur, _phaseStart, _phaseEnd, _toe, _heel=unpack({0,1,2,3,4})
	local dur=extract(L, function (col) return col(_heel)== 1 or col(_toe)==1 end, 1):row(0):copy()
	local isContactAtStart=L(_heel,0)==1 or L(_toe,0)==1
	return {isContactAtStart, dur}
end
function _isContact(dur, switchTime)
	local id=BipedPlanner._getSegmentID(switchTime, dur[2])
	local t_local=id.y
	id=id.x

	local initialContact=dur[1]
	local mod2=math.fmod(id,2)==0
	if (initialContact and mod2) or (not initialContact and not mod2) then
		return true
	end
	return false
end
