
require("config")
require("module")
require("common")


function calcCycle(contact1, contact2)
	local mid1=contact1[3]*0.5+contact1[2]*0.5
	local mid2=contact2[3]*0.5+contact2[2]*0.5
	return math.abs(mid2-mid1)
end

function getDur(contact, contact2, halfCycle)
	local mid=contact[3]*0.5+contact[2]*0.5
	local d1=contact[3]-mid
	local d2=contact[4]-contact[3]
	local mid2=contact2[3]*0.5+contact2[2]*0.5
	local d1n=contact2[2]-contact2[1]
	local d2n=mid2-contact2[2]
	return CT.vec(d1, d2, contact2[1]-contact[4], d1n, d2n)
end

function getSeq(input)
	if input.LRcontact then
		input.Lcontact=input.LRcontact
		input.Rcontact=input.LRcontact
	end

	if input.contact then
		input.Lcontact=input.contact
		input.Rcontact=input.contact
	end
	if input.contacts then
		input.Lcontact=input.contacts[1]
		input.Rcontact=input.contacts[#input.contacts]
	end

	local hL=calcCycle(input.Lcontact[1], input.Lcontact[2])
	local hR=calcCycle(input.Rcontact[1], input.Rcontact[2])

	local seq={
		cycle=getDur(input.Lcontact[1],input.Lcontact[2])
	}
	seq.cycle=seq.cycle*(1.0/30.0)

	printTable(seq)
	print(seq.cycle)
	print(seq.cycle:sum())
	--print(seq.touchOff:sum()) -- same
	seq.input=input
	return seq
end

if false then
	local input_jump={
		--         steady  heel  toe     toe  heel   steady
		LRcontact={{52, 54, 126, 132,},{148  ,150, 221, 228}},
	}
	-- for testing
	getSeq(input_jump)
end
