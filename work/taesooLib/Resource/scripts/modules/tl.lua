
do
	tl={}
	function tl.arange(i1,i2,i3)
		local v=intvectorn()
		if i2 then
			if i3 then
				v:colon(i1, i2,i3)
			else
				v:colon(i1, i2,1)
			end
		else
			v:colon(0, i1,1)
		end
		return v
	end
	function tl.linspace(i1,i2,n)
		local v=vectorn()
		v:linspace(i1,i2,n or 50)
		return v
	end
	-- convert from torch tensor to vectorn
	function tl.vec(torchtensor)
		if type(torchtensor)=='table' then
			local out=vectorn()
			out:setValues(unpack(torchtensor))
			return out
		end
		if (torchtensor:size():size()==1) then
			local out=vectorn()
			out:setValues(unpack(torch.totable(torchtensor)))
			return out
		else
			assert(torchtensor:size():size()==2 and torchtensor:size()[1]==1)
			local out=vectorn()
			out:setValues(unpack(torch.totable(torchtensor)[1]))
			return out
		end
	end
	-- intvectorn to vectorn
	function tl.asFloat(source)
		return source:asFloat()
	end
	-- selective assignment.
	-- e.g. numpy code : target[index]=target[index]*2 ->
	-- tl.assign(target, index, target:extract(index)*2) 
	function tl.assign(target, index, value)
		target:assignSelective(index, value)
	end
	function tl.reversed(source)
		local out=source:copy()
		for i=0, source:size()-1 do
			out:set(i, source(source:size()-i-1))
		end
		return out
	end
	-- convert from torch tensor to matrixn
	function tl.mat(tbl)

		if type(tbl)~='table' then
			tbl=tbl:totable()
		end
		local out=matrixn(#tbl, #tbl[1])
		for i=0, out:rows()-1 do
			out:row(i):setValues(unpack(tbl[i+1]))
		end
		return out
	end
	-- concatenate
	function tl.concatenateRows(...)
		dbg.console()
	end
	function tl.concatenateColumns(...)
		local input={...}
		local nc=0
		for i,v in ipairs(input) do
			nc=nc+v:cols()
		end
		local out=matrixn(input[1]:rows(), nc)
		nc=0
		for i,v in ipairs(input) do
			out:sub(0,0, nc, nc+v:cols()):assign(v)
			nc=nc+v:cols()
		end

		return out
	end
end
-- torch-taesooLib binding functions
if torch then
	function tl.torch(vec_or_mat)
		return torch.DoubleTensor(vec_or_mat:values())
	end
	if false then
		-- test
		-- test taesooLib <-> torch tensor conversion
		a=CT.vec(1,2,3)
		b=torch.DoubleTensor(a:values())
		c=tl.vec(b)
		print(c)
	end
	function torch.filter1d(fcn, a)

		-- torch tensor
		local tensor=fcn(torch.DoubleTensor(a:values()))
		return tl.vec(tensor)

	end
	-- simple 4-layer NN-based regressor
	Net=LUAclass()
	function Net:__init(nrows, name)
		self.nrows=nrows
		self.name=name
	end
	function Net:mapping(source, target)
		assert(source:size()==310)
		local output=self.net:forward(tl.torch(source:row()))
		target:assign(tl.vec(output))
	end
	function Net:learn(source, target)
		if true then
			source=source:sub(400,800,0,0):copy()
			target=target:sub(400,800,0,0):copy()
		end
		local net=nn.Sequential()
		--net:add(nn.Normalize(source:cols()))
		net:add(nn.Linear(source:cols(),120))
		--net:add(nn.ReLU())
		net:add(nn.ELU())
		net:add(nn.Linear(120,84))
		net:add(nn.ELU())
		net:add(nn.Linear(84,10))
		net:add(nn.ELU())
		net:add(nn.Linear(10, target:cols()))

		self.net=net

		local fn='torch_network_'..self.name..'.net', net
		if true and os.isFileExist(fn) then
			net=torch.load(fn)
			self.net=net
			-- uncomment the follwing to skip further training.
			return 
		end

		local input=torch.rand(1, source:cols())
		local output=net:forward(input)
		print(net)
		print(input)
		print(output)

		--net:zeroGradParameters() -- zero the internal gradient buffers of the network (will come to this later)
		-- https://github.com/torch/nn/blob/master/doc/criterion.md
		--local criterion = nn.ClassNLLCriterion() 
		local criterion = nn.MSECriterion()
		local trainer = nn.StochasticGradient(net, criterion)
		trainer.learningRate = 1e-2
		trainer.maxIteration = 125 -- just do 25 epochs of training.

		local trSize=source:rows() -- 1000 random input, output
		trainset = {
			size=function() return trSize end
		}
		for i=1,trSize do
			assert(not source:row(i-1):isnan())
			assert(not target:row(i-1):isnan())
			trainset[i]={
				-- 1 by m
				tl.torch(source:row(i-1):row()),
				-- 1 by n
				tl.torch(target:row(i-1):row()),
			}
		end

		for oi=1, 3 do
			print('outer:', oi)
			trainer:train(trainset)

			if true then
				local model=net
				parameters, gradParameters = model:getParameters()
				-- this need to be after :cuda and after :getParameters
				lightModel = model:clone('weight','bias','running_mean','running_std')
				-- do training, and when need to save, call
				torch.save(fn,lightModel)
			else
				torch.save(fn, net)
			end
		end
	end
end
