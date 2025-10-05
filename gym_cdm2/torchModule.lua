EnvNormalize=LUAclass()
function EnvNormalize:__init(ob_rms, clipob, gamma, epsilon)
	assert(ob_rms)
	self.ob_rms=ob_rms
	self.clipob=clipob or 10.0
	self.gamma=gamma or 0.99
	self.epsilon=epsilon or 1e-8
end
function EnvNormalize:process(obs)
	local obs = np.clip((obs - self.ob_rms.mean) / np.sqrt(self.ob_rms.var + self.epsilon), -self.clipob, self.clipob)
	return obs
end

TorchAgent=LUAclass()

function TorchAgent:__init(filename, spec_id)
	self.spec_id=spec_id
	if not TorchModule then
		-- use python3 instead.
		python.F('gym_cdm2.torchModule', 'loadAgent', spec_id, filename)
		return
	end

	if true then -- always rebuild module. 
	--if not (os.isFileExist(filename..'.module')) then
		-- this script first calculates a hash to avoid repeated conversions.
		os.execute('python3 convertPTtoTorchScript.py --env-name='..spec_id)
	end
	if os.isFileExist(filename..'.ob_rms') then
		--print('loading '..filename..'.module')
		--self.module=TorchModule(filename..".module")
		print('loading '..filename..'.ptl')
		self.module=TorchModule(filename..".ptl")

		local ob_rms={}
		ob_rms.mean=vectorn()
		ob_rms.var=vectorn()
		do
			local file=util.BinaryFile()
			file:openRead(filename..'.ob_rms')
			file:unpack(ob_rms.mean)
			file:unpack(ob_rms.var)
			file:close()
		end

		self.vec_norm=EnvNormalize(ob_rms)
	end
end

function TorchAgent:reset(userInput)
	local obs=reset(0)
	return obs
end


function TorchAgent:getAction(obs)
	if not TorchModule then
		local action=vectorn()
		-- use python3 instead.
		python.F('gym_cdm2.torchModule', 'getAction', self.spec_id, obs, action)
		return action
	end
	if self.vec_norm then
		obs=self.vec_norm:process(obs)
	end
    local action
	if self.module then
		assert(self.vec_norm)
		action=self.module:forward(obs)
	else
		print('using zero action')
		action=CT.zeros(get_dim()(1))
	end
	return action
end
