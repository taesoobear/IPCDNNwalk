MatplotLib=LUAclass()

function MatplotLib.plotFile(y, filename)
	local plotter=MatplotLib()
	if dbg.lunaType(y):sub(1,7)=='vectorn' then
		y=y:column():copy()
	end

	local numSample=y:rows()
	local xfn=CT.linspace(0,1,numSample)

	plotter:figure{1, nsubfig={1,1}, subfig_size={3,3}} -- two by one plotting area.
	plotter:add('grid(True)')
	plotter:subplot(0,0)
	for i=0, y:cols()-1 do
		plotter:plot(xfn, y:column(i))
	end
	plotter:xlabel('x')
	plotter:ylabel('y')
	--plotter:legends('x(t)', 'y(t)', 'z(t)')
	plotter:savefig(filename)
	plotter:close()
end
function MatplotLib:__init()
	print('matplotLib wrapper started')
	self.pythonCodes=array()
	self:addLine(
	[[
# -*- coding: UTF-8 -*-
from numpy import *
import matplotlib
matplotlib.use('Agg')
from pylab import *
import sys
import os
	]]
	)
end
function MatplotLib:add(...)
	local args={...}
	local c=""
	for i,arg in ipairs(args) do
		if type(arg)=='table' and getmetatable(arg) then

		else
			c=c..tostring(arg)
		end
	end
	self:addLine(c)
end
function MatplotLib:legends(...)
	local input={...}
	if type(input[1])=='table' then
		input=input[1]
	end
	
	self:add("legend(('"..table.concat(input, "','").."'))")
end
-- options example: {loc=4}
function MatplotLib:legends2(options, ...)
	local optionstr=self.optionstring(options)
	self:add("legend(('"..table.concat({...}, "','").."')"..optionstr..")")
end
-- 0-indexing. set plot region to (xth row, yth column)
function MatplotLib:subplot(x,y) 	
	self:add('ax=subplot(',self.param.nsubfig[1],',',self.param.nsubfig[2],',',(x+y*self.param.nsubfig[1])+1,')')
end
function MatplotLib:set_xlim(a,b)
	a=tostring(a)
	b=tostring(b)
	self:add('ax.set_xlim('..a..','..b..')')
end
function MatplotLib:set_ylim(a,b)
	a=tostring(a)
	b=tostring(b)
	self:add('ax.set_ylim('..a..','..b..')')
end

-- figure{figid, nsubfig={nrow, ncol}, subfig_size={width, height}}
function MatplotLib:figure(param)
	local defaultParam={
		1, -- figure id
		nsubfig={1,1},
		subfig_size={12,3},
	}
	if param then
		assert(type(param)=='table')
		param=table.merge(defaultParam, param)
		local p=param
		local fig_id=p[1]
		self:add('figure(',fig_id,', figsize=(',p.nsubfig[2]*p.subfig_size[1],
			',',p.nsubfig[1]*p.subfig_size[2],'))')
	else
		param=defaultParam
		self:add('figure(1)')
	end
	self.param=param
end

function MatplotLib:tostr(a)
	local str="("
	if type(a)=='userdata' and getmetatable(a) and getmetatable(a).luna_class=="LVec" then
		for i=1,a:size() do
			str=str..tostring(a(i))..","
		end
	elseif type(a)=='table' then
		for i=1,#a do
			str=str..tostring(a[i])..","
		end
	else
		for i=0,a:size()-1 do
			str=str..tostring(a(i))..","
		end
		-- print('unsupported format:',a)
	end
	return str..")"
end
function MatplotLib.optionstring(options)
	local optionstr='';
	for k,v in pairs(options) do
		if type(v)=='string' then
			optionstr=optionstr..','..k.."='"..v.."'"
		else
			optionstr=optionstr..','..k.."="..v..""
		end
	end
	return optionstr
end
-- options: {linestyle='dotted'}
function MatplotLib:plot(a,b, options)
	if type(a)=='table' then
		assert(#a==#b)
	else
		assert(a:size()==b:size())
	end
	if options then
		local optionstr=self.optionstring(options)
		self:add('plot('..self:tostr(a)..","..self:tostr(b)..optionstr..')')
	else
		self:add('plot('..self:tostr(a)..","..self:tostr(b)..')')
	end
end
function MatplotLib:plotSignal(b, options)
	if type(b)=='table' then
		b=CT.vec(unpack(b))
	end
	a=vectorn(b:size())
	a:colon(0,1,b:size())
	self:plot(a,b,options)
end

-- color: 'r', 'g', 'b', 'c'
-- linestyle='--' or 'dotted' or nil
function MatplotLib:plotColor(a,b, color, linestyle)
	local options={}
	options.color=color
	options.linestyle=linestyle
	self:plot(a,b, options)
end

function MatplotLib:scatter(a,b, options)
	assert(a:size()==b:size())
	if options then
		local optionstr=self.optionstring(options)
		self:add('scatter('..self:tostr(a)..","..self:tostr(b)..optionstr..')')
	else
		self:add('scatter('..self:tostr(a)..","..self:tostr(b)..')')
	end
end
function MatplotLib:savefig(fn)
	self:add('savefig("'..fn..'")')
	self.fn=fn
	if os.isUnix() then
		self:add('os.system("xdg-open '..fn..'")')
	else
		self:add('os.system("start '..fn..'")')
	end
end
function MatplotLib:xlabel(a)
	self:add('xlabel("'..a..'")')
end
function MatplotLib:ylabel(a)
	self:add('ylabel("'..a..'")')
end
function MatplotLib:addLine(l)
	self.pythonCodes:pushBack(l)
end
function MatplotLib:close()
	self:add('close(1)')
	self.fn=self.fn or 'temp'
	util.writeFile(self.fn..'.py', table.concat(self.pythonCodes,'\n'))
	os.execute('python3 '..self.fn..'.py')
	print('matplotLib wrapper finished (see '..self.fn..' )')
	self:__init()
end

   
