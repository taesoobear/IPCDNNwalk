pythonHelper=LUAclass()

function pythonHelper:__init()
	print('matplotLib wrapper started')
	self.pythonCodes=array()
	self:addLine(
	[[
from numpy import *
import matplotlib
matplotlib.use('Agg')
from pylab import *
import sys
import os
	]]
	)
end
function pythonHelper:add(...)
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
function pythonHelper:legends(...)
	self:add("legend(('"..table.concat({...}, "','").."'))")
end
-- 0-indexing. set plot region to (xth row, yth column)
function pythonHelper:subplot(x,y) 	self:add('subplot(',self.param.nsubfig[1],',',self.param.nsubfig[2],',',(x+y*self.param.nsubfig[1])+1,')')
end

-- figure{figid, nsubfig={nrow, ncol}, subfig_size={width, height}}
function pythonHelper:figure(param)
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

function pythonHelper:tostr(a)
	local str="("
	if type(a)=='userdata' and getmetatable(a) and getmetatable(a).luna_class=="LVec" then
		for i=1,a:size() do
			str=str..tostring(a(i))..","
		end
	elseif string.sub(getmetatable(a).luna_class,1,7)=='matrixn' then
		local contents={}
		local data=a
		array.pushBack(contents,'\\\n[')
		for i=0, data:rows()-1 do
			local line="["
			for j=0, data:cols()-1 do
				line=line..tostring(data(i,j))..','
			end
			line=line..'],'
			array.pushBack(contents,line)
		end
		contents[#contents]=contents[#contents]..']'
		return table.concat(contents,'\n')
	else
		for i=0,a:size()-1 do
			str=str..tostring(a(i))..","
		end
		-- print('unsupported format:',a)
	end
	return str..")"
end
function pythonHelper:plot(a,b)
	assert(a:size()==b:size())
	self:add('plot('..self:tostr(a)..","..self:tostr(b)..')')
end
function pythonHelper:savefig(fn)
	self:add('savefig("'..fn..'")')
	if os.isUnix() then
		self:add('os.system("gnome-open '..fn..'")')
	else
		self:add('os.system("start '..fn..'")')
	end
end
function pythonHelper:xlabel(a)
	self:add('xlabel("'..a..'")')
end
function pythonHelper:ylabel(a)
	self:add('ylabel("'..a..'")')
end
function pythonHelper:addLine(l)
	self.pythonCodes:pushBack(l)
end
function pythonHelper:close()
	self:add('close(1)')
	util.writeFile('temp.py', table.concat(self.pythonCodes,'\n'))
	os.execute('python temp.py')
	self.pythonCodes={}
	print('matplotLib wrapper finished')
end

Physics.MRDplotwrap=LUAclass()

function Physics.MRDplotwrap:__init()
	self.channels=array()
end
function Physics.MRDplotwrap:add(name, signal, tag)
	if signal:cols()==1 then
		self.channels:pushBack({name=name, tag=tag,signal=signal:column(0):copy()})
	elseif signal:cols()==3 then
		self.channels:pushBack({name=name..'_x', tag=tag,signal=signal:column(0):copy()})
		self.channels:pushBack({name=name..'_y', tag=tag,signal=signal:column(1):copy()})
		self.channels:pushBack({name=name..'_z', tag=tag,signal=signal:column(2):copy()})
	else
		for i=0,signal:cols()-1 do
			self.channels:pushBack({name=name..tostring(i),tag=tag, signal=signal:column(i):copy()})
		end
	end
end
function Physics.MRDplotwrap:init()
	assert(self.channels)
	self.mrdplot=Physics.MRDplot()
	local mrdplot=self.mrdplot
	local channels=self.channels
	mrdplot:initMRD(#channels, channels[1].signal:size(),1)
	for i=1,#channels do
		mrdplot.names:set(i-1, channels[i].name)
		mrdplot.units:set(i-1, channels[i].tag or " ")
		mrdplot.data:column(i-1):assign(channels[i].signal)
	end
end
function Physics.MRDplotwrap:save(fn)
	self.mrdplot:save(fn)
end
pyMrdPlot={}
pyMrdPlot.filename='debug_plot2'
function pyMrdplot_comp3(filenames, startFrame, file_titles)

   startFrame=startFrame or 0
   local mrdplots={}
   local min_rows=10000
   local tags={}
   local names={}
   for i=0, table.getn(filenames)-1 do

      mrdplots[i]=Physics.MRDplot()
      mrdplots[i]:load(filenames[i+1])

   -- mrdplot.units
   -- mrdplot.names
   -- mrdplot.data
      min_rows=math.min(mrdplots[i].data:rows(), min_rows)
	  local nchannels=mrdplots[i].names:size()
	  local p=mrdplots[i]
	  for c=0, nchannels-1 do
		  local n=p.units(c) -- 
		  if names[n] then
			  names[n]=names[n]+1
		  else
			  names[n]=1
		  end
	  end
  end
  local mrdplot=Physics.MRDplotwrap()
   for i=0, table.getn(filenames)-1 do
	  local nchannels=mrdplots[i].names:size()
	  local p=mrdplots[i]
	  for c=0, nchannels-1 do
		  local n=p.units(c) -- 
		  local tag=p.names(c)
		  if tag~="_" and tag~="" and tag~=" " and tag~=nil then
			  mrdplot:add(n.. ' '..file_titles[i+1], p.data:range(startFrame, min_rows,c,c+1), tag)
		  elseif names[n]==1 then 
			  mrdplot:add(n.. ' '..file_titles[i+1], p.data:range(startFrame, min_rows,c,c+1))
		  else
			  mrdplot:add(n.. ' '..file_titles[i+1], p.data:range(startFrame, min_rows,c,c+1), n)
		  end
	  end
  end
  mrdplot:init()
  mrdplot:save("debug_plot_compare.mrd")
  pyMrdplot3('debug_plot_compare.mrd')
  printTable(names)
end
function pyMrdplot_comp2(filenames, startFrames, file_titles, channels)
	-- comp1 과 달리 모든 프레임을 플롯한다.

   local mrdplots={}

   local min_rows=10000
   for i=0, table.getn(filenames)-1 do

      mrdplots[i]=Physics.MRDplot()
      mrdplots[i]:load(filenames[i+1])

   -- mrdplot.units
   -- mrdplot.names
   -- mrdplot.data
      min_rows=math.min(mrdplots[i].data:rows(), min_rows)
   end

   local numfiles=#filenames
   if channels==nil then
      channels={}
      local nchannels=mrdplots[0].names:size()
	  for i=0, nchannels-1 do
		  channels[i+1]={index=CT.ones(numfiles)*i,index2=i, name=mrdplots[0].units(i).." ["..mrdplots[0].names(i).."]"}
      end
  else
	  for i=1, #channels do
		  if channels[i].index==nil then
			  channels[i].index=vectorn(numfiles)
			  for j=1,numfiles do
				  channels[i].index:set(j-1,mrdplots[j-1].units:find(channels[i].key))
				  if channels[i].index(j-1) ==mrdplots[j-1].units:size() then
					  print(channels[i].key ..' not found')
					  units=mrdplots[j-1].units
					  assert(false)
				  end
			  end
		  elseif type(channels[i].index)=='number' then
			  channels[i].index=CT.ones(numfiles)*channels[i].index
		  end
	  end
   end

   if file_titles==nil then
	   file_titles={}
	   file_titles[1]=filenames[1]
	   file_titles[2]=filenames[2]
   end

   for i=0, table.getn(filenames)-1 do
	   local data=mrdplots[i].data
	   py_exec('set', 'data'..i, data:sub(startFrames[i+1], 0, 0,0))
   end

   local nsubfig=table.getn(channels)

   local width=8
   local sub_height=2.4
   py_eval("figure(1, figsize=("..tostring(width)..","..tostring(sub_height*nsubfig).."))")

   local linestyles={',"--"',',"-"'}
   for ii=0, nsubfig-1 do
      
	   py_eval("subplot("..tostring(nsubfig).."1"..tostring(ii+1)..")")

	   for jj=0, table.getn(filenames)-1 do
		   i=channels[ii+1].index(jj)
		   py_eval("t=arange(shape(get('data"..jj.."'))[0])")

		   if linestyles[jj+1] then
			   py_eval("plot(t, get('data"..jj.."')[:,"..i.."]"..linestyles[jj+1]..")")
		   else
			   py_eval("plot(t, get('data"..jj.."')[:,"..i.."])")
		   end
		   py_eval("ylabel('"..channels[ii+1].name.."')")
	   end

	   if ii==0 then
		   py_eval("legend(('"..file_titles[1].."','"..file_titles[2].."'))")
	   end

   end

   if os.isFileExist(pyMrdPlot.filename..'.png') then
	   os.deleteFiles(pyMrdPlot.filename..'.eps')
	   os.deleteFiles(pyMrdPlot.filename..'.png')
   end
   py_eval("savefig('"..pyMrdPlot.filename..".eps')")
   py_eval("savefig('"..pyMrdPlot.filename..".png')")
   py_eval("close(1)")

end


function pyMrdplot_comp(filenames, startFrame, file_titles, channels)

   startFrame=startFrame or 0
   local mrdplots={}

   local min_rows=10000
   for i=0, table.getn(filenames)-1 do

      mrdplots[i]=MRDplot()
      mrdplots[i]:load(filenames[i+1])

   -- mrdplot.units
   -- mrdplot.names
   -- mrdplot.data
      min_rows=math.min(mrdplots[i].data:rows(), min_rows)
   end

   if channels==nil then
      channels={}
      local nchannels=mrdplots[0].names:size()
	  for i=0, nchannels-1 do
		  channels[i+1]={index=i,name=mrdplots[0].units(i).." ["..mrdplots[0].names(i).."]"}
      end
   end

   if file_titles==nil then
	   file_titles={}
	   file_titles[1]=filenames[1]
	   file_titles[2]=filenames[2]
   end

   for i=0, table.getn(filenames)-1 do
	   py_exec('set', 'data'..i, mrdplots[i].data:range(startFrame, min_rows, 0, mrdplots[i].data:cols()))
   end

   local nsubfig=table.getn(channels)

   local width=8
   local sub_height=2.4
   py_eval("figure(1, figsize=("..tostring(width)..","..tostring(sub_height*nsubfig).."))")

   for ii=0, nsubfig-1 do
      
	   py_eval("subplot("..tostring(nsubfig).."1"..tostring(ii+1)..")")
	   py_eval("t=arange(shape(get('data0'))[0])")

	   for jj=0, table.getn(filenames)-1 do
		   i=channels[ii+1].index
		   if jj==1 then
			   --	    py_eval("plot(t, get('data"..jj.."')[:,"..i.."], '.')")
			   py_eval("plot(t, get('data"..jj.."')[:,"..i.."])")
		   else
			   py_eval("plot(t, get('data"..jj.."')[:,"..i.."])")
		   end
		   py_eval("ylabel('"..channels[ii+1].name.."')")
	   end

	   if ii==0 then
		   py_eval("legend(('"..file_titles[1].."','"..file_titles[2].."'))")
	   end

   end

   os.deleteFiles(pyMrdPlot.filename..'.eps')
   os.deleteFiles(pyMrdPlot.filename..'.png')
   py_eval("savefig('"..pyMrdPlot.filename..".eps')")
   py_eval("savefig('"..pyMrdPlot.filename..".png')")
   py_eval("close(1)")

end

function pyMrdplot3(filename,pngfilename, noopen)
	pngfilename=pngfilename or filename
   local mrdplot=Physics.MRDplot()
   mrdplot:load(filename)

   local names={}
   local tags={}
   for i=0, mrdplot.names:size()-1 do
	   names[i+1]=mrdplot.units(i)
	   tags[i+1]=mrdplot.names(i)
	   names[i+1]=string.gsub(names[i+1],'_',' ')
   end
   local contents={}
   array.pushBack(contents,'data=\\\n[')
   for i=0, mrdplot.data:rows()-1 do
	   local line="["
	   for j=0, mrdplot.data:cols()-1 do
		   line=line..tostring(mrdplot.data(i,j))..','
	   end
	   line=line..'],'
	   array.pushBack(contents,line)
   end
   contents[#contents]=contents[#contents]..']'
   
   array.pushBack(contents, python.ipairsToCode('names', names))
   array.pushBack(contents, python.ipairsToCode('tags', tags))

   local fn=string.sub(filename, 1,-5)
   util.writeFile(fn..'.py',table.concat(contents,'\n'))
   if os.isUnix() then
	   os.execute('python pyMrdplot.py '..fn..' 40') -- max 40 rows
	   if pngfilename then
		   pngfilename=string.sub(pngfilename,1,-5)
		   os.execute('mv '..fn..'0.png'..' '..pngfilename..'.png')
		   os.execute('mogrify -format gif -resize 50% '..pngfilename..'.png') -- to reduce file size
		   if not noopen then os.execute('gnome-open '..pngfilename..'.png') end
	   else
		   if not noopen then os.execute('gnome-open '..fn..'0.png') end
	   end
   else
	   os.execute('python pyMrdplot.py '..fn..' 4') -- max 4 rows due to a windows specific bug
	   if not noopen then os.execute('start '..fn..'0.png') end
   end

end


function pyMrdplot(filename, option)
   local mrdplot=Physics.MRDplot()
   
   mrdplot:load(filename)
   -- mrdplot.units
   -- mrdplot.names
   -- mrdplot.data

   local nsubfig=mrdplot.names:size()
   for i=0, mrdplot.names:size()-1 do
      print(mrdplot.names(i))
   end

   local sub_height=3
   py_eval("figure(1, figsize=(12,"..tostring(sub_height*nsubfig).."))")
   py_exec('set', 'data', mrdplot.data)

   for i=0, nsubfig-1 do
      py_eval("subplot("..tostring(nsubfig)..",1,"..tostring(i+1)..")")
      py_eval("t=arange(shape(get('data'))[0])")
      --py_eval("print('len', size(t))")
      py_eval("plot(t, get('data')[:,"..i.."])")
      py_eval("ylabel('"..mrdplot.units(i).." ["..mrdplot.names(i).."]')")
   end

   os.deleteFiles(pyMrdPlot.filename..'.eps')
   os.deleteFiles(pyMrdPlot.filename..'.png')
   py_eval("savefig('"..pyMrdPlot.filename..".eps')")
   py_eval("savefig('"..pyMrdPlot.filename.."')")
   py_eval("close(1)")

   if os.isUnix() then
	   os.execute('gnome-open '..pyMrdPlot.filename..'.png')
   else
	   os.execute('start '..pyMrdPlot.filename..'.png')
   end
end


   
