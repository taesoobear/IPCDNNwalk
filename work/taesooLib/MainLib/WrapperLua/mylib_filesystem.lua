

function os.createDir(path)

   if os.isUnix() then
      os.execute('mkdir "'..path..'"')
   else
      os.execute("md "..string.gsub(path, '/', '\\'))
   end
end

function os.rename(name1, name2)

   if os.isUnix() then
      os.execute('mv "'..name1..'" "'..name2..'"')
   else
      local cmd=string.gsub("move "..name1.." "..name2, '/', '\\')
      print(cmd)
      os.execute(cmd)
   end
end

function os.deleteFiles(mask)

   if os.isUnix() then
      os.execute("rm "..mask)
   else
      os.execute("del "..string.gsub(mask, '/', '\\'))
   end
end

function os.parentDir(currDir)
   return os.rightTokenize(string.gsub(currDir, "\\","/"), "/")
end

function os.relativeToAbsolutePath(folder,currDir)

	if string.sub(folder, 1,1)~="/" then
		currDir=currDir or os.currentDirectory()
		while(string.sub(folder,1,3)=="../") do
			currDir=os.parentDir(currDir)
			folder=string.sub(folder,4)
		end
		while(string.sub(folder,1,2)=="./") do
			folder=string.sub(folder,3)
		end
		if folder=="" then
			folder=currDir
		else
			folder=currDir.."/"..folder
		end
	end
	return folder
end
function os.absoluteToRelativePath(folder, currDir) -- param1: folder or file name
	if(string.sub(folder,1,1)~="/") then return folder end
	currDir=currDir or os.currentDirectory()
	local n_ddot=0
	while string.sub(folder,1,#currDir)~=currDir or currDir=="" do
		currDir=os.parentDir(currDir)
		n_ddot=n_ddot+1
	end
	local str=""
	for i=1,n_ddot do
		str=str.."../"
	end
	--print(n_ddot, currDir)
	str=str..string.sub(folder,#currDir+2)
	return string.trimSpaces(str)
end

function os.currentDirectory()
	if os.isUnix() then
		return os.capture('pwd')
	else
		return os.capture('cd')
	end
end
function os.copyFile(mask, mask2)

   if os.isUnix() or os.isMsysgit() then
	   if mask2 then
		   os.execute('cp "'..mask..'" "'..mask2..'"')
	   else
		   os.execute("cp "..mask)
	   end
   else
	   if mask2 then
		   print('copy "'..      string.gsub(mask, '/', '\\')..'" "'..string.gsub(mask2, '/', '\\')..'"')
		   os.execute('copy "'..      string.gsub(mask, '/', '\\')..'" "'..string.gsub(mask2, '/', '\\')..'"')
	   else
		   print("copy "..      string.gsub(mask, '/', '\\'))
		   os.execute("copy "..      string.gsub(mask, '/', '\\'))
	   end
   end
end

-- copy folder a to folder b (assuming folder b doesn't exists yet.)
-- otherwise, behaviors are undefined.
-- os.copyResource("../a", "../b", {'%.txt$', '%.lua$'}) 
function os.copyRecursive(srcFolder, destFolder, acceptedExt)
	if string.sub(destFolder, -1,-1)=="/" then
		destFolder=string.sub(destFolder, 1, -2)
	end

	acceptedExt=acceptedExt or os.globParam.acceptedExt
	local backup=os.globParam.acceptedExt

	os.createDir(destFolder)
	-- first copy files directly in the source folder
	if true then
		os.globParam.acceptedExt=acceptedExt
		local files=os.glob(srcFolder.."/*")
		os.globParam.acceptedExt=backup

		for i,f in ipairs(files) do
			os.copyFiles(f, destFolder.."/"..string.sub(f, #srcFolder+2))
		end
	end

	-- recursive copy subfolders
	local folders=os.globFolders(srcFolder)
	
	for i, f in ipairs(folders) do
		os.copyRecursive(srcFolder.."/"..f, destFolder.."/"..f,  acceptedExt)
	end
end
function os.copyFiles(src, dest, ext) -- copy source files to destination folder and optinally change file extensions.
   if os.isUnix() then
      os.execute('cp "'..src..'" "'..dest..'"')
   else
      local cmd="copy "..string.gsub(src, '/', '\\').." "..string.gsub(dest, '/', '\\')
      print(cmd)
      os.execute(cmd)
   end

   if ext then
      local files=os.glob(dest.."/*"..ext[1])

      for i,file in ipairs(files) do
	 if string.find(file, ext[1])~=string.len(file) then
	    os.rename(file,  string.gsub(file, ext[1], ext[2]))
	 end
      end
   end
end

function os._globWin32(attr, mask, ignorepattern)
	local cmd="dir /b/a:"..attr..' "'..string.gsub(mask, "/", "\\")..'" 2>nul'
	local cap=os.capture(cmd, true)
	local tbl=string.tokenize(cap, "\n")
	tbl[table.getn(tbl)]=nil
	local files={}
	local c=1
	local prefix=""
	--if string.find(mask, "/") then
		--prefix=os.parentDir(mask).."/"
	--else
		--prefix=""
	--end
	for i,fn in ipairs(tbl) do
		if string.sub(fn, string.len(fn))~="~" then
			if not (ignorepattern and string.isMatched(fn, ignorepattern)) then
				files[c]=prefix..fn 
				c=c+1
			end
		end
	end
	return files 
end
os.globParam={}
os.globParam.ignorePath={'^%.', '^CVS'}
os.globParam.acceptedExt={'%.txt$','%.pdf$', '%.inl$', '%.lua$', '%.cpp$', '%.c$', '%.cxx$', '%.h$', '%.hpp$', '%.py$','%.cc$'}

function os.globFolders(path) -- os.globFolders('..') list all folders in the .. folder
	if path==nil or path=="." then
		path=""
	end
	
	if os.isUnix() then
		local path='"'..path..'"'	
		if path=='""' then path="" end
		local tbl=string.tokenize(os.capture('ls -1 -p '..path.." 2> /dev/null",true), "\n")
		local tbl2=array:new()
		local ignorepath=os.globParam.ignorePath
		for i,v in ipairs(tbl) do
			if string.sub(v,-1)=="/" then
				local fdn=string.sub(v, 1,-2)
				if not string.isMatched(fdn, ignorepath) then
					tbl2:pushBack(fdn) 
				end
			end
		end
		return tbl2
	else
		return os._globWin32('D', path, os.globParam.ignorePath)
	end
end

function os._processMask(mask)
	mask=string.gsub(mask, "#", "*")
	local folder, lmask=os.rightTokenize(mask, '/',true)
	local wildcardCheckPass=true
	if not string.find(lmask, '[?*]') then -- wild card
		folder=folder..lmask
		lmask="*"
	end
	if string.sub(folder, -1)~="/" and folder~="" then
		folder=folder.."/"
	end
	return folder,lmask 
end

function os.findgrep(mask, bRecurse, pattern)
	local printFunc={ 
		iterate=function (self, v)
			util.grepFile(v, pattern)
		end
	}	
	os.find(mask, bRecurse, true, printFunc)
end
function os.find(mask, bRecurse, nomessage, printFunc) 
	local printFunc=printFunc or { iterate=function (self, v) print(v) end}
	--mask=string.gsub(mask, "#", "*")
	--mask=string.gsub(mask, "%%", "*")
	if string.find(mask, "[#%%?%*]")==nil then
		-- check if this is a file not a folder
		if os.isFileExist(mask) then
			printFunc:iterate(mask)
			return 
		else
			fn,path=os.processFileName(mask)-- fullpath
			mask=path.."/*"..fn -- try full search
		end
	end
	if not nomessage then
		io.write('globbing '..string.sub(mask,-30)..'                               \r')
	end
	if bRecurse==nil then bRecurse=false end

	if os.isUnix() then 
		local folder, lmask=os.rightTokenize(mask, '/')
		if lmask=="*" then
			mask=folder
			folder, lmask=os.rightTokenize(mask, '/')
		end
		local containsRelPath=false
		if string.find(lmask, '[?*]') then -- wild card
			containsRelPath=true
		end
		local cmd='ls -1 -p '..mask..' 2>/dev/null'
		local tbl=string.tokenize(os.capture(cmd,true), "\n")
		local lenfolder=string.len(folder)
		--print(cmd,mask,#tbl,lenfolder)
		if lenfolder==0 then lenfolder=-1 end
		local acceptedExt=deepCopyTable(os.globParam.acceptedExt)

		if string.find(mask,"%*%.") then
			local idx=string.find(mask,"%*%.")+2
			acceptedExt[#acceptedExt+1]="%."..string.sub(mask,idx)..'$'
			--print(acceptedExt[#acceptedExt])
		end

		for i=1, table.getn(tbl)-1 do
			local v=tbl[i]
			if string.sub(v,-1)~="/" and string.isMatched(v, acceptedExt) then
				if containsRelPath then
					printFunc:iterate(v)
				else
					if string.sub(mask,-1)=="/" then
						printFunc:iterate(mask..v)
					else
						printFunc:iterate(mask.."/"..v)
					end
				end
			end
		end
	else
		local folder, lmask=os._processMask(mask)
		local out=os._globWin32("-d", folder..lmask)
		local acceptedExt=deepCopyTable(os.globParam.acceptedExt)

		if string.find(mask,"%*%.") then
			local idx=string.find(mask,"%*%.")+2
			acceptedExt[#acceptedExt+1]="%."..string.sub(mask,idx)..'$'
			--print(acceptedExt[#acceptedExt])
		end
		for i=1, table.getn(out) do
			if string.isMatched(out[i], acceptedExt) then
				printFunc:iterate(folder..out[i])
			end
		end
	end
	local verbose=false
	if bRecurse then
		local folder, lmask=os._processMask(mask)
		if verbose then print(folder, lmask) end
		local subfolders=os.globFolders(folder)
		if verbose then printTable(subfolders) end
		for i=1, table.getn(subfolders) do
			local v=subfolders[i]
			os.find(folder..v..'/'..lmask, true, nomessage, printFunc)
		end
	end
	if not nomessage then
		io.write('                                                                   \r')
	end
end
function os.glob(mask, bRecurse, nomessage) -- you can use # or % instead of *. e.g. os.glob('#.jpg')

	local tbl2=array:new()
	function tbl2:iterate(v)
		--print(v)
		self:pushBack(v)
	end
	os.find(mask, bRecurse, nomessage, tbl2)
	return tbl2
end

function os.home_path()
	if os.isUnix() then
		return os.capture("echo ~")
	else
		return os.capture("echo %HOMEDRIVE%")..os.capture("echo %HOMEPATH%")
	end
end
-- returns filename, path
function os.processFileName(target)-- fullpath
	local target=string.gsub(target, '\\', '/')
	local lastSep
	local newSep=0
	local count=0
	repeat lastSep=newSep
		newSep=string.find(target, "/", lastSep+1) 	    
		count=count+1
	until newSep==nil 

	local path=string.sub(target, 0, lastSep-1)

	local filename
	if lastSep==0 then filename=string.sub(target,lastSep) path='' else filename=string.sub(target, lastSep+1) end

	return filename, path
end
function os.filename(target)
	local f=os.processFileName(target)
	return f
end

function os.isFileExist(fn)
	local f=io.open(fn,'r')
	if f==nil then return false end
	f:close()
	return true
end
function createBatchFile(fn, list, echoOff)
	local fout, msg=io.open(fn, "w")
	if fout==nil then print(msg) end

	if os.isWindows() then
		if echoOff then
			fout:write("@echo off\necho off\n")
		end
		fout:write("setlocal\n")
	end
	for i,c in ipairs(list) do
		fout:write(c.."\n")
	end
	fout:close()
end

function os.execute2(...) -- excute multiple serial operations
	execute(...)
end

function os.pexecute(...) -- excute multiple serial operations
	if os.isUnix() then
		execute(...)
	else
		local list={...}
		createBatchFile("_temp.bat",list)      
		os.execute("start _temp.bat")	
	end      
end

-- escape so that it can be used in double quotes
function os.shellEscape(str)
	if os.isUnix() then
		str=string.gsub(str, '\\', '\\\\')
		str=string.gsub(str, '"', '\\"')
		str=string.gsub(str, '%%', '\\%%')
	else
		str=string.gsub(str, '\\', '\\\\')
		str=string.gsub(str, '"', '\\"')
		str=string.gsub(str, '%$', '\$')
	end
	return str
end

function os.luaExecute(str, printCmd)
	local luaExecute
	local gotoRoot
	local endMark
	local packagepath=os.shellEscape('package.path="./OgreFltk/Resource/scripts/ui/?.lua;./OgreFltk/work/?.lua"')
	if os.isUnix() then
		luaExecute="lua -e \""..packagepath.."dofile('OgreFltk/work/mylib.lua');"
		gotoRoot="cd ../.."
		endMark="\""
	else
		luaExecute="OgreFltk\\work\\lua -e dofile('OgreFltk/work/mylib.lua');"
		gotoRoot="cd ..\\.."
		endMark=""
	end
	str=os.shellEscape(str)
	if printCmd then print(luaExecute..str..endMark) end
	execute(gotoRoot, luaExecute..str..endMark)
end
-- deprecated (the same as os.execute2)
function execute(...)
	local list={...}
	if not math.seeded then
		math.randomseed(os.time())
		math.seeded=true
	end
	if os.isUnix() then
		if #list<3 then
			local cmd=""
			for i,c in ipairs(list) do
				cmd=cmd..";"..c
			end
			--print(string.sub(cmd,2))
			os.execute(string.sub(cmd, 2))
		else
			local fn='temp/_temp'..tostring(math.random(1,10000))
			createBatchFile(fn, list)
			os.execute("sh "..fn)
			os.deleteFiles(fn)
		end
	else
		createBatchFile("_temp.bat",list,true)
		--      os.execute("cat _temp.bat")
		os.execute("_temp.bat")	
	end
end

function executeUsingNewCMD(...)
	local list={...}
	createBatchFile("_temp.bat",list,true)
	--      os.execute("cat _temp.bat")
	os.execute("cmd /c _temp.bat")	
end

function util.grepFile(fn, pattern, prefix,raw, printFunc)
	printFunc=printFunc or 
	{ 
		iterate=function(self,fn,ln,idx,line)
			print(fn..":"..ln..":"..string.trimLeft(line))
		end
	}
	prefix=prefix or ""
	pattern=string.lower(pattern)
	local fin, msg=io.open(fn, "r")
	if fin==nil then
		print(msg)
		return
	end
	local ln=1
	--local c=0
	--local lastFn, lastLn
	for line in fin:lines() do
		local lline=string.lower(line)
		local res, idx
		if raw then
			res,idx=pcall(string.find, lline, pattern)
		else
			res,idx=pcall(string.find, lline, pattern, nil,true)
		end
		if res and idx then 
			--				print(prefix..fn..":"..ln..":"..idx..":"..string.trimLeft(line))
			printFunc:iterate(prefix..fn,ln,idx,line)
			--c=c+1
			--lastFn=prefix..fn
			--lastLn=ln
		end
		ln=ln+1
	end
	fin:close()
	--if c==1 then
		--os.vi_line(lastFn, lastLn)
	--end
end
function util.grep(mask, pattern, prefix, bRecurse,raw) -- deprecated. use util.findgrep
	local list=os.glob(mask, bRecurse, true)
	for i, fn in ipairs(list) do
		util.grepFile(fn,  pattern, prefix,raw)
	end
end
function os.open(t)
	if os.isUnix() then
		os.execute('gnome-open '..t)
	else
		os.execute('start cmd/c '..t)
	end
end
