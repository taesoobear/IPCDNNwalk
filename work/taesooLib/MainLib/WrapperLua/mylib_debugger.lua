
function dbg.lunaType(c)
	if type(c)~='userdata' then
		return nil
	end
	local mt= getmetatable(c)
	if mt==nil then return nil end
	return mt.luna_class
end
function dbg.listLunaClasses(line)
	local usrCnam= string.sub(line, 7)
	local out2=''
	local out=''
	local outp=''
	
	for k,v in pairs(__luna)do
		if type(v)=='table' then
			local cname=v.luna_class
			if cname then
				local _, className=string.rightTokenize(cname,'%.')
				local nn=string.sub(k, 1, -string.len(className)-1)
				local namspac=string.gsub(nn,'_', '.')
				if namspac=='.' then namspac='' end
				if usrCnam=='' then
					out=out..namspac.. className..', '
				else
					if namspac..className==usrCnam then
						local map={__add='+', __mul='*',__div='/', __unm='-', __sub='-'}
						local lastFn='funcName'
						for kk,vv in pairs(v) do
							if type(vv)=='function' then
								if string.sub(kk,1,13)=='_property_get' then
									outp=outp .. string.sub(kk,15)..', '
								elseif string.sub(kk,1,13)=='_property_set' then
									--outp=outp .. string.sub(kk,15)..','
								else
									if map[kk] then
										out2=out2..map[kk]..', '
									else
										out2=out2..kk..', '
									end
									lastFn=kk
								end
							end
						end
						out2=out2..'\n\n Tip! you can see the function parameter types by typing "'..usrCnam..'.'..lastFn..'()"!'
						out2=out2..'\n Known bug: property names can be incorrectly displayed. "'
					end
				end
			end
		end
	end
	if outp~='' then print('Properties:\n', outp) end
	print(out)
	if out2~='' then print('Member functions:\n', out2) end
end
function dbg.readLine(cursor)
	io.write(cursor)
	util.getch=nil
	if util.getch then
		if dbg.readLineInfo==nil then
			dbg.readLineInfo={""}
			dbg.readLineInfo.currLine=0
		end
		local function save(f)
			local lines= dbg.readLineInfo
			lines[#lines+1]=f
			lines.currLine=#lines
			return f
		end
		local function goUp()
			local lines= dbg.readLineInfo
			lines.currLine=math.max(lines.currLine-1,0)
			return lines[lines.currLine+1]
		end

		local c=''
		-- getch gives 27 for non-ascii code
		while true do 
			local code=util.getch()
			local ch= string.char(code)
			if ch=='\\' then
				util.putch(ch)
				ch=string.char(util.getch())
				if ch==']' then
					print('cont')
					return 'cont'
				elseif ch=='[' then
					print('s')
					return 's'
				elseif ch=='k' then
					ch=''
					c=goUp()
					util.putch('\r'..cursor..c)
				else
					c=c..'\\'
				end
			end
			if ch=='\n' then
				util.putch('\n')
				return save(c)
			elseif code==127 then
				util.putch('\r')
				c=string.sub(c,1,-2)
				util.putch(cursor..c)
			else
				c=c..ch
				util.putch(ch)
			end
		end
		return ''
	else
		return io.read('*line')
	end
end
function dbg.traceBack(level)
   if level==nil then
      level=1
   end
   while true do
      local info=debug.getinfo(level)
      local k=info.name
      if k==nil then
	 break
      else
	 print('----------------------------------------------------------')
	 print('Level: ', level)
	 print(info.short_src..":"..info.currentline..":"..k)
	 print('Local variables:')
	 dbg.locals(level)
	 level=level+1
      end
   end
end
function os.VI_path()
	if os.isUnix() then
--		return "vim"  -- use vim in a gnome-terminal
		return "gvim"
	else
		return "gvim"
	end
end	

function os.vi_check(fn)
	local L = require "functional.list"
	local otherVim='vim'
	local servers=string.tokenize(os.capture(otherVim..' --serverlist 2>&1',true), "\n")
	local out=L.filter(function (x) return string.upper(fn)==x end,servers)
	local out_error=L.filter(function (x) return string.find(x,"Unknown option argument")~=nil end,servers)
	if #out_error>=1 then return nil end
	return #out>=1
end

function os.vi_console_close_all()
	local L = require "functional.list"
	local servers=string.tokenize(os.capture('vim --serverlist',true), "\n")
	local out=L.filter(function (x) return fn~="GVIM" end,servers)
	for i,v in ipairs(out) do
		os.execute('vim --servername "'..v..'" --remote-send ":q<CR>"')
	end
end

function os.vi_console_cmd(fn, line)
	local cc
	if line then
		cc=' +'..line..' "'..fn..'"'
	else
		cc=' "'..fn..'"'
	end
	if not os.isUnix() and os.isFileExist("C:/msysgit/msysgit/share/vim/vim73/vim.exe") then
		return '"C:/msysgit/msysgit/share/vim/vim73/vim.exe" '..cc
	end
	return 'vim '..cc
end
function os.emacs_cmd(fn, line)
	local cc
	if line then
		cc=' +'..line..' "'..fn..'"'
	else
		cc=' "'..fn..'"'
	end

	--return 'lua ~/.config/mtiler/mstiler.lua launch-gui-app emacs '..cc
	return 'emacs '..cc
end
function os.gedit_cmd(fn, line)
	local cc
	if line then
		cc=' +'..line..' "'..fn..'"'
	else
		cc=' "'..fn..'"'
	end
	return 'gedit '..cc
end
function os.emacs_client_cmd(fn, line)
	local cc
	if line then
		cc=' +'..line..' "'..fn..'"'
	else
		cc=' "'..fn..'"'
	end

	return 'emacsclient -n '..cc
end
function os.emacs_console_cmd(fn, line)
	local cc
	if line then
		cc=' +'..line..' "'..fn..'"'
	else
		cc=' "'..fn..'"'
	end

	return 'emacs -nw '..cc
end

function os.vi_readonly_console_cmd(fn, line)
	local cc
	if line then
		cc=' +'..line..' "'..fn..'"'
	else
		cc=' "'..fn..'"'
	end

	return 'vim -R -M -c ":set nomodifiable" '..cc
end
function os.vi_line(fn, line)
	if os.vi_check(fn) then
		execute(os.vi_console_cmd(fn,line))
		return
	end
	if not os.launch_vi_server() then
		print('Please launch gvim first!')
		return
	end
	local VI=os.VI_path()..' --remote-silent'
	local cmd=VI..' +'..line..' "'..fn..'"'
	--print(cmd)
	execute(cmd)
end
function os.launch_vi_server()
	local lenvipath=string.len(os.VI_path())
	if os.vi_check(string.upper(os.VI_path())) then
		print("VI server GVIM open")
		return true
	end
	if false then -- recent ubuntu gvim doesn't start up from a terminal.
		print("launching GVIM server...")
		if os.isUnix() then
			if os.VI_path()=="vim" then
				execute('cd ../..', 'gnome-terminal -e "vim --servername vim"&') -- this line is unused by default. (assumed gnome dependency)
			else
				execute('cd ../..', os.VI_path())
			end
		else
			if os.isFileExist(os.capture('echo %WINDIR%').."\\vim.bat") then
				execute('cd ..\\..', os.VI_path())
			else
				execute('cd ..\\..', "start "..os.VI_path())
			end
		end

		for i=1,10 do
			if os.vi_check(string.upper(os.VI_path())) then
				print("VI server GVIM open")
				break
			else
				print('.')
				--os.sleep(1)
			end
		end
		return true
	end
	return false
end
function os.vi(...)
	os._vi(os.VI_path(), ...)
end

function os._vi(servername, ...)
	local VI=os.VI_path() ..' --servername '..servername..' --remote-silent'
	local VI2=os.VI_path() ..' --servername '..servername..' --remote-send ":n '
	local VI3='<CR>"'

	local targets={...}

	local otherVim='vim'

	local vicwd=os.capture(otherVim..' --servername '..servername..' --remote-expr "getcwd()"')
	if vicwd=="" then
		if not os.launch_vi_server() then
			print('Please launch gvim first!')
			return
		end
		-- try one more time
		vicwd=os.capture(otherVim ..' --servername '..servername..' --remote-expr "getcwd()"')
	end
	print('vicwd=',vicwd)
	local itgt, target
	for itgt,target2 in ipairs(targets) do
		--      local target=string.sub(target2,4)
		local target=target2

		if string.find(target, '*') ~=nil or string.find(target, '?')~=nil then

			if false then
				-- open each file. too slow
				local subtgts=os.glob(target)

				local istgt,subtgt
				for istgt,subtgt in ipairs(subtgts) do
					local cmd=VI..' "'..subtgt..'"'
					if string.find(cmd,'~')==nil then
						os.execute(cmd) 
					end
				end
			elseif string.sub(target, 1,6)=="../../" and string.sub(vicwd, -10)=="taesoo_cmu" then -- fastest method 
				local cmd=VI2..string.sub(target,7)..VI3
				print(cmd)
				if os.isUnix() then
					os.execute(cmd.."&")
				else
					os.execute("start "..cmd)
				end
			else
				local lastSep
				local newSep=0
				local count=0
				repeat lastSep=newSep
					newSep=string.find(target, "/", lastSep+1) 	    
					count=count+1
				until newSep==nil 

				local path=string.sub(target, 0, lastSep-1)

				local filename
				if lastSep==0 then filename=string.sub(target,lastSep) else filename=string.sub(target, lastSep+1) end

				print(filename, path, count)

				--	    execute("cd "..path, 'find . -maxdepth 1 -name "'..filename..'"| xargs '..EMACS)


				-- if string.find(filename, "*") then
				--    execute("cd "..path, 'find '..filename..' -maxdepth 1 | xargs '..EMACS)
				-- else

				print("cd "..path, VI.." "..filename)
				if os.isUnix() then
					execute("cd "..path, "rm -f *.lua~", VI.." "..filename.."&")
				else
					execute("cd "..path, "rm -f *.lua~", "rm -f #*#", VI.." "..filename)
				end
				--	    end

			end

		else
			local cmd=VI..' "'..target..'"'
			print(cmd)
			if os.isUnix() then
				os.execute(cmd.."&")
			else
				os.execute(cmd)
			end
		end
	end
 end
function os.emacs_client(fn, line)
	local cmd=os.emacs_client_cmd(fn,line)
	print(cmd)
	local tt=os.capture(cmd.." 2>&1", true)
	print(tt)
	if select(1,string.find(tt, 'have you started the server') ) then
		print('Error detected! launching a new server...')
		local cmd
		if line then
			cmd='emacs +'..line..' "'..fn..'" --eval "(server-start)"&'
		else
			cmd='emacs "'..fn..'" --eval "(server-start)"&'
		end
		print(cmd)
		os.execute(cmd)
	end
end
function os.emacs(...)
	local targets={...}

	local EMACS
	if os.isUnix() then
		EMACS="emacs"
	else      
		EMACS="emacsclient -n"
	end

	local itgt, target
	print("emacs")
	for itgt,target2 in ipairs(targets) do
		--      local target=string.sub(target2,4)
		local target=target2

		if string.find(target, '*') ~=nil or string.find(target, '?')~=nil then

			if false then
				-- open each file. too slow
				local subtgts=os.glob(target)

				local istgt,subtgt
				for istgt,subtgt in ipairs(subtgts) do
					local cmd=EMACS..' "'..subtgt..'"'
					if string.find(cmd,'~')==nil then
						os.execute(cmd) 
					end
				end
			else
				local lastSep
				local newSep=0
				local count=0
				repeat lastSep=newSep
					newSep=string.find(target, "/", lastSep+1) 	    
					count=count+1
				until newSep==nil 

				local path=string.sub(target, 0, lastSep-1)

				local filename
				if lastSep==0 then filename=string.sub(target,lastSep) else filename=string.sub(target, lastSep+1) end

				print(filename, path, count)

				--	    execute("cd "..path, 'find . -maxdepth 1 -name "'..filename..'"| xargs '..EMACS)


				-- if string.find(filename, "*") then
				--    execute("cd "..path, 'find '..filename..' -maxdepth 1 | xargs '..EMACS)
				-- else
				execute("cd "..path, "rm -f *.lua~", "rm -f #*#", EMACS.." "..filename)
				--	    end

			end

		else
			os.execute(EMACS..' "'..target..'"')
		end
	end
end   

function os.emacs2(target)
	os.execute('find . -name "'..target..'"| xargs emacsclient -n')      
end

function os.findDotEmacsFolder()
	local candidates=
	{"C:\\Documents and Settings\\Administrator\\Application Data",
	"C:\\Documents and Settings\\sonah\\Application Data",
	"C:\\usr\\texlive\\HOME",
}
for i, candidate in ipairs(candidates) do
	if os.isFileExist(candidate.."\\.emacs") then
		return candidate
	end
end
return "c:\\DotEmacsNotFound"
end
function dbg.showCode(fn,ln)
	util.iterateFile(fn,
	{ 
		iterate=function (self, lineno, c) 
			if lineno>ln-5 and lineno<ln+5 then
				c=string.gsub(c, "\t", "    ")
				if #c > 70 then
					c=string.sub(c,1,65).."..."
				end
				if lineno==ln then
					print(lineno.."* "..c)
				else
					print(lineno.."  "..c)
				end
			end
		end
	}
	)
end

function dbg.console(msg, stackoffset)

	stackoffset=stackoffset or 0
	if(msg) then print (msg) end
	
	if dbg._consoleLevel==nil then
		dbg._consoleLevel=0
	else
		dbg._consoleLevel=dbg._consoleLevel+1
	end
      if fineLog~=nil and rank~=nil then
		  debug.sethook() -- stop all kinds of debugger
		  fineLog("dbg.console called")
		  fineLog(dbg.callstackString(1))
		  fineLog(util.tostring(dbg.locals()))
		  dbg.callstack0()
		  return
      end
	local function at(line, index)
		return string.sub(line, index, index)
	end

	local function handleStatement(statement)

		local output
		if string.find(statement, "=") and not string.find(statement, "==") then -- assignment statement
			output={pcall(loadstring(statement))}
		else -- function calls or print variables: get results
			output={pcall(loadstring("return ("..statement..")"))}
			if output[1]==false and output[2]=="attempt to call a nil value" then
				-- statement
				output={pcall(loadstring(statement))}
			end
		end

		if output[1]==false then 
			print("Error! ", output[2]) 
		else
			if type(output[1])~='boolean' then
				output[2]=output[1] -- sometimes error code is not returned for unknown reasons.
			end
			if type(output[2])=='table' then 
				if getmetatable(output[2]) and getmetatable(output[2]).__tostring then
					print(output[2])
				else
					printTable(output[2]) 
				end
			elseif output[2] then
				dbg.print(unpack(table.isubset(output, 2)))
			elseif type(output[2])=='boolean' then
				print('false')
			end
		end
	end

	local event
	while true do
		local cursor="[DEBUG"..dbg._consoleLevel.."] > "
		line=dbg.readLine(cursor)
		local cmd=at(line,1)
		local cmd_arg=tonumber(string.sub(line,2))
		if not (string.sub(line,2)=="" or cmd_arg) then
			if not ( cmd=="r" and at(line,2)==" ") then
				if not string.isOneOf(cmd, ":", ";") then
					cmd=nil
				end
			end
		end

		if cmd=="h" or string.sub(line,1,4)=="help" then --help
			print('cs[level=3]      : print callstack')
			print('c[level=1]       : print source code at a stack level')
			print(';(lua statement) : eval lua statements. e.g.) ;print(a)')
			print(':(lua statement) : eval lua statements and exit debug console. e.g.) ;dbg.startCount(10)')
			print('s[number=1]      : proceed n steps')
			print('r filename [lineno]  : run until execution of a line. filename can be a postfix substring. e.g.) r syn.lua 32')
			print('e[level=2]       : show current line (at callstack level 2) in emacs editor')
			print('v[level=2]       : show current line (at callstack level 2) in vi editor')
			print('c[level=2]       : show nearby lines (at callstack level 2) here')
			print('l[level=0]       : print local variables. Results are saved into \'l variable.')
			print("                   e.g) DEBUG]>print('l.self.mVec)")
			print('clist 		: list luna classes')
			print('clist className  : list functions in the class')
			print('cont             : exit debug mode')
			print('lua global variable     : Simply typing "a" print the content of a global variable "a".')
			print('lua local variable     : Simply typing "`a" print the content of a local variable "a".')
			print('lua statement    : run it')
		elseif line=="cont" then break
		elseif string.sub(line,1,2)=="cs" then dbg.callstack(tonumber(string.sub(line,3)) or 3)
		elseif line=="clist" or string.sub(line,1,6)=='clist ' then
			dbg.listLunaClasses(line)		
		elseif cmd=="c" then
			if cmd_arg==nil then
				local level=stackoffset
				while true do
					local info=debug.getinfo(level)
					if info then
						local a=string.sub(info.source, 1,1)
						if a=='=' or a=='[' then
							level=level+1
						elseif select(1,string.find(info.source, 'mylib_debugger.lua')) then
							level=level+1
						else
							break
						end
					else
						level=level+1
						if level>40 then break end
					end
				end
				cmd_arg=level-stackoffset+1
				print('c'..cmd_arg..':')
			end
			local level=(cmd_arg or 1)+stackoffset-1 -- -1 means 'excluding dbg.showCode'
			local info=debug.getinfo(level)
			if info then
				local a=string.sub(info.source, 1,1)
				if a=='=' or a=='[' then
					print(info.source)
				else
					local ln=info.currentline
					print(string.sub(info.source,2))
					dbg.showCode(string.sub(info.source,2),ln)
					dbg._saveLocals=dbg.locals(level+1,true)
				end
			else
				print('no such level')
			end
		elseif cmd=="v" then
			local info=debug.getinfo((cmd_arg or 1)+stackoffset-1)
			if info and info.source=="=(tail call)" then
				info=debug.getinfo((cmd_arg or 1)+stackoffset)
			end
			if info then
				--os.vi_line(string.sub(info.source,2), info.currentline)
				local fn=string.sub(info.source,2)
				fn=os.relativeToAbsolutePath(fn)
				--fn=os.absoluteToRelativePath(fn, os.relativeToAbsolutePath("../.."))
				--os.luaExecute([[os.vi_line("]]..fn..[[",]]..info.currentline..[[)]])
				os.vi_line(fn,info.currentline)
			end
		elseif cmd=="e" then
			local info=debug.getinfo((cmd_arg or 1)+stackoffset-1)
			if info then
			   --os.emacs_client(os.relativeToAbsolutePath(string.sub(info.source,2)),info.currentline)
			   os.execute(os.gedit_cmd(os.relativeToAbsolutePath(string.sub(info.source,2)),info.currentline)..'&')
			end
		elseif cmd==";" then
			handleStatement(string.sub(line,2))
		elseif cmd==":" then
			handleStatement(string.sub(line,2))
			break
		elseif cmd=="s" or cmd=="'" then
			local count=cmd_arg or 1
			event={"s", count}
			break
		elseif cmd=="r" then
			event={"r", string.sub(line, 3)}
			break
		elseif cmd=="l" then
			local level=(cmd_arg or 1)
			dbg._saveLocals=dbg.locals(level)
		else 
			statement=string.gsub(line, '``', 'dbg._saveLocals')
			statement=string.gsub(line, '`', 'dbg._saveLocals.')
			handleStatement(statement)
		end
	end

	dbg._consoleLevel=dbg._consoleLevel-1
	if event then
		if event[1]=="s" then 
			return dbg.step(event[2]) 
		elseif event[1]=="r" then
			return dbg.run(event[2])
		end
	end

end

function dbg._stepFunc (event, line)
   dbg._step=dbg._step+1
   if dbg._step==dbg._nstep then
      debug.sethook()
	  local info=debug.getinfo(2)
	  if info then
		  if select(1,string.find (info.source, 'mylib_debugger.lua')) then
			  return dbg.step(1)
		  end
		  print(info.source, line)
		  dbg.showCode(string.sub(info.source,2), line)
	  end
      return dbg.console()
   end
end
function dbg.step(n) 
   dbg._step=0
   dbg._nstep=n
   debug.sethook(dbg._stepFunc, "l")	
end
function dbg.callstack(level)
	if level==nil then
		level=1
	end
	while true do
		local info=debug.getinfo(level)
		local k=info.name
		if k==nil then
			break
		else
			print(info.short_src..":"..info.currentline..":"..k)
			level=level+1
		end
	end
end


function dbg.callstack0(level)
	if level==nil then
		level=1
	end
	while true do
		local info=debug.getinfo(level)
		if info==nil then break end
		local k=info.name
		if k==nil then
			printTable(info)
			level=level+1
		else
			print(info.short_src..":"..info.currentline..":"..k)
			level=level+1
		end
	end
end
function dbg.locals(level, noprint)
	local output={}
	if level==nil then level=1 end
	cur=1
	while true do
		if debug.getinfo(level, 'n')==nil then return output end
		k,v=debug.getlocal(level, cur)
		if k~=nil then
			output[k]=v or "(nil)"
			cur=cur+1
		else
			break
		end
	end
	if not noprint then
		os.print(output)
	end
	return output
end
function dbg.run(run_str) -- run_str example: a.lua 374 
   local tbl=string.tokenize(run_str, " ")
   local filename=tbl[1]
   local lineno=tonumber(tbl[2])

   --print(filename..","..tostring(lineno))
   if tonumber(filename)~=nil then
	   lineno=tonumber(filename)
	   filename=''
   end
   if filename=='' then
	   local info=debug.getinfo(3)
	   filename=info.source
	   if string.sub(info.source,1,1)=="=" then
		   info=debug.getinfo(4)
		   filename=info.source
	   end
   end
   print("stop at "..filename.." +",lineno)
   local strlen=string.len(filename)*-1
   dbg._runFuncParam={filename, strlen, lineno}
   debug.sethook(dbg._runFunc, "l")	
end

function dbg._runFunc (event, line)
   local src=debug.getinfo(2).source
   local param=dbg._runFuncParam

   if string.sub(src, param[2])==param[1] and (param[3]==nil or line==param[3]) then
      debug.sethook()
      print(debug.getinfo(2).source, line)
      return dbg.console()
   end
end
-- outputs counts to trace.txt
function dbg.startCount(dbgtime)
	if dbg.filePtr==nil then
		if dbgtime then
			print('Start re-counting until '..dbgtime)
		else
			print('Start counting.. ')
			print('Output will go to trace.txt')
			print('You can debug a crashing program by re-running the program using dbg.startCount(lastCount)')
			dbg.filePtr, msg=io.open("trace.txt", "w")
			if dbg.filePtr==nil then
				print(msg)
				return
			end
		end

		dbg._dbgtime=dbgtime
		dbg._count=0
		--debug.sethook(dbg.countHookF, "l")
		debug.sethook(dbg.countHookF, "c") -- much faster though less accurate
	else
	end
end
function dbg.countHookF(event)
	local _count=dbg._count 
	local _dbgtime=dbg._dbgtime

	if _dbgtime then
		if _count>_dbgtime-100 then
			local info=debug.getinfo(2)
			print('coundown', _dbgtime-_count, info.name, info.short_src, info.currentline)
			if _count==_dbgtime then
				debug.sethook()
				dbg.console()
			end
		end
	else
		local filePtr=dbg.filePtr
		filePtr:seek("set", 0)
		filePtr:write(_count)
		filePtr:flush()
	end
	dbg._count=_count+1
end

