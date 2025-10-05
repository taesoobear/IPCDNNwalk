
local isWin=string.find(string.lower(os.getenv('OS') or 'nil'),'windows')~=nil
local isCygwin=string.find(string.lower(os.getenv('PATH') or 'nil'), 'cygdrive')~=nil
if isWin then
	package.path =os.getenv('HOMEDRIVE')..os.getenv('HOMEPATH').."/bin/?.lua" .. ";"..package.path 
	package.path ="../../../MainLib/WrapperLua/?.lua" .. ";"..package.path 
	package.path ="../../MainLib/WrapperLua/?.lua" .. ";"..package.path 
	package.path ="../MainLib/WrapperLua/?.lua" .. ";"..package.path 
	if isCygwin then
		package.path =os.getenv('HOME').."/bin/?.lua" .. ";"..package.path 
	end 
	if g_make_lua_path  then
		package.path=g_make_lua_path.."../MainLib/WrapperLua/?.lua"..";"..package.path
	end
else
	package.path =os.getenv('HOME').."/bin/?.lua" .. ";"..package.path 
end
if _VERSION=='Lua 5.2' then
	require("mylib52")
else
	require("mylib")
end
function os.isCygwin()
	f=io.open('/cygdrive')
	if f then
		f:close()
		return true
	end
	return false
end
if arg[1]=='run' then
	local work='../../work'
	if INSTALL_PATH then
		work=string.sub(INSTALL_PATH, 4)
	end
	if g_execute_path then
		work=g_execute_path..'../work'
	end
	if os.isCygwin() then
		--os.execute2('cd ../../work', 'cmd /c start cmd /c '..EXE)
		os.execute2('cd '..work,  'cmd /c start run.bat '..EXE)
	else
		os.execute2('cd '..work, EXE)
	end
	return
elseif arg[1]=='luna_gen' then
	os.execute('lua ../../MainLib/WrapperLua/luna_gen.lua  ../../MainLib/WrapperLua/luna_baselib.lua')
elseif arg[1]=='pushbin' then
	os.execute('scp -P 8022 ../../work/classification.exe students@calab.hanyang.ac.kr:setup')
	return
elseif arg[1]=='pullbin' then
	os.execute('scp -P 8022 students@calab.hanyang.ac.kr:setup/classification.exe ../../work')
	return
elseif arg[1]=='pushdll' then
	os.execute('scp -P 8022 ../../work/*.dll students@calab.hanyang.ac.kr:setup/dll')
	return
elseif arg[1]=='pulldll' then
	os.execute('scp -P 8022 students@calab.hanyang.ac.kr:setup/dll/*.dll ../../work')
	return
end
if os.isCygwin() then 
	if arg[1]=='com' or arg[1]=='con' then
		if not os.isFileExist('build_win/CMakeCache.txt') then
			os.createDir('build_win')
		end

		--local path='C:\\Program Files (x86)\\Microsoft Visual Studio 10.0\\Common7\\IDE\\devenv.com'
		local path='C:\\Program Files (x86)\\Microsoft Visual Studio\\2017\\Community\\Common7\\IDE\\devenv.com'
		if not os.isFileExist(path) then
			path=string.gsub(path, ' %(x86%)', '')
		end
		if _VERSION=='Lua 5.2' then
			if arg[2]=='debug' then
				--os.createBatchFile("temp.bat", {'cd build_win', 'cmake -G "Visual Studio 10" ..', '"'..path..'" '..EXE..'.sln /Build "Debug|Win32"', 'SET /P uname=Press Enter to finish', 'exit'}, true)
				os.createBatchFile("temp.bat", {'cd build_win', 'cmake -G "Visual Studio 15 2017" ..', '"'..path..'" '..EXE..'.sln /Build "Debug|Win32"', 'SET /P uname=Press Enter to finish', 'exit'}, true)
			else
				--os.createBatchFile("temp.bat", {'cd build_win', 'cmake -G "Visual Studio 10" ..', '"'..path..'" '..EXE..'.sln /Build "Release|Win32"', 'SET /P uname=Press Enter to finish', 'exit'}, true)
				os.createBatchFile("temp.bat", {'cd build_win', 'cmake -G "Visual Studio 15 2017" ..', '"'..path..'" '..EXE..'.sln /Build "Release|Win32"', 'SET /P uname=Press Enter to finish', 'exit'}, true)
			end
		else
			if arg[2]=='debug' then
				--os.createBatchFile("temp.bat", {'cd build_win', 'cmake -G "Visual Studio 10" ...', '"'..path..'" '..EXE..'.sln /Build "Debug|Win32"', 'SET /P uname=Press Enter to finish', 'exit'}, true)
				os.createBatchFile("temp.bat", {'cd build_win', 'cmake -G "Visual Studio 15 2017" ...', '"'..path..'" '..EXE..'.sln /Build "Debug|Win32"', 'SET /P uname=Press Enter to finish', 'exit'}, true)
			else
				--os.createBatchFile("temp.bat", {'cd build_win', 'cmake -G "Visual Studio 10" ...', '"'..path..'" '..EXE..'.sln /Build "Release|Win32"', 'SET /P uname=Press Enter to finish', 'exit'}, true)
				os.createBatchFile("temp.bat", {'cd build_win', 'cmake -G "Visual Studio 15 2017" ...', '"'..path..'" '..EXE..'.sln /Build "Release|Win32"', 'SET /P uname=Press Enter to finish', 'exit'}, true)
			end
		end
		os.execute('cmd /c start temp.bat')
		return 
	elseif arg[1]=='conw' then
		if not os.isFileExist('build_win/CMakeCache.txt') then
			os.createDir('build_win')
		end
		os.execute('rm -f build_win/finished.txt')
		--os.createBatchFile("temp.bat", {'cd build_win', 'cmake ..', '"C:\\Program Files (x86)\\Microsoft Visual Studio 10.0\\Common7\\IDE\\devenv.com" '..EXE..'.sln /Build "Release|Win32"', 'echo finished > finished.txt', 'exit'}, true)
		os.createBatchFile("temp.bat", {'cd build_win', 'cmake ..', '"C:\\Program Files (x86)\\Microsoft Visual Studio\\2017\\Community\\Common7\\IDE\\devenv.com" '..EXE..'.sln /Build "Release|Win32"', 'echo finished > finished.txt', 'exit'}, true)
		os.execute('cmd /c start temp.bat')
		while not os.isFileExist('build_win/finished.txt') do
			print('waiting...')
			os.execute('sleep 2')
		end
		return 

	elseif arg[1]=='clean' then
		os.execute('rm -rf build_win')
		return 
	end
	do
		local out=os.capture('env | grep "tmp="')
		local out2=os.capture('env | grep "TMP="')
		if out2=="TMP=/tmp" and not out=="tmp=" then 
			print("Remove TMP an TEMP environment variables created by cygwin.")
			print("Otherwise, visual studio will fail to build the program")
		end
	end   

	if false then
		print("This program cannot be run in cygwin")
		print("I will open the file explorer. Run build.bat there")
		if not os.isFileExist('build.bat') then
			print("build.bat doesn't exist. Copy it from other sample folders!")
			os.execute("explorer .")
		end

		os.execute("explorer /select,build.bat")
		return 
	else
		function os.execute2(...) 
			local args={...}
			array.concat(args, {'SET /P uname=Press Enter to finish','exit'})
			os.createBatchFile("temp.bat",args,  true)
			os.execute('cmd /c start temp.bat')
		end
		function os.execute2_silent(...) 
			local args={...}
			array.concat(args, {'exit'})
			os.createBatchFile("temp.bat",args,  true)
			os.execute('cmd /c start temp.bat')
		end
	end
end

if arg[1]=="install" then
	if os.isFileExist('Release/OgreFltk.exe') then
		print('copying Release/OgreFltk.exe to work/'..EXE..'.exe')
		os.copyFile('Release/OgreFltk.exe',  '../../../work/'..EXE..'.exe')
	end
	if os.isFileExist('Debug/OgreFltk.exe') then
		print('copying Release/OgreFltk.exe to work/'..EXE..'_d.exe')
		os.copyFile('Debug/OgreFltk.exe',  '../../../work/'..EXE..'_d.exe')
	end
	-- needs python but Ctrl+C works well unlike os.execute2.
	--os.execute('python ../../work/run.py '..EXE)
elseif arg[1]=="console" then
	os.execute2('cd ../../work', 'console')
elseif not argProcessor or (argProcessor and not argProcessor(arg[1])) then
	if not os.isFileExist('build_win/CMakeCache.txt') then
		os.createDir('build_win')
	end
	if select(1, string.find(os.capture('tasklist'), 'devenv.exe')) then
		print('Visual studio is already running')
		print('Type F7 in the visual studio')
	else
		--os.execute2('cd build_win', 'cmake -G "Visual Studio 10" ..', 'start '..EXE..'.sln')
		os.execute2('cd build_win', 'cmake -G "Visual Studio 15 2017" ..', 'start '..EXE..'.sln')
	end
end
