
require("config")
require("module")
require("common")
local chosenFile=Fltk.chooseFile("Choose a file to convert", "../dump", "*.jpg", false)
chosenFile=os.absoluteToRelativePath(chosenFile)
local _, path=os.processFileName(chosenFile)

local files=os.glob(path.."/*.jpg")
table.sort(files)
if files[1]:sub(-9)~='00000.jpg' then
	for i,v in ipairs(files) do
		os.rename(v, v:sub(1,-10)..string.format("%05d.jpg", i-1))
		print('mv ',v, v:sub(1,-10)..string.format("%05d.jpg", i-1))

	end
end
os.encodeToDivx(path, 'converted.avi')
local chosenFile=Fltk.chooseFile("Choose an avi file to save", "../dump", "*.avi", true)
if chosenFile~='' then
	os.execute('cp virtualDub/converted.avi "'..chosenFile..'"')
else
	print('see work/virtualDub/converted.avi')
end
