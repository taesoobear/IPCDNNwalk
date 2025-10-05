

mp=RE.motionPanel()
mot=mp:currMotion()

discontinuity=boolN(mot:numFrames())

for i=0,mot:numFrames()-1 do
   discontinuity:set(i, mot:isConstraint(i, FootstepDetection.IS_DISCONTINUOUS))
end

mp:scrollPanel():addPanel(discontinuity:bit(), CPixelRGB8(255,255,0))

fn=Fltk.chooseFile("Choose con", "../Resource/motion/", "*.dis", true)

file=util.BinaryFile()
file:openWrite(fn)
file:pack(discontinuity)
file:close()

