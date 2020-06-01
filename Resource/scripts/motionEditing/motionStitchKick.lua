a=Motion(2,47782,47819)
b=Motion(2,18995,19050)
e=stitch(a,b, "stitch", 30,false, false, false, true, true) -- stitch forward leg
e:show()
f=stitch(a,b, "concat")
f:show("green")

