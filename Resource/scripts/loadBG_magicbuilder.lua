
rootnode =RE.ogreRootSceneNode()
bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")

dofile("../Resource/scripts/magicbuilder/loadBG_magicbuilder_layer1_donotoverwrite.lua")
dofile("../Resource/scripts/magicbuilder/loadBG_magicbuilder_layer2_donotoverwrite.lua")
dofile("../Resource/scripts/magicbuilder/loadBG_magicbuilder_layer3_donotoverwrite.lua")
dofile("../Resource/scripts/magicbuilder/loadBG_magicbuilder_layer4_donotoverwrite.lua")


RE.viewpoint():setFOVy(30.000006)
RE.viewpoint().vpos:assign({401.184546, 5572.603511, -135.577549})
RE.viewpoint().vat:assign({401.278038, 215.906525, -135.577549})
RE.viewpoint():update()
