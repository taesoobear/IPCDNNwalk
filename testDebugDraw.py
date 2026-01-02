import os, sys, pdb, math, random


import numpy as np
from libcalab_ogre3d import RE, m, lua, control

def frameMove(fElapsedTime):
	RE.updateBillboards(fElapsedTime)


def drawAll():
    this=m.getPythonWin()
    if True:
        goal=m.vector3N()

        # in centi-meter unit
        p1=m.vector3(0,100, 100)
        p2=m.vector3(0,100, 300)
        p3=m.vector3(300,100, 300)
        p4=m.vector3(300,100, 100)

        # first line segment
        goal.pushBack(p1)
        goal.pushBack(p2)

        # second
        goal.pushBack(p2)
        goal.pushBack(p3)

        goal.pushBack(p3)
        goal.pushBack(p4)

        goal.pushBack(p4)
        goal.pushBack(p1)

        thickness=10 # centimeter
        goal.translate(m.vector3(1,0,1))
        RE.drawBillboard( goal.matView(),  'skeleton1', 'solidred', thickness, 'BillboardLineList' )

    numSides=20
    RE.draw('Cylinder', m.transf(m.vector3(0,4,0)), 'cylinder1', m.vector3(2,0.5,numSides), 100, 'lightgrey_transparent')
    RE.draw('Box', m.transf(m.vector3(0,2,0)), 'box1', m.vector3(2,0.5,1), 100, 'lightgrey_transparent')
    RE.draw('WireBox', m.transf(m.vector3(0,3,0)), 'box2', m.vector3(2,0.5,1), 100, 'solidred')
    RE.draw('Text', m.vector3(0,400,0), 'cylinder1')
    RE.draw('Text', m.vector3(0,200,0), 'text2',m.vector3(1.0,0,0), 28, 'a larger red text')
    if this.findWidget('draw spheres').checkButtonValue() :
        if True: 
            # draw spheres
            pos=m.vector3(-100,20,0)
            RE.namedDraw("Sphere", pos, "ball1", "red", 5)
            pos+=m.vector3(50,0,0)
            RE.namedDraw("Sphere", pos, "ball2", "blue", 10)
            pos+=m.vector3(50,0,0)
            # draw and namedDraw has the same function signature.
            # The difference is in the caption.
            # (ball3 doesn't have any visible caption)
            RE.draw("Sphere", pos, "ball3", "green", 15)

            # after 2,4,6 seconds, these spheres will be erased.
            # no nameID necessary.
            RE.timedDraw(6, "Sphere", pos, "blue", 20)
            RE.timedDraw(4, "Sphere", pos, "green", 25)
            RE.timedDraw(2, "Sphere", pos, "red", 30)

            # both position and radius.
            RE.draw("SphereM", m.vector3(0,0,-1), "bigBall", "red_transparent", 1)

            for i in range(1, 20+1):
                RE.timedDraw(i*0.5, "Sphere", m.vector3(i*30, 0, 10), "blue", 20)

            # RE.erase("Sphere", "ball3") # to erase the sphere from the scene
            color=m.vector3(1,0,0)
            textheight=15
            RE.draw("Text",  pos+m.vector3(0,20,0), "The Ball3's caption is larger", color, textheight)

    if this.findWidget('draw lines').checkButtonValue() :
        if True:
            # draw lines
            goal=m.matrixn(36,3)
            goal.setAllValue(0)
            for ii in range(1,36+1):
                i=ii*100
                goal.set(ii-1,0, (i*(-3)/360))
                goal.set(ii-1,1, 0.7+math.sin(math.radians(ii*10)))
                goal.set(ii-1,2, 0.1)

            RE.namedDraw('Traj', goal*10.0, 'goal')
            for i in range(1,36+1):
                goal.set(i-1,1, goal.get(i-1,1)+2)

            thickness=10
            RE.namedDraw('Traj', goal*10.0, 'goal2', 'blueCircle', thickness, 'QuadListZ')
            for i in range(1,36+1):
                goal.set(i-1,1, goal.get(i-1,1)+2)
            
            # 사용자를 바라봐야하는 billboard를 그릴때는 drawBillboard 함수를 사용할 것.
            # RE.draw('Traj',...)와 동일하게 동작하지만, 시점이 바뀔때마다 다시 그린다는 차이가 있음
            # framemove에서 RE.updateBillboards호출 하는 것 빼먹으면 안됨.
            RE.drawBillboard( goal*10.0, 'line3', 'redCircle', thickness, 'QuadListV') #-- QuadListV is view-dependent -> use drawBillboard

            for i in range(1,36+1):
                goal.set(i-1,1, goal.get(i-1,1)+2)

            thickness=10 #-- 10 cm
            RE.drawBillboard( goal*10, 'goal3', 'solidwhiteTrailZTest', thickness, 'BillboardLineList' )

            for i in range(1,36+1):
                goal.set(i-1,1, goal.get(i-1,1)+2)

            if True:
                chain=goal*10
                chain.resize(chain.rows(), 8)
                color=m.vector3(1,0.5,0)

                thickness=10 # 10 cm
                for i in range(1,36+1):
                    tu=m.map(i, 1, 36, 0,1)
                    width=thickness
                    chain.set(i-1, 3, color.x)
                    chain.set(i-1, 4, color.y)
                    chain.set(i-1, 5, color.z)
                    if i>10 :
                        chain.set(i-1, 6, m.map(i, 10, 36, width,0))
                    else:
                        chain.set(i-1, 6, width)
                    chain.set(i-1, 7, tu)
                # in taesooLib-next, material doesn't work. use color values above.
                RE.drawBillboard(chain, 'goal4', 'solidwhite', thickness, 'BillboardChain' )

            for i in range(1,36+1):
                goal.set(i-1,1, goal.get(i-1,1)+2)
            RE.timedDraw(5, 'Traj', goal*10,  'solidred', thickness, 'BillboardLineList' )
    if this.findWidget('draw colored lines').checkButtonValue() :
        if True:
            # draw colored trail lines
            goal=m.vector3N()
            for i in range(1,36+1):
                goal.pushBack(m.vector3(0,100, i*10))
                goal.pushBack(m.vector3(0,100, (i+0.9)*10))
                R=m.map(i, 1,36, 1,0)
                G=0
                B=m.map(i, 1,36, 0,1)
                color=m.vector3(R, G, B)
                goal.pushBack(color)


            thickness=10 #-- 10 cm

            RE.drawBillboard( goal.matView(), 'goal3c', 'solidwhiteTrailZTest', thickness, 'ColorBillboardLineList' )

            for i in range(0,35+1):
                goal[i*3].y=goal[i*3].y+20
                goal[i*3+1].y=goal[i*3+1].y+20

            RE.drawBillboard( goal.matView(), 'goal3c2', '', thickness, 'ColorBillboardLineList' )
            
            points=m.matrixn(36, 6) #-- each row contains (R, G, B, x, y, z)
            for i in range(1,36+1) :
                points.row(i-1).setVec3(3, m.vector3(0,140, i*10))
                R=m.map(i, 1,36, 1,0)
                if RE.ogreVersion()==1:
                    G=0
                    B=m.map(i, 1,36, 0,1)
                else: # colormap.jpg (B channel represents point size)
                    G=0
                    B=10 # thickness
                color=m.vector3(R, G, B)
                points.row(i-1).setVec3(0, color)
            # draw colored points (doesn't work on ogrenext)
            thickness=10 #-- this thickness doesn't work for PointList
            #-- these points are very small so it may not be clearly visible.
            if RE.ogreVersion()==1:
                RE.draw('Traj', points, 'goal3c3', '', thickness, 'PointList')
            else:
                RE.drawBillboard( points, 'goal3c3', 'colormap', thickness, 'PointList')

            for i in range(1, 36+1):
                points.set(i-1, 4, 160); #-- adjust y

            if RE.ogreVersion()==1:
                #-- currently, the only way to adjust the point size is using a material (Point10 or Point20).
                #-- (defined in color.material)
                RE.draw( 'Traj', points, 'goal3c4', 'Point20', thickness, 'PointList')
            else:
                for i in range(1, 36+1):
                    points.set(i-1, 2, i*0.2); #-- adjust thickness
                RE.drawBillboard( points, 'goal3c4', 'colormap', thickness, 'PointList')


    if this.findWidget('draw meshes').checkButtonValue() :
        if True:
            # draw axes
            lref1=m.transf()
            lref1.rotation.assign(m.quater(math.radians(30), m.vector3(0,1,0)))
            lref1.translation.assign(m.vector3(0,40,0))
            RE.namedDraw('Axes',lref1,'lref',1)

            # draw axes (position scale =100)
            lref1=m.transf()
            lref1.rotation.assign(m.quater(math.radians(30), m.vector3(0,1,0)))
            lref1.translation.assign(m.vector3(0,0.04,0.1))
            RE.namedDraw('Axes',lref1,'lref2',100)

        if True:
            # draw arrow
            startPos=m.vector3(40,40,0)
            endPos=m.vector3(40,80,0)
            # see module.lua : drawArrow(...)
            thickness=10
            RE.draw('Arrow', startPos, endPos, 'arrowname', thickness)

        RE.draw("Box", m.transf(m.quater(1,0,0,0), m.vector3(0,1,0)), 'box', m.vector3(1,2,1)*0.1, 100)

        if True:
            # draw mesh
            global mesh, meshToEntity, entity, node # important. these need to be global so that there aren't removed immediately
            mesh=m.Geometry()
            numSegX=1
            numSegZ=1
            #mesh.initPlane(numSegX, numSegZ, 20, 20) # sizeX, sizeY
            mesh.initPlane(100, 100) # sizeX, sizeY
            meshToEntity=m.MeshToEntity(mesh, 'meshName')
            entity=meshToEntity.createEntity('entityName' )
            entity.setMaterialName("checkboard/crowdEditing")
            node=m.createChildSceneNode(m.ogreRootSceneNode(), "mesh_node")
            #node.attachObject(entity)
            #node.translate(0,60,0)

        if True:
            global mesh2, meshToEntity2, entity2, node2# important. these need to be global so that there aren't removed immediately
            # draw custom mesh
            mesh2=m.Geometry()
            for i in range(0, mesh2.numVertex()):
                print(mesh2.getVertex(i))
            mesh2.initPlane(40, 40) # sizeX, sizeY
            meshToEntity2=m.MeshToEntity(mesh2, 'meshName2')
            entity2=meshToEntity2.createEntity('entityName2' )
            entity2.setMaterialName("red")
            node2=m.createChildSceneNode(m.ogreRootSceneNode(), "nodeName2")
            node2.attachObject(entity2)
            node2.translate(0,60,0)

            # modify the mesh, and update the entity.
            mesh2.getVertex(0).y=40
            mesh2.getVertex(1).y=30
            mesh2.getVertex(2).y=20
            mesh2.getVertex(3).y=10
            mesh2.getNormal(0).assign(m.vector3(1,0,0))

    # test point list (fastest method for drawing many positions)
    
    points=m.matrixn(100, 6)
    for i in range(0,10):
        for j in range(0,10):

            points.row(i*10+j).setVec3(0, m.vector3(i/9, 0 , j/9)) # color
            points.row(i*10+j).setVec3(3, m.vector3(i*10, j*10+50 , 0)) # color
    RE.draw("Traj", points, 'points', '',0, 'PointList')


def eraseAll():
    RE.eraseAllDrawn()
    RE.removeEntityByName("mesh_node")
    RE.removeEntityByName("nodeName2")

def onCallback(w, userData):
    if w.id()=='set position':
        v=w.sliderValue()
        RE.draw("Box", m.transf(m.quater(1,0,v,0).normalized(), m.vector3(0,1+v,0)), 'box', m.vector3(1,2,1)*0.1, 100)
    elif w.id()=='eraseAll':
        eraseAll()
    elif w.id()=='drawAll':
        drawAll()
    elif w.id()=='memTest':
        for i in range(1000):
            print(i)
            eraseAll()
            m.renderOneFrame(False)
            drawAll()
            m.renderOneFrame(False)
    elif w.id()[0:5]=='draw ':
        eraseAll()
        drawAll()



this=RE.createMainWin(sys.argv)
this.create("Check_Button", 'draw spheres', 'draw spheres')
this.findWidget('draw spheres').checkButtonValue(True)

this.create("Check_Button", 'draw lines', 'draw lines (much faster)')
this.findWidget('draw lines').checkButtonValue(True)
this.create("Check_Button", 'draw colored lines', 'draw colored lines')
this.findWidget('draw colored lines').checkButtonValue(True)
this.create("Check_Button", 'draw meshes', 'draw meshes')
this.findWidget('draw meshes').checkButtonValue(True)
this.create("Value_Slider", 'set position', 'set position', 1)
this.widget(0).sliderRange(-0.1,0.5)
this.widget(0).sliderValue(0)

this.addButton('eraseAll')
this.addButton('drawAll')
this.addButton('memTest')
this.updateLayout()

drawAll()
print('ctor finished')
while True:
    if not RE.renderOneFrame(True): break

