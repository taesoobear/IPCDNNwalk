import heapq,pdb, random
import work.luamodule as lua
import work.rendermodule as RE
from easydict import EasyDict as edict # pip3 install easydict
tl=lua.taesooLib()

def isSimilar(a,b):
    if abs(a-b)<1e-9:
        return True
    return False
def new(cacheFile, mLoader, mMotionDOF, discontinuity, filterWindow=0.5, matchWindowSize=3, thr=1e-7, frameRate=30, _otherOptions=None):
    if _otherOptions==None:
        _otherOptions={}

    lua.require('moduleGraph')
    if not lua.F('checkEulerContinuity',mLoader, mMotionDOF, discontinuity):
        lua.F('fixEulerContinuity', mLoader, mMotionDOF, discontinuity)


    if RE.isFileExist(cacheFile):
        graph=RE.loadTable(cacheFile)
        param=[]
        if 'param' in graph :
            param=graph['param'] 

        if len(param)==4 and isSimilar(graph['param'][0], filterWindow)\
                and isSimilar(graph['param'][1], thr)\
                and isSimilar(graph['param'][2], frameRate)\
                and isSimilar(graph['param'][3], matchWindowSize):
            print("Creating a motion graph using "+cacheFile)

            edges=graph['edges']

            if isinstance(edges,list):
                edges=lua.toDict(edges)
            if isinstance(graph, edict):
                graph=dict(graph)
            graph['edges_old']=edges
            graph['edges']=lua.convertFloatTableToInt(edges)
            return MotionGraph(graph['nodes'], graph['edges'], mLoader, mMotionDOF, discontinuity, _otherOptions)


    # motion segmentation (모든 frame combination에서 transition cost 조사하면 느릴 수 있어서...)
    mCutState, squaredMomentum=lua.F("segmentMotionDOF", mLoader, mMotionDOF, discontinuity, frameRate, filterWindow )

    # construct graph
    # 1. create nodes
    transitionPoints=discontinuity| mCutState # bitwise or
    segments= tl.intIntervals()
    segments.runLengthEncode(transitionPoints)
    if True:
        for i in range(segments.size()):
            s=segments.startI(i)
            e=segments.endI(i)
            if e-s<7:
                if not discontinuity(s) and e<discontinuity.size() and not discontinuity(e):
                    transitionPoints.set(s, False)
                    transitionPoints.set(e, False)
                    transitionPoints.set(int((e+s)/2), True)

    #RE.addPanel(transitionPoints)

    nodes=tl.intvectorn()
    nodes.findIndex(transitionPoints, True)
    for i in range(nodes.size()):
        assert(transitionPoints(nodes(i)))
    
    distMat=calcDistMat(nodes, matchWindowSize, mLoader, mMotionDOF, discontinuity)

    if _otherOptions!=None and 'preventTransitions' in _otherOptions!=None:
        _preventTransitions=_otherOptions['preventTransitions']
        for k, v in _preventTransitions.items():

            for i in range(0, nodes.size()):
                for j in range(i+1, nodes.size()):
                    if nodes[i] in range(k[0], k[1]) and nodes[j] in range(v[0], v[1]):
                        distMat.set(i, j,1e5)
                        distMat.set(j, i,1e5)

    
    minDist=distMat.minimum()


    edges=lua.F('createMotionGraph',nodes, distMat,squaredMomentum, thr, discontinuity) 
    edges=lua.convertFloatTableToInt(edges)

    if 'var_name' in edges:
        del edges['var_name']
    else:
        edges=lua.toDict(edges)

    # simplify the data structure.
    newEdges={}
    for k, v in edges.items():
        edges_k=[None]*len(v)
        for i, vv in enumerate(v):
            edges_k[i]=vv['tgt']
        newEdges[k]=lua.ivec(edges_k)
    
    RE.saveTable({'nodes':nodes, 'edges':newEdges, 'param':[filterWindow, thr, frameRate, matchWindowSize]}, cacheFile,)
    return MotionGraph(nodes, newEdges, mLoader, mMotionDOF, discontinuity, _otherOptions)

def calcDistMat(nodes, windowSize, mLoader, mMotionDOF, discont):
    # assert(windowSize.is_integer())
    distMat=tl.matrixn(nodes.size(), nodes.size())
    distMat.setAllValue(1e5) # initialize to a large distance

    m=tl
    markers=[
            mLoader.getBoneByVoca(m.Voca.LEFTANKLE),
            mLoader.getBoneByVoca(m.Voca.RIGHTANKLE),
            mLoader.getBoneByVoca(m.Voca.LEFTKNEE),
            mLoader.getBoneByVoca(m.Voca.RIGHTKNEE),
            mLoader.getBoneByVoca(m.Voca.LEFTHIP),
            mLoader.getBoneByVoca(m.Voca.RIGHTHIP),
            mLoader.getBoneByVoca(m.Voca.HIPS),
            mLoader.getBoneByVoca(m.Voca.LEFTSHOULDER),
            mLoader.getBoneByVoca(m.Voca.RIGHTSHOULDER),
            mLoader.getBoneByVoca(m.Voca.LEFTELBOW),
            mLoader.getBoneByVoca(m.Voca.RIGHTELBOW),
            mLoader.getBoneByVoca(m.Voca.LEFTWRIST),
            mLoader.getBoneByVoca(m.Voca.RIGHTWRIST),
            mLoader.getBoneByVoca(m.Voca.CHEST2),
            mLoader.getBoneByVoca(m.Voca.HEAD)
            ]
    mocapTraj=m.matrixn(mMotionDOF.rows(), len(markers)*3)

    for i in range(mMotionDOF.rows()):
        mLoader.setPoseDOF(mMotionDOF.row(i))
        r=mocapTraj.row(i)
        for mm in range(len(markers)):
            r.setVec3(mm*3, markers[mm].getFrame().translation)

    metric=m.KovarMetric(False)
    nf=mMotionDOF.rows()
    print("creating a motion graph...")
    for i in range(0, nodes.size()):
        print(f'\r\r\r\r\r\r\r\r\r{i}/{nodes.size()}',end='')
        for j in range(i+1, nodes.size()):
            frame1=nodes(i)
            frame2=nodes(j)
            assert(frame2>frame1)
            if i!=j and frame1>=windowSize and frame2<nf-windowSize and\
            discont.range(frame1-windowSize, frame1+windowSize).count()==0 and\
            discont.range(frame2-windowSize, frame2+windowSize).count()==0 :

                dist=metric.calcDistance(\
                        mocapTraj.sub(frame1-windowSize, frame1+windowSize,0,0).toVector(),\
                        mocapTraj.sub(frame2-windowSize, frame2+windowSize,0,0).toVector())

                distMat.set(i,j, dist)
                distMat.set(j,i, dist)

    return distMat


# use the new function above to create an instance of MotionGraph
class MotionGraph:
    def __init__(self, nodes, edges, loader, motion, discontinuity, option=None):

        if option and 'blacklist' in option:
            self.blacklist=option['blacklist']
        else:
            self.blacklist=[]
        self.nodes=nodes # start frame
        self.edges=edges
        self.playInfo=None
        self.color='lightgrey_transparent'

        self.loader=loader
        self.motion=motion


        self.currNode=list(edges.keys())[0]
        nE=edges[self.currNode].size()
        for i, e in edges.items():
            if e.size()>nE:
                self.currNode=i
                nE=e.size()


        self.deltaTFs=[None]*self.nodes.size()

        self.nodes.pushBack(self.nodes[-1]) # index guard
        self.nodeTFs=[None]*self.nodes.size()

        if option and 'useOriginalRootTraj' in option and option['useOriginalRootTraj']:
            for i in range(self.nodes.size()):
                f=self.nodes(i)
                self.nodeTFs[i]=motion.row(f).toTransf(0).project2D()
        else:

            cutState=tl.boolN(motion.rows())
            cutState.setAt(self.nodes)
            newRootTraj=lua.F('calcSmoothProjectedRootTraj',motion, discontinuity, cutState)
            for i in range(self.nodes.size()):
                f=self.nodes(i)
                self.nodeTFs[i]=newRootTraj.row(f).toTransf(0).project2D()


        for i in range(self.nodes.size()-1):
            # b= a*delta
            self.deltaTFs[i]=self.nodeTFs[i].inverse()*self.nodeTFs[i+1]

    def prepareSegmentPlayback(self):
        self._makePlayInfo(self.currNode)

    def _makePlayInfo(self, currNode):
        if self.playInfo and currNode!=self.currNode+1:
            if self.color=='lightgrey_transparent':
                self.color='red_transparent'
            else:
                self.color='lightgrey_transparent'

        nodes=self.nodes

        self.playInfo=edict(
            startFrame=nodes(currNode), 
            endFrame=nodes(currNode+1), 
            color=self.color,
        )

    def randomTransition(self):
        randomIndex=random.randint(0, self.edges[self.currNode].size()-1)
        self.currNode=self.edges[self.currNode][randomIndex]
        self._makePlayInfo(self.currNode)

        print('currNode', self.currNode)
        RE.output('currNode', self.currNode)
    def alwaysTransition_simple(self):
        if self.edges[self.currNode].size()>1:
            randomIndex=random.randint(1, self.edges[self.currNode].size()-1)
        else:
            randomIndex=random.randint(0, self.edges[self.currNode].size()-1)
        self.currNode=self.edges[self.currNode][randomIndex]
        self._makePlayInfo(self.currNode)
        print('currNode', self.currNode)
        RE.output('currNode', self.currNode)

    # len(motionType)==len(mFileInfo)
    # example mFileInfo=[['run', 0, 100], ['walk', 100,200'], ['jump', 200,300]]
    def alwaysTransition(self, motionType=None, mFileInfo=None):
        if motionType==None:
            return alwaysTransition_simple(self)

        def isLegal(motionType, curr_frame_id):
            for i, info in enumerate(mFileInfo):
                if motionType[i] and curr_frame_id in range(info[1], info[2]):
                    return True
            return False

        curr_frame_id = self.nodes[self.currNode]
        need_change=not isLegal(motionType, curr_frame_id)

        print(f'need change? {need_change}')
        if self.edges[self.currNode].size()>1 and need_change:
            print(f'edge size: {self.edges[self.currNode].size()}')
            print(f'edge[currNode]: {self.edges[self.currNode]}')
            # print(f'nodes[inode]: {self.nodes[inode]}')
            for i in range(1, self.edges[self.currNode].size()):
                nextNode = self.edges[self.currNode][i]
                next_frame_id = self.nodes[nextNode]
                transit = isLegal(motionType,next_frame_id)
                if transit:
                    Index = i
                    changed = True
                    print('successfully transit!!!')
                    break
            if not transit:
                print('Plausible nodes? Yes! But no target motion node, wait until next search!')
                Index = 0
                changed = False
        else:
            print('Plausible nodes? No! Wait until next search!')
            Index=0
            changed = False
        self.currNode=self.edges[self.currNode][Index]
        self._makePlayInfo(self.currNode)
        print(f'frame index of the node: {self.nodes[self.currNode]}')

        print('currNode', self.currNode)
        RE.output('currNode', self.currNode)
        return changed

    def segmentDuration(self, node):
        return self.nodes[node+1]-self.nodes[node]

    def applySegmentDelta(self, node, TF):
        return TF*self.deltaTFs[node]

    def transitionToFirstNode(self, currTF):
        currTF.translation.y=0

        assert(currFrame==self.nodes[self.currNode+1])

        if True:
            # debug draw
            lines=tl.vector3N()
            startTF=currTF.copy()
            node=self.currNode
            for i in range(5):
                neighbour = self.edges[node][0]
                endTF=self.applySegmentDelta(neighbour, startTF)
                lines.pushBack(startTF.translation)
                lines.pushBack(endTF.translation)
                startTF=endTF
                node=neighbour

            RE.drawBillboard(lines.matView()*100,'path','red',5,'BillboardLineList')

        self.currNode=self.edges[self.currNode][0]
        self._makePlayInfo(self.currNode)

        print('currNode', self.currNode)
        RE.output('currNode', self.currNode)


    # position control
    def transitionToBestNextNode(self, currFrame, currTF, goalPos, windowSize=30):
        # create a priority queue and hash set to store visited nodes
        currTF.translation.y=0

        startFrame=self.nodes(self.currNode)

        #startTF=self.motion.row(startFrame).toTransf(0)

        assert(currFrame==self.nodes[self.currNode+1])

        source=self.currNode
        nodes=self.nodes
        queue, visited = [(0, 1e5, source, [], currTF)], set()
        heapq.heapify(queue)

        minWindowSize=min(30, windowSize)
        debugDrawVerbose=False
        debugDraw=False
        if debugDrawVerbose:
            paths=[]
        min_cost=1e5
        min_path=None
        full_search=False
        # traverse graph with BFS
        while queue:
            all_info= heapq.heappop(queue)
            (total_duration, path_cost, node, path, startTF) =all_info
            # visit the node if it was not visited before
            if full_search or node not in visited:
                visited.add(node)
                if debugDraw:
                    path = path + [(node, startTF)]
                else:
                    path = path + [node]

                # visit neighbours
                if node not in self.edges:
                    pdb.set_trace()
                for neighbour in self.edges[node]:
                    if full_search or neighbour not in visited:
                        c=self.segmentDuration(neighbour)
                        endTF=self.applySegmentDelta(neighbour, startTF)
                        if total_duration+c> minWindowSize:
                            l=tl.LineSegment(startTF.translation, endTF.translation)
                            goalPos.y=startTF.translation.y
                            t=l.minDistTime(goalPos)
                            minBound=tl.map(minWindowSize, total_duration, total_duration+c, 0, 1)
                            t=max(max(min(t, 1),minBound),0)

                            distCost=l.pos(t).distance(goalPos)
                            distCost=min(distCost, 3)

                            facingDir=endTF.rotation.rotationY()*tl.vector3(0,0,1)
                            goalDir=goalPos-endTF.translation
                            goalDir.y=0
                            goalDir.normalize()
                            facingCost=1.0-facingDir.dotProduct(goalDir)

                            rotY_mid=tl.quater()
                            rotY_mid.slerp(startTF.rotation.rotationY(), endTF.rotation.rotationY(), 0.5)
                            facingDir_mid=rotY_mid*tl.vector3(0,0,1)
                            facingCost=min(facingCost, 1.0-facingDir_mid.dotProduct(goalDir))
                            if distCost<3:
                                facingCost=facingCost*distCost/3

                            if node==neighbour: # cycles don't look good.
                                facingCost=facingCost+100

                            velocity=(endTF.translation-startTF.translation)/c*30
                            velAlignCost=-goalDir.dotProduct(velocity)

                            discount=1.0
                            if neighbour==node+1:
                                discount=0.8

                            path_cost=min(path_cost, (distCost+facingCost+ velAlignCost)*discount)

                            if neighbour in self.blacklist:
                                path_cost=1e5
                            #path_cost= (path_cost*total_duration+ c* (distCost+facingCost+ velAlignCost)*discount)/(total_duration+c)

                        new_all_info=(total_duration+c, path_cost, neighbour, path, endTF)
                        # hit the sink
                        if total_duration+c > windowSize:
                            if debugDrawVerbose:
                                new_all_info[3].append((neighbour, endTF))
                            else:
                                new_all_info[3].append(neighbour)
                            if path_cost<min_cost:
                                min_cost=path_cost
                                min_path=new_all_info
                            if debugDrawVerbose:
                                paths.append( new_all_info)
                            continue
                        heapq.heappush(queue, new_all_info)
        lines=tl.vector3N()
        if debugDrawVerbose:
            for path in paths:
                # draw all paths
                startTF=currTF.copy()
                _path=path[3]
                for i in range(1, len(_path)):
                    lines.pushBack(startTF.translation)
                    lines.pushBack(_path[i][1].translation)
                    startTF=_path[i][1]
            RE.drawBillboard(lines.matView()*100,'path','red',5,'BillboardLineList')
            print(len(paths)," paths traversed")

        if min_path==None:
            pdb.set_trace()

        if debugDraw:
            chosenGoal=min_path[4].copy()
            chosenGoal.translation.y=0
            RE.draw('Axes',chosenGoal,'chosenGoal',100)
            RE.draw('Axes',currTF,'currTF',100)

            # draw the best path
            _path=min_path[3]
            lines=tl.vector3N()
            startTF=currTF.copy()
            for i in range(1, len(_path)):
                lines.pushBack(startTF.translation)
                lines.pushBack(_path[i][1].translation)
                lines[-1].y=0.1
                lines[-2].y=0.1
                startTF=_path[i][1]
            RE.drawBillboard(lines.matView()*100,'min_path','blue',10,'BillboardLineList')


        if debugDraw:
            self.currNode=min_path[3][1][0]
        else:
            self.currNode=min_path[3][1]
        self._makePlayInfo(self.currNode)
    
        print('currNode', self.currNode)
        RE.output('currNode', self.currNode)


    # velocity control
    def transitionToBestNextNode_traj(self, currFrame, currTF, goalTraj, windowSize=30):
        # create a priority queue and hash set to store visited nodes
        currTF.translation.y=0

        startFrame=self.nodes(self.currNode)

        #startTF=self.motion.row(startFrame).toTransf(0)

        assert(currFrame==self.nodes[self.currNode+1])

        source=self.currNode
        nodes=self.nodes
        queue, visited = [(0, 0, source, [], currTF)], set()
        heapq.heapify(queue)

        minWindowSize=min(30, windowSize)
        debugDrawVerbose=False
        debugDraw=False
        if debugDrawVerbose:
            debugDraw=True
            paths=[]
        min_cost=1e9
        min_path=None
        full_search=False
        #lines=tl.vector3N()
        # traverse graph with BFS
        while queue:
            all_info= heapq.heappop(queue)
            (total_duration, path_cost, node, path, startTF) =all_info
            # visit the node if it was not visited before
            if full_search or node not in visited:
                visited.add(node)
                if debugDraw:
                    path = path + [(node, startTF)]
                else:
                    path = path + [node]

                # visit neighbours
                if node not in self.edges:
                    pdb.set_trace()

                goalTraj.column(1).setAllValue(startTF.translation.y)

                for neighbour in self.edges[node]:
                    if full_search or neighbour not in visited:
                        c=self.segmentDuration(neighbour)
                        endTF=self.applySegmentDelta(neighbour, startTF)
                        if total_duration+c> minWindowSize:
                            l=tl.LineSegment(startTF.translation, endTF.translation)

                            actualC=c
                            startPos=startTF.translation.copy()
                            startOri=startTF.rotation.copy()
                            if total_duration+c<goalTraj.rows():

                                endPos=endTF.translation.copy()
                                endPos.y=startTF.translation.y
                                endOri=endTF.rotation.copy()
                            else:
                                w=lua.F('sop.map', goalTraj.rows()-1, total_duration, total_duration+c, 0, 1)
                                endPos=tl.vector3()
                                endPos.interpolate(w, startTF.translation, endTF.translation)
                                endOri=tl.quater()
                                endOri.slerp(startTF.rotation.rotationY(), endTF.rotation.rotationY(), w)

                                actualC=goalTraj.rows()-1-total_duration

                            endOri=endOri.rotationY()

                            

                            goalPos=goalTraj.row(total_duration+actualC).toVector3(0)
                            goalOri=goalTraj.row(total_duration+actualC).toVector3(0)

                            distCost=0
                            facingCost=0
                            for f in range(actualC):
                                w=lua.F('sop.map', f, 0, actualC, 0, 1)
                                midPos=tl.vector3()
                                midPos.interpolate(w, startPos, endPos)
                                midOri=tl.quater()
                                midOri.slerp(startOri,  endOri, w)



                                facingDir=midOri*tl.vector3(0,0,1)

                                #lines.pushBack(midPos+tl.vector3(0,1,0))
                                #lines.pushBack(midPos+facingDir*0.05+tl.vector3(0,1,0))
                                goalDir=goalTraj.row(total_duration+f).toQuater(3).rotationY()*tl.vector3(0,0,1)

                                d=midPos.distance(goalTraj.row(total_duration+f ).toVector3(0))
                                distCost+=d*d
                                facingCostTerm=1.0-facingDir.dotProduct(goalDir)
                                facingCost+=facingCostTerm*facingCostTerm

                            #if node==neighbour: # cycles don't look good.
                            #    facingCost=facingCost+100

                            discount=1.0
                            if neighbour==node+1:
                                discount=0.8

                            path_cost=path_cost+ (distCost+facingCost*100)*discount
                            #path_cost=path_cost+ facingCost
                            #print('cost', distCost, facingCost)

                            if neighbour in self.blacklist:
                                path_cost+=1e5
                            #path_cost= (path_cost*total_duration+ c* (distCost+facingCost+ velAlignCost)*discount)/(total_duration+c)

                        new_all_info=(total_duration+c, path_cost, neighbour, path, endTF)
                        # hit the sink
                        if total_duration+c > windowSize:
                            if debugDraw:
                                new_all_info[3].append((neighbour, endTF))
                            else:
                                new_all_info[3].append(neighbour)
                            if path_cost<min_cost:
                                min_cost=path_cost
                                min_path=new_all_info
                            if debugDrawVerbose:
                                paths.append( new_all_info)
                            continue
                        heapq.heappush(queue, new_all_info)
        
        #RE.namedDraw('Traj', lines.matView()*100, 'facing', 'red',1 , 'LineList' )
        lines=tl.vector3N()
        if debugDrawVerbose:
            for path in paths:
                # draw all paths
                startTF=currTF.copy()
                _path=path[3]
                for i in range(1, len(_path)):
                    lines.pushBack(startTF.translation)
                    lines.pushBack(_path[i][1].translation)
                    startTF=_path[i][1]
            RE.drawBillboard(lines.matView()*100,'path','red',5,'BillboardLineList')
            print(len(paths)," paths traversed", min_cost)

        if min_path==None:
            pdb.set_trace()

        if debugDraw:
            chosenGoal=min_path[4].copy()
            chosenGoal.translation.y=0
            RE.draw('Axes',chosenGoal,'chosenGoal',100)
            RE.draw('Axes',currTF,'currTF',100)

            # draw the best path
            _path=min_path[3]
            lines=tl.vector3N()
            startTF=currTF.copy()
            for i in range(1, len(_path)):
                lines.pushBack(startTF.translation)
                lines.pushBack(_path[i][1].translation)
                lines[-1].y=0.1
                lines[-2].y=0.1
                startTF=_path[i][1]

            if debugDrawVerbose:
                RE.drawBillboard(lines.matView()*100,'min_path','blue',5,'BillboardLineList')
            else:
                RE.drawBillboard(lines.matView()*100,'min_path','blue',10,'BillboardLineList')


        if debugDraw:
            self.currNode=min_path[3][1][0]
        else:
            self.currNode=min_path[3][1]
        self._makePlayInfo(self.currNode)
    
        print('currNode', self.currNode)
        RE.output('currNode', self.currNode)


