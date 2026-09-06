

import maya.api.OpenMaya as om2
import pymel.core as pm
from collections.abc import Iterable 

class MeshOptimizer(object):
    def __init__(self):
        self.deletableEdges = []
        self.smoothableHardEdges = []
        self.mergeVerts = False
        self.angleTolerance = 0.001
        self.mergeDistance = 0.01
    
    def removeFromArray(self, components, deleteMe):
        """
        Helper that performs a simple but repetative task of removing things from MIntArrays
        """
        for i, k in enumerate(components):
            if k == deleteMe:
                components.remove(i)
        return components


    def indexToString(self, dagPath, items, componentType):
        """
        Helper to convert API component indicies to strings that Python and MEL understand
        """
        strings = []

        if not isinstance(items, Iterable):
            items = [items]

        for item in items:
            strings.append("%s.%s[%d]" % (dagPath, componentType, item))
        return strings
        

    def getVertPositions(self, dagPath):
        mesh = om2.MFnMesh(dagPath)
        
        vertIndicies = []
        vertPositions = []
        
        for vert in range(mesh.numVertices):
            vertIndicies.append(vert)
            vertPos = mesh.getPoint(vert)
            vertPositions.append((vertPos.x, vertPos.y, vertPos.z))
            
        return vertIndicies, vertPositions


    def getEdgeVector(self, mesh, edge):
        edgeVerts = mesh.getEdgeVertices(edge)
        edgeVector = (om2.MVector(mesh.getPoint(edgeVerts[0])) - om2.MVector( mesh.getPoint(edgeVerts[1])))
        return edgeVector
        
		
    def checkOutlyingEdges(self, dagPath, verts, connectedEdges):
        conVertIter = om2.MItMeshVertex(dagPath)
        while not conVertIter.isDone():
            # maya seems to crash if this list is empty!
            if verts: 
                if conVertIter.index() == verts[verts.count() - 1]:
                    outlyingEdges = conVertIter.getConnectedEdges()
                    # Deleting this edge could change the mesh in a negative way
                    # we need to check to see if it's safe
                    if outlyingEdges.count() > 4:
                        sharedEdges = list(set(outlyingEdges).intersection(connectedEdges))
                        if sharedEdges:
                            for i, edge in enumerate(sharedEdges):
                                if edge in connectedEdges:
                                    connectedEdges.remove(i)    
                        conVertIter.reset()
                    verts.remove( verts.count() - 1 )
                conVertIter.next()
				
            else:
                print("ran out of verts")
                break
            
        return connectedEdges

		
    def checkUVBorders(self, edges):
        safeEdges = []
        for edge in edges:
            numUVs = pm.ls( pm.polyListComponentConversion( edge, fromEdge=True, toUV=True ), fl=True)
            if len(numUVs) < 3:
                safeEdges.append(edge)
        return safeEdges
    
	
    def getParallelEdges(self, dagPath, vert):
        mesh = om2.MFnMesh(dagPath)
        
        vertIter = om2.MItMeshVertex(dagPath)
        while not vertIter.isDone():
            
            if vertIter.index() == vert:
                connectedEdges = vertIter.getConnectedEdges()
                # See if any edge is parallel to any other
                for i, edge1 in enumerate(connectedEdges):
                    for edge2 in connectedEdges[i+1:]:
                        
                        edge1Vector = self.getEdgeVector(mesh, edge1)
                        edge2Vector = self.getEdgeVector(mesh, edge2)
                        
                        if edge1Vector.isParallel(edge2Vector):
                            # If there are parallel edges, remove them from the list
                            connectedEdges = self.removeFromArray(connectedEdges, edge1)
                            connectedEdges = self.removeFromArray(connectedEdges, edge2)
                            
                            #connectedEdges = self.checkOutlyingEdges(dagPath, connectedVerts, connectedEdges)
                            connectedEdges = self.checkUVBorders(self.indexToString(dagPath, connectedEdges, "e"))
                            return connectedEdges
                        
            vertIter.next()
    
	# ideally merges verts around the boolean cuts but what if we don't have boolean history?
    def mergeNearbyVerts(self, dagPath, boolVertPositions):
        print("Merging verts")
        verts = om2.MFnMesh(dagPath).getPoints()
        
        mergeVerts = set(verts).difference(set(boolVertPositions))
        print("MERGE", mergeVerts)
             
    
    """
    TODO: Warn user if mesh is skinned or using vertex colors, this is not explicitly supported. 
    """
    def findBadEdges(self):
        selection = om2.MGlobal.getActiveSelectionList()
        self.deletableEdges = []

        selIter = om2.MItSelectionList(selection)
        while not selIter.isDone():
            dagPath = selIter.getDagPath()
            mesh = om2.MFnMesh(dagPath)
            edgeIter = om2.MItMeshEdge(dagPath)

            while not edgeIter.isDone():
                if not edgeIter.onBoundary():
                    faces = edgeIter.getConnectedFaces()
                    if len(faces) == 2:
                        fn0 = mesh.getPolygonNormal(faces[0], om2.MSpace.kWorld)
                        fn1 = mesh.getPolygonNormal(faces[1], om2.MSpace.kWorld)
                        if fn0.angle(fn1) <= self.angleTolerance:
                            edgeStr = self.indexToString(dagPath.fullPathName(), edgeIter.index(), 'e')
                            if edgeStr not in self.deletableEdges:
                                self.deletableEdges.append(edgeStr)
                edgeIter.next()
            
            selIter.next()

        pm.select(self.deletableEdges, replace=True)

    def deleteEdges(self):
        print("Deleting:", self.deletableEdges)
        if self.deletableEdges:
            pm.polyDelEdge(self.deletableEdges, cv=True) # polyDelEdge -cv true -ch 1 knife_cuts.e[848:895] knife_cuts.e[940:983] knife_cuts.e[1032:1079] knife_cuts.e[1118:1159];

        #TODO: implement this properly
        if self.mergeVerts:
            pm.warning("Feature not supported yet")
            #self.mergeNearbyVerts()
    

    def findBrokenTangents(self):
        """Select hard edges with matching normals on both ends."""
        selection = om2.MGlobal.getActiveSelectionList()
        dagPaths = {}

        for index in range(selection.length()):
            try:
                path = selection.getDagPath(index)
            except (RuntimeError, TypeError):
                continue

            paths = []

            if path.node().hasFn(om2.MFn.kMesh):
                paths.append(path)

            elif path.node().hasFn(om2.MFn.kTransform):
                dagIter = om2.MItDag()
                dagIter.reset(path, om2.MItDag.kDepthFirst, om2.MFn.kMesh)
                while not dagIter.isDone():
                    paths.append(dagIter.getPath())
                    dagIter.next()

            for dagPath in paths:
                if not om2.MFnDagNode(dagPath).isIntermediateObject:
                    dagPaths[dagPath.fullPathName()] = dagPath

        if not dagPaths:
            om2.MGlobal.displayError("No mesh selected.")
            return

        self.smoothableHardEdges = []
        result = om2.MSelectionList()

        for path in dagPaths.values():
            mesh = om2.MFnMesh(path)
            edgeIter = om2.MItMeshEdge(path)
            edgeIds = []

            while not edgeIter.isDone():
                if not edgeIter.onBoundary() and not edgeIter.isSmooth:
                    faces = edgeIter.getConnectedFaces()

                    if len(faces) == 2:
                        normals_match = True

                        for endpoint in (0, 1):
                            vertex_id = edgeIter.vertexId(endpoint)
                            n1 = mesh.getFaceVertexNormal(faces[0], vertex_id)
                            n2 = mesh.getFaceVertexNormal(faces[1], vertex_id)

                            if (n1.length() < 1e-12 or n2.length() < 1e-12 or n1.angle(n2) > self.angleTolerance):
                                normals_match = False
                                break

                        if normals_match:
                            edgeIds.append(edgeIter.index())

                edgeIter.next()

            if edgeIds:
                component_fn = om2.MFnSingleIndexedComponent()
                component = component_fn.create(om2.MFn.kMeshEdgeComponent)
                component_fn.addElements(edgeIds)
                result.add((path, component))

                self.smoothableHardEdges.extend("{}.e[{}]".format(path.fullPathName(), edgeId) for edgeId in edgeIds)

        om2.MGlobal.setActiveSelectionList(result, om2.MGlobal.kReplaceList)
        om2.MGlobal.displayInfo("Found {} hard edges with matching normals.".format(len(self.smoothableHardEdges)))

    def smoothHardEdges(self):
        pm.polySoftEdge(angle=180)
        pm.select(clear=True)

    def getSelectedMesh(self):
        selectedMesh = []
        for obj in pm.selected():
            meshNode = pm.listRelatives(obj, s=True, type="mesh")[0]
            selectedMesh.append(meshNode)
        return selectedMesh


class UI(MeshOptimizer):

    def __init__(self):
        super().__init__()
        windowName = "MeshOptimizer"
    
        if pm.window(windowName, exists=True, query=True):
            print("Deleting window:", pm.window(windowName, query=True, title=True))
            pm.deleteUI(windowName)
        
        pm.window(windowName, t=windowName, menuBar=True, toolbox=True)
        
        pm.menuBarLayout()
        pm.menu( label='Help' )
        pm.menuItem( label='Get help!', command=lambda *args:self.helpWindow() )
        
        # Padding to make things look nicer
        pm.frameLayout(labelVisible=False, marginHeight=10, marginWidth=10)
        
        pm.frameLayout(label="", marginHeight=5, marginWidth=5)

        pm.columnLayout(rowSpacing=10, adjustableColumn=True)
        self.angleToleranceSlider = pm.floatSliderGrp(l="Angle Tolerance", field=True, value=self.angleTolerance, minValue=0, maxValue=0.1, step=0.001,
                                                    adjustableColumn=3, columnWidth=([2,0], [3,150]), columnAttach3=["right","left","right"], 
                                                    columnOffset3=[40,-40,0], annotation="You'll probably never need to adjust this.")
        
        pm.rowLayout(numberOfColumns=2)

        self.mergeVertsCheckbox = pm.checkBoxGrp(label="Merge verts afterward", columnAlign=[1,"left"], columnAttach=[2,"left", -10], changeCommand=lambda *args:self.toggleVertMergeSlider(), value1=self.mergeVerts)
        self.mergeDistSlider = pm.floatSliderGrp(l="Vert Merge Dist", field=True, value=self.mergeDistance, step=0.01, 
                                        columnWidth=([2,0], [3,150]), adjustableColumn=3, columnAttach3=["right","left","right"], 
                                        columnOffset3=[40,-40,0], annotation="Info text", visible=False)
        pm.setParent('..')

        pm.button(l="Find Edges", h=40, c=lambda *args:self.findEdgesButton(), bgc=[0.6,0.8,0.6])
        pm.button(l="Delete Edges", h=40, c=lambda *args:self.deleteEdgesButton(), bgc=[0.6,0.8,0.6])

        pm.separator()

        pm.columnLayout(rowSpacing=10, adjustableColumn=True)
        pm.button(l="Find Smoothable Hard Edges", h=40, c=lambda *args:self.findBrokenTangents(), bgc=[0.6,0.7,0.6])
        pm.button(l="Smooth the Edges", h=40, c=lambda *args:self.smoothHardEdges(), bgc=[0.6,0.7,0.6])
        pm.setParent('..')
        
        pm.showWindow(windowName)
        
		
    def findEdgesButton(self):
        if(len(pm.selected()) == 0):
            pm.error("No objects selected.")

        self.angleTolerance = pm.floatSliderGrp(self.angleToleranceSlider, query=True, value=True)
        self.mergeDistance = pm.floatSliderGrp(self.mergeDistSlider, query=True, value=True)
        self.mergeVerts = pm.checkBoxGrp(self.mergeVertsCheckbox, query=True, value1=True)
        
        self.findBadEdges()

    def deleteEdgesButton(self):
        self.deleteEdges()

    def toggleVertMergeSlider(self):
        visibleState = pm.floatSliderGrp(self.mergeDistSlider, query=True, visible=True)
        pm.floatSliderGrp(self.mergeDistSlider, edit=True, visible=not visibleState)

    def helpWindow(self):
        windowName = "HelpWindow"
        if pm.window(windowName, exists=True, query=True):
            pm.deleteUI(windowName)
        
        pm.window(windowName, t=windowName)
        pm.columnLayout(rowSpacing=10, adjustableColumn=True)
        pm.text(l="\r\nThis tool is meant to remove superfluous edges from a model which\
        \r\ndo not add detail and can safely be optimized away without affecting the visual look.\
        \r\nit can also detect edges which can be safely smoothed to reduce vertex counts")
        pm.showWindow(windowName)

    def dropDownMenu(self):
        pass
    
	
    def openWebPage(self):
        pass
        
		
meshOptimizerWindow = UI()