import math

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

    def indexToString(self, dagPath, index, componentType):
        """
        Helper to convert API component indicies to strings that Python and MEL understand
        """
        return "%s.%s[%d]" % (dagPath, componentType, index)

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
        edgeVector = om2.MVector(mesh.getPoint(edgeVerts[0])) - om2.MVector(mesh.getPoint(edgeVerts[1]))
        return edgeVector

    def checkOutlyingEdges(self, mesh, vertIter, edgeId, candidates):
        """
        Both of the edge's verts have to vanish when the whole candidate set is
        deleted, not just when this one edge is. A vert vanishes two ways: every
        edge touching it is going too, or exactly two collinear edges survive and
        polyDelEdge -cv dissolves it. Anything else leaves a vert stranded in the
        middle of the merged face, which is the n-gon we are avoiding.
        """
        for vertId in mesh.getEdgeVertices(edgeId):
            vertIter.setIndex(vertId)
            outlyingEdges = [edge for edge in vertIter.getConnectedEdges()
                             if edge not in candidates]

            if not outlyingEdges:
                continue

            if len(outlyingEdges) != 2:
                return False

            v1 = self.getEdgeVector(mesh, outlyingEdges[0]).normal()
            v2 = self.getEdgeVector(mesh, outlyingEdges[1]).normal()

            # Collinear either way, so compare |cos(theta)| against the tolerance.
            if abs(v1 * v2) < math.cos(self.angleTolerance):
                return False

        return True

    def checkUVBorders(self, edge):
            numUVs = pm.ls(pm.polyListComponentConversion(edge, fromEdge=True, toUV=True), fl=True)
            if len(numUVs) < 3:
                return True
            else:  
                print("Fail UV check")
                return False
            
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
        dags = self.getDagPaths()
        for dag in dags:
            mesh = om2.MFnMesh(dag)
            edgeIter = om2.MItMeshEdge(dag)
            vertIter = om2.MItMeshVertex(dag)

            # Every coplanar edge starts out a candidate.
            candidates = set()
            while not edgeIter.isDone():
                if not edgeIter.onBoundary():
                    faces = edgeIter.getConnectedFaces()
                    if len(faces) == 2:
                        fn0 = mesh.getPolygonNormal(faces[0], om2.MSpace.kWorld)
                        fn1 = mesh.getPolygonNormal(faces[1], om2.MSpace.kWorld)
                        if fn0.angle(fn1) <= self.angleTolerance:
                            candidates.add(edgeIter.index())
                edgeIter.next()

            # Evicting an edge puts one of its edges back at a neighbouring vert,
            # which can strand that neighbour in turn, so keep sweeping until the
            # set stops changing. This is what lets a whole cut chain or junction
            # qualify when no single edge in it ever could on its own.
            changed = True
            while changed:
                changed = False
                for edgeId in sorted(candidates):
                    if not self.checkOutlyingEdges(mesh, vertIter, edgeId, candidates):
                        candidates.discard(edgeId)
                        changed = True

            for edgeId in sorted(candidates):
                edgeStr = self.indexToString(dag.fullPathName(), edgeId, "e")
                if edgeStr not in self.deletableEdges:
                    if self.checkUVBorders(edgeStr):
                        self.deletableEdges.append(edgeStr)

        if self.deletableEdges:
            print(f"Found {len(self.deletableEdges)} redundant edges.")
            pm.select(self.deletableEdges, replace=True)
        else:
            print(f"Found no redundant edges.")

    def deleteEdges(self):
        print("Deleting:", self.deletableEdges)
        if self.deletableEdges:
            pm.polyDelEdge(self.deletableEdges, cv=True)  

        # TODO: implement this properly
        if self.mergeVerts:
            pm.warning("Feature not supported yet")
            # self.mergeNearbyVerts()

    def findBrokenTangents(self):
        """Select hard edges with matching normals on both ends."""
        result = om2.MSelectionList()
        dags = self.getDagPaths()
        for dag in dags:
            mesh = om2.MFnMesh(dag)
            edgeIter = om2.MItMeshEdge(dag)
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
                result.add((dag, component))

                self.smoothableHardEdges.extend("{}.e[{}]".format(dag.fullPathName(), edgeId) for edgeId in edgeIds)

        om2.MGlobal.setActiveSelectionList(result, om2.MGlobal.kReplaceList)
        om2.MGlobal.displayInfo("Found {} hard edges with matching normals.".format(len(self.smoothableHardEdges)))

    def smoothHardEdges(self):
        pm.undoInfo(openChunk=True)

        sel = pm.selected()
        if sel:
            for s in sel:
                pm.select(s, replace=True)
                pm.polySoftEdge(angle=180)
                
        pm.select(clear=True)
        pm.undoInfo(closeChunk=True)

    def triangulate(self):
        print("Triangulating")
        for dag in self.getDagPaths():
            # Each split changes topology, so the iterator has to be rebuilt.
            # Loop until a full pass finds no more n-gons to cut.
            guard = 0
            while self.splitShortestDiagonal(dag):
                guard += 1
                if guard > 10000:
                    print("Triangulate: bailing out, too many splits.")
                    break

            self.findBrokenTangents()
            self.smoothHardEdges()

    def splitShortestDiagonal(self, dag):
        """Cut the shortest (i -> i+2) diagonal on the first n-gon found.

        Returns True if an edge was created, False if there is nothing left to do.
        """
        faceIter = om2.MItMeshPolygon(dag)
        while not faceIter.isDone():
            if faceIter.polygonVertexCount() > 4:
                if faceIter.isStarlike() and faceIter.isPlanar(): # don't mess around with weird geo... maybe someday.
                    verts = list(faceIter.getVertices())   # object-relative vert ids
                    points = list(faceIter.getPoints(om2.MSpace.kWorld))
                    normal = faceIter.getNormal(om2.MSpace.kWorld)
                    numPoints = len(verts)

                    shortest = (None, float('inf'))
                    for i in range(numPoints):
                        j = (i + 2) % numPoints
                        if not self.isEar(points, i, numPoints, normal):
                            continue
                        possibleNewEdgeLength = (points[i] - points[j]).length()
                        if possibleNewEdgeLength < shortest[1]:
                            shortest = (i, possibleNewEdgeLength)

                    if shortest[0] is not None:
                        i = shortest[0]
                        v1 = verts[i]
                        v2 = verts[(i + 2) % numPoints]
                        path = dag.fullPathName()
                        pm.polyConnectComponents(
                            f'{path}.vtx[{v1}]',
                            f'{path}.vtx[{v2}]')
                        return True

            faceIter.next()

        return False

    def isEar(self, points, i, numPoints, normal):
        """True if the corner at i+1 is convex, i.e. the diagonal i -> i+2
        stays inside the face. Guards against cutting outside a star-shaped n-gon."""
        a = points[i]
        b = points[(i + 1) % numPoints]
        c = points[(i + 2) % numPoints]
        return (((b - a) ^ (c - b)) * normal) > 0

    def getDagPaths(self):
        dags = []
        selection = om2.MGlobal.getActiveSelectionList()
        self.deletableEdges = []

        selIter = om2.MItSelectionList(selection)
        while not selIter.isDone():
            dags.append(selIter.getDagPath())
            selIter.next()

        return dags


class UI(MeshOptimizer):

    def __init__(self):
        super().__init__()
        windowName = "MeshOptimizer"

        if pm.window(windowName, exists=True, query=True):
            print("Deleting window:", pm.window(windowName, query=True, title=True))
            pm.deleteUI(windowName)

        pm.window(windowName, t=windowName, menuBar=True, toolbox=True)

        pm.menuBarLayout()
        pm.menu(label="Help")
        pm.menuItem(label="Get help!", command=lambda *args: self.helpWindow())

        # Padding to make things look nicer
        pm.frameLayout(labelVisible=False, marginHeight=10, marginWidth=10)

        pm.frameLayout(label="", marginHeight=5, marginWidth=5)

        pm.columnLayout(rowSpacing=10, adjustableColumn=True)
        self.angleToleranceSlider = pm.floatSliderGrp(l="Angle Tolerance", field=True, value=self.angleTolerance, minValue=0, maxValue=0.1, step=0.001,
                                                    adjustableColumn=3, columnWidth=([2,0], [3,100]), columnAttach3=["right","left","right"], 
                                                    columnOffset3=[40,-40,0], annotation="You'll probably never need to adjust this.")
        
        

        #self.mergeVertsCheckbox = pm.checkBoxGrp(label="Merge verts afterward", columnAlign=[1,"left"], columnAttach=[2,"left", -10], changeCommand=lambda *args:self.toggleVertMergeSlider(), value1=self.mergeVerts)
        self.mergeDistSlider = pm.floatSliderGrp(l="Vert Merge Dist", field=True, value=self.mergeDistance, step=0.01, 
                                        columnWidth=([2,0], [3,100]), adjustableColumn=3, columnAttach3=["right","left","right"], 
                                        columnOffset3=[40,-40,0], annotation="Info text", visible=False)

        buttonHeight = 40
        pm.rowLayout(numberOfColumns=2, ad1=True, ad2=True, generalSpacing=8)
        pm.columnLayout(rowSpacing=10, adjustableColumn=True)
        pm.button(l="Find Redundant Edges", h=35, c=lambda *args:self.findEdgesButton(), bgc=[0.6,0.8,0.6])
        pm.button(l="Delete Edges", h=35, c=lambda *args:self.deleteEdgesButton(), bgc=[0.6,0.8,0.6])
        pm.setParent('..')
        pm.button(l="Triangulate", h=80, c=lambda *args:self.triangulate(), bgc=[0.6,0.8,0.6])

        pm.setParent('..')

        pm.separator()

        pm.columnLayout(rowSpacing=10, adjustableColumn=True)
        pm.button(l="Find Smoothable Hard Edges", h=35, c=lambda *args:self.findBrokenTangents(), bgc=[0.6,0.7,0.6])
        pm.button(l="Smooth the Edges", h=35, c=lambda *args:self.smoothHardEdges(), bgc=[0.6,0.7,0.6])
        pm.setParent('..')
        
        pm.showWindow(windowName)
        
		
    def findEdgesButton(self):
        if len(pm.selected()) == 0:
            pm.error("No objects selected.")

        self.angleTolerance = pm.floatSliderGrp(self.angleToleranceSlider, query=True, value=True)
        self.mergeDistance = pm.floatSliderGrp(self.mergeDistSlider, query=True, value=True)
        #self.mergeVerts = pm.checkBoxGrp(self.mergeVertsCheckbox, query=True, value1=True)

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
        pm.text(
            l="\r\nThis tool is meant to remove superfluous edges from a model which\
        \r\ndo not add detail and can safely be optimized away without affecting the visual look.\
        \r\nit can also detect edges which can be safely smoothed to reduce vertex counts"
        )
        pm.showWindow(windowName)

    def dropDownMenu(self):
        pass

    def openWebPage(self):
        pass


meshOptimizerWindow = UI()
