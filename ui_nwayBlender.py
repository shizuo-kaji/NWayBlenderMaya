# -*- coding: utf-8 -*-
#  User interface for N-Way Blender plugin
#  @author      Shizuo KAJI
#  @date        2013/5/13

#import debugmaya
#debugmaya.startDebug()

# import modules
import maya.cmds as cmds
import pymel.core as pm

try:
#    cmds.loadPlugin("nwayDeformer.py")
#    cmds.loadPlugin("nwayDeformerFast.py")
    cmds.loadPlugin("NWayBlenderMaya");
except:
    print("Plugin already loaded")

class UI_NwayBlender:
    uiID = "NwayBlender"
    title = "NwayBlenderPlugin"
    
    def __init__(self):
        if pm.window(self.uiID, exists=True):
            pm.deleteUI(self.uiID)
        win = pm.window(self.uiID, title=self.title, menuBar=True)
        with win:
            pm.menu( label='Create', tearOff=True )
#            pm.menuItem( label="NwayPy", c=pm.Callback( self.initPlugin, "nwayDeformer") )
#            pm.menuItem( label="NwayFast", c=pm.Callback( self.initPlugin, "nwayDeformerFast") )
            pm.menuItem( label="Nway", c=pm.Callback( self.initPlugin, "nway") )
            pm.menu( label='file', tearOff=True )
            #            pm.menuItem( label="Save mesh", c=pm.Callback( self.saveMesh) )
            #pm.menuItem( label="Load mesh", c=pm.Callback( self.loadMesh) )
            self._parentLayout = pm.columnLayout( adj=True )
            with self._parentLayout:
                self.createUISet()
                    
    def createUISet(self):
        self._childLayout = pm.columnLayout( adj=True )
        with self._childLayout:
            pm.text(l="Click cage mesh first, then shift+click target mesh, and click the menu item.")
#            for deformerType in ["nwayDeformer","nwayDeformerFast","nway"]:
            for deformerType in ["nway"]:
                deformers = pm.ls(type=deformerType)
                for node in deformers:
                    frameLayout = pm.frameLayout( label=node.name(), collapsable = True)
                    with frameLayout:
                        with pm.rowLayout(numberOfColumns=1) :
                            pm.button( l="Del", c=pm.Callback( self.deleteNode, node))
                        for j in range(node.blendMesh.numElements()):
                            with pm.rowLayout(numberOfColumns=1) :
                                pm.attrFieldSliderGrp(label=node.blendWeight[j].getAlias(), min=-1.0, max=2.0, attribute= node.blendWeight[j])

    def initPlugin(self, deformerType):
        meshes = pm.selected(tr=1)
        if not meshes:
            return
        pm.select( meshes[-1])
        deformer = pm.ls(cmds.deformer(type=deformerType)[0])[0]
        for i in range(len(meshes)-1):
            deformer.bw[i].set(0.0)
            shape=meshes[i].getShapes()[0]
            cmds.connectAttr(shape+".outMesh", deformer+".blendMesh[%s]" %(i))
            pm.aliasAttr(meshes[i].name(), deformer.bw[i].name())
        self.updateUI()
        
    # delete deformer node
    def deleteNode(self,node):
        cmds.delete(node.name())
        self.updateUI()

    def updateUI(self):
        # update UI
        pm.deleteUI( self._childLayout )
        pm.setParent(self._parentLayout)
        self.createUISet()
        return
    
    def saveMesh(self):
        filedir=os.path.abspath(os.path.dirname(__file__))+"/"
        meshes = pm.selected(tr=1)[0]
        shape=meshes.getShapes()[0]
        p=mesh.getPositions(shape.name())
        tri=mesh.getTriangleList(shape.name())
        np.save(filedir+"test"+".pts", p)
        np.save(filedir+"test"+".tri", tri)
#        np.save(filedir+shape.name()+".pts", p)
#        np.save(filedir+shape.name()+".tri", tri)
        print(filedir+shape.name()+": saved")
        return
    
    def loadMesh(self):
        tet=np.load(filedir+"test"+".tet")
#        tet=np.load(filedir+shape.name()+".tet")
        print(tet)
        return
    