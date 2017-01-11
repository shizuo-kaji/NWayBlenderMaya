# -*- coding: utf-8 -*-
#  weight controller using barycentric coordinate
#  @author      Shizuo KAJI
#  @date        2013/5/13


# Maya modules
import maya.cmds as cmds
import pymel.core as pm

# set project path
#import os
try:
#    cmds.loadPlugin(os.path.abspath(os.path.dirname(__file__))+"/weightControllerNode.py")
    cmds.loadPlugin("weightController")
except:
    print("Plugin already loaded")
    
class UI_WeightController:
    uiID = "WeightController"
    title = "WeightControllerCreator"

    deformers = []
    
    ## Constructor
    def __init__(self):
        if pm.window(self.uiID, exists=True):
            pm.deleteUI(self.uiID)

        win = pm.window(self.uiID, title=self.title, menuBar=True)
        with win:
            pm.menu( label='Plugin', tearOff=True )
            pm.menuItem( label="Create", c=pm.Callback( self.initPlugin ) )
            self._parentLayout = pm.columnLayout( adj=True )
            with self._parentLayout:
                self.createUISet()

    def createUISet(self):
        self._childLayout = pm.columnLayout( adj=True )
        with self._childLayout:
            pm.text(l="select vertex objects, then click Create. the last object becomes the controller")
            self.deformers = pm.ls(type="weightController")
            for deformer in self.deformers:
                frameLayout = pm.frameLayout( label=deformer.name(), collapsable = True)
                with frameLayout:
                    with pm.rowLayout(numberOfColumns=3) :
                        pm.button( l="Del", c=pm.Callback( self.deleteNode, deformer))
                        list = cmds.listConnections(deformer.name(), t='nway');
                        pm.button( l="Out2NWay", c=pm.Callback(self.connectOut, deformer))
            
    def initPlugin(self):
        meshes = pm.selected(tr=1)
        if not meshes:
            return
        node=cmds.createNode("weightController")
        pnode = pm.ls(node)[0]
        cmds.connectAttr(meshes[-1]+".translate", node+".lc")
        for i in range(len(meshes)-1):
            cmds.connectAttr(meshes[i]+".translate", node+".v[%s]" %(i))
        self.updateUI()

    def connectOut(self, deformer):
        node = pm.ls('nway*')
        for j in range(cmds.getAttr(node[0]+'.blendWeight',size=True)):
            cmds.connectAttr(deformer+'.outputs[%s]'%(j), node[0]+".blendWeight[%s]" %(j))
        self.updateUI()

    def updateUI(self):
        pm.deleteUI( self._childLayout )
        pm.setParent(self._parentLayout)
        self.createUISet()

    def deleteNode(self,node):
        cmds.delete(node.name())
        self.updateUI()


