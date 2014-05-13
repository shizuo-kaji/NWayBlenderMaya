#pragma once

#pragma comment(linker, "/export:initializePlugin /export:uninitializePlugin")

#include <maya/MFnPlugin.h>

#include <numeric>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

class weightControllerNode : public MPxNode
{
public:
    weightControllerNode() {};
    static  void*   creator();
    static  MStatus initialize();
    virtual MStatus     compute( const MPlug&, MDataBlock&);
    static MTypeId      id;
    static MString      nodeName;
    static MObject      aOutputs;
	static MObject		aVertices;
	static MObject		aLocator;

private:
};