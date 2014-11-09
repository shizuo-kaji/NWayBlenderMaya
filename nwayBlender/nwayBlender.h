#pragma once
#pragma comment(linker, "/export:initializePlugin /export:uninitializePlugin")

#include <maya/MFnPlugin.h>

#include <numeric>
#include <Eigen/Sparse>
#include <unsupported/Eigen/MatrixFunctions>

#include "affinelib.h"
#include "tetrise.h"
#include "MeshMaya.h"
#include "ARAP.h"
#include "deformerConst.h"

using namespace Eigen;

// deformer
class nwayDeformerNode : public MPxDeformerNode{
public:
    nwayDeformerNode(): numMesh(0), tetMode(-1), rotationCosistency(false), dim(0), blendMode(-1), isError(0) {};
    virtual MStatus deform( MDataBlock& data, MItGeometry& itGeo, const MMatrix &localToWorldMatrix, unsigned int mIndex );
    static  void*   creator();
    void    postConstructor();
    static  MStatus initialize();
 
    static MTypeId      id;
    static MString      nodeName;
    static MObject      aTetMode;
    static MObject      aBlendMode;
	static MObject		aBlendMesh;
	static MObject		aWeight;
    static MObject      aIteration;
    static MObject      aRotationConsistency;
    static MObject      aVisualiseEnergy;
    static MObject      aVisualisationMultiplier;
    static MObject      aEnergy;
    static MObject      aInitRotation;

private:
	std::vector<Matrix4d> PI;
	std::vector< std::vector<Matrix3d> > GL,logGL,R,logR,S,logS;
	std::vector< std::vector<Vector4d> > quat;
	std::vector< std::vector<Vector3d> > L;
    std::vector< std::vector<int> > adjacencyList;
    std::vector<int> tetList;
    std::vector<edge> edgeList;
    std::vector<vertex> vertexList;
    std::vector<int> faceList;
    std::vector<Vector3d> pts;
    SpSolver solver;
    SpMat constraintMat;
	int numMesh;
    short tetMode, blendMode, isError;
    bool rotationCosistency;
    int dim;  // total number of points including ghost ones
};