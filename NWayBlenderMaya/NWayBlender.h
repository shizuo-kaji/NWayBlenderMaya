#pragma once
#pragma comment(linker, "/export:initializePlugin /export:uninitializePlugin")

#include <maya/MFnPlugin.h>

#include <numeric>
#include <Eigen/SparseLU>
#include <Eigen/SparseCholesky>

#include "affinelib.h"

typedef Eigen::SparseMatrix<double> SpMat;
typedef Eigen::Triplet<double> T;

using namespace Eigen;


class nwayDeformerNode : public MPxDeformerNode
{
public:
    nwayDeformerNode(): numMesh(0), numTet(0), numPts(0) {};
    virtual MStatus deform( MDataBlock& data, MItGeometry& itGeo, const MMatrix &localToWorldMatrix, unsigned int mIndex );
    static  void*   creator();
    static  MStatus initialize();
 
    static MTypeId      id;
    static MString      nodeName;
    static MObject      aBlendMode;
	static MObject		aBlendMesh;
	static MObject		aWeight;

private:
    void readMatrixArray(MArrayDataHandle& handle, std::vector<Matrix4d>& m);
	void tetMatrix(const MPointArray& p, const MIntArray& triangles, std::vector<Matrix4d>& m);
	void arapHI(const std::vector<Matrix4d>& PI, const MIntArray& triangles, int dim);
	void arapAt(const std::vector< std::vector<Matrix3d> >& logR, const std::vector< std::vector<Matrix3d> >& logS,
		const std::vector< std::vector<Vector3d> >& L, const std::vector<double>& weight, std::vector<Matrix4d>& At);
	void arapG(const std::vector< Matrix4d>& At, const std::vector<Matrix4d>& PI,
		 const MIntArray& triangles,MatrixXd& G);
	
	std::vector<Matrix4d> PI;
	std::vector< std::vector<Matrix3d> > logR;
	std::vector< std::vector<Matrix3d> > logS;
	std::vector< std::vector<Vector3d> > L;
	MIntArray triangles;
    MPointArray pts;
    SimplicialLDLT<SpMat> solver;
//    SparseLU<SpMat> solver;
	int numMesh;
	int numPts;
    int numTet;
};