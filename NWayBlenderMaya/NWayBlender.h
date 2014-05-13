#pragma once
#pragma comment(linker, "/export:initializePlugin /export:uninitializePlugin")

#include <maya/MFnPlugin.h>

#include <numeric>
#include <Eigen/SparseLU>

#include "affinelib.h"

typedef Eigen::SparseMatrix<float> SpMat;
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
	void tetMatrix(const MPointArray& p, const MIntArray& triangles, std::vector<Matrix4f>& m);
	void arapHI(const std::vector<Matrix4f>& PI, const MIntArray& triangles, int dim);
	void arapAt(const std::vector< std::vector<Matrix3f> >& logR, const std::vector< std::vector<Matrix3f> >& logS,
		const std::vector< std::vector<Vector3f> >& L, const std::vector<float>& weight, std::vector<Matrix4f>& At);
	void arapG(const std::vector< Matrix4f>& At, const std::vector<Matrix4f>& PI,
		 const MIntArray& triangles,MatrixXf& G);
	
	std::vector<Matrix4f> PI;
	std::vector< std::vector<Matrix3f> > logR;
	std::vector< std::vector<Matrix3f> > logS;
	std::vector< std::vector<Vector3f> > L;
	MIntArray triangles;
    MPointArray pts;
    SparseLU<SpMat> solver;
	unsigned int numMesh;
	unsigned int numPts;
    unsigned int numTet;
};