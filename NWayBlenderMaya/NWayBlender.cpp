/**
 * @file NWayBlender.cpp
 * @brief N-Way shape blend deformer plugin for Maya
 * @section LICENSE The MIT License
 * @section requirements:  Eigen 3:  http://eigen.tuxfamily.org/
 * @section Autodesk Maya: http://www.autodesk.com/products/autodesk-maya/overview
 * @section (included) AffineLib: https://github.com/shizuo-kaji/AffineLib
 * @version 0.10
 * @date  1/Dec/2012
 * @author Shizuo KAJI
 */

#include "StdAfx.h"
#include "NWayBlender.h"

using namespace Eigen;

MTypeId nwayDeformerNode::id( 0x00000300 );
MString nwayDeformerNode::nodeName( "nway" );
MObject nwayDeformerNode::aBlendMode;
MObject nwayDeformerNode::aBlendMesh;
MObject nwayDeformerNode::aWeight;

void* nwayDeformerNode::creator() { return new nwayDeformerNode; }
 
MStatus nwayDeformerNode::deform( MDataBlock& data, MItGeometry& itGeo, const MMatrix &localToWorldMatrix, unsigned int mIndex )
{
	MObject thisNode = thisMObject();
    MStatus status;
    MThreadUtils::syncNumOpenMPThreads();    // for OpenMP
    
	MArrayDataHandle hWeight = data.inputArrayValue(aWeight);
    MArrayDataHandle hBlendMesh = data.inputArrayValue(aBlendMesh);
    int nnumMesh = hBlendMesh.elementCount();
    if(numPts == 0){   // first time
        // load target mesh
        MArrayDataHandle hInput = data.outputArrayValue( input, &status );
        CHECK_MSTATUS_AND_RETURN_IT( status );
        status = hInput.jumpToElement( mIndex );
        CHECK_MSTATUS_AND_RETURN_IT( status );
        MObject oInputGeom = hInput.outputValue().child( inputGeom ).asMesh();
        MFnMesh inputMesh( oInputGeom );
        inputMesh.getPoints( pts );
		numPts=pts.length();
        // get face tetrahedra
        MIntArray count;
        inputMesh.getTriangles( count, triangles );
		numTet=triangles.length()/3;
		std::vector<Matrix4f> P(numTet);
        PI.resize(numTet);
        tetMatrix(pts, triangles, P);
        // prepare ARAP solver
		for(unsigned int i=0;i<numTet;i++)
			PI[i] = P[i].inverse();
		int dim=numTet+numPts;
        arapHI(PI, triangles, dim);
        return MS::kSuccess;
    }
	// if blend mesh is added, compute log for each tet
    logR.resize(nnumMesh);
    logS.resize(nnumMesh);
    L.resize(nnumMesh);
	for(unsigned int j=numMesh; j<nnumMesh; j++){
        hBlendMesh.jumpToElement(j);
        MFnMesh blendMesh(hBlendMesh.inputValue().asMesh());
	    MPointArray pts;
        blendMesh.getPoints( pts );
		if(numPts != pts.length()){
            MGlobal::displayInfo("incompatible mesh");
            return MS::kFailure;
        }
		std::vector<Matrix4f> Q(numTet);
        tetMatrix(pts, triangles, Q);
		Matrix4f aff;
        Matrix3f R,U;
        Vector3f s;
		logR[j].resize(numTet);
		logS[j].resize(numTet);
		L[j].resize(numTet);
        for(unsigned int i=0;i<numTet;i++)  {
            aff=PI[i]*Q[i];
            Matrixlib::parametriseGL(aff.block(0,0,3,3), logS[j][i] ,R);
            logR[j][i]=Matrixlib::logSO(R);
            L[j][i]=Matrixlib::transPart(aff);
		}
	}
    numMesh=nnumMesh;
    // load weights
    std::vector<float> weight(numMesh);
	if(hWeight.elementCount() != numMesh) { return MS::kSuccess; }
	for(unsigned int i=0;i<numMesh;i++){
		hWeight.jumpToArrayElement(i);
		weight[i]=hWeight.inputValue().asFloat();
	}
	// compute ideal affine
	int dim=numTet+numPts;
	std::vector<Matrix4f> At(numTet);
    arapAt(logR, logS, L, weight, At);
    // solve ARAP
	MatrixXf G=MatrixXf::Zero(dim,3);
    arapG(At, PI, triangles, G);
    MatrixXf Sol = solver.solve(G);
	for(unsigned int i=0;i<numPts;i++){
		pts[i].x=Sol(i,0);
		pts[i].y=Sol(i,1);
		pts[i].z=Sol(i,2);
	}
    itGeo.setAllPositions(pts);

    return MS::kSuccess;
}


// construct face tetrahedra by adding normals to face triangles
void nwayDeformerNode::tetMatrix(const MPointArray& p, const MIntArray& triangles, std::vector<Matrix4f>& m){
    MVector u, v, q;
    for(unsigned int i=0;i<numTet;i++)
    {
        u=p[triangles[3*i+1]]-p[triangles[3*i]];
        v=p[triangles[3*i+2]]-p[triangles[3*i]];
        q=u^v;
        q.normalize();
        
        m[i] << p[triangles[3*i]].x, p[triangles[3*i]].y, p[triangles[3*i]].z, 1,
        p[triangles[3*i+1]].x, p[triangles[3*i+1]].y, p[triangles[3*i+1]].z, 1,
        p[triangles[3*i+2]].x, p[triangles[3*i+2]].y, p[triangles[3*i+2]].z, 1,
        q[0]+p[triangles[3*i]].x,q[1]+p[triangles[3*i]].y, q[2]+p[triangles[3*i]].z,1;
    }
}
// ARAP slover
void nwayDeformerNode::arapHI(const std::vector<Matrix4f>& PI, const MIntArray& triangles, int dim){
    std::vector<T> tripletList;
    tripletList.reserve(dim*dim);
    Matrix4f Hlist;
	Matrix4f diag=Matrix4f::Identity();
	diag(3,3)=EPSILON;
    int s,t;
	for(unsigned int i=0;i<numTet;i++){
		Hlist=PI[i].transpose()*diag*PI[i];
		for(unsigned int j=0;j<4;j++){
            if(j==3){
                s=numPts+i;
            }else{
                s=triangles[3*i+j];
            }
			for(unsigned int k=0;k<4;k++){
                if(k==3){
                    t=numPts+i;
                }else{
                    t=triangles[3*i+k];
                }
                tripletList.push_back(T(t,s,Hlist(j,k)));
			}
		}
	}
    SpMat mat(dim,dim);
    mat.setFromTriplets(tripletList.begin(), tripletList.end());
    solver.compute(mat);
}
// blend matrices
void nwayDeformerNode::arapAt(const std::vector< std::vector<Matrix3f> >& logR, const std::vector< std::vector<Matrix3f> >& logS,
		const std::vector< std::vector<Vector3f> >& L, const std::vector<float>& weight, std::vector<Matrix4f>& At)
{
    Matrix3f lR,lS,A;
    Vector3f lL;
    for(unsigned int i=0;i<numTet;i++)
    {
        lR = Matrix3f::Zero();
        lS = Matrix3f::Zero();
        lL = Vector3f::Zero();
        for(unsigned int j=0;j<numMesh;j++)
        {
            lR += weight[j]*logR[j][i];
            lS += weight[j]*logS[j][i];
            lL += weight[j]*L[j][i];
        }
        A=Matrixlib::expSym(lS)*Matrixlib::expSO(lR);
        At[i]=Matrixlib::affine(A,lL);
    }
}

// ARAP
void nwayDeformerNode::arapG(const std::vector<Matrix4f>& At, const std::vector<Matrix4f>& PI,
	 const MIntArray& triangles, MatrixXf& G)
{
    Matrix4f Glist;
	Matrix4f diag=Matrix4f::Identity();
	diag(3,3)=EPSILON;
    for(unsigned int i=0;i<numTet;i++)
    {
		Glist=At[i].transpose()*diag*PI[i];
        for(unsigned int k=0;k<3;k++)
        {
            for(unsigned int j=0;j<3;j++)
            {
                G(triangles[3*i+j],k) += Glist(k,j);
            }
            G(numPts+i,k) += Glist(k,3);
        }
    }
}


// plugin (un)initialiser
MStatus nwayDeformerNode::initialize()
{
    MFnTypedAttribute tAttr;
    MFnNumericAttribute nAttr;

	aBlendMesh = tAttr.create("blendMesh", "bm", MFnData::kMesh);
    tAttr.setArray(true);
    tAttr.setUsesArrayDataBuilder(true); 
    addAttribute(aBlendMesh);
	attributeAffects( aBlendMesh, outputGeom );

	aWeight = nAttr.create("blendWeight", "bw", MFnNumericData::kFloat, 0.0);
    nAttr.setArray(true);
    nAttr.setKeyable(true);
    nAttr.setStorable(true);
    nAttr.setUsesArrayDataBuilder(true);
    addAttribute(aWeight);
	attributeAffects( aWeight, outputGeom );

    return MS::kSuccess;
}


 
MStatus initializePlugin( MObject obj )
{
    MStatus status;
    MFnPlugin plugin( obj, "Shizuo KAJI", "0.1", "Any");
 
    status = plugin.registerNode( nwayDeformerNode::nodeName, nwayDeformerNode::id, nwayDeformerNode::creator, nwayDeformerNode::initialize, MPxNode::kDeformerNode );
    CHECK_MSTATUS_AND_RETURN_IT( status );
 
    return status;
}
 
MStatus uninitializePlugin( MObject obj )
{
    MStatus   status;
    MFnPlugin plugin( obj );
 
    status = plugin.deregisterNode( nwayDeformerNode::id );
    CHECK_MSTATUS_AND_RETURN_IT( status );
 
    return status;
}