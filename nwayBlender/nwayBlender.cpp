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
#include "nwayBlender.h"
#include <set>
#include <queue>

// parametrisation mode
#define BM_SRL 0
#define BM_SES 1
#define BM_LOG3 3
#define BM_LOG4 4
#define BM_QSL 5
#define BM_AFF 10
#define BM_OFF -1

// error codes
#define ERROR_ARAP_PRECOMPUTE 1


using namespace Eigen;
using namespace AffineLib;
using namespace Tetrise;

MTypeId nwayDeformerNode::id( 0x00000300 );
MString nwayDeformerNode::nodeName( "nwayBlender" );
MObject nwayDeformerNode::aBlendMode;
MObject nwayDeformerNode::aTetMode;
MObject nwayDeformerNode::aBlendMesh;
MObject nwayDeformerNode::aWeight;
MObject nwayDeformerNode::aIteration;
MObject nwayDeformerNode::aRotationConsistency;
MObject nwayDeformerNode::aVisualiseEnergy;
MObject nwayDeformerNode::aVisualisationMultiplier;
MObject nwayDeformerNode::aEnergy;

// blend matrices
template<typename T>
void blendMatList(const std::vector< std::vector<T> >& A, const std::vector<double>& weight, std::vector<T>& X){
    int numMesh= (int)A.size();
    if(numMesh == 0) return;
    int numTet= (int)A[0].size();
    for(int i=0;i<numTet;i++){
        X[i].setZero();
        for(int j=0;j<numMesh;j++){
            X[i] += weight[j]*A[j][i];
        }
    }
}

template<typename T>
void blendMatLinList(const std::vector< std::vector<T> >& A, const std::vector<double>& weight, std::vector<T>& X){
    int numMesh= (int)A.size();
    if(numMesh == 0) return;
    int numTet= (int)A[0].size();
    for(int i=0;i<numTet;i++){
        double sum = 0.0;
        X[i].setZero();
        for(int j=0;j<numMesh;j++){
            X[i] += weight[j] * A[j][i];
            sum += weight[j];
        }
        X[i] += (1.0-sum) * T::Identity();
    }
}

// blend quaternion linearly
void blendQuatList(const std::vector< std::vector<Vector4d> >& A, const std::vector<double>& weight,
               std::vector<Vector4d>& X){
    int numMesh= (int)A.size();
    if(numMesh == 0) return;
    int numTet= (int)A[0].size();
    Vector4d I(0,0,0,1);
    for(int i=0;i<numTet;i++){
        double sum = 0.0;
        X[i].setZero();
        for(int j=0;j<numMesh;j++){
            X[i] += weight[j] * A[j][i];
            sum += weight[j];
        }
        X[i] += (1.0-sum) * I;
        X[i].normalized();
    }
}


// 
void* nwayDeformerNode::creator() { return new nwayDeformerNode; }
 
MStatus nwayDeformerNode::deform( MDataBlock& data, MItGeometry& itGeo, const MMatrix &localToWorldMatrix, unsigned int mIndex )
{
	MObject thisNode = thisMObject();
    MStatus status;
    MThreadUtils::syncNumOpenMPThreads();    // for OpenMP
    
    if(isError>0) return MS::kFailure;
    
    MArrayDataHandle hBlendMesh = data.inputArrayValue(aBlendMesh);
    short numIter = data.inputValue( aIteration ).asShort();
    short nblendMode = data.inputValue( aBlendMode ).asShort();
    short ntetMode = data.inputValue( aTetMode ).asShort();
    double visualisationMultiplier = data.inputValue(aVisualisationMultiplier).asDouble();
    bool visualiseEnergy = data.inputValue( aVisualiseEnergy ).asBool();
	bool nrotationCosistency = data.inputValue( aRotationConsistency ).asBool();
    if( nrotationCosistency != rotationCosistency){
        numMesh = 0;
        rotationCosistency = nrotationCosistency;
    }
    MPointArray Mpts;
    itGeo.allPositions(Mpts);
    int nnumMesh = hBlendMesh.elementCount();
    int numPts = Mpts.length();
    int numTet = (int)tetList.size()/4;
    // initialisation
    if(tetMode != ntetMode){
        tetMode = ntetMode;
        numMesh = 0;
        // point list
        pts.resize(numPts);
        for(int i=0;i<numPts;i++){
            pts[i] << Mpts[i].x, Mpts[i].y, Mpts[i].z;
        }
		std::vector<Matrix4d> P;
        getMeshData(data, input, inputGeom, mIndex, tetMode, pts, tetList, faceList, edgeList, vertexList, P);
        dim = removeDegenerate(tetMode, numPts, tetList, faceList, edgeList, vertexList, P);
        makeAdjacencyList(tetMode, tetList, edgeList, vertexList, adjacencyList);
        makeTetMatrix(tetMode, pts, tetList, faceList, edgeList, vertexList, P);
        // prepare ARAP solver
        numTet = (int)tetList.size()/4;
        PI.resize(numTet);
		for(int i=0;i<numTet;i++){
			PI[i] = P[i].inverse().eval();
        }
        std::vector<double> tetWeight(numTet,1.0);
        std::vector< std::map<int,double> > constraint(0);
        //constraint[0][0]=1.0;
        isError = ARAPprecompute(PI, tetList, tetWeight, constraint, EPSILON, dim, constraintMat, solver);
        if(isError>0) return MS::kFailure;
    }
	// if blend mesh is added, compute log for each tet
    logR.resize(nnumMesh); logS.resize(nnumMesh);
    R.resize(nnumMesh); S.resize(nnumMesh);
    GL.resize(nnumMesh); logGL.resize(nnumMesh);
    quat.resize(nnumMesh);
    L.resize(nnumMesh);
    // for recomputation of parametrisation
    if(numMesh>nnumMesh || nblendMode != blendMode){
        numMesh =0;
        blendMode = nblendMode;
    }
	for(int j=numMesh; j<nnumMesh; j++){
        hBlendMesh.jumpToElement(j);
        MFnMesh blendMesh(hBlendMesh.inputValue().asMesh());
	    MPointArray Mbpts;
        blendMesh.getPoints( Mbpts );
		if(numPts != Mbpts.length()){
            MGlobal::displayInfo("incompatible mesh");
            return MS::kFailure;
        }
        std::vector<Vector3d> bpts(numPts);
        for(int i=0;i<numPts;i++){
            bpts[i] << Mbpts[i].x, Mbpts[i].y, Mbpts[i].z;
        }
		std::vector<Matrix4d> Q(numTet);
        makeTetMatrix(tetMode, bpts, tetList, faceList, edgeList, vertexList, Q);
		logR[j].resize(numTet); logS[j].resize(numTet);
		R[j].resize(numTet); S[j].resize(numTet);
		GL[j].resize(numTet); logGL[j].resize(numTet);
		quat[j].resize(numTet);
		L[j].resize(numTet);
        for(int i=0;i<numTet;i++)  {
            Matrix4d aff=PI[i]*Q[i];
            GL[j][i]=aff.block(0,0,3,3);
            L[j][i]=transPart(aff);
            parametriseGL(GL[j][i], logS[j][i] ,R[j][i]);
        }
        if( blendMode == BM_LOG3){
            for(int i=0;i<numTet;i++)
                logGL[j][i]=GL[j][i].log();
        }else if( blendMode == BM_QSL){
            for(int i=0;i<numTet;i++){
                S[j][i]=expSym(logS[j][i]);
                Quaternion<double> q(R[j][i].transpose());
                quat[j][i] << q.x(), q.y(), q.z(), q.w();
            }
        }
        // traverse tetrahedra to compute continuous log of rotation
        if(rotationCosistency){
            std::set<int> remain;
            std::queue<int> later;
            std::vector<Vector3d> prevN(numTet, Vector3d::Zero());
            std::vector<double> prevTheta(numTet, 0.0);
            for(int i=0;i<numTet;i++){
                remain.insert(remain.end(),i);
            }
            while(!remain.empty()){
                int next;
                if( !later.empty()){
                    next = later.front();
                    later.pop();
                    remain.erase(next);
                }else{
                    next = *remain.begin();
                    remain.erase(remain.begin());
                }
                logR[j][next]=logSOc(R[j][next],prevTheta[next],prevN[next]);
                for(int k=0;k<adjacencyList[next].size();k++){
                    int f=adjacencyList[next][k];
                    if(remain.erase(f)>0){
                        prevN[f]=prevN[next];
                        prevTheta[f]=prevTheta[next];
                        later.push(f);
                    }
                }
            }
        }else{
            for(int i=0;i<numTet;i++)
                logR[j][i] = logSO(R[j][i]);
        }
	}
    numMesh=nnumMesh;
    if(numMesh == 0) return MS::kSuccess;

    // load weights
    std::vector<double> weight(numMesh);
    MArrayDataHandle hWeight = data.inputArrayValue(aWeight);
	if(hWeight.elementCount() != numMesh) { return MS::kSuccess; }
	for(int i=0;i<numMesh;i++){
		hWeight.jumpToArrayElement(i);
		weight[i]=hWeight.inputValue().asDouble();
	}
	// compute ideal affine
    std::vector<Vector3d> new_pts(numPts);
	std::vector<Matrix4d> A(numTet);
    std::vector<Matrix3d> AR(numTet),AS(numTet);
    std::vector<Vector3d> AL(numTet);
    
    blendMatList(L, weight, AL);
    if(blendMode==BM_SRL){
        blendMatList(logR, weight, AR);
        blendMatList(logS, weight, AS);
        #pragma omp parallel for
        for(int i=0;i<numTet;i++){
            AR[i] = expSO(AR[i]);
            AS[i] = expSym(AS[i]);
        }
    }else if(blendMode == BM_LOG3){  // log
        blendMatList(logGL, weight, AR);
        #pragma omp parallel for
        for(int i=0;i<numTet;i++){
            AR[i] = AR[i].exp();
            AS[i] = Matrix3d::Identity();
        }
    }else if(blendMode == BM_QSL){ // quaternion
        std::vector<Vector4d> Aq(numTet);
        blendMatLinList(S, weight, AS);
        blendQuatList(quat, weight, Aq);
        #pragma omp parallel for
        for(int i=0;i<numTet;i++){
            Quaternion<double> Q(Aq[i]);
            AR[i] = Q.matrix().transpose();
        }
    }else if(blendMode == BM_AFF){ // linear
        blendMatLinList(GL, weight, AR);
        for(int i=0;i<numTet;i++){
            AS[i] = Matrix3d::Identity();            
        }
    }else{
        return MS::kFailure;
    }
    
    MatrixXd G(dim+1,3),Sol;
    std::vector<double> tetEnergy(numTet);
    // iterate to determine vertices position
    for(int k=0;k<numIter;k++){
        for(int i=0;i<numTet;i++){
            A[i]=pad(AS[i]*AR[i],AL[i]);
        }
        // solve ARAP
        std::vector<Vector3d> constraintVector(0);
        std::vector<double> tetWeight(numTet,1.0);
        //constraintVector[0]=pts[0];
        ARAPSolve(A, PI, tetList, tetWeight, constraintVector, EPSILON, dim, constraintMat, solver, Sol);
        
        // set new vertices position
        for(int i=0;i<numPts;i++){
            new_pts[i][0]=Sol(i,0);
            new_pts[i][1]=Sol(i,1);
            new_pts[i][2]=Sol(i,2);
        }
        // if iteration continues
        if(k+1<numIter || visualiseEnergy){
            std::vector<Matrix4d> Q(numTet);
            makeTetMatrix(tetMode, new_pts, tetList, faceList, edgeList, vertexList, Q);
            Matrix3d S;
            #pragma omp parallel for
            for(int i=0;i<numTet;i++)  {
                polarHigham((PI[i]*Q[i]).block(0,0,3,3), S, AR[i]);
//                tetEnergy[i] = (S-Matrix3d::Identity()).squaredNorm();
                tetEnergy[i] = (S-AS[i]).squaredNorm();
            }
        }
    }
    // set new vertex position
    for(int i=0;i<numPts;i++){
        Mpts[i].x=Sol(i,0);
        Mpts[i].y=Sol(i,1);
        Mpts[i].z=Sol(i,2);
    }
    itGeo.setAllPositions(Mpts);
    
    // set vertex color according to ARAP energy
    if(visualiseEnergy){
        std::vector<double> ptsEnergy;
        makePtsWeightList(tetMode, numPts, tetList, faceList, edgeList, vertexList, tetEnergy, ptsEnergy);
        //double max_energy = *std::max_element(ptsEnergy.begin(), ptsEnergy.end());
        outputAttr(data, aEnergy, ptsEnergy);
        for(int i=0;i<numPts;i++){
            ptsEnergy[i] *= visualisationMultiplier;     //  or /= max_energy
        }
        visualise(data, outputGeom, ptsEnergy);
    }

    return MS::kSuccess;
}


// plugin (un)initialiser
MStatus nwayDeformerNode::initialize()
{
    MFnTypedAttribute tAttr;
    MFnNumericAttribute nAttr;
    MFnEnumAttribute eAttr;

	aBlendMesh = tAttr.create("blendMesh", "mesh", MFnData::kMesh);
    tAttr.setArray(true);
    tAttr.setUsesArrayDataBuilder(true); 
    addAttribute(aBlendMesh);
	attributeAffects( aBlendMesh, outputGeom );

	aWeight = nAttr.create("blendWeight", "bw", MFnNumericData::kDouble, 0.0);
    nAttr.setArray(true);
    nAttr.setKeyable(true);
    nAttr.setStorable(true);
    nAttr.setUsesArrayDataBuilder(true);
    addAttribute(aWeight);
	attributeAffects( aWeight, outputGeom );

	aRotationConsistency = nAttr.create( "rotationConsistency", "rc", MFnNumericData::kBoolean, false );
    nAttr.setStorable(true);
    addAttribute( aRotationConsistency );
    attributeAffects( aRotationConsistency, outputGeom );

	aVisualiseEnergy = nAttr.create( "visualiseEnergy", "ve", MFnNumericData::kBoolean, false );
    nAttr.setStorable(true);
    addAttribute( aVisualiseEnergy );
    attributeAffects( aVisualiseEnergy, outputGeom );

	aVisualisationMultiplier = nAttr.create("visualisationMultiplier", "vmp", MFnNumericData::kDouble, 1.0);
    nAttr.setStorable(true);
	addAttribute( aVisualisationMultiplier );
	attributeAffects( aVisualisationMultiplier, outputGeom );
    
    aBlendMode = eAttr.create( "blendMode", "bm", BM_SRL );
    eAttr.addField( "expSO+expSym", BM_SRL );
    eAttr.addField( "logmatrix3", BM_LOG3 );
    eAttr.addField( "quat+linear", BM_QSL );
    eAttr.addField( "linear", BM_AFF );
    eAttr.addField( "off", BM_OFF );
    addAttribute( aBlendMode );
    attributeAffects( aBlendMode, outputGeom );

    aTetMode = eAttr.create( "tetMode", "tm", TM_FACE );
    eAttr.addField( "face", TM_FACE );
    eAttr.addField( "edge", TM_EDGE );
    eAttr.addField( "vertex", TM_VERTEX );
    eAttr.addField( "vface", TM_VFACE );
    addAttribute( aTetMode );
    attributeAffects( aTetMode, outputGeom );

	aIteration = nAttr.create("iteration", "it", MFnNumericData::kShort, 1);
    addAttribute(aIteration);
    attributeAffects(aIteration, outputGeom);
    
    // this shouldn't affect outputGeom
	aEnergy = nAttr.create("energy", "energy", MFnNumericData::kDouble, 0.0);
    nAttr.setArray(true);
    nAttr.setKeyable(true);
    nAttr.setStorable(true);
    nAttr.setUsesArrayDataBuilder(true);
    addAttribute(aEnergy);

    // Make the deformer weights paintable
    MGlobal::executeCommand( "makePaintable -attrType multiFloat -sm deformer nwayBlender weights;" );
 
    return MS::kSuccess;
}


// this deformer also changes colours
void nwayDeformerNode::postConstructor(){
	setDeformationDetails(kDeformsColors);
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
