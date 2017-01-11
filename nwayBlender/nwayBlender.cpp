/**
 * @file NWayBlender.cpp
 * @brief N-Way shape blend deformer plugin for Maya
 * @section LICENSE The MIT License
 * @section requirements:  Eigen 3:  http://eigen.tuxfamily.org/
 * @section Autodesk Maya: http://www.autodesk.com/products/autodesk-maya/overview
 * @section (included) AffineLib: https://github.com/shizuo-kaji/AffineLib
 * @version 0.20
 * @date  18/Jul/2015
 * @author Shizuo KAJI
 */

#include "StdAfx.h"
#include "nwayBlender.h"
#include <set>
#include <queue>
#include <ctime>


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
MObject nwayDeformerNode::aInitRotation;
MObject nwayDeformerNode::aAreaWeighted;
MObject nwayDeformerNode::aARAP;

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
//            clock_t clock_start=clock();

    MObject thisNode = thisMObject();
    MStatus status;
    MThreadUtils::syncNumOpenMPThreads();    // for OpenMP
    
    MArrayDataHandle hBlendMesh = data.inputArrayValue(aBlendMesh);
    short numIter = data.inputValue( aIteration ).asShort();
    short nblendMode = data.inputValue( aBlendMode ).asShort();
    short tetMode = data.inputValue( aTetMode ).asShort();
    double visualisationMultiplier = data.inputValue(aVisualisationMultiplier).asDouble();
    bool visualiseEnergy = data.inputValue( aVisualiseEnergy ).asBool();
	bool nrotationCosistency = data.inputValue( aRotationConsistency ).asBool();
    MPointArray Mpts;
    itGeo.allPositions(Mpts);
    int nnumMesh = hBlendMesh.elementCount();
    int numPts = Mpts.length();
    // initialisation
    if(!data.isClean(aARAP)){
//        clock_t clock_start=clock();
        numMesh = 0;
        // point list
        pts.resize(numPts);
        for(int i=0;i<numPts;i++){
            pts[i] << Mpts[i].x, Mpts[i].y, Mpts[i].z;
        }
        // make tetrahedral structure
        getMeshData(data, input, inputGeom, mIndex, tetMode, pts, mesh.tetList, faceList, edgeList, vertexList, mesh.tetMatrix, mesh.tetWeight);
        mesh.dim = removeDegenerate(tetMode, numPts, mesh.tetList, faceList, edgeList, vertexList, mesh.tetMatrix);
        makeTetMatrix(tetMode, pts, mesh.tetList, faceList, edgeList, vertexList, mesh.tetMatrix, mesh.tetWeight);
        makeAdjacencyList(tetMode, mesh.tetList, edgeList, vertexList, adjacencyList);
        mesh.numTet = (int)mesh.tetList.size()/4;
        mesh.computeTetMatrixInverse();
        // prepare ARAP solver
        if(!data.inputValue( aAreaWeighted ).asBool()){
            mesh.tetWeight.clear();
            mesh.tetWeight.resize(mesh.numTet,1.0);
        }
        mesh.constraintWeight.resize(1);
        mesh.constraintWeight[0] = std::make_pair(0,1.0);
        mesh.constraintVal.resize(1,3);
        mesh.constraintVal(0,0) = pts[0][0];
        mesh.constraintVal(0,1) = pts[0][1];
        mesh.constraintVal(0,2) = pts[0][2];
        
        isError = mesh.ARAPprecompute();
        status = data.setClean(aARAP);
        //        MString es="Init timing: ";
//        double timing=(double)(clock()- clock_start)/CLOCKS_PER_SEC;
//        es += timing;
//        MGlobal::displayInfo(es);
    }
    if(isError>0) return MS::kFailure;
	// if blend mesh is added, compute log for each tet
    logR.resize(nnumMesh); logS.resize(nnumMesh);
    R.resize(nnumMesh); S.resize(nnumMesh);
    GL.resize(nnumMesh); logGL.resize(nnumMesh);
    quat.resize(nnumMesh);
    L.resize(nnumMesh);
    // for recomputation of parametrisation
    if(numMesh>nnumMesh || nblendMode != blendMode || nrotationCosistency != rotationCosistency){
        numMesh =0;
        blendMode = nblendMode;
        rotationCosistency = nrotationCosistency;
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
        makeTetMatrix(tetMode, bpts, mesh.tetList, faceList, edgeList, vertexList, Q, dummy_weight);
		logR[j].resize(mesh.numTet); logS[j].resize(mesh.numTet);
		R[j].resize(mesh.numTet); S[j].resize(mesh.numTet);
		GL[j].resize(mesh.numTet); L[j].resize(mesh.numTet);
        for(int i=0;i<mesh.numTet;i++)  {
            Matrix4d aff=mesh.tetMatrixInverse[i]*Q[i];
            GL[j][i]=aff.block(0,0,3,3);
            L[j][i]=transPart(aff);
            parametriseGL(GL[j][i], logS[j][i] ,R[j][i]);
        }
        if( blendMode == BM_LOG3){
            logGL[j].resize(mesh.numTet);
            for(int i=0;i<mesh.numTet;i++)
                logGL[j][i]=GL[j][i].log();
        }else if( blendMode == BM_SQL){
            quat[j].resize(mesh.numTet);
            for(int i=0;i<mesh.numTet;i++){
                S[j][i]=expSym(logS[j][i]);
                Quaternion<double> q(R[j][i].transpose());
                quat[j][i] << q.x(), q.y(), q.z(), q.w();
            }
        }else if( blendMode == BM_SlRL){
            for(int i=0;i<mesh.numTet;i++){
                S[j][i]=expSym(logS[j][i]);
            }
        }
        // traverse tetrahedra to compute continuous log of rotation
        if(rotationCosistency){
            std::set<int> remain;
            std::queue<int> later;
            // load initial rotation from the attr
            Matrix3d initR;
            double angle = data.inputValue(aInitRotation).asDouble();
            initR << 0,M_PI * angle/180.0,0,  -M_PI * angle/180.0,0,0, 0,0,0;
            std::vector<Matrix3d> prevSO(mesh.numTet, initR);
            // create the adjacency graph to traverse
            for(int i=0;i<mesh.numTet;i++){
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
                logR[j][next]=logSOc(R[j][next],prevSO[next]);
                for(int k=0;k<adjacencyList[next].size();k++){
                    int f=adjacencyList[next][k];
                    if(remain.erase(f)>0){
                        prevSO[f]=logR[j][next];
                        later.push(f);
                    }
                }
            }
        }else{
            for(int i=0;i<mesh.numTet;i++)
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
	std::vector<Matrix4d> A(mesh.numTet);
    std::vector<Matrix3d> AR(mesh.numTet),AS(mesh.numTet);
    std::vector<Vector3d> AL(mesh.numTet);
    
    blendMatList(L, weight, AL);
    if(blendMode==BM_SRL){
        blendMatList(logR, weight, AR);
        blendMatList(logS, weight, AS);
        #pragma omp parallel for
        for(int i=0;i<mesh.numTet;i++){
            AR[i] = expSO(AR[i]);
            AS[i] = expSym(AS[i]);
        }
    }else if(blendMode == BM_LOG3){  // log
        blendMatList(logGL, weight, AR);
        #pragma omp parallel for
        for(int i=0;i<mesh.numTet;i++){
            AR[i] = AR[i].exp();
            AS[i] = Matrix3d::Identity();
        }
    }else if(blendMode == BM_SQL){ // quaternion
        std::vector<Vector4d> Aq(mesh.numTet);
        blendMatLinList(S, weight, AS);
        blendQuatList(quat, weight, Aq);
        #pragma omp parallel for
        for(int i=0;i<mesh.numTet;i++){
            Quaternion<double> Q(Aq[i]);
            AR[i] = Q.matrix().transpose();
        }
    }else if(blendMode == BM_SlRL){ // expSO+linear Sym
        blendMatList(logR, weight, AR);
        blendMatLinList(S, weight, AS);
        #pragma omp parallel for
        for(int i=0;i<mesh.numTet;i++){
            AR[i] = expSO(AR[i]);
        }
    }else if(blendMode == BM_AFF){ // linear
        blendMatLinList(GL, weight, AR);
        for(int i=0;i<mesh.numTet;i++){
            AS[i] = Matrix3d::Identity();            
        }
    }else{
        return MS::kFailure;
    }
    
    std::vector<double> tetEnergy(mesh.numTet);
    // iterate to determine vertices position
    for(int k=0;k<numIter;k++){
        for(int i=0;i<mesh.numTet;i++){
            A[i]=pad(AS[i]*AR[i],AL[i]);
        }
        // solve ARAP
        mesh.ARAPSolve(A);
        
        // set new vertices position
        for(int i=0;i<numPts;i++){
            new_pts[i][0]=mesh.Sol(i,0);
            new_pts[i][1]=mesh.Sol(i,1);
            new_pts[i][2]=mesh.Sol(i,2);
        }
        // if iteration continues
        if(k+1<numIter || visualiseEnergy){
            makeTetMatrix(tetMode, new_pts, mesh.tetList, faceList, edgeList, vertexList, Q, dummy_weight);
            Matrix3d S,R;
            #pragma omp parallel for
            for(int i=0;i<mesh.numTet;i++)  {
                polarHigham((mesh.tetMatrixInverse[i]*Q[i]).block(0,0,3,3), S, AR[i]);
                tetEnergy[i] = (S-AS[i]).squaredNorm();
            }
        }
    }
    // set new vertex position
    for(int i=0;i<numPts;i++){
        Mpts[i].x=mesh.Sol(i,0);
        Mpts[i].y=mesh.Sol(i,1);
        Mpts[i].z=mesh.Sol(i,2);
    }
    itGeo.setAllPositions(Mpts);
    
    // set vertex color according to ARAP energy
    if(visualiseEnergy){
        std::vector<double> ptsEnergy;
        makePtsWeightList(tetMode, numPts, mesh.tetList, faceList, edgeList, vertexList, tetEnergy, ptsEnergy);
        //double max_energy = *std::max_element(ptsEnergy.begin(), ptsEnergy.end());
        outputAttr(data, aEnergy, ptsEnergy);
        for(int i=0;i<numPts;i++){
            ptsEnergy[i] *= visualisationMultiplier;     //  or /= max_energy
        }
        visualise(data, outputGeom, ptsEnergy);
    }

//    MString es="Runtime timing: ";
//    double timing=(double)(clock()- clock_start)/CLOCKS_PER_SEC;
//    es += timing;
//    MGlobal::displayInfo(es);

    return MS::kSuccess;
}


// plugin (un)initialiser
MStatus nwayDeformerNode::initialize()
{
    MFnTypedAttribute tAttr;
    MFnNumericAttribute nAttr;
    MFnEnumAttribute eAttr;
    MFnMatrixAttribute mAttr;

    // this attr will be dirtied when ARAP recomputation is needed
    aARAP = nAttr.create( "arap", "arap", MFnNumericData::kBoolean, true );
    nAttr.setStorable(false);
    nAttr.setKeyable(false);
    nAttr.setHidden(true);
    addAttribute( aARAP );
    
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

    aInitRotation = nAttr.create("initRotation", "ir", MFnNumericData::kDouble);
    addAttribute(aInitRotation);
    attributeAffects( aInitRotation, outputGeom );

    aVisualiseEnergy = nAttr.create( "visualiseEnergy", "ve", MFnNumericData::kBoolean, false );
    nAttr.setStorable(true);
    addAttribute( aVisualiseEnergy );
    attributeAffects( aVisualiseEnergy, outputGeom );

    aAreaWeighted = nAttr.create( "areaWeighted", "aw", MFnNumericData::kBoolean, false );
    nAttr.setStorable(true);
    addAttribute( aAreaWeighted );
    attributeAffects( aAreaWeighted, outputGeom );
    attributeAffects( aAreaWeighted, aARAP );

    aVisualisationMultiplier = nAttr.create("visualisationMultiplier", "vmp", MFnNumericData::kDouble, 1.0);
    nAttr.setStorable(true);
	addAttribute( aVisualisationMultiplier );
	attributeAffects( aVisualisationMultiplier, outputGeom );
    
    aBlendMode = eAttr.create( "blendMode", "bm", BM_SRL );
    eAttr.addField( "expSO+expSym", BM_SRL );
    eAttr.addField( "logmatrix3", BM_LOG3 );
    eAttr.addField( "quat+linear", BM_SQL );
    eAttr.addField( "expSO+linear", BM_SlRL );
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
    attributeAffects( aTetMode, aARAP );
    
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
