/**
 * @file weightController.cpp
 * @brief weightController plugin for Maya
 * @section LICENSE The MIT License
 * @section requirements:  Eigen 3:  http://eigen.tuxfamily.org/
 * @section Autodesk Maya: http://www.autodesk.com/products/autodesk-maya/overview
 * @section Based on the paper by Kai Hormann
 * @version 0.10
 * @date  1/Nov/2013
 * @author Shizuo KAJI
 */


#include "StdAfx.h"
#include "weightController.h"

#define EPSILON 0.000001

using namespace Eigen;

MTypeId weightControllerNode::id( 0x00000400 );
MString weightControllerNode::nodeName( "weightController" );
MObject weightControllerNode::aOutputs;
MObject weightControllerNode::aLocator;
MObject weightControllerNode::aVertices;

void* weightControllerNode::creator() { return new weightControllerNode; }
 
MStatus weightControllerNode::compute( const MPlug& plug, MDataBlock& dataBlock){
	MObject thisNode = thisMObject();
    MStatus status;
    if(plug != aOutputs || ! plug.isElement()){    return MS::kSuccess; }
    float3 &uu = dataBlock.inputValue(aLocator).asFloat3();
    Vector3f loc(uu[0],uu[1],uu[2]);
    MArrayDataHandle hVertices=dataBlock.inputArrayValue(aVertices);
    int num=hVertices.elementCount();
    if(num==0){return MS::kSuccess; }
        // compute the weight
    std::vector< Vector3f > v(num);
    std::vector< float > r(num);
    for(int i=0;i<num;i++){
        float3 &uu=hVertices.inputValue().asFloat3();
        Vector3f u(uu[0],uu[1],uu[2]);
        v[i] = u-loc;
        r[i] = v[i].norm();
        if(i<num-1){ hVertices.next(); }
    }
    std::vector<float> w(num, 0.0f), A(num), D(num);
    for(int i=0;i<num;i++){
        int j=(i+1)% num;
        A[i]=(v[i].cross(v[j])).norm();
        D[i]=v[i].dot(v[j]);
    }
    bool flag=true;
    // check if it is on the boundary
    for(int i=0;i<num;i++){
        if(r[i]<EPSILON){     // at a vertex
            w[i]=1.0;
            flag=false;
            break;
        }else if(abs(A[i])<EPSILON && D[i] < 0){   // on an edge
            int j=(i+1) % num;
            w[i]=r[j];
            w[j]=r[i];
            flag=false;
            break;
        }
    }
    // if it is not on the boundary
    if(flag){
        for(int i=0;i<num;i++){
            int k=(i-1+num)% num;
            if(fabs(A[k])>EPSILON)
                w[i]+=(r[k]-D[k]/r[i])/A[k];
            if(fabs(A[i])>EPSILON)
                w[i]+=(r[(i+1)% num]-D[i]/r[i])/A[i];
        }
    }
    float sum=0.0;
    for(unsigned int i=0;i<num;i++)
        sum += w[i];
    for(unsigned int i=0;i<num;i++)
        w[i] /= sum;
// writing output
    MPlug wPlug(thisNode, aOutputs);
    MDataHandle wHandle = wPlug.constructHandle(dataBlock);
    MArrayDataHandle arrayHandle(wHandle, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MArrayDataBuilder arrayBuilder = arrayHandle.builder(&status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    for(unsigned int i = 0; i < num; i++) {
        MDataHandle handle = arrayBuilder.addElement(i,&status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        handle.set(w[i]);
    }
    status = arrayHandle.set(arrayBuilder);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    wPlug.setValue(wHandle);
    dataBlock.setClean(plug);
    
    return MS::kSuccess;
}


// init of plugin
MStatus weightControllerNode::initialize(){
    MFnNumericAttribute nAttr;

    aOutputs = nAttr.create("outputs", "out", MFnNumericData::kFloat, 0.0);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);
    addAttribute(aOutputs);

    aLocator = nAttr.create("locator", "lc", MFnNumericData::k3Float);
    addAttribute(aLocator);
    attributeAffects( aLocator, aOutputs );

    aVertices = nAttr.create("vertices", "v", MFnNumericData::k3Float);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);
    addAttribute(aVertices);
    attributeAffects( aVertices, aOutputs);
 
    return MS::kSuccess;
}
 
MStatus initializePlugin( MObject obj ){
    MStatus status;
    MFnPlugin plugin( obj, "Shizuo KAJI", "0.1", "CREST");
 
    status = plugin.registerNode(weightControllerNode::nodeName , weightControllerNode::id, weightControllerNode::creator, weightControllerNode::initialize );
    CHECK_MSTATUS_AND_RETURN_IT( status );
 
    return status;
}
 
MStatus uninitializePlugin( MObject obj ){
    MStatus   status;
    MFnPlugin plugin( obj );
 
    status = plugin.deregisterNode( weightControllerNode::id );
    CHECK_MSTATUS_AND_RETURN_IT( status );
 
    return status;
}
