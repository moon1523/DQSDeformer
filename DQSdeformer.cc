#include <iostream>
#include <functions.h>
#include "G4RotationMatrix.hh"
#include "G4SystemOfUnits.hh"
#include "G4Timer.hh"
#include "KinectData.hh"
int main(int argc, char** argv){
    KinectData kinect;
    kinect.ReadKinectData("Posture.txt");
    kinect.Deform();

    G4Timer timer;
    vector<Point3> verts, outVerts;
    map<int, vector<vector<int>>> faces;
    auto tessPair = ReadObj("RightArm1.obj", verts, faces);
    vector<vector<double>> weights;
    SetWeights2(verts, weights, faces, tessPair);

    vector<Point3> nodes, outNodes;
    map<int, vector<int>> attributes;
    vector<vector<double>> weightsN;
    vector<pair<vector<int>, double>> eleVol;
    ReadNode("RightArm1.2.node", nodes);
    ReadEle2("RightArm1.2.ele", nodes, eleVol, attributes);
    SetWeights3(nodes, weightsN, attributes);
    WriteEle("weights.ele", eleVol, nodes, weightsN);

    //vector<vector<int>> faces;
   // vector<vector<double>> weights;
   //ReadPly("rightArm5.ply", verts, faces);
   // system("tetgen -q rightArm.ply");


    G4RotationMatrix rot = G4RotationMatrix::IDENTITY;

    for(int i=0;i<15;i++){
        cout<<"angle-->"<<i*10+10<<endl;
        rot.rotateX(-10*degree);
        Transfo tf;
        tf.set_mat3(Mat3(rot.col1().getX(), rot.col2().getX(), rot.col3().getX(),
                         rot.col1().getY(), rot.col2().getY(), rot.col3().getY(),
                         rot.col1().getZ(), rot.col2().getZ(), rot.col3().getZ()));
        Dual_quat_cu dq(tf);
        timer.Start();
        dual_quat_deformer(verts, outVerts, dq, weights);
        timer.Stop();
        cout<<"time(PM) --> "<<timer.GetRealElapsed()<<endl;

        timer.Start();
        dual_quat_deformer(nodes, outNodes, dq, weightsN);
        timer.Stop();
        cout<<"time(TM) --> "<<timer.GetRealElapsed()<<endl;
        DetectTangled(outNodes, eleVol);

        string fileN = "boneTest0_" + to_string(i);//+".ply";
        PrintObj(fileN, outVerts, faces);
        //PrintPly(fileN, outVerts, faces);
        PrintNode(fileN, outNodes);
    }
    return 0;
}
