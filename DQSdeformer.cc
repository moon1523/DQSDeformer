#include <iostream>
#include <functions.h>
#include "G4RotationMatrix.hh"
#include "G4SystemOfUnits.hh"
#include "G4Timer.hh"
#include "KinectData.hh"
#include "Polygon.hh"
int main(int argc, char** argv){
    //KinectData kinect;
    //kinect.ReadKinectData("Posture.txt");
    //kinect.Deform();
    string objF, objWeight, tetF, tetWeight;
    for(int i=1; i<argc;i++){
        if     (string(argv[i])=="-pol") {
            objF = argv[++i]; objWeight = argv[++i];
        }
        else if(string(argv[i])=="-tet") {
            tetF = argv[++i]; tetWeight = argv[++i];
        }
    }

    G4Timer timer;
    //read polygon data
    Polygon polygon;
    vector<map<int, double>> polWeights;
    if(objF.size()){
        cout<<"Reading polygon data.."<<flush; timer.Start();
        polygon = Polygon(objF);
        timer.Stop(); cout<<timer.GetRealElapsed()<<" ("<<polygon.GetVerts().size()<<")"<<endl;

        cout<<"Reading weight data.."<<flush; timer.Start();
        polWeights = ReadWeights(objWeight);
        timer.Stop(); cout<<timer.GetRealElapsed()<<" ("<<polWeights.size()<<")"<<endl;
    }
    vector<Point3> polVertices = polygon.GetVerts();

    //initialize joint info. & dual quaternions
    vector<Vec3> jointCenter; vector<int> jointParent;
    ifstream ifs("jointInfo");
    if(!ifs.is_open()) {cerr<<"There is no jointInfo"<<endl; return 1;}
    cout<<"Reading jointInfo file.."<<flush; timer.Start();
    string dump; int parentID; double x, y, z;
    getline(ifs, dump);
    while(ifs>>dump>>dump>>parentID>>x>>y>>z){
        jointCenter.push_back(Vec3(x, y, z));
        jointParent.push_back(parentID);
    }ifs.close();
    timer.Stop(); cout<<timer.GetRealElapsed()<<" ("<<jointCenter.size()<<")"<<endl;
    vector<Dual_quat_cu> dual_quat; dual_quat.reserve(jointCenter.size());

    //start deformation
    for(int n=0;;n++){
        for(int i=0;i<jointCenter.size();i++)
            dual_quat[i] = Dual_quat_cu::identity();
        double x,y,z;
        int id; double angle;
        cout<<"id: "; cin>>id; if(id<0) break;
        cin.clear();cin.ignore(256, '\n');
        std::string axisStr;
        std::cout<<"axis: ";
        std::getline(std::cin, axisStr);
        stringstream buff(axisStr); buff>>x>>y>>z;
        cout<<"degree: "; cin>>angle; angle *= deg;

        Transfo tf; tf.rotate(jointCenter[jointParent[id]], Vec3(x,y,z), angle);
        dual_quat[id] = Dual_quat_cu(tf);
        for(int i=0;i<jointCenter.size();i++){
            dual_quat[i] = dual_quat[jointParent[i]];
        }
        vector<Point3> newPolVerts;
        dual_quat_deformer(polVertices,newPolVerts,dual_quat,polWeights);
        polygon.SetVerts(newPolVerts);
        polygon.PrintOBJ(polygon.GetName()+to_string(n)+".obj");
    }


    // vector<vector<double>> weights = ReadWeights(weightsF);

/*

    vector<Point3> nodes, outNodes;
    map<int, vector<int>> attributes;
    vector<vector<double>> weightsN;
    vector<pair<vector<int>, double>> eleVol;
    ReadNode("RightArm1.2.node", nodes);
    ReadEle2("RightArm1.2.ele", nodes, eleVol, attributes);
    SetWeights3(nodes, weightsN, attributes);
    WriteEle("weights.ele", eleVol, nodes, weightsN);

   // vector<vector<int>> faces;
   // vector<vector<double>> weights;
   // ReadPly("rightArm5.ply", verts, faces);
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
    }*/
    return 0;
}
