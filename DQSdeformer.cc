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
   // getline(ifs, dump);
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
        stringstream ss(axisStr); ss>>x>>y>>z;
        cout<<"degree: "; cin>>angle; angle *= deg;
        Transfo tf = Transfo::rotate(jointCenter[id], Vec3(x,y,z), angle);
        dual_quat[id] = Dual_quat_cu(tf);
        for(int i=id+1;i<jointCenter.size();i++){
            if(i==jointCenter.size()) break;
            dual_quat[i] = dual_quat[jointParent[i]];
        }
        vector<Point3> newPolVerts;
        cout<<"Start deformation.."<<flush; timer.Start();
        dual_quat_deformer(polVertices,newPolVerts,dual_quat,polWeights);
        timer.Stop(); cout<<timer.GetRealElapsed()<<endl;
        polygon.SetVerts(newPolVerts);
        polygon.PrintOBJ(polygon.GetName()+to_string(n)+".obj");
    }

    return 0;
}
