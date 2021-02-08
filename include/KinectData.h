#ifndef KINECTDATA_HH
#define KINECTDATA_HH

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
//#include "G4ThreeVector.hh"
//#include "dual_quat_cu.hpp"
#include <map>
#include <utility>
//#include "G4SystemOfUnits.hh"

using namespace std;
typedef std::pair<Tbx::Point3, Tbx::Quat_cu> JointData;

class KinectData
{
public:
    KinectData();
    ~KinectData() {}

    void ReadKinectData(string fileName);
    void AssignVertices(vector<G4ThreeVector> vertices, int id);
    void Deform();

private:
    //Temporary to test on skeleton
    void ReadObj(string objFile);
    vector<Tbx::Point3> DeformAFrame(map<int, JointData> aFrame);
    map<int, Tbx::Vec3> CalculateNewCenters(map<int, JointData> aFrame);

private:
    vector<pair<Tbx::Point3, int>>   verticesVec;
    map<int, vector<vector<int>>>    faceMap;
    map<int, double>                 jointLength;
    map<int, Tbx::Vec3>            centers;
    //KinectData
    vector<map<int, JointData>> postureData;
    map<int, Tbx::Quat_cu>      defaultRot;

    //tool
    void GetRotQuat(Tbx::Quat_cu &rot, Tbx::Point3 start, Tbx::Point3 end, Tbx::Vec3 toward) {
        Tbx::Vec3 vec = end - start;
        Tbx::Vec3 axis = vec.cross(toward).normalized();
        double theta = acos(vec.dot(toward) / (vec.norm()*toward.norm()));
         rot=Tbx::Quat_cu(axis, theta);
    }
    void PrintObj(string name, const vector<pair<Tbx::Point3, int>> verts, const map<int,vector<vector<int>>> faces)
    {
        ofstream ofs(name+".obj");
        for(auto v:verts)ofs<<"v "<<v.first<<endl;
        ofs<<endl;
        for(auto shell:faces){
            ofs<<"g "<<shell.first<<endl<<endl;
            for(auto face:shell.second)
                ofs<<"f "<<face[0]<<" "<<face[1]<<" "<<face[2]<<endl;
            ofs<<endl;
        }ofs.close();
    }
    void PrintObj(string name, const vector<Tbx::Point3> verts, const map<int,vector<vector<int>>> faces)
    {
        ofstream ofs(name+".obj");
        for(auto v:verts)ofs<<"v "<<v<<endl;
        ofs<<endl;
        for(auto shell:faces){
            ofs<<"g "<<shell.first<<endl<<endl;
            for(auto face:shell.second)
                ofs<<"f "<<face[0]<<" "<<face[1]<<" "<<face[2]<<endl;
            ofs<<endl;
        }ofs.close();
    }

};

#endif // KINECTDATA_HH
