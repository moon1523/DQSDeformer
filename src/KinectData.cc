#include "KinectData.hh"
#include <sstream>
#include "G4SystemOfUnits.hh"

KinectData::KinectData()
{
   ReadObj("skeleton.obj");
}

void KinectData::ReadKinectData(string fileName){
    ifstream ifs(fileName);
    if(!ifs.is_open()) {cerr<<"There is no "+fileName<<endl; exit(1);}
    cout<<"Reading "+fileName<<flush;
    while(1){
        if(ifs.eof()) break;
        map<int, JointData> frameData;
        string dump;
        for(int i=0;i<32;i++){
             getline(ifs, dump);
            if(dump.size()<5) break;
            stringstream ss(dump);
            int id;
            double x, y, z, w;
            Tbx::Point3 point;
            ss>>id>>point>>x>>y>>z>>w>>dump;
            if(id!=i) {
                cerr<<"DATAFILE ERROR: check joint IDs, "<<i<<" is missing"<<endl;
                exit(2);
            }
            frameData[id] = JointData(point*mm, Tbx::Quat_cu(x, y, z, w));
        }
        getline(ifs, dump);
        postureData.push_back(frameData);
    }
    ifs.close();
    cout<<"..."<<postureData.size()<<" frames"<<endl;
}

void KinectData::ReadObj(string objFile){
    ifstream ifs(objFile);
    if(!ifs.is_open()) {cerr<<"There is no "+objFile<<endl; exit(1);}

    cout<<"Reading "+objFile<<flush;
    Tbx::Point3 point;
    int id, a, b, c;
    vector<Tbx::Point3> vertVec;
    string dump;

    while(getline(ifs, dump)){
        stringstream ss(dump);
        ss>>dump;
        if(dump=="v"){
            ss>>point;
            vertVec.push_back(point*cm);
        }
        else if(dump=="g"){
            ss>>id;
            if(id<100){
                for(auto v:vertVec) verticesVec.push_back(make_pair(v, id));
                faceMap[id] = {};
            }else{
                for(auto v:vertVec) verticesVec.push_back(make_pair(v, id-100));
                Tbx::Vec3 center;
                for(auto v:vertVec) center += v.to_vec3();
                center /= vertVec.size();
                centers[id-100]=center;
            }
            vertVec.clear();
        }
        else if(dump=="f"){
           // if(id<0) continue;
            ss>>a>>b>>c;
            faceMap[id].push_back({a, b, c});
        }
    }ifs.close();

    //calibration (default Rot)

    for(auto c:centers) defaultRot[c.first] = Tbx::Quat_cu();
    auto xAxis = Tbx::Vec3(1,0,0);
    auto zAxis = Tbx::Vec3(0,0,1);
    //torso
    defaultRot[0]  = Tbx::Quat_cu(xAxis, 90*deg) * Tbx::Quat_cu(zAxis, 90*deg);
    defaultRot[1]  = defaultRot[0];
    defaultRot[2]  = defaultRot[0];
    defaultRot[3]  = defaultRot[0];
    defaultRot[26] = defaultRot[0];
    //right arm
    defaultRot[4] = Tbx::Quat_cu(xAxis, 90*deg);
    defaultRot[5] = Tbx::Quat_cu(xAxis, 90*deg)*Tbx::Quat_cu(zAxis, -90*deg);
    defaultRot[6] = defaultRot[5];
    defaultRot[7] = Tbx::Quat_cu(xAxis, 180*deg)*Tbx::Quat_cu(zAxis, -90*deg);
    //left arm
    defaultRot[11] = Tbx::Quat_cu(xAxis, -90*deg);
    defaultRot[12] = Tbx::Quat_cu(xAxis, -90*deg)*Tbx::Quat_cu(zAxis, 90*deg);
    defaultRot[13] = defaultRot[12];
    defaultRot[14] = Tbx::Quat_cu(zAxis, 90*deg);
    //right leg
    defaultRot[18] = defaultRot[0];
    defaultRot[19] = defaultRot[0];
    defaultRot[20] = defaultRot[0];
    //left leg
    defaultRot[22] = Tbx::Quat_cu(xAxis, -90*deg)*Tbx::Quat_cu(zAxis, -90*deg);
    defaultRot[23] = defaultRot[22];
    defaultRot[24] = defaultRot[22];

    //joint center adjust.
    for(auto &v:verticesVec)
        v.first = defaultRot[v.second].rotate(v.first+(-centers[v.second]));

    map<int, Tbx::Vec3> centerTemp;
    centerTemp[1] = defaultRot[0].rotate(centers[1]-centers[0]);
    centerTemp[2] = defaultRot[1].rotate(centers[2]-centers[1]);
    centerTemp[3] = defaultRot[2].rotate(centers[3]-centers[2]);
    centerTemp[26] = defaultRot[3].rotate(centers[26]-centers[3]);
    centerTemp[4] = defaultRot[2].rotate(centers[4]-centers[2]);
    centerTemp[5] = defaultRot[4].rotate(centers[5]-centers[4]);
    centerTemp[6] = defaultRot[5].rotate(centers[6]-centers[5]);
    centerTemp[7] = defaultRot[6].rotate(centers[7]-centers[6]);
    centerTemp[11] = defaultRot[2].rotate(centers[11]-centers[2]);
    centerTemp[12] = defaultRot[11].rotate(centers[12]-centers[11]);
    centerTemp[13] = defaultRot[12].rotate(centers[13]-centers[12]);
    centerTemp[14] = defaultRot[13].rotate(centers[14]-centers[13]);
    centerTemp[18] = defaultRot[0].rotate(centers[18]-centers[0]);
    centerTemp[19] = defaultRot[18].rotate(centers[19]-centers[18]);
    centerTemp[20] = defaultRot[19].rotate(centers[20]-centers[19]);
    centerTemp[22] = defaultRot[0].rotate(centers[22]-centers[0]);
    centerTemp[23] = defaultRot[22].rotate(centers[23]-centers[22]);
    centerTemp[24] = defaultRot[23].rotate(centers[24]-centers[23]);
    centers.clear();
    centers = centerTemp;

    /*
    vector<Tbx::Point3> points;
    points.push_back(Tbx::Point3(-0.5, -0.5, -0.5 ));
    points.push_back(Tbx::Point3( -0.5, -0.5, 0.5 ));
    points.push_back(Tbx::Point3( -0.5, 0.5, 0.5 ));
    points.push_back(Tbx::Point3( -0.5, 0.5, -0.5 ));
    points.push_back(Tbx::Point3( 30, -0.5, -0.5 ));
    points.push_back(Tbx::Point3( 30, 0.5, -0.5 ));
    points.push_back(Tbx::Point3( 30, 0.5, 0.5 ));
    points.push_back(Tbx::Point3( 30, -0.5, 0.5 ));

    points.push_back(Tbx::Point3( -0.5, -0.5, -0.5 ));
    points.push_back(Tbx::Point3( -0.5, -0.5, 0.5 ));
    points.push_back(Tbx::Point3( -0.5, 20, 0.5 ));
    points.push_back(Tbx::Point3( -0.5, 20, -0.5 ));
    points.push_back(Tbx::Point3( 0.5, -0.5, -0.5 ));
    points.push_back(Tbx::Point3( 0.5, 20, -0.5 ));
    points.push_back(Tbx::Point3( 0.5, 20, 0.5 ));
    points.push_back(Tbx::Point3( 0.5, -0.5, 0.5 ));

    points.push_back(Tbx::Point3( -0.5, -0.5, -0.5 ));
    points.push_back(Tbx::Point3( -0.5, -0.5, 10 ));
    points.push_back(Tbx::Point3( -0.5, 0.5, 10 ));
    points.push_back(Tbx::Point3( -0.5, 0.5, -0.5 ));
    points.push_back(Tbx::Point3( 0.5, -0.5, -0.5 ));
    points.push_back(Tbx::Point3( 0.5, 0.5, -0.5 ));
    points.push_back(Tbx::Point3( 0.5, 0.5, 10 ));
    points.push_back(Tbx::Point3( 0.5, -0.5, 10 ));

    for(auto rot:defaultRot){
        std::stringstream ss;
        ss << "ply" << std::endl;
        ss << "format ascii 1.0" << std::endl;
        ss << "comment Exported by RapidForm" << std::endl;
        ss << "element vertex 24" << std::endl;
        ss << "property double x" << std::endl;
        ss << "property double y" << std::endl;
        ss << "property double z" << std::endl;
        ss << "element face 18" << std::endl;
        ss << "property list uchar int vertex_index" << std::endl;
        ss << "end_header" << std::endl;
        for(auto p:points)
            ss<<Tbx::Quat_cu(rot.second.to_matrix3().inverse()).rotate(p)<<endl;
            //ss<<rot.second.to_matrix3().inverse()*p<<endl;
        ss << "4 0 1 2 3" << std::endl;
        ss << "4 4 5 6 7" << std::endl;
        ss << "4 0 3 5 4" << std::endl;
        ss << "4 3 2 6 5" << std::endl;
        ss << "4 2 1 7 6" << std::endl;
        ss << "4 1 0 4 7" << std::endl;

        ss << "4 8 9 10 11" << std::endl;
        ss << "4 12 13 14 15" << std::endl;
        ss << "4 8 11 13 12" << std::endl;
        ss << "4 11 10 14 13" << std::endl;
        ss << "4 10 9 15 14" << std::endl;
        ss << "4 9 8 12 15" << std::endl;

        ss << "4 16 17 18 19" << std::endl;
        ss << "4 20 21 22 23" << std::endl;
        ss << "4 16 19 21 20" << std::endl;
        ss << "4 19 18 22 21" << std::endl;
        ss << "4 18 17 23 22" << std::endl;
        ss << "4 17 16 20 23" << std::endl;
        ofstream ofs(to_string(rot.first)+".ply");
        ofs<<ss.str();
        ofs.close();
    }*/


    /*defaultRot[0].to_matrix3().inverse()
    //for(auto rot:defaultRot) cout<<rot.first<<":"<<endl<<rot.second.to_matrix3()<<endl;

    for(auto &v:verticesVec)
        v.first = centers[v.second] + defaultRot[v.second].rotate(v.first);

    PrintObj("test.obj",verticesVec, faceMap);

*/
    cout<<"...done"<<endl;
}

void KinectData::Deform(){
    int count(0);
    for(auto aData:postureData){
        PrintObj(to_string(count++),DeformAFrame(aData),faceMap);
    }
}

vector<Tbx::Point3> KinectData::DeformAFrame(map<int, JointData> aFrame){
    vector<Tbx::Point3> deformed;
    map<int, Tbx::Vec3> newCenters = CalculateNewCenters(aFrame);

    for(auto vert:verticesVec){
        deformed.push_back(aFrame[vert.second].second.rotate(vert.first) + newCenters[vert.second]);
    }
    return deformed;
}

map<int, Tbx::Vec3> KinectData::CalculateNewCenters(map<int, JointData> aFrame){
    map<int, Tbx::Vec3> newCenters;
    newCenters[0] = aFrame[0].first;
    newCenters[1] = aFrame[0].second.rotate(centers[1])+newCenters[0];
    newCenters[2] = aFrame[1].second.rotate(centers[2])+newCenters[1];
    newCenters[3] = aFrame[2].second.rotate(centers[3])+newCenters[2];
    newCenters[26] = aFrame[3].second.rotate(centers[26])+newCenters[3];
    newCenters[4] = aFrame[2].second.rotate(centers[4])+newCenters[2];
    newCenters[5] = aFrame[4].second.rotate(centers[5])+newCenters[4];
    newCenters[6] = aFrame[5].second.rotate(centers[6])+newCenters[5];
    newCenters[7] = aFrame[6].second.rotate(centers[7])+newCenters[6];
    newCenters[11] = aFrame[2].second.rotate(centers[11])+newCenters[2];
    newCenters[12] = aFrame[11].second.rotate(centers[12])+newCenters[11];
    newCenters[13] = aFrame[12].second.rotate(centers[13])+newCenters[12];
    newCenters[14] = aFrame[13].second.rotate(centers[14])+newCenters[13];
    newCenters[18] = aFrame[0].second.rotate(centers[18])+newCenters[0];
    newCenters[19] = aFrame[18].second.rotate(centers[19])+newCenters[18];
    newCenters[20] = aFrame[19].second.rotate(centers[20])+newCenters[19];
    newCenters[22] = aFrame[0].second.rotate(centers[22])+newCenters[0];
    newCenters[23] = aFrame[22].second.rotate(centers[23])+newCenters[22];
    newCenters[24] = aFrame[23].second.rotate(centers[24])+newCenters[23];
    return newCenters;
}
