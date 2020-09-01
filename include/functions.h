#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include "dual_quat_cu.hpp"
#include "G4TessellatedSolid.hh"
#include "G4TriangularFacet.hh"
#include "G4SystemOfUnits.hh"
#include "G4ThreeVector.hh"
using namespace std;
using namespace Tbx;

//Specification
pair<G4TessellatedSolid*, G4TessellatedSolid*> ReadObj(string name, vector<Point3> &verts, map<int,vector<vector<int>>> &faces){
    //initialization
    verts.clear(); faces.clear();
    //read file
    ifstream ifs(name);
    if(!ifs.is_open()) {cerr<<"There is no "+name<<endl; exit(1);}

    cout<<"Reading "+name<<flush;
    G4ThreeVector point;
    int id, a, b, c;
    vector<G4ThreeVector> vertVec;
    string dump;
    G4TessellatedSolid* tess0 = new G4TessellatedSolid();
    G4TessellatedSolid* tess1 = new G4TessellatedSolid();
    while(getline(ifs, dump)){
        stringstream ss(dump);
        ss>>dump;
        if(dump=="v"){
            ss>>point;
            vertVec.push_back(point);
        }
        else if(dump=="g"){
            ss>>id;
            faces[id] = {};
        }
        else if(dump=="f"){
            ss>>a>>b>>c;
            faces[id].push_back({a, b, c});
            if(id==0) tess0->AddFacet(new G4TriangularFacet(vertVec[a-1],vertVec[b-1],vertVec[c-1],ABSOLUTE));
            else if(id==1) tess1->AddFacet(new G4TriangularFacet(vertVec[a-1],vertVec[b-1],vertVec[c-1],ABSOLUTE));
        }
    }ifs.close();
    tess0->SetSolidClosed(true);
    tess1->SetSolidClosed(true);
    for(auto v:vertVec) verts.push_back(Point3(double(v.getX()), double(v.getY()), double(v.getZ())));
    cout<<" - done"<<endl;
    return make_pair(tess0, tess1);
}


void WriteEle(string name, vector<pair<vector<int>, double>> eleVol, vector<Point3> nodes, vector<vector<double>> weights){
    ofstream ofs(name);
    ofs<<eleVol.size()<<"  4  1"<<endl;
    int count(0);
    for(auto ele:eleVol){
        if(nodes[ele.first[0]].z+nodes[ele.first[1]].z+nodes[ele.first[2]].z+nodes[ele.first[3]].z>8) continue;
        int n(0);
        double avg(0.);
        for(int i=0;i<4;i++){
            if(weights[ele.first[i]][0]==0) continue;
            if(weights[ele.first[i]][0]==1) continue;
            avg += weights[ele.first[i]][0];
            n++;
        }
        if(n>0) avg/=n;
        else avg = weights[ele.first[0]][0];
        ofs<<count<<" "<<ele.first[0]<<" "<<ele.first[1]<<" "<<ele.first[2]<<" "<<ele.first[3]<<" "<<int(avg*100)<<endl;count++;
    }
    ofs.close();
}

void SetWeights(const vector<Point3> verts, vector<vector<double>> &weights){
    cout<<"Setting weights"<<flush;
    weights.clear();
/*    vector<Point3> verts0, verts1;
    vector<vector<int>> faces0, faces1;
    ReadPly("0.ply", verts0, faces0);
    ReadPly("1.ply", verts1, faces1);
    G4TessellatedSolid tess0, tess1;
    vector<G4ThreeVector> verts0_g4, verts1_g4;
    for(auto vert:verts0) verts0_g4.push_back(G4ThreeVector(vert.x, vert.y, vert.z));
    for(auto vert:verts1) verts1_g4.push_back(G4ThreeVector(vert.x, vert.y, vert.z));

    for(auto face:faces0){
        tess0.AddFacet(new G4TriangularFacet(verts0_g4[face[0]],
                                             verts0_g4[face[1]],
                                             verts0_g4[face[2]],
                                             ABSOLUTE));
    }tess0.SetSolidClosed(true);
    for(auto face:faces1){
        tess1.AddFacet(new G4TriangularFacet(verts1_g4[face[0]],
                                             verts1_g4[face[1]],
                                             verts1_g4[face[2]],
                                             ABSOLUTE));
    }tess1.SetSolidClosed(true);
    for(auto vert:verts){
        if(vert.y>5) {weights.push_back({0,1}); continue;}
        if(vert.y<-5) {weights.push_back({1,0}); continue;}
        G4ThreeVector point(vert.x, vert.y, vert.z);
        double d0_in = tess0.DistanceToIn(point);
        double d0_out = tess0.DistanceToOut(point);
        double d0 = (d0_in>d0_out)? d0_in:d0_out;
        double d1_in = tess1.DistanceToIn(point);
        double d1_out = tess1.DistanceToOut(point);
        double d1 = (d1_in>d1_out)? d1_in:d1_out;
//        cout<<vert<<scientific<<": "<<d0<<" "<<d1;getchar();
        d0 *= d0; d1 *= d1;
        double r = d0/(d0+d1);
        cout<<point.getY()<<"\t"<<r<<endl;
 //        cout<<r*r<<endl; getchar();

        weights.push_back({1-r, r});
    }*/

    double margin(5);
    double constant = -0.5/(margin*margin);
    for(auto vert:verts){
        if(vert.y>margin) {weights.push_back({0,1}); continue;}
        if(vert.y<-margin) {weights.push_back({1,0}); continue;}
        if(vert.y>0){
           double w = constant*vert.y*(vert.y-margin*2) + 0.5;
           weights.push_back({1-w, w});
        }else{
            double w = constant*vert.y*(vert.y+margin*2) + 0.5;
            weights.push_back({w, 1-w});
        }
    }
    cout<<"-done"<<endl;
}


double DistanceTo(vector<Point3> verts, Point3 point)
{
    double distance(3);
    for(auto v:verts){
        Vec3 vec = point-v;
        if(abs(vec.y)>distance) continue;
        if(abs(vec.x)>distance) continue;
        if(abs(vec.z)>distance) continue;
        double dist2=vec.norm_squared();
        if(dist2<distance*distance) distance = sqrt(dist2);
    }
    return distance;
}

void SetWeights2(const vector<Point3> verts, vector<vector<double>> &weights, map<int,vector<vector<int>>> &faces, pair<G4TessellatedSolid*, G4TessellatedSolid*>tessPair)
{
    cout<<"Setting weights"<<flush;
    weights.clear();

    //quad
    double margin(5);
    double constant = -0.5/(margin*margin);
    for(auto vert:verts){
        if(vert.y>margin) {weights.push_back({0,1}); continue;}
        if(vert.y<-margin) {weights.push_back({1,0}); continue;}
        if(vert.y>0){
           double w = constant*vert.y*(vert.y-margin*2) + 0.5;
           weights.push_back({1-w, w});
        }else{
            double w = constant*vert.y*(vert.y+margin*2) + 0.5;
            weights.push_back({w, 1-w});
        }
    }

    //vert ext.
    vector<int> verts0, verts1;
    for(auto face:faces[0]){verts0.push_back(face[0]);verts0.push_back(face[1]);verts0.push_back(face[2]);}
    for(auto face:faces[1]){verts1.push_back(face[0]);verts1.push_back(face[1]);verts1.push_back(face[2]);}
    sort(verts0.begin(), verts0.end());
    verts0.erase(unique(verts0.begin(), verts0.end()), verts0.end());
    sort(verts1.begin(), verts1.end());
    verts1.erase(unique(verts1.begin(), verts1.end()), verts1.end());
    for(auto &id:verts0)id--;
    for(auto &id:verts1)id--;

    //bone rigid
    for(auto id:verts0)weights[id] = {1,0};
    for(auto id:verts1)weights[id] = {0,1};

    vector<Point3> upperVec, lowerVec;
    for(auto id:verts0)upperVec.push_back(verts[id]);
    for(auto id:verts1)lowerVec.push_back(verts[id]);

    //gradation
    set<int> rigidNodes(verts0.begin(), verts0.end());
    rigidNodes.insert(verts1.begin(), verts1.end());
    for(size_t n=0;n<verts.size();n++){
        if(rigidNodes.find(n)!=rigidNodes.end()) continue;
        double upDist = DistanceTo(upperVec, verts[n]);
        double lowDist = DistanceTo(lowerVec, verts[n]);
        if(upDist<2 && lowDist<2){
            double ratio = 1./(upDist+lowDist);
            weights[n] = {lowDist*ratio, upDist*ratio};
        }
        else if(upDist<2){
            double alpha = (2-upDist)*0.5;
            weights[n][0] *= (1.-alpha);
            weights[n][1] *= (1.-alpha);
            weights[n][0] += alpha;
        }
        else if(lowDist<2){
            double alpha = (2-lowDist)*0.5;
            weights[n][0] *= (1.-alpha);
            weights[n][1] *= (1.-alpha);
            weights[n][1] += alpha;
        }
    }


    cout<<"-done"<<endl;
}

void SetWeights3(const vector<Point3> nodes, vector<vector<double>> &weights, map<int, vector<int>> attributes)
{
    cout<<"Setting weights"<<flush;
    weights.clear();

    //quad
    double margin(5);
    double constant = -0.5/(margin*margin);
    for(auto vert:nodes){
        if(vert.y>margin) {weights.push_back({0,1}); continue;}
        if(vert.y<-margin) {weights.push_back({1,0}); continue;}
        if(vert.y>0){
           double w = constant*vert.y*(vert.y-margin*2) + 0.5;
           weights.push_back({1-w, w});
        }else{
            double w = constant*vert.y*(vert.y+margin*2) + 0.5;
            weights.push_back({w, 1-w});
        }
    }

    //bone rigid
    int upperBone(2), lowerBone(3);
    for(auto id:attributes[upperBone])weights[id] = {1,0};
    for(auto id:attributes[lowerBone])weights[id] = {0,1};

    vector<Point3> upperVec, lowerVec;
    for(auto id:attributes[upperBone])upperVec.push_back(nodes[id]);
    for(auto id:attributes[lowerBone])lowerVec.push_back(nodes[id]);

    //gradation
    set<int> rigidNodes(attributes[upperBone].begin(), attributes[upperBone].end());
    rigidNodes.insert(attributes[lowerBone].begin(), attributes[lowerBone].end());
    for(size_t n=0;n<nodes.size();n++){
        if(rigidNodes.find(n)!=rigidNodes.end()) continue;
        double upDist = DistanceTo(upperVec, nodes[n]);
        double lowDist = DistanceTo(lowerVec, nodes[n]);
        if(upDist<2 && lowDist<2){
            double ratio = 1./(upDist+lowDist);
            weights[n] = {lowDist*ratio, upDist*ratio};
        }
        else if(upDist<2){
            double alpha = (2-upDist)*0.5;
            weights[n][0] *= (1.-alpha);
            weights[n][1] *= (1.-alpha);
            weights[n][0] += alpha;
        }
        else if(lowDist<2){
            double alpha = (2-lowDist)*0.5;
            weights[n][0] *= (1.-alpha);
            weights[n][1] *= (1.-alpha);
            weights[n][1] += alpha;
        }
    }

    cout<<"-done"<<endl;
}

std::vector<std::map<int, double>> ReadWeights(string fileName){
    ifstream ifs(fileName);
    string dump;
    stringstream ss;
    std::map<int, double> wMap;
    std::vector<std::map<int, double>> weights;
    while(getline(ifs, dump)){
        ss.clear(); wMap.clear();
        ss.str(dump);
        int id; double w;
        ss>>dump;
        while(ss>>id){
            ss>>w;
            wMap[id] = w;
        }
        weights.push_back(wMap);
    }ifs.close();
    return weights;
}

void ReadPly(string name, vector<Point3> &verts, vector<vector<int>> &faces)
{
    //initialization
    verts.clear(); faces.clear();

    //read file
    ifstream ifs(name);
    if(!ifs.is_open()) {cerr<<"There is no "<<name<<endl; exit(1);}

    Point3 point;
    string dump;
    int nb_verts(0);
    int nb_faces(0);
    while(1){
        getline(ifs, dump);
        if(dump.find("element vertex")!=string::npos){
            stringstream ss(dump);
            ss>>dump>>dump>>nb_verts;
        }
        else if(dump.find("element face")!=string::npos){
            stringstream ss(dump);
            ss>>dump>>dump>>nb_faces;
        }
        else if(dump.find("end_header")!=string::npos) break;
    }
    cout<<"Start reading "+name+"...(v:"<<nb_verts<<"/f:"<<nb_faces<<")"<<flush;
    for(int i=0;i<nb_verts;i++){
        ifs>>point;
        verts.push_back(point);
    }
    for(int i=0;i<nb_faces;i++){
        int a, b, c;
        ifs>>dump>>a>>b>>c;
        vector<int> face = {a, b, c};
        faces.push_back(face);
    }
    ifs.close();
    cout<<"-done"<<endl;
}

void PrintPly(string name, vector<Point3> verts, vector<vector<int>> faces)
{
    ofstream ofs(name);
    ofs<<"ply"<<endl;
    ofs<<"format ascii 1.0"<<endl;
    ofs<<"comment Exported by RapidForm"<<endl;
    ofs<<"element vertex "<<verts.size()<<endl;
    ofs<<"property double x"<<endl;
    ofs<<"property double y"<<endl;
    ofs<<"property double z"<<endl;
    ofs<<"element face "<<faces.size()<<endl;
    ofs<<"property list uchar int vertex_index"<<endl;
    ofs<<"end_header"<<endl;
    for(Point3 vert:verts) ofs<<vert<<endl;
    for(vector<int> face:faces) ofs<<"3 "<<face[0]<<" "<<face[1]<<" "<<face[2]<<endl;
    ofs.close();
    cout<<"Printed "+name<<endl;
}

void ReadNode(string name, vector<Point3> &nodes)
{
    nodes.clear();
    ifstream ifsNode(name);
    if(!ifsNode.is_open()) {cerr<<"There is no "<<name<<endl; exit(1);}
    int nb_node;
    string dump;
    getline(ifsNode, dump);
    stringstream ss(dump);
    ss>>nb_node;
    Point3 point;
    for(int i=0;i<nb_node;i++){
        ifsNode>>dump>>point;
        nodes.push_back(point);
    }ifsNode.close();
}

void ReadEle(string name, map<int, vector<int>> &attributes)
{
    attributes.clear();
    ifstream ifsEle(name);
    if(!ifsEle.is_open()){cerr<<"There is no "<<name<<endl; exit(1);}
    int nb_ele;
    string dump;
    getline(ifsEle, dump);
    stringstream ss(dump);
    ss>>nb_ele;
    int a, b, c, d, id(-100);
    for(int i=0;i<nb_ele;i++){
        getline(ifsEle, dump);
        stringstream ss1(dump);
        ss1>>dump>>a>>b>>c>>d>>id;
        if(attributes.find(id)==attributes.end()) attributes[id]={a, b, c, d};
        else {
            attributes[id].push_back(a);
            attributes[id].push_back(b);
            attributes[id].push_back(c);
            attributes[id].push_back(d);
        }
    }ifsEle.close();
    for(auto &att:attributes){
        sort(att.second.begin(), att.second.end());
        att.second.erase(unique(att.second.begin(), att.second.end()), att.second.end());
    }
}

void ReadEle2(string name, vector<Point3> nodes, vector<pair<vector<int>, double>> &eleVol, map<int, vector<int>> &attributes)
{
    attributes.clear();
    eleVol.clear();
    ifstream ifsEle(name);
    if(!ifsEle.is_open()){cerr<<"There is no "<<name<<endl; exit(1);}
    int nb_ele;
    string dump;
    getline(ifsEle, dump);
    stringstream ss(dump);
    ss>>nb_ele;
    int a, b, c, d, id(-100);
    for(int i=0;i<nb_ele;i++){
        getline(ifsEle, dump);
        stringstream ss1(dump);
        ss1>>dump>>a>>b>>c>>d>>id;
        Vec3 aa = nodes[a]-nodes[d];
        Vec3 bb = nodes[b]-nodes[d];
        Vec3 cc = nodes[c]-nodes[d];
        double vol = (aa.cross(bb)).dot(cc);
        vector<int> elements = {a, b, c, d};
        eleVol.push_back(make_pair(elements, vol));
        if(attributes.find(id)==attributes.end()) attributes[id]={a, b, c, d};
        else {
            attributes[id].push_back(a);
            attributes[id].push_back(b);
            attributes[id].push_back(c);
            attributes[id].push_back(d);
        }
    }ifsEle.close();
    for(auto &att:attributes){
        sort(att.second.begin(), att.second.end());
        att.second.erase(unique(att.second.begin(), att.second.end()), att.second.end());
    }
}

void DetectTangled(const vector<Point3> nodes, vector<pair<vector<int>, double>> eleVol)
{
    double volDiff(0);
    double tangledVol(0);
    int count(0);
    for(auto ele:eleVol){
        Vec3 aa = nodes[ele.first[0]] - nodes[ele.first[3]];
        Vec3 bb = nodes[ele.first[1]] - nodes[ele.first[3]];
        Vec3 cc = nodes[ele.first[2]] - nodes[ele.first[3]];
        double vol = (aa.cross(bb)).dot(cc);
        if(ele.second*vol>0) volDiff+=abs(vol)-abs(ele.second);
        else {count++; tangledVol+=vol/6.;}
    }
    cout<<"   Tangled: "<<count<<", "<<tangledVol<<"   "<<"Volume Diff: "<<volDiff/6.<<endl;;
}

void PrintNode(string name, const vector<Point3> nodes)
{
    ofstream ofsNode(name+".node");
    ofsNode<<nodes.size()<<"  3  0  0"<<endl;
    for(size_t i=0;i<nodes.size();i++){
        ofsNode<<i<<" "<<nodes[i]<<endl;
    }ofsNode.close();
}


void PrintObj(string name, const vector<Point3> verts, const map<int,vector<vector<int>>> faces)
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

void dual_quat_deformer(const std::vector<Point3>& in_verts,
                        std::vector<Point3>& out_verts,
                        Dual_quat_cu& dual_quat,
                        const std::vector< std::vector<double> >& weights)
{
    out_verts.clear();
    for(unsigned v = 0; v < in_verts.size(); ++v)
    {
        double w0 = weights[v][0];
        double w1 = weights[v][1];
        if( dual_quat.rotation().dot( Dual_quat_cu::identity().rotation() ) < 0.f )
            w1 *= -1.f;
        Dual_quat_cu dq_blend = Dual_quat_cu::identity()*w0 + dual_quat*w1;

        // Compute animated position
        Point3 vi = dq_blend.transform( in_verts[v] );
        out_verts.push_back(vi);
    }
}

void dual_quat_deformer(const std::vector<Point3>& in_verts,
                        std::vector<Point3>& out_verts,
                        const std::vector<Dual_quat_cu>& dual_quat,
                        const std::vector<std::map<int, double>>& weights)
{
    out_verts.clear();
    for(unsigned v = 0; v < in_verts.size(); ++v)
    {
        Dual_quat_cu dq_blend;
        bool first(true);
        Quat_cu q0;

        for(auto w:weights[v]){
            if(first){
                dq_blend = dual_quat[w.first] * w.second;
                q0 = dual_quat[w.first].rotation();
                first = false;
                continue;
            }
            if( dual_quat[w.first].rotation().dot( q0 ) < 0.f )
                dq_blend = dq_blend + dual_quat[w.first] * (-w.second);
            else dq_blend = dq_blend + dual_quat[w.first] * w.second;
        }

        // Compute animated position
        Point3 vi = dq_blend.transform( in_verts[v] );
        out_verts.push_back(vi);
    }
}
