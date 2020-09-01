#include "Polygon.hh"
#include <fstream>

Polygon::Polygon():isValid(false)
{}

Polygon::Polygon(string fileName):isValid(false)
{if(fileName.empty()) return;
 ReadOBJ(fileName);}

void Polygon::ReadOBJ(string fileName){
    isValid = true;

    if (verts.size())
        cerr<<"WARNING: " + fileName + " data overwrites the "+objName+" data."<<endl;

    //initialization
    verts.clear(); faceBuff.clear();
    objName = fileName;

    //read file
    ifstream ifs(fileName);
    if(!ifs.is_open()) {cerr<<"There is no "+fileName<<endl; exit(1);}

    double x, y, z;
    string dump;
    while(getline(ifs, dump)){
        stringstream ss(dump);
        ss>>dump;
        if(dump=="v"){
            ss>>x>>y>>z;
            verts.push_back(Point3(x, y, z));
        }
        else if(dump=="g"){
            string shellName = ss.str();
            shellName = shellName.substr(2, shellName.size()-2);
            faceBuff<<endl<<ss.str()<<endl;
            faceBuff<<"usemtl "<<shellName<<endl;
            faceBuff<<"s"<<endl<<endl;
        }
        else if(dump=="f"){
            faceBuff<<ss.str()<<endl;
        }
    }ifs.close();
}

void Polygon::PrintOBJ(string fileName){
    cout<<"Writing "+fileName<<endl;
    ofstream ofs(fileName);
    ofs<<"mtllib "+objName.substr(0, objName.size()-3) + "mtl"<<endl<<endl;
    for(auto v:verts)
        ofs<<"v "<<v<<endl;
    ofs<<faceBuff.str();
}
