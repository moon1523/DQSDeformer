#ifndef POLYGON_HH
#define POLYGON_HH

#include <string>
#include <map>
#include <vector>
#include "dual_quat_cu.hpp"

using namespace std;
using namespace Tbx;

class Polygon
{
public:
    Polygon();
    Polygon(string fileName);
    void ReadOBJ(string fileName);
    void PrintOBJ(string fileName);
    vector<Point3> GetVerts() {return verts;}
    void SetVerts(vector<Point3> _verts) {verts = _verts;}
    string GetName() {return objName.substr(0, objName.size()-4);}

private:
    string objName;
    bool isValid;
    vector<Point3> verts;
    stringstream faceBuff;
};

#endif // POLYGON_HH
