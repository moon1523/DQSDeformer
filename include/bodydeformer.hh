#ifndef BODYDEFORMER_HH
#define BODYDEFORMER_HH

#include <map>
#include <string>
#include "functions.h"

using namespace std;
class BodyDeformer
{
public:
    BodyDeformer();
    void SetVerts(vector<Point3> _verts) {verts=_verts;}
    void SetWeights(vector<map<int, double>> _weights){weights=_weights;}
    void SetJointCenter(vector<Vec3> _jCenters){jCenters=_jCenters;}

private:
    vector<Point3> verts;
    vector<map<int, double>> weights;
    vector<Vec3> jCenters;
};

#endif // BODYDEFORMER_HH
