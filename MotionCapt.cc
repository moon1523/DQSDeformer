#include <iostream>
#include <functions.h>
#include <opencv2/viz.hpp>
#include <opencv2/viz/widget_accessor.hpp>

#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
#include <vtkActor.h>
#include "G4RotationMatrix.hh"
#include "G4SystemOfUnits.hh"
#include "G4Timer.hh"
#include "KinectData.hh"
//#include "Polygon.hh"
#include "bodytracking.hh"
#include "bodydeformer.hh"

class WPoly : public cv::viz::Widget3D
{
public:
    WPoly(){}
    WPoly(const string & fileName);
    vtkSmartPointer<vtkPolyData> GetPolyData() {return reader->GetOutput();}
    void WritePolyData(string name) {
        writer->SetFileName(name.c_str());
        writer->SetInputConnection(reader->GetOutputPort());
        writer->Write();
    }
    void Initialize(const string & fileName);
    void UpdateVerts(vector<Point3> _p);
private:
    vtkPLYReader* reader;
    vtkPLYWriter* writer;
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter;
    vtkSmartPointer<vtkActor> actor;
};

/**
 * @function TriangleWidget::TriangleWidget
 * @brief Constructor
 */
WPoly::WPoly(const string & fileName)
{
    Initialize(fileName);
}
void WPoly::Initialize(const string &fileName){
    transformFilter =
            vtkSmartPointer<vtkTransformPolyDataFilter>::New();

    reader = vtkPLYReader::New();
    writer = vtkPLYWriter::New();
    reader->SetFileName (fileName.c_str());
    reader->Update ();
    vtkSmartPointer<vtkPolyData> polyData = reader->GetOutput ();
    // Create mapper and actor
    mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    // Store this actor in the widget in order that visualizer can access it
    cv::viz::WidgetAccessor::setProp(*this, actor);
}
void WPoly::UpdateVerts(vector<Point3> _p){
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->ShallowCopy(reader->GetOutput()->GetPoints());
    for (int i = 0; i < points->GetNumberOfPoints(); i++)
     {
        //double pp[3]; points->GetPoint(i,pp); pp[2]+=1;
        double p[3] = {_p[i].x,_p[i].y,_p[i].z};
        points->SetPoint(i, p);
     }
}

bool quitChk(false); bool writePLY(false);
void  KeyboardViz3d(const cv::viz::KeyboardEvent &w, void *t)
{
   switch (w.code) {
   case 'q':
       quitChk=true;
       break;
   case 's':
       offViz=true;
       break;
   case 'r':
       writePLY=true;
       break;
    }
}

void PrintUsage(){
    cout<<"Usage:./MotionCapt [option] [phantomFile] [weight] [jointCenters]"<<endl;
    exit(1);
}

void GeneratePLYandJointInfo(string objFileName);
int main(int argc, char** argv){
    //Arguments
    string plyF, plyWeight, jointF; //objF, objWeight, tetF, tetWeight,
    if(argc==1) PrintUsage();
    for(int i=1; i<argc;i++){
        if     (string(argv[i])=="-ply") {
            plyF = argv[++i]; plyWeight = argv[++i]; jointF = argv[++i];
        }
        else if     (string(argv[i])=="-conv") {
//            GeneratePLYandJointInfo(string(argv[++i]));
            SkelTest(string(argv[++i]));
        }
//        else if(string(argv[i])=="-tet") {
//            tetF = argv[++i]; tetWeight = argv[++i];
//        }
        else PrintUsage();
    }

    G4Timer timer;

    //Read ply file
    WPoly poly;
    vector<map<int, double>> plyWeights;
    vector<Point3> plyVerts;
    if(plyF.size()){
        cout<<"Reading PLY data.."<<flush; timer.Start();
        poly.Initialize(plyF);
        vtkSmartPointer<vtkPoints> points = poly.GetPolyData()->GetPoints();
        for(int i=0;i<points->GetNumberOfPoints();i++){
            double* p = points->GetPoint(i);
            plyVerts.push_back(Point3(p[0],p[1],p[2]));
        }
        timer.Stop(); cout<<timer.GetRealElapsed()<<" ("<<plyVerts.size()<<")"<<endl;

        cout<<"Reading weight data.."<<flush; timer.Start();
        plyWeights = ReadWeights(plyWeight);
        timer.Stop(); cout<<timer.GetRealElapsed()<<" ("<<plyWeights.size()<<")"<<endl;
    }

    //Read Joint info.
    map<int, Vec3> jCen = ReadJointF(jointF);
    map<int, int>  parentJ = ReadKinectJoint("KinectJoint");

    //init. viewer
    cv::viz::Viz3d myWindow("PLY viewer");
    myWindow.showWidget("model PLY", poly);
    cv::Vec3f cam_pos(-300,0,-500), cam_focal_point(0,0,200), cam_y_dir(0,1,0);
    cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    myWindow.setViewerPose(cam_pose);
    myWindow.registerKeyboardCallback(KeyboardViz3d,&myWindow);
    myWindow.spinOnce(1, true);


    // Start body tracking
    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_5;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "Get depth camera calibration failed!");
    int depthWidth = sensorCalibration.depth_camera_calibration.resolution_width;
    int depthHeight = sensorCalibration.depth_camera_calibration.resolution_height;

    // Create Body Tracker
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    //tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_CPU;
    tracker_config.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
    VERIFY(k4abt_tracker_create(&sensorCalibration, tracker_config, &tracker), "Body tracker initialization failed!");
    // Initialize the 3d window controller
    Window3dWrapper window3d;
    window3d.Create("3D Visualization", sensorCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);

    int calibFrame(0);
    map<int, Vec3> uprightOrien;
    map<int, double> calibLengths;
    while (s_isRunning)
    {
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0

        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // timeout_in_ms is set to 0. Return immediately no matter whether the sensorCapture is successfully added
            // to the queue or not.
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);

            // Release the sensor capture once it is no longer needed.
            k4a_capture_release(sensorCapture);

            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
            break;
        }

        // Pop Result from Body Tracker
        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            /************* Successfully get a body tracking result, process the result here ***************/
            VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);

            if(calibSwitch && k4abt_frame_get_num_bodies(bodyFrame)){
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, 0, &body.skeleton), "Get skeleton from body frame failed!");
                k4a_quaternion_t q = body.skeleton.joints[0].orientation;
                Quat_cu root(q.wxyz.w,q.wxyz.x,q.wxyz.y,q.wxyz.z);
                root.normalize();
                Quat_cu rootInv = root.conjugate();
                for(auto j:jCen){
                    if(j.first==0) continue;
                    k4a_float3_t t = body.skeleton.joints[j.first].position;
                    k4a_float3_t t0 = body.skeleton.joints[parentJ[j.first]].position;
                    Vec3 orientation = Vec3(t.xyz.x,t.xyz.y,t.xyz.z) - Vec3(t0.xyz.x,t0.xyz.y,t0.xyz.z);
                    uprightOrien[j.first] += rootInv.rotate(orientation);
                    calibLengths[j.first] += orientation.norm();
                }
                cout<<"\rCalibration frame #"<<calibFrame++<<flush;
            }
            //Release the bodyFrame
            k4abt_frame_release(bodyFrame);
        }

        window3d.SetLayout3d(s_layoutMode);
        window3d.SetJointFrameVisualization(s_visualizeJointFrame);
        window3d.Render();
    }

    //calib.
    map<int, double> lengths;
    for(auto j:jCen){
        if(j.first==0) continue;
        lengths[j.first] = (j.second-jCen[parentJ[j.first]]).norm();
    }

    if(calibFrame>0){
    Mat3 calibRot;
    calibRot.c=-1; calibRot.d=-1; calibRot.h=-1;
    for(auto &ort:uprightOrien){
        ort.second /= calibFrame*10;
        ort.second = calibRot*ort.second;
    }

    for(auto &l:calibLengths){
        l.second /= calibFrame*10;
    }
    map<int, Vec3> calibCen;
    auto calibTF = CalculateScalingVec(jCen, calibCen, parentJ, calibLengths);
    vector<Point3> scaledVertices;
    ScaleToActor(plyVerts,scaledVertices,calibTF,plyWeights);
    //plyVerts = scaledVertices;
    poly.UpdateVerts(scaledVertices);
    myWindow.spinOnce();

    lengths=calibLengths;
    plyVerts = scaledVertices;
    jCen = calibCen;
}
    //orientation alignment for limbs
    map<int, Vec3> desiredOrt;
    vector<int> groups={12,13,5,6,22,23,18,19};
    for(int id:groups) desiredOrt[id] = Vec3(0,1,0);
    desiredOrt[4] = Vec3(1,0,0);
    desiredOrt[11] = Vec3(-1,0,0);
    map<int, Transfo> alignTf;
    for(auto j:jCen){
        if(desiredOrt.find(j.first)==desiredOrt.end()) alignTf[j.first] = Transfo::identity();
        else {
            Vec3 v = jCen[j.first+1]-j.second;
            alignTf[j.first] = GetRotMatrix(v,desiredOrt[j.first]);
        }
    }

    //preprocessing for KINECT data
    map<int,Transfo> preTransfo;
    Transfo tfX(Mat3::rotate(Vec3(1,0,0), 90*deg));
    Transfo tfX1(Mat3::rotate(Vec3(-1,0,0), 90*deg));
    Transfo tfX2(Mat3::rotate(Vec3(1,0,0), 180*deg));
    Transfo tfZ(Mat3::rotate(Vec3(0,0,1), 90*deg));
    Transfo tfZ1(Mat3::rotate(Vec3(0,0,-1), 90*deg));
    //cout<<rotMat;getchar();
    Transfo tf1=tfX*tfZ;
    groups = {0,1,2,3,18,19,20,26};
    for(int id:groups) preTransfo[id]=tf1*alignTf[id];
    groups = {22,23,24};
    tf1=tfX1*tfZ1;
    for(int id:groups) preTransfo[id]=tf1*alignTf[id];
    groups = {5,6};
    tf1=tfX*tfZ1;
    for(int id:groups) preTransfo[id]=tf1*alignTf[id];
    groups = {12,13};
    tf1=tfX1*tfZ;
    for(int id:groups) preTransfo[id]=tf1*alignTf[id];
    groups = {11};
    for(int id:groups) preTransfo[id]=tfX1*alignTf[id];
    groups = {4};
    for(int id:groups) preTransfo[id]=tfX*alignTf[id];
    groups = {7};
    for(int id:groups) preTransfo[id]=tfX2*tfZ1*alignTf[id];
    groups = {14};
    for(int id:groups) preTransfo[id]=tfZ*alignTf[id];


/*
    ifstream ifsSph("sphere.ply");
    string str_temp; int vNum, fNum;
    while(ifsSph>>str_temp){
        if(str_temp=="vertex") ifsSph>>vNum;
        else if(str_temp=="face") ifsSph>>fNum;
        else if(str_temp=="end_header") break;
    }
    vector<Vec3> vVec;
    vector<vector<int>> fVec;
    for(int i=0;i<vNum;i++){
        double x,y,z;
        ifsSph>>x>>y>>z;
        vVec.push_back(Vec3(x,y,z));
    }
    for(int i=0;i<fNum;i++){
        int x,y,z,id;
        ifsSph>>id>>x>>y>>z;
        fVec.push_back({x,y,z});
    }ifsSph.close();

    map<int,Vec3> centers;
    centers[0] = Vec3();
    for(auto ort:uprightOrien){
        centers[ort.first] = centers[parentJ[ort.first]]+ort.second + jCen[0];
    }
    double torsoX, torsoY;
    torsoY = ((jCen[5].y+jCen[12].y)-(jCen[22].y+jCen[18].y))/
             ((centers[5].y+centers[12].y)-(centers[22].y+centers[18].y));
    torsoX = (jCen[5].x-jCen[12].x)/(centers[5].x-centers[12].x);

    vector<int> scaleJ = {1,2,3,26,11,12,4,5};
    for(int id:scaleJ) centers[id].y*=torsoY;
    scaleJ = {4,5,11,12};
    for(int id:scaleJ) centers[id].x*=torsoX;

    ofstream ofs("centers.obj");
    int count(0);
    for(auto cc:centers){
        for(auto v:vVec){
            Vec3 vv = v+ cc.second;
            ofs<<"v "<<vv.x<<" "<<vv.y<<" "<<vv.z<<endl;
        }
        ofs<<"g "<<cc.first<<endl;
        int offSet = vVec.size()*count;
        for(auto f:fVec){
            ofs<<"f "<<f[0]+offSet+1<<" "<<f[1]+offSet+1<<" "<<f[2]+offSet+1<<endl;
        }
        count++;
    }ofs.close();*/

    s_isRunning = true;
    vector<Point3> newPolVerts;

    //while (s_isRunning)
    bool test(true);
    int frameNo(0);
    for(;;)
    {
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0

        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // timeout_in_ms is set to 0. Return immediately no matter whether the sensorCapture is successfully added
            // to the queue or not.
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);

            // Release the sensor capture once it is no longer needed.
            k4a_capture_release(sensorCapture);

            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
            break;
        }

        // Pop Result from Body Tracker
        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            if(offViz) VisualizeResult(bodyFrame, window3d, depthWidth, depthHeight);
            else if(k4abt_frame_get_num_bodies(bodyFrame)){
                k4abt_body_t body;
                k4abt_frame_get_body_skeleton(bodyFrame, 0, &body.skeleton);
                map<int, Vec3> newCenters;
                map<int, Vec3> jointOrt;
                for(auto j:jCen){
                    k4a_float3_t t = body.skeleton.joints[j.first].position;
                    newCenters[j.first] = Vec3(t.xyz.x,t.xyz.y,t.xyz.z)*0.1;
                    if(j.first==0) continue;
                    jointOrt[j.first]=newCenters[j.first]-newCenters[parentJ[j.first]];
                    jointOrt[j.first].normalize();
                }
                for(auto &cen:newCenters){
                    if(cen.first==0) continue;
                    cen.second = newCenters[parentJ[cen.first]] + jointOrt[cen.first]*lengths[cen.first];
                }
                map<int, Dual_quat_cu> duQautVec;

                for(auto j:jCen){    
                    k4a_quaternion_t q = body.skeleton.joints[j.first].orientation;
                    Quat_cu q1(q.wxyz.w,q.wxyz.x,q.wxyz.y,q.wxyz.z);
                    q1.normalize();
                    //Transfo tf = Transfo::translate(newCenters[j.first])*(Transfo(q1.to_matrix3())*preTransfo[j.first])*Transfo::translate(-j.second);
                    Transfo tf = Transfo::translate(newCenters[j.first])*(Transfo(q1.to_matrix3())*preTransfo[j.first])*Transfo::translate(-j.second);
                    duQautVec[j.first] =
                     Dual_quat_cu(tf.normalized());
                   // Dual_quat_cu(Quat_cu(q.wxyz.w,q.wxyz.x,q.wxyz.y,q.wxyz.z), j.second);
                }

                dual_quat_deformer(plyVerts,newPolVerts,duQautVec,plyWeights);
                poly.UpdateVerts(newPolVerts);
                if(test){
                    for(int i=0;i<1000;i++)
                        myWindow.spinOnce();
                    test=false;
                }
                myWindow.spinOnce(100);
                if(writePLY){
                    poly.WritePolyData(to_string(frameNo)+".ply");
                    cout<<endl<<"printed "+to_string(frameNo)+".ply"<<endl;
                    writePLY = false;
                }
                cout<<"\rframe #"<<frameNo++<<flush;
            }

            //Release the bodyFrame
            k4abt_frame_release(bodyFrame);
        }

        if(offViz){
            window3d.SetLayout3d(s_layoutMode);
            window3d.SetJointFrameVisualization(s_visualizeJointFrame);
            window3d.Render();
        }
    }
    /*
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
        dual_quat_deformer(plyVerts,newPolVerts,dual_quat,plyWeights);
        timer.Stop(); cout<<timer.GetRealElapsed()<<endl;
        poly.UpdateVerts(newPolVerts);
        myWindow.spinOnce(1, true);
        while(1){
            myWindow.spinOnce(1);
            if(quitChk){
                quitChk = false;
                break;
            }
        }
    }

*/
    std::cout << "Finished body tracking processing!" << std::endl;

    window3d.Delete();
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    return 0;
}
void GeneratePLYandJointInfo(string objFileName){
    ifstream ifs(objFileName);
    if(!ifs.is_open()) {cerr<<"There is no "+objFileName<<endl; exit(1);}

    string dump;
    vector<G4ThreeVector> vertices, plyV;
    vector<vector<int>> faces;
    G4ThreeVector aPoint;
    int a, b, c;
    bool printPLY(false);
    map<int, G4ThreeVector> jointCenters;
    while(getline(ifs, dump)){
        stringstream ss(dump);
        ss>>dump;
        if(dump=="v"){
            ss>>aPoint;
            vertices.push_back(aPoint);
        }
        else if(dump=="g"){
            string shellName;
            ss>>shellName;
            if(shellName=="skin"){
                printPLY = true;
                plyV = vertices;
                vertices.clear();
            }
            else{
                printPLY = false;
                G4ThreeVector center;
                for(auto v:vertices) center += v;
                center /= vertices.size();
                vertices.clear();
                jointCenters[atoi(shellName.c_str())] = center;
            }
        }
        else if(dump=="f"){
            if(!printPLY) continue;
            ss>>a>>b>>c;
            faces.push_back({a-1,b-1,c-1});
        }
    }ifs.close();
    cout<<"Imported "+objFileName<<endl;
    ofstream ofs(objFileName.substr(0,objFileName.size()-3)+"ply");
    ofs<<"ply"<<endl;
    ofs<<"format ascii 1.0"<<endl;
    ofs<<"comment Exported by RapidForm"<<endl;
    ofs<<"element vertex "<<plyV.size()<<endl;
    ofs<<"property float x"<<endl;
    ofs<<"property float y"<<endl;
    ofs<<"property float z"<<endl;
    ofs<<"element face "<<faces.size()<<endl;
    ofs<<"property list uchar int vertex_index"<<endl;
    ofs<<"end_header"<<endl;
    for(G4ThreeVector v:plyV)
        ofs<<v.getX()<<" "<<v.getY()<<" "<<v.getZ()<<endl;
    for(auto f:faces)
        ofs<<"3 "<<f[0]<<" "<<f[1]<<" "<<f[2]<<endl;
    ofs.close();
    cout<<objFileName.substr(0,objFileName.size()-3)+"ply was generated"<<endl;

    ofstream ofs2(objFileName.substr(0,objFileName.size()-3)+"joint");
    for(auto joint:jointCenters)
        ofs2<<joint.first<<" "<<joint.second.getX()<<" "<<joint.second.getY()<<" "<<joint.second.getZ()<<endl;
    ofs2.close();
    cout<<objFileName.substr(0,objFileName.size()-3)+"joint was generated"<<endl;
    exit(2);
}
