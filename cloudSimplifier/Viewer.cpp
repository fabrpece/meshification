#include <functional>
#include <GL/glew.h>
#include <GL/glut.h>
#include <QKeyEvent>
#include <QMessageBox>
#include <QFileDialog>
#include <QInputDialog>
#include <chrono>
#include <sstream>
#include <fstream>
#include "Viewer.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <aruco/cameraparameters.h>
#include <aruco/markerdetector.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#define PAOLO_COORD_SYS

using namespace Eigen;
typedef std::chrono::high_resolution_clock myclock;

struct sort_PCL_points_x
{
    inline bool operator() (const pcl::PointXYZRGB p1, const pcl::PointXYZRGB p2)
    {
        return (p1.x < p2.x);
    }
};

struct sort_PCL_points_y
{
    inline bool operator() (const pcl::PointXYZRGB p1, const pcl::PointXYZRGB p2)
    {
        return (p1.y < p2.y);
    }
};

struct sort_PCL_points_z
{
    inline bool operator() (const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2)
    {
        return (p1.z < p2.z);
    }
};

static
void refineCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud, double leafSize = 0.025, int compressionType=0)
{
    //typedef std::chrono::high_resolution_myclock myclock;

    // Create the Voxel grid filtering object
    int old_pts = cloud.points.size();
    const auto t0_vx = myclock::now();
    std::cout << "Voxel grid filtering..." << std::flush;
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB> ());
    sor.setInputCloud(cloud.makeShared());
    sor.setLeafSize (leafSize, leafSize, leafSize);
    sor.filter(*cloud_filtered);
    cloud.points.clear();
    for(int i=0; i<cloud_filtered->points.size();++i)
      cloud.points.push_back(cloud_filtered->points[i]);
    cloud.height = cloud_filtered->height;
    cloud.width = cloud_filtered->width;
    cloud.is_dense = cloud_filtered->is_dense;
    const auto t1_vx = myclock::now();
    //std::cout << "done in " << (t1_vx-t0_vx).count()/1000 << "ms. Ratio: " << cloud.points.size() << "/" << old_pts <<std::endl;
    std::cout << "done in " << std::chrono::duration_cast<std::chrono::milliseconds>(t1_vx - t0_vx).count() << "ms. Ratio: " << cloud.points.size() << "/" << old_pts <<std::endl;


    //Downsample based on octree structure
    const auto t0_ds = myclock::now();
    std::cout << "Downsampling cloud..." << std::flush;
    pcl::io::compression_Profiles_e compressionProfile;

    switch(compressionType)
    {
        case 0:
            compressionProfile = pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR;
            break;
        case 1:
            compressionProfile = pcl::io::MED_RES_OFFLINE_COMPRESSION_WITH_COLOR;
            break;
        case 2:
            compressionProfile = pcl::io::HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR;
            break;
        default:
            compressionProfile = pcl::io::LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR;
            break;
    }

    // instantiate point cloud compression for encoding and decoding

    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> (compressionProfile);
    pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB> ();
    std::stringstream compressedData;
    // output pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut (new pcl::PointCloud<pcl::PointXYZRGB> ());
    // compress point cloud
    PointCloudEncoder->encodePointCloud (cloud.makeShared(), compressedData);
    // decompress point cloud
    PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
    old_pts = cloud.points.size();
    cloud.points.clear();
    for(int i=0; i<cloudOut->points.size();++i)
        cloud.points.push_back(cloudOut->points[i]);
    cloud.height = cloudOut->height;
    cloud.width = cloudOut->width;
    cloud.is_dense = cloudOut->is_dense;
    delete PointCloudEncoder;
    delete PointCloudDecoder;
    const auto t1_ds = myclock::now();
    //std::cout << "done in " << (t1_ds-t0_ds).count()/1000 << "ms. Ratio: " <<  cloud.points.size() << "/" << old_pts <<std::endl;
    std::cout << "done in " << std::chrono::duration_cast<std::chrono::milliseconds>(t1_ds - t0_ds).count() << "ms. Ratio: " << cloud.points.size() << "/" << old_pts <<std::endl;

    //Outlier removal
    const auto t0_or = myclock::now();
    std::cout << "Removing Outliers..." << std::flush;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_outlier (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor2;
    sor2.setInputCloud (cloud.makeShared());
    //What this means is that all points who have a distance larger than 1 standard deviation
    //of the mean distance to teh query point will be marked as outliers and removed
    sor2.setMeanK (50);
    sor2.setStddevMulThresh (1.0);
    sor2.filter(*cloud_filtered_outlier);
    old_pts = cloud.points.size();
    cloud.points.clear();
    for(int i=0; i<cloud_filtered_outlier->points.size();++i)
      cloud.points.push_back(cloud_filtered_outlier->points[i]);
    cloud.height = cloud_filtered_outlier->height;
    cloud.width = cloud_filtered_outlier->width;
    cloud.is_dense = cloud_filtered_outlier->is_dense;
    const auto t1_or = myclock::now();
    //std::cout << "done in " << (t1_or-t0_or).count()/1000 << "ms. Ratio: " <<  cloud.points.size() << "/" << old_pts <<std::endl;
    std::cout << "done in " << std::chrono::duration_cast<std::chrono::milliseconds>(t1_or - t0_or).count() << "ms. Ratio: " << cloud.points.size() << "/" << old_pts <<std::endl;
/*
    pcl::PLYWriter writer;
    writer.write("compressed.ply",cloud,true);
*/
}

static
void saveCloud(const char* ofilename, const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB> inverted_cloud;
    inverted_cloud.points.reserve(cloud.points.size());
    for(int i=0; i<cloud.points.size(); ++i)
    {
        pcl::PointXYZRGB or_pt = cloud.points[i];
        pcl::PointXYZRGB inverted;
        /*
        pt.x = -y;
        pt.y = z;
        pt.z = -x;
        */
        inverted.x = -or_pt.z;
        inverted.y = -or_pt.x;
        inverted.z = or_pt.y;
        inverted.rgb = or_pt.rgb;
        inverted_cloud.points.push_back(inverted);
    }
    pcl::io::savePLYFile(ofilename, inverted_cloud,true);
}

static
void triangulate(const pcl::PointCloud<pcl::PointXYZRGB>& in_cloud)
{
      //convert to XYZ only
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      cloud->points.clear();

      for(int i=0; i< in_cloud.points.size(); ++i)
      {
          pcl::PointXYZ pt;
          pt.x = in_cloud.points[i].x;
          pt.y = in_cloud.points[i].y;
          pt.z = in_cloud.points[i].z;
          cloud->points.push_back(pt);

      }
      // Normal estimation*
      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (cloud);
      n.setInputCloud (cloud);
      n.setSearchMethod (tree);
      n.setKSearch (20);
      n.compute (*normals);
      //* normals should not contain the point normals + surface curvatures

      // Concatenate the XYZ and normal fields*
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
      pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
      //* cloud_with_normals = cloud + normals

      // Create search tree*
      pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
      tree2->setInputCloud (cloud_with_normals);

      // Initialize objects
      pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
      pcl::PolygonMesh triangles;

      // Set the maximum distance between connected points (maximum edge length)
      gp3.setSearchRadius (0.25);

      // Set typical values for the parameters
      gp3.setMu (2.5);
      gp3.setMaximumNearestNeighbors (100);
      gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
      gp3.setMinimumAngle(M_PI/18); // 10 degrees
      gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
      gp3.setNormalConsistency(false);

      // Get result
      gp3.setInputCloud (cloud_with_normals);
      gp3.setSearchMethod (tree2);
      gp3.reconstruct (triangles);

      pcl::io::savePLYFile("test.ply",triangles);

      // Additional vertex information
      std::vector<int> parts = gp3.getPartIDs();
      std::vector<int> states = gp3.getPointStates();
}

static
bool readPly(const std::string& fname, pcl::PointCloud<pcl::PointXYZRGB>& cloud, double modelview[16])
{
    std::ifstream fileIn(fname, std::ios_base::binary);
    if (fileIn.is_open() == false) {
        std::cerr << "Cannot open file: " << fname << std::endl;
        return false;
    }

    std::string line;
    bool is_bin = false;
    int tot_points = 0;
    /*read header:
    ply
    format ascii 1.0
    element vertex 246283
    property float x
    property float y
    property float z
    property uchar red
    property uchar green
    property uchar blue
    end_header
    */
    bool has_alpha = false;
    std::getline(fileIn,line); //ply
    while (line != std::string("end_header")) {
        std::getline(fileIn,line);
        std::istringstream iss(line);
        std::string tmp;
        iss >> tmp;
        if (tmp == "comment") {
            iss >> tmp;
            if (tmp != "model_view")
                continue;
            for (int i = 0; i < 16; ++i)
                iss >> modelview[i];
        }
        else if (tmp == "format") { //check if this is the type
            iss >> tmp;
            if (tmp != "ascii")
                is_bin = true;
        }
        else if(tmp == "property")
        {
            iss >> tmp;
            if (tmp == "uchar")
            {
                iss >>tmp;
                if (tmp == "alpha")
                    has_alpha = true;
            }
        }
        else if (tmp == "element") {
            iss >> tmp;
            if (tmp != "vertex")
                continue;
            iss >> tot_points;
            cloud.points.reserve(tot_points);
        }
    }

    for (int i = 0; i < tot_points && fileIn; ++i) {
        float x, y, z;
        unsigned char r, g, b,a;
        if (is_bin == true) {
            fileIn.read((char*)&x, sizeof(float));
            fileIn.read((char*)&y, sizeof(float));
            fileIn.read((char*)&z, sizeof(float));
            fileIn.read((char*)&r, sizeof(unsigned char));
            fileIn.read((char*)&g, sizeof(unsigned char));
            fileIn.read((char*)&b, sizeof(unsigned char));
            if(has_alpha)
                fileIn.read((char*)&a, sizeof(unsigned char));
        } else {
            std::getline(fileIn,line);
            std::istringstream iss(line);
            iss >> x >> y >> z >> r >> g >> b;
        }

        pcl::PointXYZRGB pt(r, g, b);
        pt.x = -y;
        pt.y = z;
        pt.z = -x;
        cloud.points.push_back(pt);
    }

    fileIn.close();

    std::sort(cloud.points.begin(), cloud.points.end(),
    [](const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) {
        return (p1.z < p2.z);
    });

    return cloud.points.size() == tot_points;
}

static
std::string getFileExtension(const std::string& FileName)
{
    if(FileName.find_last_of(".") != std::string::npos)
        return FileName.substr(FileName.find_last_of(".")+1);
    return "";
}

static
bool calibrate(double* modelview,
               const std::string& calibration_image,
               const std::string& extrinsics_file,
               float marker_size = 0.289)
{

    aruco::CameraParameters cam_params;
    aruco::MarkerDetector marker_detector;
    cv::Mat frame = cv::imread(calibration_image.c_str());
    cam_params.readFromXMLFile(extrinsics_file.c_str());
    std::vector<aruco::Marker> markers;
    marker_detector.detect(frame, markers, cam_params, marker_size, false);
    Eigen::Map<Eigen::Matrix4d> mv_matrix(modelview);
    mv_matrix.setIdentity();
    bool marker_found = false;
    if(markers.size()>0)
    {
        for (auto& m : markers) {
            if(!marker_found)
                marker_found = true;
            m.draw(frame, cv::Scalar(255, 0, 0));
    #ifdef PAOLO_COORD_SYS
            cv::Mat rot_src = m.Rvec.clone(), rot;
            rot_src.at<float>(1, 0) *= -1.0f;
            rot_src.at<float>(2, 0) *= -1.0f;
            cv::Rodrigues(rot_src, rot);
            Eigen::Matrix3d r = Eigen::Map<Eigen::Matrix3f>((float*)rot.ptr()).cast<double>();
            Eigen::Vector3d t(-m.Tvec.at<float>(0, 0), m.Tvec.at<float>(1, 0), m.Tvec.at<float>(2, 0));
            Eigen::Affine3d a = Eigen::Affine3d::Identity();
            a.rotate(r).translate(t);
            Eigen::Map<Eigen::Matrix4d> mv_matrix(modelview);
            mv_matrix = a.matrix();
    #else
            m.glGetModelViewMatrix(modelview);
    #endif
        }
    }
    std::cout << "comment model_view";
    for(int i=0; i<16;++i)
        std::cout << " " << modelview[i];
    std::cout << std::endl;
    cv::imshow("Marker location", frame);
    cv::waitKey(30);
    return marker_found;
}

Viewer::Viewer(const int w, const int h, QWidget* parent):
    QGLViewer(parent),
    width(w), height(h),
    n_vertices(0),
    voxelGridLeafSize(0.025),
    pclCompressionType(0),
    toDownsample(false),
    drawSprite(false),
    doPointSmooth(true)
{
    setWindowTitle("Simplified Point Cloud Viewer");
    Eigen::Map<Eigen::Matrix4d> mv(modelview);
    mv.setIdentity();
    loadCloud();
    getAllOptions();
}

Viewer::~Viewer()
{
    glDeleteBuffers(2, vbo);
    glDeleteVertexArrays(1, vao);

    bool ok;
    QStringList options;
    options.push_back(QString("No"));
    options.push_back(QString("Yes"));
    QString save = QInputDialog::getItem(this, tr("Select if saving the cloud"),
                                                        tr("Save the cloud?"), options,
                                                        0, false, &ok);
    if(save == QString("Yes"))
    {
        QString fileName = QFileDialog::getSaveFileName(NULL,tr("Save PLY file"), ".", tr("*.ply"));
        if(!fileName.isEmpty())
        {
            QByteArray byteArray = fileName.toUtf8();
            const char* ofilename = byteArray.constData();
            saveCloud(ofilename, cloud);
        }
    }
}

void Viewer::getAllOptions()
{
    bool ok;
    QStringList options;
    options.push_back(QString("Do Not Downsample"));
    options.push_back(QString("Downsample"));
    QString downsample_selected = QInputDialog::getItem(this, tr("Select if downsampling the cloud"),
                                                        tr("Downsample the cloud?"), options,
                                                        0, false, &ok);

    if(downsample_selected == QString("Downsample"))
    {
        toDownsample = true;
        double voxel_grid_leaf_size = QInputDialog::getDouble(this, tr("Select voxel grid leaf size"),
                                                 tr("voxel grid leaf size"), 0.025,
                                                 0.0,2.0,4, &ok);
        if(ok)
            voxelGridLeafSize = voxel_grid_leaf_size;

        QStringList compressions;
        compressions.push_back(QString("Low"));
        compressions.push_back(QString("Medium"));
        compressions.push_back(QString("High"));
        QString selected = QInputDialog::getItem(this, tr("Select PCL compression type"),
                                                            tr("PCL compression type"), compressions,
                                                            0, false, &ok);
        if(ok)
        {
            for(int i=0; i<compressions.size(); ++i)
            {
                if(selected == compressions[i])
                    pclCompressionType = i;
            }
        }
    }

    options.clear();
    options.push_back(QString("Do Not Calibrate"));
    options.push_back(QString("Calibrate"));
    QString calib_selected = QInputDialog::getItem(this, tr("Select if calibrating the cloud"),
                                                        tr("Calibrate the cloud?"), options,
                                                        0, false, &ok);
    if(calib_selected == QString("Calibrate"))
    {
        QString fileName1 = QFileDialog::getOpenFileName(NULL,tr("Open YML file"), ".", tr("*.yml"));
        QByteArray byteArray1 = fileName1.toUtf8();
        std::string calib_file(byteArray1.constData());

        QString fileName2 = QFileDialog::getOpenFileName(NULL,tr("Open calibration image"), ".", tr("*.jpg *jpeg *png *bmp"));
        QByteArray byteArray2 = fileName2.toUtf8();
        std::string image_file(byteArray2.constData());

        double marker_size = QInputDialog::getDouble(this, tr("Select marker size"),
                                                 tr("Marker size"), 0.2890,
                                                 0.0,1.0,4, &ok);
        if(!ok)
            marker_size = marker_size;

        calibrate(modelview,
                  image_file,
                  calib_file,
                  (float)marker_size);
    }
}

void Viewer::init()
{

    setKeyDescription(Qt::Key_F, "Toogles FPS");
    setKeyDescription(Qt::Key_W, "Toggles wireframe");
    setKeyDescription(Qt::Key_T, "Toggles textures");
    setKeyDescription(Qt::Key_S, "Save cloud");
    setKeyDescription(Qt::Key_P, "Triangulate and save cloud");
    setKeyDescription(Qt::Key_O, "Toggles point sprites");

    glewInit();
    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_CULL_FACE);
    glFrontFace(GL_CW);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat ambient[4] = {0.2f, 0.2f, 0.2f, 1.0f};
    GLfloat diffuse[4] = {0.6f, 0.6f, 0.6f, 1.0f};
    GLfloat specular[4] = {0.0f, 0.0f, 0.0f, 1.0f};
    GLfloat position[4] = {0.0f, 4.0f, -5.0f};
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_POSITION, position);
    glDisable(GL_LIGHTING);
    glEnable(GL_TEXTURE_2D);
    setSceneCenter(qglviewer::Vec(0, 0, 4));
    setSceneRadius(5);
    camera()->fitSphere(qglviewer::Vec(0, 0, 4), 5);
    camera()->setFOVToFitScene();
    setAnimationPeriod(1);
    startAnimation();

    const auto ret = ::glewInit();
    if (ret != GLEW_OK)
        throw std::runtime_error("Unable to initialize OpenGL extensions.");

    glGenVertexArrays(1, vao);
    glGenBuffers(2, vbo);
    fillVBOs();
}

void Viewer::loadCloud()
{
    QString fileName = QFileDialog::getOpenFileName(NULL,tr("Open PLY/PCD file"), ".", tr("*.ply *.pcd"));
    QByteArray byteArray = fileName.toUtf8();
    const char* fname = byteArray.constData();

    //typedef std::chrono::high_resolution_myclock myclock;
    const auto t0 = myclock::now();
    if(getFileExtension(fname) == "pcd") {
        pcl::PCDReader reader;
        reader.read(fname, cloud);
    }
    else if(getFileExtension(fname) == "ply") {
        readPly(fname, cloud, modelview);
    }

    const auto t1 = myclock::now();
    std::cout << "Static model loaded in " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms." <<std::endl;
}

void Viewer::fillVBOs(bool isBGR)
{
    const auto t0 = myclock::now();
    if(toDownsample)
    {
        int or_points = cloud.points.size();
        refineCloud(cloud, voxelGridLeafSize, pclCompressionType);
        float perc_removed = 100.0 - ((float)(cloud.points.size()/(float)or_points)*100.0);
        std::cout << "Removed " <<  perc_removed << "% of the input points" <<std::endl;
    }


    int old_pts = cloud.points.size();
    if (old_pts == 0)
        return;

    n_vertices = cloud.points.size();
    std::vector<float> VBO_cloud_pos; VBO_cloud_pos.reserve(n_vertices);
    std::vector<unsigned char> VBO_cloud_cols; VBO_cloud_cols.reserve(n_vertices);

    for(int i = 0; i < n_vertices; ++i) {
        //position
        VBO_cloud_pos.push_back( cloud.points[i].x);
        VBO_cloud_pos.push_back( cloud.points[i].y);
        VBO_cloud_pos.push_back( cloud.points[i].z);
        //colour
        int32_t rgb_val = *(int32_t*)(&cloud.points[i].rgb);
        if(!isBGR)
        {
            VBO_cloud_cols.push_back( ((unsigned char) (rgb_val >> 16)) );//r
            VBO_cloud_cols.push_back( ((unsigned char) (rgb_val >> 8)) );//g
            VBO_cloud_cols.push_back( ((unsigned char) (rgb_val >> 0)) );//b
        }
        else
        {
            VBO_cloud_cols.push_back( ((unsigned char) (rgb_val >> 0)) );//r
            VBO_cloud_cols.push_back( ((unsigned char) (rgb_val >> 8)) );//g
            VBO_cloud_cols.push_back( ((unsigned char) (rgb_val >> 16)) );//b
        }

    }

    //write everything in the VAO
    glBindVertexArray(vao[0]);
    //enable vertex, colour and normals
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);

    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) *  n_vertices * 3, &VBO_cloud_pos[0], GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0,0);

    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLbyte) * n_vertices * 3, &VBO_cloud_cols[0], GL_STATIC_DRAW);
    glColorPointer(3, GL_UNSIGNED_BYTE, 0, 0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    const auto t1 = myclock::now();
    std::cout << "Static model moved to gpu in " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms." <<std::endl;

}

void Viewer::enablePointSprite()
{

    //Make space for particle limits and fill it from OGL call.
    GLfloat sizes[2];
    glGetFloatv(GL_ALIASED_POINT_SIZE_RANGE, sizes);

    glEnable(GL_DEPTH_TEST);
    //glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    float quadratic[] =  { 1.0, 5.0, 5.0};
    glPointParameterfvARB( GL_POINT_DISTANCE_ATTENUATION_ARB, quadratic );
    glPointParameterfARB( GL_POINT_FADE_THRESHOLD_SIZE_ARB, 60.0 );

    if(doPointSmooth)
        glEnable(GL_POINT_SMOOTH);
    else
        glDisable(GL_POINT_SMOOTH);
    glPointSize( 50 );

    //Tell it the max and min sizes we can use using our pre-filled array.
    glPointParameterfARB( GL_POINT_SIZE_MIN_ARB, sizes[0] );
    glPointParameterfARB( GL_POINT_SIZE_MAX_ARB, sizes[1] );
    glTexEnvf( GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE );

}

void Viewer::disablePointSprite()
{
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

//http://www.opengl.org/discussion_boards/showthread.php/129547-Particle-System-Depth-and-Blend
void Viewer::draw()
{
    if (n_vertices == 0)
        loadCloud();

    GLboolean lighting_state = glIsEnabled(GL_LIGHTING);
    if (lighting_state == GL_TRUE)
        glDisable(GL_LIGHTING);

    glPushMatrix();
    glMultMatrixd(modelview);
    glBindVertexArray(vao[0]);
    float pointsize = 1;
    glGetFloatv(GL_POINT_SIZE,&pointsize);
    glPointSize(2.5);
    if(drawSprite)
        enablePointSprite();
    glDrawArrays(GL_POINTS, 0, n_vertices);
    glPointSize(pointsize);
    glBindVertexArray(0);
    if(drawSprite)
        disablePointSprite();
    glPopMatrix();

    if (lighting_state == GL_TRUE)
        glEnable(GL_LIGHTING);
}

void Viewer::keyPressEvent(QKeyEvent* e)
{
    if (e->key() == Qt::Key_W) {
        GLint previous_polygon_mode[2];
        glGetIntegerv(GL_POLYGON_MODE, previous_polygon_mode);
        const int mode = previous_polygon_mode[0] == GL_LINE ? GL_FILL : GL_LINE;
        glPolygonMode(GL_FRONT_AND_BACK, mode);
        updateGL();
    } else if (e->key() == Qt::Key_T) {
        if (glIsEnabled(GL_TEXTURE_2D) == GL_TRUE) {
            glDisable(GL_TEXTURE_2D);
            glEnable(GL_LIGHTING);
        }
        else {
            glEnable(GL_TEXTURE_2D);
            glDisable(GL_LIGHTING);
        }
        updateGL();
    } else if (e->key() == Qt::Key_L) {
        if (glIsEnabled(GL_LIGHTING) == GL_FALSE)
        {
            glEnable(GL_LIGHTING);
            glEnable(GL_LIGHT0);
        }
        else
        {
            glDisable(GL_LIGHTING);
            glDisable(GL_LIGHT0);
        }
        updateGL();
    } else if (e->key() == Qt::Key_S) {

        QString fileName = QFileDialog::getSaveFileName(NULL,tr("Save PLY file"), ".", tr("*.ply"));
        if(!fileName.isEmpty())
        {
            QByteArray byteArray = fileName.toUtf8();
            const char* ofilename = byteArray.constData();
            saveCloud(ofilename, cloud);
        }

    } else if(e->key() == Qt::Key_P)
    {
        triangulate(cloud);
    }
    else if(e->key() == Qt::Key_O)
    {
        drawSprite = !drawSprite;
        updateGL();
    }
    else if(e->key() == Qt::Key_I)
    {
        doPointSmooth = !doPointSmooth;
        updateGL();
    }
    else
        QGLViewer::keyPressEvent(e);
}
