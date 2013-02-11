#include <chrono>
#include <sstream>
#include <fstream>
#include <GL/glew.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
#include <Eigen/Core>
#include "StaticModel.hpp"

static
std::string getFileExtension(const std::string& FileName)
{
    if(FileName.find_last_of(".") != std::string::npos)
        return FileName.substr(FileName.find_last_of(".")+1);
    return "";
}

StaticModel::StaticModel() :
    n_vertices(0)
{
    Eigen::Map<Eigen::Matrix4f> mv(modelview);
    mv.setIdentity();
    glGenVertexArrays(1, vao);
    glGenBuffers(2, vbo);
}

StaticModel::~StaticModel()
{
    glDeleteBuffers(2, vbo);
    glDeleteVertexArrays(1, vao);
}

static
bool readPly(const std::string& fname, pcl::PointCloud<pcl::PointXYZRGB>& cloud, float modelview[16]);

void StaticModel::load(const char *fname)
{
    typedef std::chrono::high_resolution_clock clock;
    const auto t0 = clock::now();
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    if(getFileExtension(fname) == "pcd") {
        pcl::PCDReader reader;
        reader.read(fname, cloud);
    }
    else if(getFileExtension(fname) == "ply") {
        //pcl::PLYReader reader;
        //reader.read(fname.c_str(), *cloud);
        readPly(fname, cloud, modelview);
    }

    n_vertices = cloud.points.size();
    if (cloud.points.size() == 0)
        return;

    const auto t1 = clock::now();
    std::cout << "Static model loaded in " << (t1-t0).count()/1000 << "ms" <<std::endl;
    std::vector<float> VBO_cloud_pos(cloud.points.size() * 3);
    std::vector<unsigned char> VBO_cloud_cols(cloud.points.size() * 3);

    int k = 0;
    for(int i = 0; i < cloud.points.size() * 3; i += 3) {
        VBO_cloud_pos[i] = cloud.points[k].x;
        VBO_cloud_pos[i+1] = cloud.points[k].y;
        VBO_cloud_pos[i+2] = cloud.points[k].z;
        int32_t rgb_val = *(int32_t*)(&cloud.points[k].rgb);
        VBO_cloud_cols[i] = ((unsigned char) (rgb_val >> 16));//r
        VBO_cloud_cols[i+1] = ((unsigned char) (rgb_val >> 8));//g
        VBO_cloud_cols[i+2] = ((unsigned char) (rgb_val >> 0));//b
        ++k;
    }

    glBindVertexArray(vao[0]);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) *  cloud.points.size() * 3, &VBO_cloud_pos[0], GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0,0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLbyte) * cloud.points.size() * 3, &VBO_cloud_cols[0], GL_STATIC_DRAW);
    glColorPointer(3, GL_UNSIGNED_BYTE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void StaticModel::draw()
{
    if (n_vertices == 0)
        return;
    GLboolean lighting_state = glIsEnabled(GL_LIGHTING);
    if (lighting_state == GL_TRUE)
        glDisable(GL_LIGHTING);
    glPushMatrix();
    glMultMatrixf(modelview);
    glBindVertexArray(vao[0]);
    glDrawArrays(GL_POINTS, 0, n_vertices);
    glBindVertexArray(0);
    glPopMatrix();
    if (lighting_state == GL_TRUE)
        glEnable(GL_LIGHTING);
}

static
bool readPly(const std::string& fname, pcl::PointCloud<pcl::PointXYZRGB>& cloud, float modelview[16])
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
        unsigned char r, g, b;
        if (is_bin == true) {
            fileIn.read((char*)&x, sizeof(float));
            fileIn.read((char*)&y, sizeof(float));
            fileIn.read((char*)&z, sizeof(float));
            fileIn.read((char*)&r, sizeof(unsigned char));
            fileIn.read((char*)&g, sizeof(unsigned char));
            fileIn.read((char*)&b, sizeof(unsigned char));
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
    return cloud.points.size() == tot_points;
}
