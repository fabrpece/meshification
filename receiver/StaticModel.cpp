#include <chrono>
#include <sstream>
#include <fstream>
#include <GL/glew.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include "StaticModel.hpp"

static
std::string getFileExtension(const std::string& filename)
{
    const auto p = filename.find_last_of('.');
    return p != std::string::npos ? filename.substr(p + 1) : "";
}

StaticModel::StaticModel() :
    n_vertices(0),
    use_point_smooth(false)
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

void StaticModel::load(const char* fname)
{
    typedef std::chrono::high_resolution_clock clock;
    const auto t0 = clock::now();
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    if(getFileExtension(fname) == "pcd") {
        pcl::PCDReader reader;
        reader.read(fname, cloud);
    }
    else if(getFileExtension(fname) == "ply") {
        readPly(fname, cloud, modelview);
    }

    n_vertices = cloud.points.size();
    if (cloud.points.size() == 0)
        return;

    std::vector<float> positions;
    positions.reserve(n_vertices * 3);
    std::vector<unsigned char> colors;
    colors.reserve(n_vertices * 3);

    for(int i = 0; i < n_vertices; ++i) {
        positions.push_back(cloud.points[i].x);
        positions.push_back(cloud.points[i].y);
        positions.push_back(cloud.points[i].z);
        const int32_t rgb_val = *(int32_t*)(&cloud.points[i].rgb);
        colors.push_back(static_cast<unsigned char>(rgb_val >> 16));//r
        colors.push_back(static_cast<unsigned char>(rgb_val >> 8));//g
        colors.push_back(static_cast<unsigned char>(rgb_val >> 0));//b
    }

    glBindVertexArray(vao[0]);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) *  n_vertices * 3, &positions[0], GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0,0);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(GLbyte) * n_vertices * 3, &colors[0], GL_STATIC_DRAW);
    glColorPointer(3, GL_UNSIGNED_BYTE, 0, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    const auto t1 = clock::now();
    std::cout << "Static model loaded in " << std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() << "ms." << std::endl;
}

void StaticModel::tooglePointSmooth()
{
    use_point_smooth = !use_point_smooth;
}

static
void enablePointSprite(const bool usePointSmooth)
{
    //Make space for particle limits and fill it from OGL call.
    GLfloat sizes[2];
    glGetFloatv(GL_ALIASED_POINT_SIZE_RANGE, sizes);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    float quadratic[] =  {1.0f, 5.0f, 5.0f};
    glPointParameterfvARB(GL_POINT_DISTANCE_ATTENUATION_ARB, quadratic);
    glPointParameterfARB(GL_POINT_FADE_THRESHOLD_SIZE_ARB, 80.0f);
    if (usePointSmooth)
        glEnable(GL_POINT_SMOOTH);
    else
        glDisable(GL_POINT_SMOOTH);
    glPointSize(70);

    //Tell it the max and min sizes we can use using our pre-filled array.
    glPointParameterfARB(GL_POINT_SIZE_MIN_ARB, sizes[0]);
    glPointParameterfARB(GL_POINT_SIZE_MAX_ARB, sizes[1]);
    glTexEnvf(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
}

static
void disablePointSprite()
{
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

void StaticModel::draw() const
{
    if (n_vertices == 0)
        return;
    GLboolean lighting_state = glIsEnabled(GL_LIGHTING);
    if (lighting_state == GL_TRUE)
        glDisable(GL_LIGHTING);
    glPushMatrix();
    glMultMatrixf(modelview);
    glBindVertexArray(vao[0]);
    float pointsize = 0.0f;
    glGetFloatv(GL_POINT_SIZE, &pointsize);
    glPointSize(2.5);
    enablePointSprite(use_point_smooth);
    glDrawArrays(GL_POINTS, 0, n_vertices);
    disablePointSprite();
    glPointSize(pointsize);
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
        else if (tmp == "property") {
            iss >> tmp;
            if (tmp == "uchar") {
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
        unsigned char r, g, b, a;
        if (is_bin == true) {
            fileIn.read((char*)&x, sizeof(float));
            fileIn.read((char*)&y, sizeof(float));
            fileIn.read((char*)&z, sizeof(float));
            fileIn.read((char*)&r, sizeof(unsigned char));
            fileIn.read((char*)&g, sizeof(unsigned char));
            fileIn.read((char*)&b, sizeof(unsigned char));
            if (has_alpha == true)
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
    return cloud.points.size() == tot_points;
}
