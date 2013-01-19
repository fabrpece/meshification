#include <string>
#include <fstream>
#include <stdexcept>
#include <array>
#include <GL/glew.h>
#include <QKeyEvent>
#include <QImage>
#include <QMessageBox>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include "Viewer.hpp"
#include "Camera.hpp"

const std::string name = "virtual_kinect";

Viewer::Viewer(QWidget* parent) :
    QGLViewer(parent),
    n_elements(0),
    depth_stream((name + ".depth").c_str(), std::ios::out | std::ios::binary),
    color_stream((name + ".rgb").c_str(), std::ios::out | std::ios::binary)
{
    setWindowTitle("Virtual Kinect");
}

Viewer::~Viewer()
{
    glDeleteVertexArrays(vao_size, vao);
    glDeleteBuffers(vbo_size, vbo);
}

void Viewer::init()
{
    ::glewInit();
    glGenVertexArrays(vao_size, vao);
    glGenBuffers(vbo_size, vbo);
    std::array<float, 18> camera_buffer {
	0, 0, 0,
	0.1f, 0.1f, -0.2f,
	-0.1f, 0.1f, -0.2f,
	-0.1f, -0.1f, -0.2f,
	0.1f, -0.1f, -0.2f,
	0.1f, 0.1f, -0.2f
    };
    n_cam_elements = camera_buffer.size() / 3;
    glBindVertexArray(vao[1]);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[3]);
    glBufferData(GL_ARRAY_BUFFER, camera_buffer.size() * sizeof(float), camera_buffer.data(), GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glBindVertexArray(0);
    cam.reset(new Camera(name, 640, 480));
}

void Viewer::draw()
{
    double mv[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, mv);
    cam->set_modelview_matrix(mv);
    draw_model();
    //glDisable(GL_DEPTH_TEST);
    //glColor3f(0, 1, 0);
    //glEnable(GL_DEPTH_TEST);
    cam->render(std::bind(&Viewer::draw_model, this));
    //glDisable(GL_LIGHTING);
    //cam->draw(std::bind(&Viewer::draw_camera, this));
    //cam->draw_cloud();
    //glEnable(GL_LIGHTING);
    shot();
}

void Viewer::keyPressEvent(QKeyEvent* e)
{
    if (e->key() == Qt::Key_S && e->modifiers() == Qt::AltModifier) {
	shot();
	updateGL();
    } else if (e->key() == Qt::Key_W) {
	GLint previous_polygon_mode[2];
	glGetIntegerv(GL_POLYGON_MODE, previous_polygon_mode);
	const int mode = previous_polygon_mode[0] == GL_LINE ? GL_FILL : GL_LINE;
	glPolygonMode(GL_FRONT_AND_BACK, mode);
	updateGL();
    } else
	QGLViewer::keyPressEvent(e);
}

void Viewer::draw_model() const
{
    glColor3f(1, 1, 1);
    glBindVertexArray(vao[0]);
    glDrawElements(GL_TRIANGLES, n_elements, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void Viewer::draw_camera() const
{
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBindVertexArray(vao[1]);
    glDrawArrays(GL_TRIANGLE_FAN, 0, n_cam_elements);
    glBindVertexArray(0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Viewer::load_model(const QString& filename)
{
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(filename.toAscii().data(), aiProcess_Triangulate | aiProcess_GenSmoothNormals);
    if (scene == 0)
	throw std::logic_error("Unable to load the model file " + filename.toStdString());
    if (scene->HasMeshes() == false)
	throw std::logic_error("The model doesn't contain meshes");
    aiMesh* mesh = scene->mMeshes[0];
    if (mesh->mPrimitiveTypes != aiPrimitiveType_TRIANGLE)
	throw std::logic_error("The mesh must be composed of only triangles");
    std::vector<unsigned> indices(3 * mesh->mNumFaces);
    for (int i = 0; i < mesh->mNumFaces; ++i) {
	const aiFace& face = mesh->mFaces[i];
	if (face.mNumIndices != 3)
	    throw std::logic_error("Only triangular primitives are admitted");
	std::copy(face.mIndices, face.mIndices + 3, indices.begin() + 3 * i);
    }
    std::vector<float> vertices;
    vertices.reserve(3 * mesh->mNumVertices);
    for (int i = 0; i < mesh->mNumVertices; ++i) {
	const aiVector3D& vertex = mesh->mVertices[i];
	for (int j = 0; j < 3; ++j)
	    vertices.push_back(vertex[j]);
    }
    std::vector<float>normals;
    normals.reserve(3 * mesh->mNumVertices);
    for (int i = 0; i < mesh->mNumVertices; ++i) {
	const aiVector3D& normal = mesh->mNormals[i];
	for (int j = 0; j < 3; ++j)
	    normals.push_back(normal[j]);
    }
    glBindVertexArray(vao[0]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbo[0]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned) * indices.size(), &indices[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
    glVertexPointer(3, GL_FLOAT, 0, 0);
    glEnableClientState(GL_VERTEX_ARRAY);
    glBindBuffer(GL_ARRAY_BUFFER, vbo[2]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * normals.size(), &normals[0], GL_STATIC_DRAW);
    glNormalPointer(GL_FLOAT, 0, 0);
    glEnableClientState(GL_NORMAL_ARRAY);
    glBindVertexArray(0);
    n_elements = indices.size();
}

void Viewer::shot()
{
    std::vector<unsigned char> rgb;
    std::vector<float> point_cloud;
    cam->get_frame(rgb, point_cloud);
    const int n_vertices = point_cloud.size() / 3;
    std::vector<unsigned short> depth(n_vertices);
    for (int i = 0; i < n_vertices; ++i)
	depth[i] = static_cast<unsigned short>(point_cloud[3 * i + 2] * 1000.0f);
    depth_stream.write((const char*)depth.data(), 2 * depth.size());
    depth_stream << std::flush;
    color_stream.write((const char*)rgb.data(), rgb.size());
    color_stream << std::flush;
    //QImage image(&rgb[0], cam->get_width(), cam->get_height(), QImage::Format_RGB888);
    //if (image.mirrored().save(QString("frame%1.png").arg(cam->get_name().c_str())) == false)
	//QMessageBox::warning(this, "Grab error", "Unable to write the snapshot");
}
