#ifndef MESH_H
#define MESH_H

#include <QOpenGLWidget>
#include <QVector>

// TO MODIFY
class Point
{
public:
    double _x;
    double _y;
    double _z;

    Point():_x(),_y(),_z() {}
    Point(float x_, float y_, float z_):_x(x_),_y(y_),_z(z_) {}
};

class Vertex
{
public:
    double _x;
    double _y;
    double _z;
    int indF;

    Vertex():_x(),_y(),_z() {}
    Vertex(float x_, float y_, float z_):_x(x_),_y(y_),_z(z_) {}
    Vertex(const Vertex &) = default;
};

class Face
{
public:
    int _v1;
    int _v2;
    int _v3;
    int indF1;
    int indF2;
    int indF3;

    Face(): _v1(), _v2(), _v3() {}
    Face(int v1_, int v2_, int v3_): _v1(v1_), _v2(v2_), _v3(v3_) {}
    int findVertexIndex(int v) {
        if (v == _v1) {
            return 1;
        }
        if (v == _v2) {
            return 2;
        }
        return 3;

    }
    int nextCCWiseFace(int vertexIndex) {
        if (vertexIndex == 1) {
            return indF2;
        }
        if (vertexIndex == 2) {
            return indF3;
        }
        return indF1;
    }
    int nextCCWiseVertex(int vertexIndex) {
        if (vertexIndex == 1) {
            return _v2;
        }
        if (vertexIndex == 2) {
            return _v3;
        }
        return _v1;
    }
};

class Circulator_on_vertices
{
    QVector<Vertex> vertices;
    Vertex vertex;
    int startIndex;
    int currentIndex;
    int endIndex;
public:
    Circulator_on_vertices(QVector<Vertex> vertices, Vertex vertex): vertices(vertices), vertex(vertex) {
        startIndex = 0;
        currentIndex = 0;
        endIndex = vertices.length();
    }
    Circulator_on_vertices& operator++() {
        if (currentIndex == endIndex) {
            currentIndex = 0;
        } else {
            currentIndex++;
        }
        return *this;
    }
    Vertex get() {
        return vertices[currentIndex];
    }
    int getIndex() {
        return currentIndex;
    }
};

class Circulator_on_faces
{
    QVector<Face> faces;
    Vertex vertex;
    int startIndex;
    int currentIndex;
    int endIndex;
public:
    Circulator_on_faces(QVector<Face> faces, Vertex vertex): faces(faces), vertex(vertex) {
        startIndex = 0;
        currentIndex = 0;
        endIndex = faces.length();
    }
    Circulator_on_faces& operator++() {
        if (currentIndex == endIndex) {
            currentIndex = 0;
        } else {
            currentIndex++;
        }
        return *this;
    }
    Face get() {
        return faces[currentIndex];
    }
    int getIndex() {
        return currentIndex;
    }
};

class Mesh
{
  // (Q ou STL)Vector of vertices
    QVector<Vertex> _vertices;
  // (Q ou STL)Vector of faces
    QVector<Face> _faces;
  // Those who do not know about STL Vectors should have a look at cplusplus.com examples
public:
    Mesh(); // Constructors automatically called to initialize a Mesh (default strategy)
    Mesh(QVector<Vertex> vertices_, QVector<Face> faces_): _vertices(vertices_), _faces(faces_) {}
    //~Mesh(); // Destructor automatically called before a Mesh is destroyed (default strategy)

    // funtions to add faces and vertices to the mesh
    void addVertex(Vertex v) {_vertices.push_back(v);};
    void addFace(Face f) {_faces.push_back(f);};

    // calculate Laplace Beltrami
    QVector<QVector<float>> LPBeltrami();

    // circulating through the mesh
    Circulator_on_faces incident_faces(Vertex &v, int vertexPos) {
        QVector<Face> incidentFaces;
        // we first get the face the vertex is linked to
        Face currentFace = _faces[v.indF];
        incidentFaces.push_back(currentFace);
        int currentIndex = -1;
        // then we go counter clock wise through the faces
        while (currentIndex != v.indF) {
            currentIndex = currentFace.nextCCWiseFace(currentFace.findVertexIndex(vertexPos));
            currentFace = _faces[currentIndex];
            incidentFaces.push_back(currentFace);
        }
        return Circulator_on_faces(incidentFaces,v);
    }

    Circulator_on_vertices adjacent_vertices(Vertex &v, int vertexPos) {
        QVector<Vertex> adjacentVertices;
        Circulator_on_faces circF = incident_faces(v, vertexPos);
        Face fToExplore = circF.get();
        adjacentVertices.push_back(_vertices[fToExplore.nextCCWiseVertex(fToExplore.findVertexIndex(vertexPos))]);
        ++circF;
        while (circF.getIndex() != 0) {
            Face fToExploreW = circF.get();
            adjacentVertices.push_back(_vertices[fToExploreW.nextCCWiseVertex(fToExploreW.findVertexIndex(vertexPos))]);
            ++circF;
        }
        return Circulator_on_vertices(adjacentVertices,v);
    }



    // loading from .obj files and drawing
    void loadMesh();
    void drawMesh();
    void drawMeshWireFrame();
    //bool mesh::compareEdges(const int &edge1, const int &edge2)
    void simplifyMesh(float threshold);
};



class GeometricWorld //Here used to create a singleton instance
{
  QVector<Point> _bBox;  // Bounding box
public :
  GeometricWorld();
  void draw();
  void drawWireFrame();
  // ** TP Can be extended with further elements;
  Mesh _mesh;
};


#endif // MESH_H
