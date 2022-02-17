#include "mesh.h"
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

GeometricWorld::GeometricWorld()
{
    double width=0.5, depth=0.6, height=0.8;
    _bBox.push_back(Point(-0.5*width,-0.5*depth,-0.5*height)); //0
    _bBox.push_back(Point(-0.5*width,0.5*depth,-0.5*height)); // 1
    _bBox.push_back(Point(0.5*width,-0.5*depth,-0.5*height)); // 2
    _bBox.push_back(Point(-0.5*width,-0.5*depth,0.5*height)); // 3
    _mesh = Mesh();
    _mesh.loadMesh();
}

// The following functions could be displaced into a module OpenGLDisplayGeometricWorld that would include mesh.h

// Draw a Point
void glPointDraw(const Point & p) {
    glVertex3f(p._x, p._y, p._z);
}

// Draw a Vertex
void glPointDraw(const Vertex & v) {
    glVertex3f(v._x, v._y, v._z);
}

//Example with a bBox
void GeometricWorld::draw() {
    glColor3d(1,0,0);
    glBegin(GL_TRIANGLES);
    glPointDraw(_bBox[0]);
    glPointDraw(_bBox[1]);
    glPointDraw(_bBox[2]);
    glEnd();

    glColor3d(0,1,0);
    glBegin(GL_TRIANGLES);
    glPointDraw(_bBox[0]);
    glPointDraw(_bBox[2]);
    glPointDraw(_bBox[3]);
    glEnd();

    glColor3d(0,0,1);
    glBegin(GL_TRIANGLES);
    glPointDraw(_bBox[0]);
    glPointDraw(_bBox[3]);
    glPointDraw(_bBox[1]);
    glEnd();

    //glColor3d(1,1,0);
}

void Mesh::loadMesh() {
    map<pair<int, int>, pair<int, int>> vertexFaceMatch;
    fstream newfile;
    int count = 0, verticesCount, facesCount;
    newfile.open("../queen.off",ios::in); //open a file to perform read operation using file object
    if (newfile.is_open()){ //checking whether the file is open
       string tp;
       while(getline(newfile, tp)){ //read data from file object and put it into string.
          if (count == 0) {
              int first_whitespace_pos = tp.find(" ");
              verticesCount = stoi(tp.substr(0, first_whitespace_pos));
              facesCount = stoi(tp.substr(first_whitespace_pos+1, tp.length() - first_whitespace_pos));
          } else {
            if (count <= verticesCount) {
                // separate file line according to whitespaces to get all coordinates of vertex
                int first_whitespace_pos = tp.find(" ");
                int second_whitespace_pos = tp.find(" ", first_whitespace_pos+1);
                float P1 = stof(tp.substr(0, first_whitespace_pos));
                float P2 = stof(tp.substr(first_whitespace_pos+1, second_whitespace_pos-first_whitespace_pos));
                float P3 = stof(tp.substr(second_whitespace_pos+1, tp.length()-second_whitespace_pos));
                Vertex v(P1, P2, P3);
                addVertex(v);
            } else if (count <= verticesCount + facesCount) {
                // separate string according to whitespaces in .off file
                int first_whitespace_pos = tp.find(" ");
                int second_whitespace_pos = tp.find(" ", first_whitespace_pos+1);
                int third_whitespace_pos = tp.find(" ", second_whitespace_pos+1);
                // get indexes of vertexes for face with above separation
                int indV1 = stoi(tp.substr(first_whitespace_pos+1, second_whitespace_pos-first_whitespace_pos));
                int indV2 = stoi(tp.substr(second_whitespace_pos+1, third_whitespace_pos-second_whitespace_pos));
                int indV3 = stoi(tp.substr(third_whitespace_pos+1, tp.length()-third_whitespace_pos));
                Face f(indV1, indV2, indV3);


                // match vertex to face
                Vertex v1 = _vertices[indV1];
                Vertex v2 = _vertices[indV2];
                Vertex v3 = _vertices[indV3];
                int indF = count - verticesCount - 1;
                v1.indF = indF;
                v2.indF = indF;
                v3.indF = indF;

                // match face to adjacent face
                pair<int, int> edge1 = make_pair(indV1, indV2);
                pair<int, int> edge2 = make_pair(indV1, indV3);
                pair<int, int> edge3 = make_pair(indV2, indV3);

                // for each edge of the face, need to check if there is an adjacent face
                if (vertexFaceMatch.find(edge1) == vertexFaceMatch.end() ) {
                  // not found
                  // we add the edge/(faceIndex, oppositeVertexInd) to the map
                  vertexFaceMatch.insert({edge1, make_pair(indF, 2)});
                } else {
                  // found
                  // we then know what is the opposite face
                  int oppFaceInd = vertexFaceMatch.at(edge1).first;
                  int vertexInd = vertexFaceMatch.at(edge1).second;
                  f.indF3 = oppFaceInd;
                  Face oppFace = _faces[oppFaceInd];
                  if (vertexInd == 1) {
                    oppFace.indF1 = indF;
                  }
                  if (vertexInd == 2) {
                    oppFace.indF2 = indF;
                  }
                  if (vertexInd == 3) {
                    oppFace.indF3 = indF;
                  }
                  _faces[oppFaceInd] = oppFace;
                }

                // second edge
                if (vertexFaceMatch.find(edge2) == vertexFaceMatch.end() ) {
                  vertexFaceMatch.insert({edge2, make_pair(indF, 1)});
                } else {
                    int oppFaceInd = vertexFaceMatch.at(edge2).first;
                    int vertexInd = vertexFaceMatch.at(edge2).second;
                    f.indF2 = oppFaceInd;
                    Face oppFace = _faces[oppFaceInd];
                    if (vertexInd == 1) {
                      oppFace.indF1 = indF;
                    }
                    if (vertexInd == 2) {
                      oppFace.indF2 = indF;
                    }
                    if (vertexInd == 3) {
                      oppFace.indF3 = indF;
                    }
                    _faces[oppFaceInd] = oppFace;
                }

                // last edge
                if (vertexFaceMatch.find(edge3) == vertexFaceMatch.end() ) {
                  vertexFaceMatch.insert({edge3, make_pair(indF, 0)});
                } else {
                    int oppFaceInd = vertexFaceMatch.at(edge3).first;
                    int vertexInd = vertexFaceMatch.at(edge3).second;
                    f.indF1 = oppFaceInd;
                    Face oppFace = _faces[oppFaceInd];
                    if (vertexInd == 1) {
                      oppFace.indF1 = indF;
                    }
                    if (vertexInd == 2) {
                      oppFace.indF2 = indF;
                    }
                    if (vertexInd == 3) {
                      oppFace.indF3 = indF;
                    }
                    _faces[oppFaceInd] = oppFace;
                }
                addFace(f);
            }
          }
          count++;
       }
       newfile.close(); //close the file object.
    }
}

float norm(Vertex vect1, Vertex vect2) {
    // norm between two vectors
    // using the Vertex class to store vectors because can be interpreted in a similar way
    return sqrt((vect2._x - vect1._x)*(vect2._x - vect1._x) + (vect2._y - vect1._y)*(vect2._y - vect1._y) + (vect2._z - vect1._z)*(vect2._z - vect1._z));
}

float angle(Vertex v1, Vertex v2, Vertex v3) {
    // find angle between (v2,v1) and (v2, v3)
    Vertex vect1(v1._x - v2._x, v1._y - v2._y, v1._z - v2._z);
    Vertex vect2(v3._x - v2._x, v3._y - v2._y, v3._z - v2._z);
    return norm(vect1, vect2);
}

QVector<QVector<float>> Mesh::LPBeltrami() {
    QVector<QVector<float>> LPBRes;
    QVectorIterator<Vertex> itV(_vertices);
    int count = 0;
        while (itV.hasNext()) {
            Vertex v = itV.next();
            int totalArea = 0;
            int totalSumX = 0;
            int totalSumY = 0;
            int totalSumZ = 0;

            // two circulators on triangles and three on vertices
            // worse performance but easier to understand and code
            Circulator_on_faces circF = incident_faces(v, count);
            Circulator_on_faces circF_ahead = incident_faces(v, count);
            ++circF_ahead;

            Circulator_on_vertices circV = adjacent_vertices(v, count);
            Circulator_on_vertices circV_ahead = adjacent_vertices(v, count);
            Circulator_on_vertices circV_ahead2 = adjacent_vertices(v, count);
            ++circV_ahead;
            ++circV_ahead2;
            ++circV_ahead2;
            bool whileNotStarted = true;
            while (circV.getIndex() != 0 || whileNotStarted) {
                whileNotStarted = false;
                Vertex currentVertex = circV_ahead.get();
                float alpha_ij = angle(v, circV.get(), circV_ahead.get());
                float beta_ij = angle(v, circV_ahead2.get(), circV_ahead.get());

                // calculating laplacian
                totalSumX += (cos(alpha_ij)/sin(alpha_ij) + cos(beta_ij)/sin(beta_ij))*(currentVertex._x - v._x);
                totalSumY += (cos(alpha_ij)/sin(alpha_ij) + cos(beta_ij)/sin(beta_ij))*(currentVertex._y - v._y);
                totalSumZ += (cos(alpha_ij)/sin(alpha_ij) + cos(beta_ij)/sin(beta_ij))*(currentVertex._z - v._z);

                // calculating area of triangle
                totalArea += 1/2 * norm(v, circV.get()) * norm(circV.get(), circV_ahead.get()) * sin(alpha_ij);
            }
            totalSumX /= totalArea;
            totalSumY /= totalArea;
            totalSumZ /= totalArea;

            QVector<float> Laplacian;
            Laplacian.push_back(totalSumX);
            Laplacian.push_back(totalSumY);
            Laplacian.push_back(totalSumZ);
            LPBRes.push_back(Laplacian);
    }
    return LPBRes;
}

Mesh::Mesh() {}

//Example with a wireframe bBox
void GeometricWorld::drawWireFrame() {
    glColor3d(0,1,0);
    glBegin(GL_LINE_STRIP);
    glPointDraw(_bBox[0]);
    glPointDraw(_bBox[1]);
    glEnd();
    glColor3d(0,0,1);
    glBegin(GL_LINE_STRIP);
    glPointDraw(_bBox[0]);
    glPointDraw(_bBox[2]);
    glEnd();
    glColor3d(1,0,0);
    glBegin(GL_LINE_STRIP);
    glPointDraw(_bBox[0]);
    glPointDraw(_bBox[3]);
    glEnd();
}

// ne SUROTUT pas supprimer dans les vector des faces quand on merge deux vertex
// utiliser un tableau de boolean pour savoir si les faces sont utilisees ou pas
// on fait une map ou pour chaque face on précise quel sommet est parti ou pour update les infos de voisinage SI BESOIN, c'est rare et cher
// quand on contracte l'arrpete, les faces disparaissent et les voisins changent, il faut donc pleins de vairables locales en indices
// variables nécessaires : indice global sommet 1, indice global sommet 2, indice face 1, indice face 2, les quatres faces voisines des deux faces en question pour update leur voisins,
// il faut aussi savoir c'est quel voisin (voisin 1, 2 ou 3) (faire une fonction pour ça)
// instruction type : faces[ifgh].setvoisin(ir) = ifhd

/*
bool Mesh::compareEdges(const pair<int, int> &edge1, const pair<int, int> &edge2) {
    int ind1Edge1 = edge1.first;
    int ind2Edge1 = edge1.second;

}

void Mesh::simplifyMesh(float threshold) {
    // threshold defines the minimum size for edges
    QVector<pair<int,int>> smallestEdges;


}*/
void Mesh::drawMesh() {
    QVector<QVector<float>> LPB = LPBeltrami();
    QVectorIterator<Face> itNumbers(_faces);
        while (itNumbers.hasNext()) {
            Face f = itNumbers.next();
            Vertex v1 = _vertices[f._v1];
            Vertex v2 = _vertices[f._v2];
            Vertex v3 = _vertices[f._v3];

            glColor3d(1,1,1);
            glBegin(GL_TRIANGLES);
            glPointDraw(v1);
            glPointDraw(v2);
            glPointDraw(v3);
            glEnd();
        }
}

void Mesh::drawMeshWireFrame() {
    QVectorIterator<Face> itNumbers(_faces);
        while (itNumbers.hasNext()) {
            Face f = itNumbers.next();
            Vertex v1 = _vertices[f._v1];
            Vertex v2 = _vertices[f._v2];
            Vertex v3 = _vertices[f._v3];

            glColor3d(1,0,0);
            glBegin(GL_TRIANGLES);
            glPointDraw(v1);
            glPointDraw(v2);
            glPointDraw(v3);
            glEnd();

            glBegin(GL_LINE_STRIP);
            glPointDraw(v1);
            glPointDraw(v2);
            glEnd();
            glBegin(GL_LINE_STRIP);
            glPointDraw(v1);
            glPointDraw(v3);
            glEnd();
            glBegin(GL_LINE_STRIP);
            glPointDraw(v2);
            glPointDraw(v3);
            glEnd();
        }
}
