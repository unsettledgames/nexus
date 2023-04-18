#include "textureextractor.h"

#include <vcg/math/similarity2.h>
#include <vcg/space/rect_packer.h>

#include <texture-defrag/defrag_mesh.h>
#include "../texture-defrag/src/texture_optimization.h"
#include "../texture-defrag/src/packing.h"
#include "../texture-defrag/src/utils.h"
#include "../texture-defrag/src/mesh_attribute.h"
#include "../texture-defrag/src/seam_remover.h"
#include "../texture-defrag/src/texture_rendering.h"

#include "tmesh.h"

#include <iostream>
#include <GL/glew.h>
#include <QOpenGLContext>
#include <QOffscreenSurface>
#include <QPainter>

#include <Instrumentor.h>

using namespace std;

namespace nx
{
    QImage TextureExtractor::Extract(TMesh& mesh, std::vector<QImage>& toDefrag, float &error, float &pixelXedge)
    {
        Defrag::Mesh defragMesh;
        Defrag::AlgoParameters ap;

        ap.matchingThreshold = 2;
        ap.boundaryTolerance = 0.2f;
        ap.distortionTolerance = 0.5f;
        ap.globalDistortionThreshold = 0.025f;
        ap.UVBorderLengthReduction = 0.0;
        ap.offsetFactor = 5;
        ap.timelimit = 10;

        Defragment(toDefrag, mesh, defragMesh, ap);

        auto texSizes = Pack(defragMesh);

        QImage ret = Render(defragMesh, texSizes);

        {
            PROFILE_SCOPE("CopyBackMesh");
            tri::Allocator<Defrag::Mesh>::CompactEveryVector(defragMesh);

            assert(defragMesh.vn == defragMesh.vert.size() && defragMesh.vn == defragMesh.VN());
            assert(defragMesh.fn == defragMesh.face.size() && defragMesh.fn == defragMesh.FN());

            mesh.face.resize(defragMesh.FN());
            mesh.fn = defragMesh.FN();
            mesh.vert.resize(defragMesh.VN());
            mesh.vn = defragMesh.VN();

            for (int i = 0; i < defragMesh.VN(); ++i) {
                mesh.vert[i].P().X() = defragMesh.vert[i].P().X();
                mesh.vert[i].P().Y() = defragMesh.vert[i].P().Y();
                mesh.vert[i].P().Z() = defragMesh.vert[i].P().Z();
            }

            for (int i = 0; i < defragMesh.FN(); ++i) {
                for (int k = 0; k < 3; ++k) {
                    mesh.face[i].V(k) = &mesh.vert[defragMesh.face[i].cV(k) - &(*defragMesh.vert.begin())];
                    mesh.face[i].P(k) = defragMesh.face[i].P(k);
                    mesh.face[i].node = defragMesh.face[i].node;

                    mesh.face[i].WT(k).U() = defragMesh.face[i].cWT(k).U();
                    mesh.face[i].WT(k).V() = defragMesh.face[i].cWT(k).V();
                }
            }
        }

        {
            PROFILE_SCOPE("ComputeError");
            float pdx = 1/(float)ret.width();
            float pdy = 1/(float)ret.height();
            float pdx2 = pdx*pdx;
            error = 0.0;
            pixelXedge = 0.0f;
            for(auto &face: mesh.face) {
                for(int k = 0; k < 3; k++) {
                    int j = (k==2)?0:k+1;

                    float edge = vcg::SquaredNorm(face.P(k) - face.P(j));
                    float pixel = vcg::SquaredNorm(face.V(k)->T().P() - face.V(j)->T().P())/pdx2;
                    pixelXedge += pixel;
                    if(pixel > 10) pixel = 10;
                    if(pixel < 1)
                        error += edge;
                    else
                        error += edge/pixel;
                }
            }
            pixelXedge = sqrt(pixelXedge/mesh.face.size()*3);
            error = sqrt(error/mesh.face.size()*3);
        }

        //compute area waste
        double areausage = 0;
        double pdx = 1.0 / ret.width();
        double pdy = 1.0 / ret.height();

        static double sum = 0.0;
        static int n = 0;

        std::cout << "PDX: " << pdx << std::endl;
        std::cout << "PDY: " << pdy << std::endl;

        for(int i = 0; i < mesh.face.size(); i++) {
            auto &face = mesh.face[i];
            auto V0 = face.WT(0).P();
            auto V1 = face.WT(1).P();
            auto V2 = face.WT(2).P();

            V0.X() /= pdx;
            V0.Y() /= pdy;

            V1.X() /= pdx;
            V1.Y() /= pdy;

            V2.X() /= pdx;
            V2.Y() /= pdy;

            areausage += (V2 - V0)^(V2 - V1)/2;
        }

        QPainter painter(&ret);
        painter.setPen(QColor(255,0,255));
        std::cout << "Painter ok" << std::endl;
        for(int i = 0; i < mesh.face.size(); i++) {
            auto &face = mesh.face[i];

            for(int k = 0; k < 3; k++) {
                int j = (k==2)?0:k+1;
                float x0 = face.WT(k).P().X()/pdx;
                float y0 = (1.0f-face.WT(k).P().Y())/pdy;
                float x1 = face.WT(j).P().X()/pdx;
                float y1 = (1.0f-face.WT(j).P().Y())/pdy;

                painter.drawLine(x0, y0, x1, y1);
            }
        }

        sum += (100.0 * areausage) / ((double)ret.width() * ret.height());
        n++;

        std::cout << "Curr average: " << sum / n << std::endl;

        std::cout << "area usage: " << areausage << std::endl;
        std::cout << "texture area: " << ret.width() * ret.height() << std::endl;
        std::cout << "Percentage of used area: " << (100.0 * areausage) / ((double)ret.width() * ret.height()) << std::endl;

        return ret;
    }

    void TextureExtractor::Defragment(const std::vector<QImage>& toDefrag, TMesh& mesh, Defrag::Mesh& defragMesh, Defrag::AlgoParameters& algoParams)
    {
        PROFILE_SCOPE("DefragmentTextures");

        // Create TextureObject
        m_Textures = std::make_shared<Defrag::TextureObject>();
        for (auto& img : toDefrag)
            m_Textures->AddImage(img);

        // Create Defrag::Mesh
        {
            PROFILE_SCOPE("CopyMesh");
            // build mesh object
            auto fi = tri::Allocator<Defrag::Mesh>::AddFaces(defragMesh, mesh.FN());
            auto vi = tri::Allocator<Defrag::Mesh>::AddVertices(defragMesh, mesh.VN());

            for (int i = 0; i < mesh.vert.size(); ++i) {
                vi->P().X() = mesh.vert[i].P().X();
                vi->P().Y() = mesh.vert[i].P().Y();
                vi->P().Z() = mesh.vert[i].P().Z();
                ++vi;
            }

            for (int i = 0; i < mesh.face.size(); ++i) {
                defragMesh.face[i].node = mesh.face[i].node;

                for (int k = 0; k < 3; ++k) {
                    fi->V(k) = &defragMesh.vert[mesh.face[i].cV(k) - &(*mesh.vert.begin())];
                    fi->WT(k).U() = mesh.face[i].cWT(k).U();
                    fi->WT(k).V() = mesh.face[i].cWT(k).V();

                    if (m_Level != 0)
                        fi->WT(k).N() = m_FaceTexToPatchTex[m_Patches[m_Nodes[mesh.face[i].node].first_patch].texture];
                }
                ++fi;
            }

            for (auto& f : defragMesh.face)
                f.SetMesh();
        }

        // Clean mesh
        {
            PROFILE_SCOPE("PrepareMesh");
            tri::UpdateTopology<Defrag::Mesh>::FaceFace(defragMesh);
            tri::UpdateNormal<Defrag::Mesh>::PerFaceNormalized(defragMesh);
            tri::UpdateNormal<Defrag::Mesh>::PerVertexNormalized(defragMesh);

            Defrag::ScaleTextureCoordinatesToImage(defragMesh, m_Textures);

            // Prepare mesh
            int vndupIn;
            Defrag::PrepareMesh(defragMesh, &vndupIn);
            Defrag::ComputeWedgeTexCoordStorageAttribute(defragMesh);
        }

        // After this function is called, graph holds a reference to the textureObject
        {
            PROFILE_SCOPE("CreateGrap");
            m_Graph = Defrag::ComputeGraph(defragMesh, m_Textures);

            for (auto& c : m_Graph->charts)
                m_RegionFlipped[c.first] = c.second->UVFlipped();

            // ensure all charts are oriented coherently, and then store the wtc attribute
            Defrag::ReorientCharts(m_Graph);
        }

        {
            PROFILE_SCOPE("ExecuteAlgorithm");

            Defrag::AlgoStateHandle state = InitializeState(m_Graph, algoParams);

            Defrag::GreedyOptimization(m_Graph, state, algoParams);
            int vndupOut;
            Defrag::Finalize(m_Graph, &vndupOut);

            // Rotate charts
            for (auto entry : m_Graph->charts) {
                Defrag::ChartHandle chart = entry.second;
                double zeroResamplingChartArea;
                int anchor = Defrag::RotateChartForResampling(chart, state->changeSet, m_RegionFlipped, false, &zeroResamplingChartArea);
                if (anchor != -1) {
                    m_AnchorMap[chart] = anchor;
                }
            }
        }
    }

    std::vector<Defrag::TextureSize> TextureExtractor::Pack(Defrag::Mesh& defragMesh)
    {
        // Pack the atlas
        std::vector<Defrag::TextureSize> texszVec;
        std::vector<Defrag::ChartHandle> chartsToPack;
        {
            PROFILE_SCOPE("AtlasPacking");
            for (auto& entry : m_Graph->charts) {
                if (entry.second->AreaUV() != 0) {
                    chartsToPack.push_back(entry.second);
                } else {
                    for (auto fptr : entry.second->fpVec) {
                        for (int j = 0; j < fptr->VN(); ++j) {
                            fptr->V(j)->T().P() = Point2d::Zero();
                            fptr->V(j)->T().N() = 0;
                            fptr->WT(j).P() = Point2d::Zero();
                            fptr->WT(j).N() = 0;
                        }
                    }
                }
            }

            int npacked = Defrag::Pack(chartsToPack, m_Textures, texszVec);

            // Some charts weren't packed
            if (npacked < (int) chartsToPack.size()) {
                cout << "Couldn't pack " << chartsToPack.size() - npacked << " charts" << endl;
            }
        }

        {
            PROFILE_SCOPE("PolishTexture");
            // Trim & shift
            Defrag::TrimTexture(defragMesh, texszVec, false);
            Defrag::IntegerShift(defragMesh, chartsToPack, texszVec, m_AnchorMap, m_RegionFlipped);
        }

        return texszVec;
    }

    QImage TextureExtractor::Render(Defrag::Mesh& defragMesh, std::vector<Defrag::TextureSize>& texszVec)
    {
        std::vector<std::shared_ptr<QImage>> newTextures;
        {
            PROFILE_SCOPE("RenderTexture");
            cout << "Init OpenGL" << endl;
            static bool contextInited = false;
            // Create dummy OpenGL context
            QOpenGLContext glContext;

            QSurfaceFormat format;
            format.setMajorVersion(3);
            format.setMinorVersion(0);

            glContext.setFormat(format);
            glContext.create();

            QOffscreenSurface surface;
            surface.setFormat(format);
            surface.create();

            glContext.makeCurrent(&surface);
            glContext.supportsThreadedOpenGL();

            // init glew
            glewExperimental = GL_TRUE;
            auto glewInited = glewInit();

            std::cout << "Rendering texture" << std::endl;
            newTextures = Defrag::RenderTexture(defragMesh, m_Textures, texszVec, true, Defrag::RenderMode::Linear);
        }

        return *newTextures[0];
    }
}
