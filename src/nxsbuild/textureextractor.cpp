#include "textureextractor.h"
#include "texturerenderer.h"

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

#include <Instrumentor.h>

using namespace std;

namespace nx
{
    QImage TextureExtractor::Extract(TMesh& mesh, std::vector<QImage>& textures, float &error, float &pixelXedge,
                                     float& avgUsage, ParametrizationAlgo paramAlgo, PackingAlgo packingAlgo, RenderingAlgo renderingAlgo)
    {
        Defrag::AlgoParameters ap;

        ap.matchingThreshold = 2;
        ap.boundaryTolerance = 0.2f;
        ap.distortionTolerance = 0.5f;
        ap.globalDistortionThreshold = 0.025f;
        ap.UVBorderLengthReduction = 0.0;
        ap.offsetFactor = 5;
        ap.timelimit = 10;

        Parametrize(textures, mesh, ap, paramAlgo);
        std::cout << "Defragmented" << std::endl;

        m_TexSizes = Pack(mesh, packingAlgo);

        TextureRenderer::JobData data;
        data.Extractor = this;
        data.OutMesh = &mesh;

        m_Renderer->AddJob(&data);
        {
            QMutexLocker lock(&data.ConditionMutex);
            data.Condition.wait(&data.ConditionMutex);
        }

        QImage ret = m_FinalImage;

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

        //#define DEBUG_TRIANGLES
#ifdef DEBUG_TRIANGLES
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
#endif
        sum += (100.0 * areausage) / ((double)ret.width() * ret.height());
        n++;
        avgUsage = sum / n;

        std::cout << "Curr average: " << avgUsage << std::endl;

        return ret;
    }

    void TextureExtractor::Parametrize(const std::vector<QImage>& toDefrag, TMesh& mesh,
                                       Defrag::AlgoParameters& algoParams, ParametrizationAlgo algo)
    {
        PROFILE_SCOPE("DefragmentTextures");

        // Create TextureObject
        cout << "N textures: " << toDefrag.size() << endl;
        m_Textures = std::make_shared<Defrag::TextureObject>();
        for (auto& img : toDefrag) {
            m_Textures->AddImage(img);
        }

        // Create Defrag::Mesh
        {
            PROFILE_SCOPE("CopyMesh");
            NxsToDefragMesh(m_DefragMesh, mesh);
        }

        // Clean mesh
        {
            PROFILE_SCOPE("PrepareMesh");
            tri::UpdateTopology<Defrag::Mesh>::FaceFace(m_DefragMesh);
            tri::UpdateNormal<Defrag::Mesh>::PerFaceNormalized(m_DefragMesh);
            tri::UpdateNormal<Defrag::Mesh>::PerVertexNormalized(m_DefragMesh);

            Defrag::ScaleTextureCoordinatesToImage(m_DefragMesh, m_Textures);

            // Prepare mesh
            int vndupIn;
            Defrag::PrepareMesh(m_DefragMesh, &vndupIn);
            Defrag::ComputeWedgeTexCoordStorageAttribute(m_DefragMesh);
        }

        // After this function is called, graph holds a reference to the textureObject
        {
            PROFILE_SCOPE("CreateGrap");
            m_Graph = Defrag::ComputeGraph(m_DefragMesh, m_Textures);

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
        //DefragToNxsMesh(mesh, defragMesh);
    }

    std::pair<int, int> TextureExtractor::Pack(TMesh& mesh, PackingAlgo algo)
    {
        std::vector<Defrag::ChartHandle> chartsToPack;
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

        /*
        float H = 0;
        for (auto chart : chartsToPack)
            H += chart.UVBox().DimY();
        H /= m_Graph->charts.size();

        std::vector<uint32_t> classes(chartsToPack.size());
        for (uint32_t i=0; i<chartsToPack.size(); i++)
        {
            float chartHeight =chartsToPack[i]->UVBox().DimY();
            if (chartHeight < H)
                classes[i] = std::ceil(std::log2(H / chartHeight)) - 1;
            else
                classes[i] = 0;
        }

        uint32_t xSign = 0;
        std::set<uint32_t> xPos = {};
        std::unordered_map<uint32_t, std::vector<uint32_t>> admissibleVerticals;

        for (uint32_t i=0; i<classes.size(); i++)
        {
            for (uint32_t j=0; j<std::pow(2, classes[i])-1; j++)
            {
                admissibleVerticals[classes[j]]
            }
        }

        for (uint32_t i=0; i<chartsToPack.size(); i++)
        {

        }

        std::sort(classes.begin(), classes.end(), [](uint32_t a, uint32_t b){return a > b;});
        */
        cout << "Copied (Pack)" << endl;

        // Pack the atlas
        std::vector<Defrag::TextureSize> texszVec;
        {
            int npacked = Defrag::Pack(chartsToPack, m_Textures, texszVec);
            std::cout << "pack end" << endl;

            // Some charts weren't packed
            if (npacked < (int) chartsToPack.size()) {
                cout << "Couldn't pack " << chartsToPack.size() - npacked << " charts" << endl;
            }
        }

        {
            PROFILE_SCOPE("PolishTexture");
            // Trim & shift
            Defrag::TrimTexture(m_DefragMesh, texszVec, false);
            Defrag::IntegerShift(m_DefragMesh, chartsToPack, texszVec, m_AnchorMap, m_RegionFlipped);
        }

        cout << "Tex size vec: " << texszVec.size() << endl;

        std::pair<int, int> ret = {texszVec[0].w, texszVec[0].h};
        //DefragToNxsMesh(mesh, defragMesh);

        return ret;
    }

    void TextureExtractor::Render(TMesh* mesh)
    {
        std::cout << "Rendering..." << std::endl;
        std::vector<std::shared_ptr<QImage>> newTextures;
        {
            std::vector<Defrag::TextureSize> sizes = {{m_TexSizes.first, m_TexSizes.second}};
            std::cout << "Size: " << m_TexSizes.first << "," << m_TexSizes.second << endl;
            newTextures = Defrag::RenderTexture(m_DefragMesh, m_Textures, sizes, true, Defrag::RenderMode::Linear);
            std::cout << "New texts" << endl;
        }

        DefragToNxsMesh(*mesh, m_DefragMesh);
        m_FinalImage = QImage(*newTextures[0]);

        std::cout << "Rendered" << std::endl;
    }

    void TextureExtractor::DefragToNxsMesh(TMesh& nxs, Defrag::Mesh& defrag)
    {
        assert(defrag.vn == defrag.vert.size() && defrag.vn == defrag.VN());
        assert(defrag.fn == defrag.face.size() && defrag.fn == defrag.FN());

        nxs.face.resize(defrag.FN());
        nxs.fn = defrag.FN();
        nxs.vert.resize(defrag.VN());
        nxs.vn = defrag.VN();

        for (int i = 0; i < defrag.VN(); ++i) {
            nxs.vert[i].P().X() = defrag.vert[i].P().X();
            nxs.vert[i].P().Y() = defrag.vert[i].P().Y();
            nxs.vert[i].P().Z() = defrag.vert[i].P().Z();
        }

        for (int i = 0; i < defrag.FN(); ++i) {
            nxs.face[i].node = defrag.face[i].node;
            nxs.face[i].SetFlags(defrag.face[i].cFlags());
            nxs.face[i].id = defrag.face[i].id;

            for (int k = 0; k < 3; ++k) {
                nxs.face[i].V(k) = &nxs.vert[defrag.face[i].cV(k) - &(*defrag.vert.begin())];
                nxs.face[i].P(k) = defrag.face[i].P(k);

                nxs.face[i].WT(k).U() = defrag.face[i].cWT(k).U();
                nxs.face[i].WT(k).V() = defrag.face[i].cWT(k).V();
                nxs.face[i].WT(k).N() = 0;

                nxs.face[i].V(k)->T().P() = defrag.face[i].V(k)->T().P();
            }
        }
    }

    void TextureExtractor::NxsToDefragMesh(Defrag::Mesh& defrag, TMesh& nxs)
    {
        // build mesh object
        auto fi = tri::Allocator<Defrag::Mesh>::AddFaces(defrag, nxs.FN());
        auto vi = tri::Allocator<Defrag::Mesh>::AddVertices(defrag, nxs.VN());

        for (int i = 0; i < nxs.vert.size(); ++i) {
            vi->P().X() = nxs.vert[i].P().X();
            vi->P().Y() = nxs.vert[i].P().Y();
            vi->P().Z() = nxs.vert[i].P().Z();

            ++vi;
        }

        for (int i = 0; i < nxs.face.size(); ++i) {
            defrag.face[i].node = nxs.face[i].node;

            for (int k = 0; k < 3; ++k) {
                fi->V(k) = &defrag.vert[nxs.face[i].cV(k) - &(*nxs.vert.begin())];
                fi->V(k)->T().P() = nxs.face[i].V(k)->T().P();

                fi->WT(k).U() = nxs.face[i].cWT(k).U();
                fi->WT(k).V() = nxs.face[i].cWT(k).V();
                fi->WT(k).N() = 0;

                if (m_Level != 0)
                    fi->WT(k).N() = m_FaceTexToPatchTex[m_Patches[m_Nodes[nxs.face[i].node].first_patch].texture];
            }
            ++fi;
        }

        for (auto& f : defrag.face)
            f.SetMesh();
    }
}
