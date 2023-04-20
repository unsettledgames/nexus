#ifndef TEXTUREEXTRACTOR_H
#define TEXTUREEXTRACTOR_H

#include <QImage>
#include <texture-defrag/types.h>
#include "../nexus/src/common/nexusdata.h"
#include <texture-defrag/texture_object.h>
#include <texture-defrag/seam_remover.h>

#include <map>
#include <unordered_map>

namespace nx
{
    class TMesh;

    class TextureExtractor
    {
    public:
        enum class ParametrizationAlgo{Defrag = 0};
        enum class PackingAlgo{Defrag = 0, Tetris};
        enum class RenderingAlgo{Defrag = 0};

        TextureExtractor(const std::vector<Patch> patches, const std::vector<Node> nodes,
            uint32_t level, std::unordered_map<int, int>& faceToPatchTexture) :
            m_Level(level), m_FaceTexToPatchTex(faceToPatchTexture), m_Patches(patches), m_Nodes(nodes){}

        QImage Extract(TMesh& mesh, std::vector<QImage>& toDefrag, float &error, float &pixelXedge, float& avgUsage,
                       ParametrizationAlgo, PackingAlgo packingAlgo, RenderingAlgo renderingAlgo);

    private:
        void Parametrize(const std::vector<QImage>& toDefrag, TMesh& mesh,
                         Defrag::AlgoParameters& algoParams, ParametrizationAlgo algo);
        std::pair<int, int> Pack(TMesh& mesh, PackingAlgo algo);
        QImage Render(TMesh& mesh, std::pair<int, int>& texSize, RenderingAlgo algo);

        void Defragment(TMesh& mesh);

        std::pair<int, int> PackDefrag(TMesh& mesh);
        std::pair<int, int> PackTetris(TMesh& mesh);

        void RenderDefrag(TMesh& mesh);

        void DefragToNxsMesh(TMesh& nxs, Defrag::Mesh& defrag);
        void NxsToDefragMesh(Defrag::Mesh& defrag, TMesh& nxs);

    private:
        uint32_t m_Level;

        std::vector<Patch> m_Patches;
        std::vector<Node> m_Nodes;

        Defrag::GraphHandle m_Graph;
        Defrag::TextureObjectHandle m_Textures;
        Defrag::Mesh m_DefragMesh;

        std::map<Defrag::ChartHandle, int> m_AnchorMap;
        std::map<Defrag::RegionID, bool> m_RegionFlipped;
        std::unordered_map<int, int> m_FaceTexToPatchTex;
    };
}


#endif // TEXTUREEXTRACTOR_H
