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
        TextureExtractor(const std::vector<Patch> patches, const std::vector<Node> nodes,
            uint32_t level, std::unordered_map<int, int>& faceToPatchTexture) :
            m_Level(level), m_FaceTexToPatchTex(faceToPatchTexture), m_Patches(patches), m_Nodes(nodes){}


        QImage Extract(TMesh& mesh, std::vector<QImage>& toDefrag, float &error, float &pixelXedge);

    private:
        void Defragment(const std::vector<QImage>& toDefrag, TMesh& mesh, Defrag::Mesh& defragMesh, Defrag::AlgoParameters& algoParams);

        std::vector<Defrag::TextureSize> Pack(Defrag::Mesh& defragMesh);

        QImage Render(Defrag::Mesh& mesh, std::vector<Defrag::TextureSize>& texSize);

    private:
        uint32_t m_Level;

        std::vector<Patch> m_Patches;
        std::vector<Node> m_Nodes;

        Defrag::GraphHandle m_Graph;
        Defrag::TextureObjectHandle m_Textures;

        std::map<Defrag::ChartHandle, int> m_AnchorMap;
        std::map<Defrag::RegionID, bool> m_RegionFlipped;
        std::unordered_map<int, int> m_FaceTexToPatchTex;
    };
}


#endif // TEXTUREEXTRACTOR_H
