/*
Nexus

Copyright(C) 2012 - Federico Ponchio
ISTI - Italian National Research Council - Visual Computing Lab

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License (http://www.gnu.org/licenses/gpl.txt)
for more details.
*/
#include <QDebug>
#include <QThreadPool>
#include <QRunnable>
#include <QFileInfo>
#include <QPainter>
#include <QImage>
#include <QDir>
#include <QImageWriter>
#include <GL/glew.h>
#include <QOpenGLWidget>
#include "vertex_cache_optimizer.h"

#include "nexusbuilder.h"
#include "kdtree.h"
#include "meshstream.h"
// Nexus meshes
#include "mesh.h"
#include "tmesh.h"

#include <texture-defrag/defrag_mesh.h>
#include <texture-defrag/texture_object.h>
#include <vcg/math/similarity2.h>
#include <vcg/space/rect_packer.h>

#include "../texture-defrag/src/texture_object.h"
#include "../texture-defrag/src/texture_optimization.h"
#include "../texture-defrag/src/packing.h"
#include "../texture-defrag/src/utils.h"
#include "../texture-defrag/src/mesh_attribute.h"
#include "../texture-defrag/src/seam_remover.h"
#include "../texture-defrag/src/texture_rendering.h"

#include <QApplication>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <iostream>

#define NXS_FORMAT_VERSION  3

using namespace std;

namespace nx
{
    static qint64 pad(qint64 s) {
        const qint64 padding = NEXUS_PADDING;
        qint64 m = (s-1) & ~(padding -1);
        return m + padding;
    }

    unsigned int nextPowerOf2 ( unsigned int n )
    {
        unsigned count = 0;

        // First n in the below condition
        // is for the case where n is 0
        if (n && !(n & (n - 1)))
            return n;

        while( n != 0)
        {
            n >>= 1;
            count += 1;
        }

        return 1 << count;
    }


    NodeBox::NodeBox(KDTree *tree, uint32_t block) {
        for(int k = 0; k < 3; k++)
            axes[k] = tree->axes[k];
        box = tree->block_boxes[block];
    }

    bool NodeBox::isIn(vcg::Point3f &p) {
        return KDTree::isIn(axes, box, p);
    }

    vector<bool> NodeBox::markBorders(Node &node, vcg::Point3f *p, uint16_t *f) {
        vector<bool> border(node.nvert, false);
        for(int i = 0; i < node.nface; i++) {
            bool outside = false;
            for(int k = 0; k < 3; k++) {
                uint16_t index = f[i*3 + k];
                outside |= !isIn(p[index]);
            }
            if(outside)
                for(int k = 0; k < 3; k++) {
                    uint16_t index = f[i*3 + k];
                    border[index] = true;
                }
        }
        return border;
    }

    NexusBuilder::NexusBuilder(quint32 components): chunks("cache_chunks"), scaling(0.5), useNodeTex(true), tex_quality(92), nodeTex("cache_tex") {

        Signature &signature = header.signature;
        signature.vertex.setComponent(VertexElement::COORD, Attribute(Attribute::FLOAT, 3));
        if(components & FACES)     //ignore normals for meshes
            signature.face.setComponent(FaceElement::INDEX, Attribute(Attribute::UNSIGNED_SHORT, 3));
        if(components & NORMALS)
            signature.vertex.setComponent(VertexElement::NORM, Attribute(Attribute::SHORT, 3));
        if(components & COLORS)
            signature.vertex.setComponent(VertexElement::COLOR, Attribute(Attribute::BYTE, 4));
        if(components & TEXTURES)
            signature.vertex.setComponent(FaceElement::TEX, Attribute(Attribute::FLOAT, 2));

        header.version = NXS_FORMAT_VERSION;
        header.signature = signature;
        header.nvert = header.nface = header.n_nodes = header.n_patches = header.n_textures = 0;

        nodeTex.open();
    }

    NexusBuilder::NexusBuilder(Signature &signature): chunks("cache_chunks"), scaling(0.5) {
        header.version = NXS_FORMAT_VERSION;
        header.signature = signature;
        header.nvert = header.nface = header.n_nodes = header.n_patches = header.n_textures = 0;
    }

    void NexusBuilder::initAtlas(const std::vector<QImage>& textures) {
        if(textures.size()) {
            atlas.addTextures(textures);
        }
    }

    bool NexusBuilder::initAtlas(std::vector<LoadTexture> &textures) {
        if(textures.size()) {
            bool success = atlas.addTextures(textures);
            if(!success)
                return false;
        }
        return true;
    }

    void NexusBuilder::create(KDTree *tree, Stream *stream, uint top_node_size) {
        Node sink;
        sink.first_patch = 0;
        nodes.push_back(sink);

        int level = 0;
        int last_top_level_size = 0;
        do {
            cout << "Creating level " << level << endl;
            tree->clear();
            if(level % 2) tree->setAxesDiagonal();
            else tree->setAxesOrthogonal();

            tree->load(stream);
            stream->clear();

            createLevel(tree, stream, level);
            level++;
            if(skipSimplifyLevels <= 0 && last_top_level_size != 0 && stream->size()/(float)last_top_level_size > 0.9f) {
                cout << "Stream: " << stream->size() << " Last top level size: " << last_top_level_size << endl;
                cout << "Larger top level, most probably to high parametrization fragmentation.\n";
                break;
            }
            last_top_level_size = stream->size();
            skipSimplifyLevels--;
        } while(stream->size() > top_node_size);

        reverseDag();
        saturate();
    }

    class UnionFind {
    public:
        std::vector<int> parents;
        void init(int size) {
            parents.resize(size);
            for(int i = 0; i < size; i++)
                parents[i] = i;
        }

        int root(int p) {
            while(p != parents[p])
                p = parents[p] = parents[parents[p]];
            return p;
        }
        void link(int p0, int p1) {
            int r0 = root(p0);
            int r1 = root(p1);
            parents[r1] = r0;
        }
        int compact(std::vector<int> &node_component) { //change numbering of the connected components, return number
            node_component.resize(parents.size());
            std::map<int, int> remap;// inser root here and order them.
            for(size_t i = 0; i < parents.size(); i++) {
                int root = i;
                while(root != parents[root])
                    root = parents[root];
                parents[i] = root;
                node_component[i] = remap.emplace(root, remap.size()).first->second;
            }
            return remap.size();
        }

    };

    std::vector<QImage> NexusBuilder::extractNodeTex(TMesh &mesh, std::vector<QImage>& toDefrag, int level, float &error, float &pixelXedge) {
        cout << "Extracting textures" << endl;
        // Set default parameters for defragmenter
        Defrag::AlgoParameters ap;

        ap.matchingThreshold = 2;
        ap.boundaryTolerance = 0.2f;
        ap.distortionTolerance = 0.5f;
        ap.globalDistortionThreshold = 0.025f;
        ap.UVBorderLengthReduction = 0.0;
        ap.offsetFactor = 5;
        ap.timelimit = 10;

        // Finally create TextureObject
        Defrag::TextureObjectHandle textureObject = std::make_shared<Defrag::TextureObject>();
        cout << "N textures: " << toDefrag.size() << endl;

        for (auto& img : toDefrag)
            textureObject->AddImage(img);
        cout << "loaded textures" << endl;

        cout << "Copy mesh" << endl;

        // Create Defrag::Mesh
        Defrag::Mesh defragMesh;
        // build mesh object
        auto fi = tri::Allocator<Defrag::Mesh>::AddFaces(defragMesh, mesh.FN());
        auto vi = tri::Allocator<Defrag::Mesh>::AddVertices(defragMesh, mesh.VN());

        for (int i = 0; i < mesh.VN(); ++i) {
            vi->P().X() = mesh.vert[i].P().X();
            vi->P().Y() = mesh.vert[i].P().Y();
            vi->P().Z() = mesh.vert[i].P().Z();
            ++vi;
        }

        for (int i = 0; i < mesh.FN(); ++i) {
            for (int k = 0; k < 3; ++k) {
                fi->V(k) = &defragMesh.vert[mesh.face[i].cV(k) - &(*mesh.vert.begin())];
                fi->WT(k).U() = mesh.face[i].cWT(k).U();
                fi->WT(k).V() = mesh.face[i].cWT(k).V();
                fi->WT(k).N() = mesh.face[i].cWT(k).N();
            }
            ++fi;
        }

        for (auto& f : defragMesh.face)
            f.SetMesh();        

        // Clean mesh
        tri::UpdateTopology<Defrag::Mesh>::FaceFace(defragMesh);
        tri::UpdateNormal<Defrag::Mesh>::PerFaceNormalized(defragMesh);
        tri::UpdateNormal<Defrag::Mesh>::PerVertexNormalized(defragMesh);

        cout << "Cleaned mesh" << endl;

        Defrag::ScaleTextureCoordinatesToImage(defragMesh, textureObject);

        // Prepare mesh
        int vndupIn;
        Defrag::PrepareMesh(defragMesh, &vndupIn);
        Defrag::ComputeWedgeTexCoordStorageAttribute(defragMesh);

        // After this function is called, graph holds a reference to the textureObject
        Defrag::GraphHandle graph = Defrag::ComputeGraph(defragMesh, textureObject);
        std::map<Defrag::RegionID, bool> flipped;
        for (auto& c : graph->charts)
            flipped[c.first] = c.second->UVFlipped();

        // ensure all charts are oriented coherently, and then store the wtc attribute
        Defrag::ReorientCharts(graph);

        std::map<Defrag::ChartHandle, int> anchorMap;
        Defrag::AlgoStateHandle state = InitializeState(graph, ap);

        Defrag::GreedyOptimization(graph, state, ap);
        int vndupOut;
        Defrag::Finalize(graph, &vndupOut);

        /* [USELESS?]
        bool colorize = true;

        if (colorize)
            tri::UpdateColor<Defrag::Mesh>::PerFaceConstant(m, vcg::Color4b(91, 130, 200, 255));
        */

        // Rotate charts
        for (auto entry : graph->charts) {
            Defrag::ChartHandle chart = entry.second;
            double zeroResamplingChartArea;
            int anchor = Defrag::RotateChartForResampling(chart, state->changeSet, flipped, false, &zeroResamplingChartArea);
            if (anchor != -1) {
                anchorMap[chart] = anchor;
            }
        }

        // Pack the atlas
        // Discard zero-area charts
        std::vector<Defrag::ChartHandle> chartsToPack;
        for (auto& entry : graph->charts) {
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

        std::vector<Defrag::TextureSize> texszVec;
        int npacked = Defrag::Pack(chartsToPack, textureObject, texszVec);

        // Some charts weren't packed
        if (npacked < (int) chartsToPack.size()) {
            cout << "Couldn't pack " << chartsToPack.size() - npacked << " charts" << endl;
        }

        cout << "Trim & shift" << endl;
        // Trim & shift
        Defrag::TrimTexture(defragMesh, texszVec, false);
        Defrag::IntegerShift(defragMesh, chartsToPack, texszVec, anchorMap, flipped);

        cout << "Init OpenGL" << endl;
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
        std::vector<std::shared_ptr<QImage>> newTextures = Defrag::RenderTexture(defragMesh, textureObject, texszVec, true,
            Defrag::RenderMode::Linear);

        cout << "Rendered" << endl;

        mesh.face.resize(defragMesh.FN());
        mesh.fn = defragMesh.FN();
        mesh.vert.resize(defragMesh.VN());
        mesh.vn = defragMesh.VN();

        cout << "Resized buffers" << endl;

        for (int i = 0; i < defragMesh.VN(); ++i) {
            mesh.vert[i].P().X() = defragMesh.vert[i].P().X();
            mesh.vert[i].P().Y() = defragMesh.vert[i].P().Y();
            mesh.vert[i].P().Z() = defragMesh.vert[i].P().Z();
        }

        cout << "Copied vertices" << endl;

        for (int i = 0; i < defragMesh.FN(); ++i) {
            for (int k = 0; k < 3; ++k) {
                mesh.face[i].V(k) = &mesh.vert[defragMesh.face[i].cV(k) - &(*defragMesh.vert.begin())];
                mesh.face[i].P(k) = defragMesh.face[i].P(k);

                mesh.face[i].WT(k).U() = defragMesh.face[i].cWT(k).U();
                mesh.face[i].WT(k).V() = defragMesh.face[i].cWT(k).V();
                mesh.face[i].WT(k).N() = defragMesh.face[i].cWT(k).N();
            }
        }

        cout << "Copied faces" << endl;
        std::vector<QImage> ret;
        for (uint32_t i=0; i<newTextures.size(); i++)
            ret.push_back(*newTextures[i]);
        return ret;
    }

    void NexusBuilder::createCloudLevel(KDTreeCloud *input, StreamCloud *output, int level) {

        for(uint block = 0; block < input->nBlocks(); block++) {
            Cloud cloud = input->get(block);
            assert(cloud.size() < (1<<16));
            if(cloud.size() == 0) continue;

            Mesh mesh;
            mesh.load(cloud);

            int target_points = cloud.size()*scaling;
            std::vector<AVertex> deleted = mesh.simplifyCloud(target_points);

            //save node in nexus temporary structure
            quint32 mesh_size = mesh.serializedSize(header.signature);
            mesh_size = pad(mesh_size);
            quint32 chunk = chunks.addChunk(mesh_size);
            uchar *buffer = chunks.getChunk(chunk);
            quint32 patch_offset = patches.size();

            std::vector<Patch> node_patches;
            mesh.serialize(buffer, header.signature, node_patches);

            //patches will be reverted later, but the local order is important because of triangle_offset
            std::reverse(node_patches.begin(), node_patches.end());
            patches.insert(patches.end(), node_patches.begin(), node_patches.end());

            quint32 current_node = nodes.size();
            nx::Node node = mesh.getNode();
            node.offset = chunk; //temporaryle remember which chunk belongs to which node
            node.error = mesh.averageDistance();
            node.first_patch = patch_offset;
            nodes.push_back(node);
            boxes.push_back(NodeBox(input, block));

            //we pick the deleted vertices from simplification and reprocess them.

            swap(mesh.vert, deleted);
            mesh.vn = mesh.vert.size();

            Splat *vertices = new Splat[mesh.vn];
            mesh.getVertices(vertices, current_node);

            for(int i = 0; i < mesh.vn; i++) {
                Splat &s = vertices[i];
                output->pushVertex(s);
            }

            delete []vertices;
        }
    }



    /*
      Commented because the gain is negligible ( and the code is not correct either, there is
      some problem in saving the trianglws... */


    class Worker: public QRunnable {
    public:
        int level;
        uint block;
        KDTreeSoup *input;
        StreamSoup *output;
        NexusBuilder &builder;

        Worker(NexusBuilder &_builder, KDTreeSoup *in, StreamSoup *out, uint _block, int _level):
            builder(_builder), input(in), output(out), block(_block), level(_level) {}

    protected:
        void run() {
            builder.processBlock(input, output, block, level);
        }
    };


    void NexusBuilder::processBlock(KDTreeSoup *input, StreamSoup *output, uint block, int level) {
        TMesh mesh;
        Mesh mesh1;

        quint32 mesh_size;
        int ntriangles = 0;

        {
            QMutexLocker locker(&m_input);
            Soup soup = input->get(block); //soup is memory allocated by input, lock is needed.
            assert(soup.size() < (1<<16));
            if(soup.size() == 0) return;

            ntriangles = soup.size();
            if(!hasTextures()) {
                mesh1.load(soup);
            } else {
                mesh.load(soup);
            }
        }


        if(!hasTextures()) {
            //no need to mutex the input, it won't change anything.
            input->lock(mesh1, block);
            mesh_size = mesh1.serializedSize(header.signature);

        } else {
            mesh.splitSeams(header.signature);
            mesh_size = mesh.serializedSize(header.signature);

            static int n = 0;
            QString texname = "Loaded" + QString::number(n) +  ".jpg";

            QImageWriter rewriter(texname, "jpg");
            rewriter.setQuality(100);
            if (level == 0)
                rewriter.write(originalTextures[0]);
            mesh.savePlyTex("Loaded" + QString::number(n) + ".ply", texname);
            n++;
        }
        mesh_size = pad(mesh_size);

        uchar *buffer = new uchar[mesh_size];
        std::vector<Patch> node_patches;

        float error;
        float pixelXedge;
        if(!hasTextures()) {
            cout << "Untextured" << endl;
            mesh1.serialize(buffer, header.signature, node_patches);
        } else {
            if(useNodeTex) {
                mesh.createPatch(header.signature, node_patches);

                // Load texture data
                std::vector<QImage> packedTextures;
                if (level == 0) {
                    packedTextures = extractNodeTex(mesh, originalTextures, level, error, pixelXedge);
                }
                else
                {
                    cout << "Textured" << endl;
                    // Get used textures
                    std::vector<int> toDefrag;
                    for (auto& patch : node_patches) {
                        patch.texture = patches[nodes[patch.node].first_patch].texture;
                        toDefrag.push_back(patch.texture);
                    }

                    std::vector<QImage> texImages(toDefrag.size());

                    {
                        QMutexLocker locker(&m_textures);

                        for (uint32_t i=0; i<toDefrag.size(); i++)
                        {
                            Texture tex = textures[toDefrag[i]];
                            uint64_t offset = tex.getBeginOffset();
                            uint64_t endOffset = (i == textures.size() - 1) ? nodeTex.size() : tex.getEndOffset();

                            uint64_t dataSize = endOffset - offset;
                            nodeTex.seek(offset);

                            uint8_t* data;
                            auto bytes = nodeTex.read(dataSize);
                            data = (uint8_t*)bytes.data();
                            texImages[i].loadFromData(data, dataSize);

                            // Convert format if necessary
                            if (texImages[i].format() != QImage::Format_RGB888)
                                texImages[i].convertToFormat(QImage::Format_RGB888);
                        }

                        nodeTex.seek(nodeTex.size());
                    }

                    packedTextures = extractNodeTex(mesh, texImages, level, error, pixelXedge);
                }

                mesh.splitSeams(header.signature);
                mesh.serialize(buffer, header.signature, node_patches);

                Texture t;
                {
                    QMutexLocker locker(&m_textures);
                    t.offset = nodeTex.size()/NEXUS_PADDING;

                    output_pixels += packedTexture.width()*packedTexture.height();

                    QImageWriter writer(&nodeTex, "jpg");
                    writer.setQuality(tex_quality);
    #if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
                    writer.setOptimizedWrite(true);
                    writer.setProgressiveScanWrite(true);
    #endif
                    writer.write(packedTexture);

                    quint64 size = pad(nodeTex.size());
                    nodeTex.resize(size);
                    nodeTex.seek(size);
                }
                {
                    QMutexLocker locker(&m_builder);
                    textures.push_back(t);
                    for(Patch &patch: node_patches)
                        patch.texture = textures.size()-1; //last texture inserted
                }

                #define DEBUG_TEXTURES
    #ifdef DEBUG_TEXTURES
                cout << "Saving test meshes and textures" << endl;
                static int counter = 0;

                QString texname = QString::number(counter) + ".jpg";
                QImageWriter rewriter(texname, "jpg");
                rewriter.setQuality(tex_quality);
                rewriter.write(packedTexture);

                mesh.textures.push_back(texname.toStdString());
                mesh.savePlyTex(QString::number(counter) + ".ply", texname);
                counter++;
    #endif
            }

            //VICIUOS TRICK: we could save only a texture every 2 geometry levels since the patch is contained also to a parent node.
            //we could store the texture in the parent nodes i and have it good also for the children node.
            //but only starting from the bottom.

        }
        quint32 chunk;
        //done serializing, move the data to the chunk.
        {
            QMutexLocker locker(&m_chunks);
            chunk = chunks.addChunk(mesh_size);
            uchar *chunk_buffer = chunks.getChunk(chunk);
            memcpy(chunk_buffer, buffer, mesh_size);
            chunks.dropChunk(chunk); //no neede anymore
        }
        delete []buffer;

        nx::Node node;
        if(!hasTextures())
            node = mesh1.getNode(); //get node data before simplification
        else
            node = mesh.getNode();

        int nface;
        {
            if(!hasTextures()) {
                mesh1.lockVertices();
                { //needed only if Mesh::QUADRICS
                    QMutexLocker locker(&m_texsimply);
                    mesh1.quadricInit();
                }
                error = mesh1.simplify(ntriangles*scaling, Mesh::QUADRICS);
                nface = mesh1.fn;

            } else {
                QMutexLocker locker(&m_texsimply);
                int nvert = ntriangles*scaling;

                if(skipSimplifyLevels > 0)
                    nvert = ntriangles;

                else if(nvert < 64) //don't simplify too much!
                    nvert = 64;

                float e = mesh.simplify(nvert, TMesh::QUADRICS);
                if(!useNodeTex)
                    error = e;
                nface = mesh.fn;
            }
        }

        quint32 current_node;
        {
            QMutexLocker locker(&m_builder);

            //patches will be reverted later, but the local order is important because of triangle_offset
            quint32 patch_offset = patches.size();
            std::reverse(node_patches.begin(), node_patches.end());
            patches.insert(patches.end(), node_patches.begin(), node_patches.end());

            current_node = nodes.size();
            node.offset = chunk;          //temporarily remember which chunk belongs to which node
            node.error = error;

            node.first_patch = patch_offset;

            nodes.push_back(node);
            boxes.push_back(NodeBox(input, block));
        }


        //Simplify and stream the meshes
        Triangle *triangles = new Triangle[nface];
        //streaming the output
        if(!hasTextures()) {
            mesh1.getTriangles(triangles, current_node);
        } else {
            mesh.getTriangles(triangles, current_node);
        }

        {
            QMutexLocker locker(&m_output);
            for(int i = 0; i < nface; i++) {
                Triangle &t = triangles[i];
                if(!t.isDegenerate())
                    output->pushTriangle(triangles[i]);
                else
                    cout << "Degenerate" << endl;
            }
        }
        delete []triangles;
    }

    void NexusBuilder::createMeshLevel(KDTreeSoup *input, StreamSoup *output, int level) {
        atlas.buildLevel(level);
        if(level > 0)
            atlas.flush(level-1);


       QThreadPool pool;
        pool.setMaxThreadCount(n_threads);

        for(uint block = 0; block < input->nBlocks(); block++) {
            Worker *worker = new Worker(*this, input, output, block, level);
            pool.start(worker);
        }
        pool.waitForDone();
    }


    void NexusBuilder::createLevel(KDTree *in, Stream *out, int level) {
        KDTreeSoup *isSoup = dynamic_cast<KDTreeSoup *>(in);
        if(!isSoup) {
            KDTreeCloud *input = dynamic_cast<KDTreeCloud *>(in);
            StreamCloud *output = dynamic_cast<StreamCloud *>(out);
            createCloudLevel(input, output, level);
        } else {
            KDTreeSoup *input = dynamic_cast<KDTreeSoup *>(in);
            StreamSoup *output = dynamic_cast<StreamSoup *>(out);
            createMeshLevel(input, output, level);
        }
    }

    void NexusBuilder::saturate() {
        //we do not have the 'backlinks' so we make a depth first traversal
        //TODO! BIG assumption: the nodes are ordered such that child comes always after parent
        for(int node = nodes.size()-2; node >= 0; node--)
            saturateNode(node);

        //nodes.front().error = nodes.front().sphere.Radius()/2;
        nodes.back().error = 0;
    }

    void NexusBuilder::testSaturation() {

        //test saturation:
        for(uint n = 0; n < nodes.size()-1; n++) {
            Node &node = nodes[n];
            vcg::Sphere3f &sphere = node.sphere;
            for(uint p = node.first_patch; p < node.last_patch(); p++) {
                Patch &patch = patches[p];
                Node &child = nodes[patch.node];
                vcg::Sphere3f s = child.sphere;
                float dist = (sphere.Center() - s.Center()).Norm();
                float R = sphere.Radius();
                float r = s.Radius();
                assert(sphere.IsIn(child.sphere));
                assert(child.error < node.error);
            }
        }
    }

    void NexusBuilder::reverseDag() {

        std::reverse(nodes.begin(), nodes.end());
        std::reverse(boxes.begin(), boxes.end());
        std::reverse(patches.begin(), patches.end());

        //first reversal: but we point now to the last_patch, not the first
        for(uint i = 0; i < nodes.size(); i++)
            nodes[i].first_patch = patches.size() -1 - nodes[i].first_patch;

        //get the previous node last +1 to become the first
        for(uint i = nodes.size()-1; i >= 1; i--)
            nodes[i].first_patch = nodes[i-1].first_patch +1;
        nodes[0].first_patch  = 0;

        //and reversed the order of the nodes.
        for(uint i = 0; i < patches.size(); i++) {
            patches[i].node = nodes.size() - 1 - patches[i].node;
        }
    }


    void NexusBuilder::save(QString filename) {

        //cout << "Saving to file " << qPrintable(filename) << endl;
        //cout << "Input squaresize " << sqrt(input_pixels) <<  " Output size " << sqrt(output_pixels) << "\n";

        file.setFileName(filename);
        if(!file.open(QIODevice::ReadWrite | QIODevice::Truncate))
            throw QString("could not open file " + filename);

        if(header.signature.vertex.hasNormals() && header.signature.face.hasIndex())
            uniformNormals();

        if(textures.size())
            textures.push_back(Texture());

        header.nface = 0;
        header.nvert = 0;
        header.n_nodes = nodes.size();
        header.n_patches = patches.size();

        header.n_textures = textures.size();
        header.version = NXS_FORMAT_VERSION;

        //find roots and adjust error
        uint32_t nroots = header.n_nodes;
        for(uint32_t j = 0; j < nroots; j++) {
            for(uint32_t i = nodes[j].first_patch; i < nodes[j].last_patch(); i++)
                if(patches[i].node < nroots)
                    nroots = patches[i].node;
            nodes[j].error = nodes[j].tight_radius;
        }

        header.sphere = vcg::Sphere3f();
        for(uint32_t i = 0; i < nroots; i++)
            header.sphere.Add(nodes[i].tightSphere());

        for(uint i = 0; i < nodes.size()-1; i++) {
            nx::Node &node = nodes[i];
            header.nface += node.nface;
            header.nvert += node.nvert;
        }

        quint64 size = sizeof(Header)  +
                nodes.size()*sizeof(Node) +
                patches.size()*sizeof(Patch) +
                textures.size()*sizeof(Texture);
        size = pad(size);
        quint64 index_size = size;

        std::vector<quint32> node_chunk; //for each node the corresponding chunk
        for(quint32 i = 0; i < nodes.size()-1; i++)
            node_chunk.push_back(nodes[i].offset);

        //compute offsets and store them in nodes
        for(uint i = 0; i < nodes.size()-1; i++) {
            nodes[i].offset = size/NEXUS_PADDING;
            quint32 chunk = node_chunk[i];
            size += chunks.chunkSize(chunk);
        }
        nodes.back().offset = size/NEXUS_PADDING;

        if(textures.size()) {
            if(!useNodeTex) {
                for(uint i = 0; i < textures.size()-1; i++) {
                    quint32 s = textures[i].offset;
                    textures[i].offset = size/NEXUS_PADDING;
                    size += s;
                    size = pad(size);
                }
                textures.back().offset = size/NEXUS_PADDING;

            } else { //texture.offset keeps the index in the nodeTex temporay file (already padded and in NEXUS_PADDING units
                if(header.signature.flags & Signature::Flags::DEEPZOOM) {
                    //just fix the last one
                    textures.back().offset = nodeTex.size()/NEXUS_PADDING;

                } else { //texture.offset holds the size of each texture
                    for(uint i = 0; i < textures.size()-1; i++)
                        textures[i].offset += size/NEXUS_PADDING;
                    size += nodeTex.size();
                    textures.back().offset = size/NEXUS_PADDING;
                }
            }
        }

        qint64 r = file.write((char*)&header, sizeof(Header));
        if(r == -1)
            throw(file.errorString());
        assert(nodes.size());
        file.write((char*)&(nodes[0]), sizeof(Node)*nodes.size());
        if(patches.size())
            file.write((char*)&(patches[0]), sizeof(Patch)*patches.size());
        if(textures.size())
            file.write((char*)&(textures[0]), sizeof(Texture)*textures.size());
        file.seek(index_size);

        //NODES
        QString basename = filename.left(filename.size() - 4) + "_files";
        if(header.signature.flags & Signature::Flags::DEEPZOOM) {
            QDir dir;
            dir.mkdir(basename);
        }
        for(uint i = 0; i < node_chunk.size(); i++) {
            quint32 chunk = node_chunk[i];
            uchar *buffer = chunks.getChunk(chunk);
            optimizeNode(i, buffer);
            if(header.signature.flags & Signature::Flags::DEEPZOOM) {
                QFile nodefile(QString("%1/%2.nxn").arg(basename).arg(i));
                nodefile.open(QFile::WriteOnly);
                nodefile.write((char*)buffer, chunks.chunkSize(chunk));
            } else
                file.write((char*)buffer, chunks.chunkSize(chunk));
        }

        //TEXTURES
        //	QString basename = filename.left(filename.length()-4);
        //compute textures offsetse
        //Image should store all the mipmaps
        if(textures.size()) {
            if(useNodeTex) {
                //todo split into pieces.
                if(header.signature.flags & Signature::Flags::DEEPZOOM) {
                    for(uint i = 0; i < textures.size()-1; i++) {
                        quint32 s = textures[i].offset*NEXUS_PADDING;
                        quint32 size = textures[i+1].offset*NEXUS_PADDING -s;

                        nodeTex.seek(s);
                        auto buffer = nodeTex.read(size);

                        QFile texfile(QString("%1/%2.jpg").arg(basename).arg(i));
                        texfile.open(QFile::WriteOnly);
                        texfile.write(buffer);
                    }
                } else {
                    nodeTex.seek(0);

                    qint64 buffer_size = 64*1<<20; //64 MB
                    do {
                        auto buffer = nodeTex.read(buffer_size);
                        if(!buffer.size())
                            break;

                        bool success = file.write(buffer);
                        if(!success)
                            throw QString("failed writing texture data from temporary file.");
                    } while(1);
                }
            } else {
                for(int i = 0; i < textures.size()-1; i++) {
                    Texture &tex = textures[i];
                    assert(tex.offset == file.pos()/NEXUS_PADDING);
                    QFile image(images[i]);
                    if(!image.open(QFile::ReadOnly))
                        throw QString("could not load img %1").arg(images[i]);
                    bool success = file.write(image.readAll());
                    if(!success)
                        throw QString("failed writing texture %1").arg(images[i]);
                    //we should use texture.offset instead.
                    quint64 s = file.pos();
                    s = pad(s);
                    file.resize(s);
                    file.seek(s);
                }
            }
        }
        if(textures.size())
            for(int i = 0; i < textures.size()-1; i++) {
                QFile::remove(QString("nexus_tmp_tex%1.png").arg(i));
            }
        cout << "Saving to file " << qPrintable(filename) << endl;
        file.close();
    }


    //include sphere of the children and ensure error s bigger.
    void NexusBuilder::saturateNode(quint32 n) {
        const float epsilon = 1.01f;

        nx::Node &node = nodes[n];
        for(quint32 i = node.first_patch; i < node.last_patch(); i++) {
            nx::Patch &patch = patches[i];
            if(patch.node == nodes.size()-1) //sink, get out
                return;

            nx::Node &child = nodes[patch.node];
            if(node.error <= child.error)
                node.error = child.error*epsilon;

            //we cannot just add the sphere, because it moves the center and the tight radius will be wrong
            if(!node.sphere.IsIn(child.sphere)) {
                float dist = (child.sphere.Center() - node.sphere.Center()).Norm();
                dist += child.sphere.Radius();
                if(dist > node.sphere.Radius())
                    node.sphere.Radius() = dist;
            }
        }
        node.sphere.Radius() *= epsilon;
    }

    void NexusBuilder::optimizeNode(quint32 n, uchar *chunk) {
        return;
        Node &node = nodes[n];
        assert(node.nface);

        uint16_t *faces = (uint16_t  *)(chunk + node.nvert*header.signature.vertex.size());

        uint start =  0;
        for(uint i = node.first_patch; i < node.last_patch(); i++) {
            Patch &patch = patches[i];
            uint end = patch.triangle_offset;
            uint nface = end - start;
            //optimizing vertex cache.
            quint16 *triangles = new quint16[nface*3];

            bool success = vmath::vertex_cache_optimizer::optimize_post_tnl(24, faces + 3*start, nface, node.nvert,  triangles);
            if(success)
                memcpy(faces + start, triangles, 3*sizeof(quint16)*nface);
            else
                cout << "Failed cache optimization" << endl;
            delete []triangles;
            start = end;
        }
    }

    /* extracts vertices in origin which intersects destination box */
    void NexusBuilder::appendBorderVertices(uint32_t origin, uint32_t destination, std::vector<NVertex> &vertices) {
        Node &node = nodes[origin];
        uint32_t chunk = node.offset; //chunk index was stored here.


        //if origin != destination do not flush cache, it would invalidate pointers.
        uchar *buffer = chunks.getChunk(chunk, origin != destination);

        vcg::Point3f *point = (vcg::Point3f *)buffer;
        int size = sizeof(vcg::Point3f) +
                header.signature.vertex.hasTextures()*sizeof(vcg::Point2f) +
                header.signature.vertex.hasColors() * sizeof(vcg::Color4b);
        vcg::Point3s *normal = (vcg::Point3s *)(buffer + size * node.nvert);
        uint16_t *face = (uint16_t *)(buffer + header.signature.vertex.size()*node.nvert);

        NodeBox &nodebox = boxes[origin];

        vector<bool> border = nodebox.markBorders(node, point, face);
        for(int i = 0; i < node.nvert; i++) {
            if(border[i])
                vertices.push_back(NVertex(origin, i, point[i], normal + i));
        }
    }


    void NexusBuilder::uniformNormals() {
        cout << "Unifying normals\n";
        /*
        level 0: for each node in the lowest level:
                load the neighboroughs
                find common vertices (use lock to find the vertices)
                average normals

        level > 0: for every other node (goind backwards)
                load child nodes
                find common vertices (use lock, it's way faster!)
                copy normal
        */

        std::vector<NVertex> vertices;

        uint32_t sink = nodes.size()-1;
        for(int t = sink-1; t > 0; t--) {
            Node &target = nodes[t];

            vcg::Box3f box = boxes[t].box;
            //box.Offset(box.Diag()/100);
            box.Offset(box.Diag()/10);

            vertices.clear();
            appendBorderVertices(t, t, vertices);

            bool last_level = (patches[target.first_patch].node == sink);

            if(last_level) {//find neighboroughs among the same level

                for(int n = t-1; n >= 0; n--) {
                    Node &node = nodes[n];
                    if(patches[node.first_patch].node != sink) continue;
                    if(!box.Collide(boxes[n].box)) continue;

                    appendBorderVertices(n, t, vertices);
                }

            } else { //again among childrens.

                for(uint p = target.first_patch; p < target.last_patch(); p++) {
                    uint n = patches[p].node;

                    appendBorderVertices(n, t, vertices);
                }
            }

            if(!vertices.size()) { //this is possible, there might be no border at all.
                continue;
            }

            sort(vertices.begin(), vertices.end());

            uint start = 0;
            while(start < vertices.size()) {
                NVertex &v = vertices[start];

                uint last = start+1;
                while(last < vertices.size() && vertices[last].point == v.point)
                    last++;

                if(last_level && last - start > 1) { //average all normals
                    vcg::Point3f normalf(0, 0, 0);
                    for(uint k = start; k < last; k++) {
                        for(int l = 0; l < 3; l++)
                            normalf[l] += (*vertices[k].normal)[l];
                    }
                    normalf.Normalize();
                    //convert back to shorts
                    vcg::Point3s normals;
                    for(int l = 0; l < 3; l++)
                        normals[l] = (short)(normalf[l]*32766);

                    for(uint k = start; k < last; k++)
                        *vertices[k].normal = normals;

                } else //just copy from first one (coming from lower level due to sorting
                    for(uint k = start; k < last; k++)
                        *vertices[k].normal =*v.normal;

                start = last;
            }


            /*		for(uint k = 0; k < vertices.size(); k++) {
                NVertex &v = vertices[k];
                if(v.point != previous) {
                    //uniform normals;
                    if(k - last > 1) {   //more than 1 vertex to unify

                        if(last_level) {     //average all normals of the coincident points.

                            vcg::Point3f normalf(0, 0, 0);

                            for(uint j = last; j < k; j++)
                                for(int l = 0; l < 3; l++)
                                    normalf[l] += (*vertices[j].normal)[l];
                            normalf.Normalize();

                            //convert back to shorts
                            vcg::Point3s normals;
                            for(int l = 0; l < 3; l++)
                                normals[l] = (short)(normalf[l]*32766);

                            for(uint j = last; j < k; j++)
                                *vertices[j].normal = normals;

                        } else { //just copy the first one (it's from the lowest level, due to sort)
                            if(vertices[k-1].node == t) //unless this vertex do not belongs to t.
                                *vertices[k-1].normal = *vertices[last].normal;
                        }
                    }
                    previous = v.point;
                    last = k;
                }
            }*/
        }
    }
}

