#include "texturerenderer.h"
#include "textureextractor.h"

#include <QOffscreenSurface>
#include <QPainter>
#include <QThread>

#include <iostream>
#include <Instrumentor.h>

using namespace std;

namespace nx
{
    TextureRenderer::TextureRenderer()
    {
        PROFILE_SCOPE("RenderTexture");
        cout << "Init OpenGL" << endl;
        static bool contextInited = false;

        // Create OpenGL context
        QSurfaceFormat format;
        format.setMajorVersion(3);
        format.setMinorVersion(0);

        m_GLContext.setFormat(format);
        m_GLContext.create();

        m_GLSurface.setFormat(format);
        m_GLSurface.blockSignals(true);
        m_GLSurface.create();

        m_GLContext.makeCurrent(&m_GLSurface);

        // init glew
        glewExperimental = GL_TRUE;
        auto glewInited = glewInit();
    }

    bool TextureRenderer::Start(uint32_t toProcess)
    {
        if (m_Jobs.size() > 0)
            std::cout << "FATAL: rendering jobs not finished" << std::endl;
        m_Processed = 0;
        std::cout << "[RENDERER] Tex renderer started" << std::endl;
        while (m_Processed < toProcess)
        {

            TextureRenderer::JobData* job = nullptr;
            {
                QMutexLocker lock(&m_QueueMutex);
                if (m_Jobs.size() == 0)
                    m_Condition.wait(&m_QueueMutex);

                job = m_Jobs.front();
                m_Jobs.pop();
            }

            job->Extractor->Render(job->OutMesh);
            job->Condition.wakeAll();

            std::cout << "[RENDERER] Finished!" << std::endl;
            m_Processed++;
            std::cout << "Processed " << m_Processed << " out of " << toProcess << std::endl;
        }
        std::cout << "finished everything" << std::endl;
        return true;
    }
}

