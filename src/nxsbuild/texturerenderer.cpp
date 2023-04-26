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

    void TextureRenderer::Start(uint32_t toProcess)
    {
        while (m_Processed < toProcess)
        {
            TextureRenderer::JobData* job;
            {
                QMutexLocker lock(&m_QueueMutex);
                m_Condition.wait(&m_QueueMutex);
            }

            {
                QMutexLocker lock(&m_QueueMutex);
                if (m_Jobs.size() > 0)
                {
                    job = m_Jobs.back();
                    m_Jobs.pop();
                }
            }

            m_GLContext.moveToThread(QThread::currentThread());
            m_GLContext.makeCurrent(&m_GLSurface);

            job->Extractor->Render(job->OutMesh);
            std::cout << "RENDERING FINISHED!" << std::endl;
            job->Finished = true;

            m_Processed++;
        }

        m_Processed = 0;
    }
}

