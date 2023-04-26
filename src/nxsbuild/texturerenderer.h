#ifndef TEXTURERENDERER_H
#define TEXTURERENDERER_H

#include <queue>
#include <QWaitCondition>
#include <QMutexLocker>
#include <QMutex>

#include <GL/glew.h>
#include <QOpenGLContext>
#include <QOffscreenSurface>

namespace nx
{
    class TextureExtractor;
    class TMesh;

    class TextureRenderer
    {
    public:
        struct JobData
        {
            TextureExtractor* Extractor;
            TMesh* OutMesh;
            bool Finished = false;
        };

        TextureRenderer();
        TextureRenderer(const TextureRenderer& other) = default;

        void Start(uint32_t toProcess);

        inline void AddJob(JobData* data)
        {
            {
                QMutexLocker lock(&m_QueueMutex);
                m_Jobs.push(data);
                m_Condition.wakeOne();
            }
        }

        inline uint32_t ProcessedCount() {return m_Processed;}

    private:
        std::queue<JobData*> m_Jobs;
        uint32_t m_Processed = 0;

        QWaitCondition m_Condition;
        QMutex m_QueueMutex;

        QOpenGLContext m_GLContext;
        QOffscreenSurface m_GLSurface;
    };
}


#endif // TEXTURERENDERER_H
