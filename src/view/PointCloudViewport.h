#ifndef NEWELL_VIEW_POINTCLOUDVIEWPORT_H
#define NEWELL_VIEW_POINTCLOUDVIEWPORT_H

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLWidget>
#include <QPoint>
#include <QVector3D>

#include <cstddef>
#include <vector>

class PointCloud;

class PointCloudViewport : public QOpenGLWidget
{
public:
    explicit PointCloudViewport(QWidget *parent = nullptr);
    ~PointCloudViewport() override;

    void setPointCloud(const PointCloud *pointCloud);
    void clearPointCloud();

protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void rebuildGeometry(const PointCloud *pointCloud);
    void rebuildAxisGeometry();
    void uploadPendingPointCloud();
    void uploadPendingAxisGeometry();
    void drawAxis();
    void fitViewToPointCloud();
    [[nodiscard]] double computeAxisExtent() const;
    [[nodiscard]] QMatrix4x4 modelViewProjectionMatrix() const;

    const PointCloud *pointCloud_ = nullptr;
    std::vector<float> pointData_;
    std::vector<float> axisData_;
    std::size_t pointCount_ = 0;

    bool glInitialized_ = false;
    bool pointDataDirty_ = false;
    bool axisDataDirty_ = false;

    QOpenGLShaderProgram shaderProgram_;
    QOpenGLBuffer vertexBuffer_{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer axisVertexBuffer_{QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject vertexArrayObject_;
    QOpenGLVertexArrayObject axisVertexArrayObject_;

    QVector3D cloudCenter_{0.0f, 0.0f, 0.0f};
    float cloudRadius_ = 1.0f;
    float yawDegrees_ = 0.0f;
    float pitchDegrees_ = 0.0f;
    float zoomDistance_ = 3.0f;
    QVector3D panOffset_{0.0f, 0.0f, 0.0f};
    QPoint lastMousePosition_;
};

#endif // NEWELL_VIEW_POINTCLOUDVIEWPORT_H
