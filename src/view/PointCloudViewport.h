#ifndef NEWELL_VIEW_POINTCLOUDVIEWPORT_H
#define NEWELL_VIEW_POINTCLOUDVIEWPORT_H

#include "model/geometry/Point3d.h"

#include <QMatrix4x4>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <QPoint>
#include <QPointF>
#include <QOpenGLWidget>
#include <QRect>
#include <QVector4D>
#include <QVector3D>

#include <cstddef>
#include <optional>
#include <vector>

class QKeyEvent;
class PointCloud;
class PointCloudViewport : public QOpenGLWidget
{
    Q_OBJECT

public:
    explicit PointCloudViewport(QWidget *parent = nullptr);
    ~PointCloudViewport() override;

    void setPointCloud(const PointCloud *pointCloud, bool fitView = true);
    void clearPointCloud();
    void setSelectionModeEnabled(bool enabled);
    void setSelectedPointIndices(const std::vector<std::size_t> &selectedPointIndices);
    void setSelectionInteractionEnabled(bool enabled);

protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

signals:
    void pointPicked(std::size_t pointIndex);
    void areaPointsSelected(const std::vector<std::size_t> &pointIndices);
    void areaPointsDeselected(const std::vector<std::size_t> &pointIndices);

private:
    enum class InteractionMode
    {
        Idle,
        Orbiting,
        Panning,
        BoxSelecting,
        BoxDeselecting,
    };

    void rebuildGeometry(const PointCloud *pointCloud);
    void rebuildSelectedGeometry();
    void rebuildAxisGeometry();
    void uploadPendingPointCloud();
    void uploadPendingSelectedPoints();
    void uploadPendingAxisGeometry();
    void drawAxis();
    void fitViewToPointCloud();
    void drawSelectionOverlay();
    void beginInteraction(InteractionMode mode, const QPoint &position);
    void updateSelectionInteraction(const QPoint &position);
    void resetInteraction();
    [[nodiscard]] bool isSelectionInteractionActive() const;
    [[nodiscard]] bool selectionGesturePassedDragThreshold() const;
    [[nodiscard]] QRect normalizedSelectionRectangle() const;
    [[nodiscard]] std::vector<std::size_t> collectPointsInSelectionArea() const;
    [[nodiscard]] std::optional<QPointF> projectedScreenPoint(const Point3d &point) const;
    [[nodiscard]] QVector3D currentForward() const;
    [[nodiscard]] QVector3D currentTarget() const;
    [[nodiscard]] QVector3D currentEye() const;
    [[nodiscard]] QMatrix4x4 projectionMatrix() const;
    [[nodiscard]] QMatrix4x4 viewMatrix() const;
    [[nodiscard]] float zoomDistanceStep() const;
    [[nodiscard]] std::optional<QVector3D> rayDirectionForScreenPoint(const QPointF &screenPoint) const;
    [[nodiscard]] std::optional<QVector3D> intersectCursorRayWithViewPlane(
        const QPointF &screenPoint,
        const QVector3D &planePoint,
        const QVector3D &planeNormal) const;
    [[nodiscard]] double computeAxisExtent() const;
    [[nodiscard]] QMatrix4x4 modelViewProjectionMatrix() const;
    [[nodiscard]] QVector4D projectPoint(const Point3d &point) const;
    [[nodiscard]] std::optional<std::size_t> pickPointAt(const QPoint &screenPosition) const;

    const PointCloud *pointCloud_ = nullptr;
    std::vector<float> pointData_;
    std::vector<float> selectedPointData_;
    std::vector<float> axisData_;
    std::size_t pointCount_ = 0;
    std::size_t selectedPointCount_ = 0;
    std::vector<std::size_t> selectedPointIndices_;

    bool glInitialized_ = false;
    bool pointDataDirty_ = false;
    bool selectedPointDataDirty_ = false;
    bool axisDataDirty_ = false;
    bool selectionModeEnabled_ = false;
    bool selectionInteractionEnabled_ = true;
    InteractionMode interactionMode_ = InteractionMode::Idle;

    QOpenGLShaderProgram shaderProgram_;
    QOpenGLBuffer vertexBuffer_{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer selectedVertexBuffer_{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer axisVertexBuffer_{QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject vertexArrayObject_;
    QOpenGLVertexArrayObject selectedVertexArrayObject_;
    QOpenGLVertexArrayObject axisVertexArrayObject_;

    QVector3D cloudCenter_{0.0f, 0.0f, 0.0f};
    float cloudRadius_ = 1.0f;
    float yawDegrees_ = 0.0f;
    float pitchDegrees_ = 0.0f;
    float zoomDistance_ = 3.0f;
    QVector3D panOffset_{0.0f, 0.0f, 0.0f};
    QPoint lastMousePosition_;
    QPoint selectionStartPosition_;
    QPoint selectionCurrentPosition_;
};

#endif // NEWELL_VIEW_POINTCLOUDVIEWPORT_H
