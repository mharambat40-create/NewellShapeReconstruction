#include "view/PointCloudViewport.h"

#include "model/geometry/BoundingBox3d.h"
#include "model/geometry/Point3d.h"
#include "model/geometry/PointCloud.h"
#include "model/geometry/TriangleMesh.h"

#include <Eigen/Geometry>

#include <QColor>
#include <QApplication>
#include <QCoreApplication>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QOpenGLExtraFunctions>
#include <QPainter>
#include <QVector4D>
#include <QWheelEvent>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <optional>

namespace
{
constexpr float kDefaultPointSize = 5.0f;
constexpr float kSelectedPointSize = 12.0f;
constexpr float kAxisLineWidth = 1.0f;
constexpr float kMinZoomDistance = 0.05f;
constexpr float kRotationDirection = -1.0f;
constexpr float kRotationSpeed = 0.5f;
constexpr float kPanDirection = 1.0f;
constexpr float kZoomFactor = 0.1f;
constexpr float kPickTolerancePixels = 10.0f;
constexpr float kSelectionOverlayLineWidth = 1.5f;
constexpr int kSelectionClickThresholdPixels = 3;

const QColor kViewportBackgroundColor("#F8F8F8");
const QColor kSelectionOverlayFillColor(36, 92, 230, 38);
const QColor kSelectionOverlayStrokeColor(36, 92, 230, 180);
const QVector3D kPointCloudColor(0.235f, 0.255f, 0.275f); // #3C4146
const QVector3D kSelectedPointColor(0.851f, 0.122f, 0.122f); // #D91F1F
const QVector3D kMeshColor(0.72f, 0.73f, 0.74f);

const QVector3D kAxisXColor(0.851f, 0.122f, 0.122f); // #D91F1F
const QVector3D kAxisYColor(0.122f, 0.678f, 0.180f); // #1FAD2E
const QVector3D kAxisZColor(0.141f, 0.361f, 0.902f); // #245CE6

constexpr float kDefaultAxisLength = 1.0f;
constexpr float kVerticalFieldOfViewDegrees = 45.0f;
constexpr float kDefaultYawDegrees = 135.0f;
constexpr float kDefaultPitchDegrees = -27.9383527f;

QVector3D computeForwardVector(float yawDegrees, float pitchDegrees)
{
    const float yawRadians = qDegreesToRadians(yawDegrees);
    const float pitchRadians = qDegreesToRadians(pitchDegrees);

    return QVector3D(
        std::cos(pitchRadians) * std::cos(yawRadians),
        std::cos(pitchRadians) * std::sin(yawRadians),
        std::sin(pitchRadians));
}

bool hasBoxDeselectionModifier(Qt::KeyboardModifiers modifiers)
{
#ifdef Q_OS_MACOS
    const bool commandMappedToMeta =
        QCoreApplication::testAttribute(Qt::AA_MacDontSwapCtrlAndMeta);
    return commandMappedToMeta
        ? modifiers.testFlag(Qt::MetaModifier)
        : modifiers.testFlag(Qt::ControlModifier);
#else
    return modifiers.testFlag(Qt::MetaModifier) || modifiers.testFlag(Qt::ControlModifier);
#endif
}

bool isCommandKey(Qt::Key key)
{
#ifdef Q_OS_MACOS
    const bool commandMappedToMeta =
        QCoreApplication::testAttribute(Qt::AA_MacDontSwapCtrlAndMeta);
    return commandMappedToMeta ? key == Qt::Key_Meta : key == Qt::Key_Control;
#else
    Q_UNUSED(key)
    return false;
#endif
}
}

PointCloudViewport::PointCloudViewport(QWidget *parent)
    : QOpenGLWidget(parent)
{
    setMinimumSize(480, 320);
    yawDegrees_ = kDefaultYawDegrees;
    pitchDegrees_ = kDefaultPitchDegrees;
    rebuildAxisGeometry();
}

PointCloudViewport::~PointCloudViewport()
{
    makeCurrent();
    vertexArrayObject_.destroy();
    vertexBuffer_.destroy();
    selectedVertexArrayObject_.destroy();
    selectedVertexBuffer_.destroy();
    axisVertexArrayObject_.destroy();
    axisVertexBuffer_.destroy();
    meshVertexArrayObject_.destroy();
    meshVertexBuffer_.destroy();
    doneCurrent();
}

void PointCloudViewport::setPointCloud(const PointCloud *pointCloud, bool fitView)
{
    pointCloud_ = pointCloud;
    rebuildGeometry(pointCloud);
    rebuildSelectedGeometry();
    if (fitView) {
        fitViewToPointCloud();
    }
    rebuildAxisGeometry();
    update();
}

void PointCloudViewport::clearPointCloud()
{
    pointCloud_ = nullptr;
    pointData_.clear();
    pointCount_ = 0;
    selectedPointData_.clear();
    selectedPointIndices_.clear();
    selectedPointCount_ = 0;
    rebuildMeshGeometry(nullptr);
    pointDataDirty_ = true;
    selectedPointDataDirty_ = true;
    cloudCenter_ = QVector3D(0.0f, 0.0f, 0.0f);
    cloudRadius_ = 1.0f;
    yawDegrees_ = kDefaultYawDegrees;
    pitchDegrees_ = kDefaultPitchDegrees;
    zoomDistance_ = 3.0f;
    panOffset_ = QVector3D(0.0f, 0.0f, 0.0f);
    rebuildAxisGeometry();
    update();
}

void PointCloudViewport::setTriangleMesh(const TriangleMesh *triangleMesh)
{
    rebuildMeshGeometry(triangleMesh);
    update();
}

void PointCloudViewport::clearTriangleMesh()
{
    rebuildMeshGeometry(nullptr);
    update();
}

void PointCloudViewport::initializeGL()
{
    auto *gl = context()->extraFunctions();
    gl->initializeOpenGLFunctions();
    gl->glEnable(GL_DEPTH_TEST);
    gl->glEnable(GL_PROGRAM_POINT_SIZE);

    shaderProgram_.addShaderFromSourceCode(
        QOpenGLShader::Vertex,
        R"(#version 330 core
           layout(location = 0) in vec3 position;
           uniform mat4 u_mvp;
           uniform float u_pointSize;
           void main()
           {
               gl_Position = u_mvp * vec4(position, 1.0);
               gl_PointSize = u_pointSize;
           })");

    shaderProgram_.addShaderFromSourceCode(
        QOpenGLShader::Fragment,
        R"(#version 330 core
           out vec4 fragColor;
           uniform vec3 u_color;
           void main()
           {
               fragColor = vec4(u_color, 1.0);
           })");

    shaderProgram_.link();

    meshShaderProgram_.addShaderFromSourceCode(
        QOpenGLShader::Vertex,
        R"(#version 330 core
           layout(location = 0) in vec3 position;
           layout(location = 1) in vec3 normal;
           uniform mat4 u_mvp;
           out vec3 v_normal;
           void main()
           {
               gl_Position = u_mvp * vec4(position, 1.0);
               v_normal = normal;
           })");
    meshShaderProgram_.addShaderFromSourceCode(
        QOpenGLShader::Fragment,
        R"(#version 330 core
           in vec3 v_normal;
           out vec4 fragColor;
           uniform vec3 u_color;
           void main()
           {
               vec3 lightDirection = normalize(vec3(0.35, -0.45, 0.82));
               float diffuse = 0.32 + 0.68 * abs(dot(normalize(v_normal), lightDirection));
               fragColor = vec4(u_color * diffuse, 1.0);
           })");
    meshShaderProgram_.link();

    vertexArrayObject_.create();
    vertexBuffer_.create();
    selectedVertexArrayObject_.create();
    selectedVertexBuffer_.create();
    axisVertexArrayObject_.create();
    axisVertexBuffer_.create();
    meshVertexArrayObject_.create();
    meshVertexBuffer_.create();

    glInitialized_ = true;
    pointDataDirty_ = true;
    selectedPointDataDirty_ = true;
    axisDataDirty_ = true;
    meshDataDirty_ = true;
}

void PointCloudViewport::resizeGL(int width, int height)
{
    Q_UNUSED(width)
    Q_UNUSED(height)
    rebuildAxisGeometry();
}

void PointCloudViewport::paintGL()
{
    auto *gl = context()->extraFunctions();
    gl->glClearColor(
        static_cast<float>(kViewportBackgroundColor.redF()),
        static_cast<float>(kViewportBackgroundColor.greenF()),
        static_cast<float>(kViewportBackgroundColor.blueF()),
        1.0f);
    gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!glInitialized_) {
        return;
    }

    uploadPendingPointCloud();
    uploadPendingSelectedPoints();
    uploadPendingAxisGeometry();
    uploadPendingMesh();

    shaderProgram_.bind();
    shaderProgram_.setUniformValue("u_mvp", modelViewProjectionMatrix());
    drawAxis();
    shaderProgram_.release();

    if (meshVertexCount_ > 0) {
        meshShaderProgram_.bind();
        meshShaderProgram_.setUniformValue("u_mvp", modelViewProjectionMatrix());
        meshShaderProgram_.setUniformValue("u_color", kMeshColor);
        meshVertexArrayObject_.bind();
        gl->glDrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(meshVertexCount_));
        meshVertexArrayObject_.release();
        meshShaderProgram_.release();
    }

    if (pointCount_ > 0) {
        shaderProgram_.bind();
        shaderProgram_.setUniformValue("u_mvp", modelViewProjectionMatrix());
        shaderProgram_.setUniformValue("u_pointSize", kDefaultPointSize);
        shaderProgram_.setUniformValue("u_color", kPointCloudColor);

        gl->glDepthFunc(GL_LEQUAL);
        vertexArrayObject_.bind();
        gl->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(pointCount_));
        vertexArrayObject_.release();

        if (selectedPointCount_ > 0) {
            shaderProgram_.setUniformValue("u_pointSize", kSelectedPointSize);
            shaderProgram_.setUniformValue("u_color", kSelectedPointColor);
            selectedVertexArrayObject_.bind();
            gl->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(selectedPointCount_));
            selectedVertexArrayObject_.release();
        }
        gl->glDepthFunc(GL_LESS);
        shaderProgram_.release();
    }

    drawSelectionOverlay();
}

void PointCloudViewport::keyPressEvent(QKeyEvent *event)
{
#ifdef Q_OS_MACOS
    if (isCommandKey(static_cast<Qt::Key>(event->key()))) {
        event->accept();
        return;
    }
#endif

    QOpenGLWidget::keyPressEvent(event);
}

void PointCloudViewport::keyReleaseEvent(QKeyEvent *event)
{
#ifdef Q_OS_MACOS
    if (isCommandKey(static_cast<Qt::Key>(event->key()))) {
        event->accept();
        return;
    }
#endif

    QOpenGLWidget::keyReleaseEvent(event);
}

void PointCloudViewport::mousePressEvent(QMouseEvent *event)
{
    const Qt::KeyboardModifiers modifiers = event->modifiers() | QGuiApplication::keyboardModifiers();
    lastMousePosition_ = event->pos();
    if (selectionModeEnabled_ &&
        selectionInteractionEnabled_ &&
        event->button() == Qt::LeftButton) {
        beginInteraction(
            hasBoxDeselectionModifier(modifiers) ? InteractionMode::BoxDeselecting
                                                 : InteractionMode::BoxSelecting,
            event->pos());
        event->accept();
        return;
    }

    if (event->button() == Qt::RightButton) {
        beginInteraction(InteractionMode::Orbiting, event->pos());
        event->accept();
        return;
    }

    if (event->button() == Qt::MiddleButton) {
        beginInteraction(InteractionMode::Panning, event->pos());
        event->accept();
        return;
    }

#ifdef Q_OS_MACOS
    if (event->button() == Qt::LeftButton) {
        beginInteraction(InteractionMode::PendingMacPanning, event->pos());
        event->accept();
        return;
    }
#endif

    QOpenGLWidget::mousePressEvent(event);
}

void PointCloudViewport::mouseMoveEvent(QMouseEvent *event)
{
    const QPoint delta = event->pos() - lastMousePosition_;
    lastMousePosition_ = event->pos();

    if (interactionMode_ == InteractionMode::BoxSelecting ||
        interactionMode_ == InteractionMode::BoxDeselecting) {
        updateSelectionInteraction(event->pos());
        event->accept();
        return;
    }

    if (interactionMode_ == InteractionMode::Orbiting) {
        yawDegrees_ += static_cast<float>(delta.x()) * kRotationSpeed * kRotationDirection;
        pitchDegrees_ += static_cast<float>(delta.y()) * kRotationSpeed * kRotationDirection;
        pitchDegrees_ = std::clamp(pitchDegrees_, -89.0f, 89.0f);
        event->accept();
        update();
        return;
    }

#ifdef Q_OS_MACOS
    if (interactionMode_ == InteractionMode::PendingMacPanning) {
        if (!panGesturePassedDragThreshold(event->pos())) {
            event->accept();
            return;
        }

        interactionMode_ = InteractionMode::Panning;
        panningCursorActive_ = true;
        setCursor(Qt::ClosedHandCursor);
        panCameraByScreenDelta(event->pos() - interactionStartPosition_);
        event->accept();
        return;
    }
#endif

    if (interactionMode_ == InteractionMode::Panning) {
        panCameraByScreenDelta(delta);
        event->accept();
        return;
    }

    QOpenGLWidget::mouseMoveEvent(event);
}

void PointCloudViewport::mouseReleaseEvent(QMouseEvent *event)
{
    if (interactionMode_ == InteractionMode::BoxSelecting &&
        event->button() == Qt::LeftButton) {
        updateSelectionInteraction(event->pos());
        const bool passedDragThreshold = selectionGesturePassedDragThreshold();
        const std::vector<std::size_t> pointIndices =
            passedDragThreshold ? collectPointsInSelectionArea() : std::vector<std::size_t>{};
        resetInteraction();
        event->accept();

        if (!passedDragThreshold) {
            if (selectionModeEnabled_) {
                if (const auto pickedPointIndex = pickPointAt(event->pos())) {
                    emit pointPicked(*pickedPointIndex);
                }
            }
            return;
        }

        emit areaPointsSelected(pointIndices);
        return;
    }

    if (interactionMode_ == InteractionMode::BoxDeselecting &&
        event->button() == Qt::LeftButton) {
        updateSelectionInteraction(event->pos());
        const std::vector<std::size_t> pointIndices =
            selectionGesturePassedDragThreshold() ? collectPointsInSelectionArea()
                                                  : std::vector<std::size_t>{};
        resetInteraction();
        event->accept();
        if (!pointIndices.empty()) {
            emit areaPointsDeselected(pointIndices);
        }
        return;
    }

    if (interactionMode_ == InteractionMode::Orbiting &&
        event->button() == Qt::RightButton) {
        resetInteraction();
        event->accept();
        return;
    }

    if (interactionMode_ == InteractionMode::Panning &&
        event->button() == Qt::MiddleButton) {
        resetInteraction();
        event->accept();
        return;
    }

#ifdef Q_OS_MACOS
    if ((interactionMode_ == InteractionMode::PendingMacPanning ||
         interactionMode_ == InteractionMode::Panning) &&
        event->button() == Qt::LeftButton) {
        resetInteraction();
        event->accept();
        return;
    }
#endif

    QOpenGLWidget::mouseReleaseEvent(event);
}

void PointCloudViewport::wheelEvent(QWheelEvent *event)
{
    const QPoint angleDelta = event->angleDelta();
    if (angleDelta.y() == 0) {
        return;
    }

    const QVector3D targetBefore = currentTarget();
    const QVector3D forwardBefore = currentForward();
    const std::optional<QVector3D> anchorBefore = intersectCursorRayWithViewPlane(
        event->position(),
        targetBefore,
        forwardBefore);

    const float wheelSteps = static_cast<float>(angleDelta.y()) / 120.0f;
    zoomDistance_ = std::max(kMinZoomDistance, zoomDistance_ - wheelSteps * zoomDistanceStep());

    if (anchorBefore) {
        if (const std::optional<QVector3D> anchorAfter = intersectCursorRayWithViewPlane(
                event->position(),
                targetBefore,
                forwardBefore)) {
            panOffset_ += *anchorBefore - *anchorAfter;
        }
    }

    rebuildAxisGeometry();
    update();
}

void PointCloudViewport::setSelectionInteractionEnabled(bool enabled)
{
    selectionInteractionEnabled_ = enabled;

    if (!enabled) {
        resetInteraction();
    }
}

void PointCloudViewport::beginInteraction(InteractionMode mode, const QPoint &position)
{
    interactionMode_ = mode;
    lastMousePosition_ = position;
    interactionStartPosition_ = position;
    if (mode == InteractionMode::BoxSelecting ||
        mode == InteractionMode::BoxDeselecting) {
        selectionStartPosition_ = position;
        selectionCurrentPosition_ = position;
    }

    grabMouse();
    update();
}

void PointCloudViewport::panCameraByScreenDelta(const QPoint &delta)
{
    const QVector3D worldUp(0.0f, 0.0f, 1.0f);
    const QVector3D forward = computeForwardVector(yawDegrees_, pitchDegrees_).normalized();

    QVector3D right = QVector3D::crossProduct(forward, worldUp);
    if (right.lengthSquared() < 1.0e-6f) {
        right = QVector3D(1.0f, 0.0f, 0.0f);
    } else {
        right.normalize();
    }

    QVector3D cameraUp = QVector3D::crossProduct(right, forward);
    if (cameraUp.lengthSquared() < 1.0e-6f) {
        cameraUp = worldUp;
    } else {
        cameraUp.normalize();
    }

    const float aspectRatio = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
    const float halfVerticalFovRadians = qDegreesToRadians(kVerticalFieldOfViewDegrees * 0.5f);
    const float visibleHalfHeight = zoomDistance_ * std::tan(halfVerticalFovRadians);
    const float visibleHalfWidth = visibleHalfHeight * aspectRatio;

    const float worldUnitsPerPixelX = width() > 0
        ? (2.0f * visibleHalfWidth / static_cast<float>(width()))
        : 0.0f;
    const float worldUnitsPerPixelY = height() > 0
        ? (2.0f * visibleHalfHeight / static_cast<float>(height()))
        : 0.0f;

    panOffset_ +=
        (-right * static_cast<float>(delta.x()) * worldUnitsPerPixelX +
         cameraUp * static_cast<float>(delta.y()) * worldUnitsPerPixelY) *
        kPanDirection;
    rebuildAxisGeometry();
    update();
}

void PointCloudViewport::updateSelectionInteraction(const QPoint &position)
{
    selectionCurrentPosition_ = position;
    update();
}

void PointCloudViewport::resetInteraction()
{
    interactionMode_ = InteractionMode::Idle;

    if (panningCursorActive_) {
        unsetCursor();
        panningCursorActive_ = false;
    }

    if (QWidget::mouseGrabber() == this) {
        releaseMouse();
    }

    update();
}

bool PointCloudViewport::selectionGesturePassedDragThreshold() const
{
    return std::abs(selectionCurrentPosition_.x() - selectionStartPosition_.x()) >
            kSelectionClickThresholdPixels ||
        std::abs(selectionCurrentPosition_.y() - selectionStartPosition_.y()) >
            kSelectionClickThresholdPixels;
}

bool PointCloudViewport::panGesturePassedDragThreshold(const QPoint &position) const
{
    return (position - interactionStartPosition_).manhattanLength() >= QApplication::startDragDistance();
}

bool PointCloudViewport::isSelectionInteractionActive() const
{
    return interactionMode_ == InteractionMode::BoxSelecting ||
        interactionMode_ == InteractionMode::BoxDeselecting;
}

void PointCloudViewport::rebuildGeometry(const PointCloud *pointCloud)
{
    pointData_.clear();
    pointCount_ = 0;

    if (!pointCloud || pointCloud->empty()) {
        pointDataDirty_ = true;
        return;
    }

    pointData_.reserve(pointCloud->pointCount() * 3U);
    for (const Point3d &point : pointCloud->points()) {
        pointData_.push_back(static_cast<float>(point.x()));
        pointData_.push_back(static_cast<float>(point.y()));
        pointData_.push_back(static_cast<float>(point.z()));
    }

    pointCount_ = pointCloud->pointCount();
    pointDataDirty_ = true;
}

void PointCloudViewport::rebuildSelectedGeometry()
{
    selectedPointData_.clear();
    selectedPointCount_ = 0;

    if (!pointCloud_ || pointCloud_->empty() || selectedPointIndices_.empty()) {
        selectedPointDataDirty_ = true;
        return;
    }

    selectedPointData_.reserve(selectedPointIndices_.size() * 3U);
    for (const std::size_t pointIndex : selectedPointIndices_) {
        if (pointIndex >= pointCloud_->pointCount()) {
            continue;
        }

        const Point3d &point = pointCloud_->points()[pointIndex];
        selectedPointData_.push_back(static_cast<float>(point.x()));
        selectedPointData_.push_back(static_cast<float>(point.y()));
        selectedPointData_.push_back(static_cast<float>(point.z()));
    }

    selectedPointCount_ = selectedPointData_.size() / 3U;
    selectedPointDataDirty_ = true;
}

void PointCloudViewport::rebuildMeshGeometry(const TriangleMesh *triangleMesh)
{
    meshData_.clear();
    meshVertexCount_ = 0U;
    if (!triangleMesh || triangleMesh->empty() || !triangleMesh->hasValidIndices()) {
        meshDataDirty_ = true;
        return;
    }

    meshData_.reserve(triangleMesh->triangleCount() * 18U);
    for (const Triangle &triangle : triangleMesh->triangles()) {
        const Point3d &first = triangleMesh->vertices()[triangle.vertexIndices[0]];
        const Point3d &second = triangleMesh->vertices()[triangle.vertexIndices[1]];
        const Point3d &third = triangleMesh->vertices()[triangle.vertexIndices[2]];
        Eigen::Vector3d faceNormal =
            (second.vector() - first.vector()).cross(third.vector() - first.vector());
        if (faceNormal.squaredNorm() <= 1.0e-24) {
            continue;
        }
        faceNormal.normalize();

        for (const Point3d *vertex : {&first, &second, &third}) {
            meshData_.push_back(static_cast<float>(vertex->x()));
            meshData_.push_back(static_cast<float>(vertex->y()));
            meshData_.push_back(static_cast<float>(vertex->z()));
            meshData_.push_back(static_cast<float>(faceNormal.x()));
            meshData_.push_back(static_cast<float>(faceNormal.y()));
            meshData_.push_back(static_cast<float>(faceNormal.z()));
        }
    }

    meshVertexCount_ = meshData_.size() / 6U;
    meshDataDirty_ = true;
}

void PointCloudViewport::rebuildAxisGeometry()
{
    const float axisLength = static_cast<float>(computeAxisExtent());

    axisData_ = {
        -axisLength, 0.0f, 0.0f,   axisLength, 0.0f, 0.0f,
        0.0f, -axisLength, 0.0f,   0.0f, axisLength, 0.0f,
        0.0f, 0.0f, -axisLength,   0.0f, 0.0f, axisLength,
    };

    axisDataDirty_ = true;
}

void PointCloudViewport::uploadPendingPointCloud()
{
    if (!pointDataDirty_ || !glInitialized_) {
        return;
    }

    auto *gl = context()->extraFunctions();

    vertexArrayObject_.bind();
    vertexBuffer_.bind();
    vertexBuffer_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vertexBuffer_.allocate(pointData_.data(), static_cast<int>(pointData_.size() * sizeof(float)));

    shaderProgram_.bind();
    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    shaderProgram_.release();

    vertexBuffer_.release();
    vertexArrayObject_.release();
    pointDataDirty_ = false;
}

void PointCloudViewport::uploadPendingSelectedPoints()
{
    if (!selectedPointDataDirty_ || !glInitialized_) {
        return;
    }

    auto *gl = context()->extraFunctions();

    selectedVertexArrayObject_.bind();
    selectedVertexBuffer_.bind();
    selectedVertexBuffer_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    selectedVertexBuffer_.allocate(
        selectedPointData_.data(),
        static_cast<int>(selectedPointData_.size() * sizeof(float)));

    shaderProgram_.bind();
    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    shaderProgram_.release();

    selectedVertexBuffer_.release();
    selectedVertexArrayObject_.release();
    selectedPointDataDirty_ = false;
}

void PointCloudViewport::uploadPendingAxisGeometry()
{
    if (!axisDataDirty_ || !glInitialized_) {
        return;
    }

    auto *gl = context()->extraFunctions();

    axisVertexArrayObject_.bind();
    axisVertexBuffer_.bind();
    axisVertexBuffer_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    axisVertexBuffer_.allocate(axisData_.data(), static_cast<int>(axisData_.size() * sizeof(float)));

    shaderProgram_.bind();
    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    shaderProgram_.release();

    axisVertexBuffer_.release();
    axisVertexArrayObject_.release();
    axisDataDirty_ = false;
}

void PointCloudViewport::uploadPendingMesh()
{
    if (!meshDataDirty_ || !glInitialized_) {
        return;
    }

    auto *gl = context()->extraFunctions();
    meshVertexArrayObject_.bind();
    meshVertexBuffer_.bind();
    meshVertexBuffer_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    meshVertexBuffer_.allocate(meshData_.data(), static_cast<int>(meshData_.size() * sizeof(float)));

    meshShaderProgram_.bind();
    gl->glEnableVertexAttribArray(0);
    gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
    gl->glEnableVertexAttribArray(1);
    gl->glVertexAttribPointer(
        1,
        3,
        GL_FLOAT,
        GL_FALSE,
        6 * sizeof(float),
        reinterpret_cast<const void *>(3 * sizeof(float)));
    meshShaderProgram_.release();

    meshVertexBuffer_.release();
    meshVertexArrayObject_.release();
    meshDataDirty_ = false;
}

void PointCloudViewport::drawAxis()
{
    auto *gl = context()->extraFunctions();

    gl->glLineWidth(kAxisLineWidth);
    shaderProgram_.setUniformValue("u_pointSize", 1.0f);

    axisVertexArrayObject_.bind();

    shaderProgram_.setUniformValue("u_color", kAxisXColor);
    gl->glDrawArrays(GL_LINES, 0, 2);

    shaderProgram_.setUniformValue("u_color", kAxisYColor);
    gl->glDrawArrays(GL_LINES, 2, 2);

    shaderProgram_.setUniformValue("u_color", kAxisZColor);
    gl->glDrawArrays(GL_LINES, 4, 2);

    axisVertexArrayObject_.release();
}

void PointCloudViewport::fitViewToPointCloud()
{
    yawDegrees_ = kDefaultYawDegrees;
    pitchDegrees_ = kDefaultPitchDegrees;
    panOffset_ = QVector3D(0.0f, 0.0f, 0.0f);

    if (!pointCloud_ || pointCloud_->empty()) {
        cloudCenter_ = QVector3D(0.0f, 0.0f, 0.0f);
        cloudRadius_ = 1.0f;
        zoomDistance_ = 3.0f;
        return;
    }

    const std::optional<BoundingBox3d> boundingBox = pointCloud_->boundingBox();
    if (!boundingBox || !boundingBox->isValid()) {
        cloudCenter_ = QVector3D(0.0f, 0.0f, 0.0f);
        cloudRadius_ = 1.0f;
        zoomDistance_ = 3.0f;
        return;
    }

    const Eigen::Vector3d center = boundingBox->center();
    cloudCenter_ = QVector3D(
        static_cast<float>(center.x()),
        static_cast<float>(center.y()),
        static_cast<float>(center.z()));

    cloudRadius_ = static_cast<float>(boundingBox->diagonalLength() * 0.5);
    if (cloudRadius_ < 0.5f) {
        cloudRadius_ = 0.5f;
    }

    zoomDistance_ = cloudRadius_ * 2.5f;
}

void PointCloudViewport::drawSelectionOverlay()
{
    if (!isSelectionInteractionActive()) {
        return;
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(QPen(kSelectionOverlayStrokeColor, kSelectionOverlayLineWidth));
    painter.setBrush(kSelectionOverlayFillColor);

    if (interactionMode_ == InteractionMode::BoxSelecting ||
        interactionMode_ == InteractionMode::BoxDeselecting) {
        const QRect selectionRectangle = normalizedSelectionRectangle();
        if (selectionRectangle.isValid()) {
            painter.drawRect(selectionRectangle);
        }
    }
}

QRect PointCloudViewport::normalizedSelectionRectangle() const
{
    return QRect(selectionStartPosition_, selectionCurrentPosition_).normalized();
}

std::vector<std::size_t> PointCloudViewport::collectPointsInSelectionArea() const
{
    std::vector<std::size_t> selectedIndices;
    if (!pointCloud_ || pointCloud_->empty()) {
        return selectedIndices;
    }

    if (interactionMode_ == InteractionMode::BoxSelecting ||
        interactionMode_ == InteractionMode::BoxDeselecting) {
        const QRect selectionRectangle = normalizedSelectionRectangle();
        if (selectionRectangle.width() < 2 && selectionRectangle.height() < 2) {
            return selectedIndices;
        }

        selectedIndices.reserve(pointCloud_->pointCount());
        for (std::size_t pointIndex = 0; pointIndex < pointCloud_->pointCount(); ++pointIndex) {
            const std::optional<QPointF> screenPoint =
                projectedScreenPoint(pointCloud_->points()[pointIndex]);
            if (screenPoint && selectionRectangle.contains(screenPoint->toPoint())) {
                selectedIndices.push_back(pointIndex);
            }
        }

        return selectedIndices;
    }

    return selectedIndices;
}

std::optional<QPointF> PointCloudViewport::projectedScreenPoint(const Point3d &point) const
{
    if (width() <= 0 || height() <= 0) {
        return std::nullopt;
    }

    QVector4D clipPoint = projectPoint(point);
    if (qFuzzyIsNull(clipPoint.w())) {
        return std::nullopt;
    }

    clipPoint /= clipPoint.w();
    if (clipPoint.z() < -1.0f || clipPoint.z() > 1.0f) {
        return std::nullopt;
    }

    const float screenX = (clipPoint.x() * 0.5f + 0.5f) * static_cast<float>(width());
    const float screenY = (1.0f - (clipPoint.y() * 0.5f + 0.5f)) * static_cast<float>(height());
    if (screenX < 0.0f || screenX > static_cast<float>(width()) ||
        screenY < 0.0f || screenY > static_cast<float>(height())) {
        return std::nullopt;
    }

    return QPointF(screenX, screenY);
}

double PointCloudViewport::computeAxisExtent() const
{
    const double aspectRatio = height() > 0
        ? static_cast<double>(width()) / static_cast<double>(height())
        : 1.0;
    const double halfVerticalFovRadians =
        static_cast<double>(kVerticalFieldOfViewDegrees) * std::numbers::pi_v<double> / 360.0;

    const double visibleHalfHeight = static_cast<double>(zoomDistance_) * std::tan(halfVerticalFovRadians);
    const double visibleHalfWidth = visibleHalfHeight * aspectRatio;
    const double visibleRadius = std::max(visibleHalfWidth, visibleHalfHeight);

    const double sceneOffset =
        static_cast<double>(cloudCenter_.length()) +
        static_cast<double>(panOffset_.length()) +
        static_cast<double>(cloudRadius_);

    return std::max(
        static_cast<double>(kDefaultAxisLength),
        (sceneOffset + visibleRadius) * 2.0);
}

QVector3D PointCloudViewport::currentForward() const
{
    return computeForwardVector(yawDegrees_, pitchDegrees_).normalized();
}

QVector3D PointCloudViewport::currentTarget() const
{
    return cloudCenter_ + panOffset_;
}

QVector3D PointCloudViewport::currentEye() const
{
    return currentTarget() - currentForward() * zoomDistance_;
}

QMatrix4x4 PointCloudViewport::projectionMatrix() const
{
    QMatrix4x4 projection;
    const float aspectRatio = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
    projection.perspective(
        kVerticalFieldOfViewDegrees,
        aspectRatio,
        0.01f,
        std::max(1000.0f, zoomDistance_ + cloudRadius_ * 10.0f));
    return projection;
}

QMatrix4x4 PointCloudViewport::viewMatrix() const
{
    QMatrix4x4 view;
    view.lookAt(currentEye(), currentTarget(), QVector3D(0.0f, 0.0f, 1.0f));
    return view;
}

float PointCloudViewport::zoomDistanceStep() const
{
    return std::max(0.05f, std::max(cloudRadius_, 1.0f) * kZoomFactor);
}

std::optional<QVector3D> PointCloudViewport::rayDirectionForScreenPoint(const QPointF &screenPoint) const
{
    if (width() <= 0 || height() <= 0) {
        return std::nullopt;
    }

    bool invertible = false;
    const QMatrix4x4 inverse = (projectionMatrix() * viewMatrix()).inverted(&invertible);
    if (!invertible) {
        return std::nullopt;
    }

    const float ndcX = (2.0f * static_cast<float>(screenPoint.x()) / static_cast<float>(width())) - 1.0f;
    const float ndcY = 1.0f - (2.0f * static_cast<float>(screenPoint.y()) / static_cast<float>(height()));

    QVector4D nearPoint = inverse * QVector4D(ndcX, ndcY, -1.0f, 1.0f);
    QVector4D farPoint = inverse * QVector4D(ndcX, ndcY, 1.0f, 1.0f);
    if (qFuzzyIsNull(nearPoint.w()) || qFuzzyIsNull(farPoint.w())) {
        return std::nullopt;
    }

    nearPoint /= nearPoint.w();
    farPoint /= farPoint.w();

    const QVector3D direction = (farPoint.toVector3D() - nearPoint.toVector3D()).normalized();
    if (direction.lengthSquared() < 1.0e-6f) {
        return std::nullopt;
    }

    return direction;
}

std::optional<QVector3D> PointCloudViewport::intersectCursorRayWithViewPlane(
    const QPointF &screenPoint,
    const QVector3D &planePoint,
    const QVector3D &planeNormal) const
{
    const std::optional<QVector3D> rayDirection = rayDirectionForScreenPoint(screenPoint);
    if (!rayDirection) {
        return std::nullopt;
    }

    const QVector3D rayOrigin = currentEye();
    const float denominator = QVector3D::dotProduct(*rayDirection, planeNormal);
    if (std::abs(denominator) < 1.0e-5f) {
        return std::nullopt;
    }

    const float distanceToPlane =
        QVector3D::dotProduct(planePoint - rayOrigin, planeNormal) / denominator;
    if (distanceToPlane <= 0.0f) {
        return std::nullopt;
    }

    return rayOrigin + *rayDirection * distanceToPlane;
}

QMatrix4x4 PointCloudViewport::modelViewProjectionMatrix() const
{
    return projectionMatrix() * viewMatrix();
}

void PointCloudViewport::setSelectionModeEnabled(bool enabled)
{
    selectionModeEnabled_ = enabled;
    if (!enabled) {
        resetInteraction();
    }
}

void PointCloudViewport::setSelectedPointIndices(const std::vector<std::size_t> &selectedPointIndices)
{
    selectedPointIndices_ = selectedPointIndices;
    std::sort(selectedPointIndices_.begin(), selectedPointIndices_.end());
    selectedPointIndices_.erase(
        std::remove_if(
            selectedPointIndices_.begin(),
            selectedPointIndices_.end(),
            [this](std::size_t index) {
                return !pointCloud_ || index >= pointCloud_->pointCount();
            }),
        selectedPointIndices_.end());
    selectedPointIndices_.erase(
        std::unique(selectedPointIndices_.begin(), selectedPointIndices_.end()),
        selectedPointIndices_.end());

    rebuildSelectedGeometry();
    update();
}

QVector4D PointCloudViewport::projectPoint(const Point3d &point) const
{
    return modelViewProjectionMatrix() *
        QVector4D(
            static_cast<float>(point.x()),
            static_cast<float>(point.y()),
            static_cast<float>(point.z()),
            1.0f);
}

std::optional<std::size_t> PointCloudViewport::pickPointAt(const QPoint &screenPosition) const
{
    if (!pointCloud_ || pointCloud_->empty() || width() <= 0 || height() <= 0) {
        return std::nullopt;
    }

    const float toleranceSquared = kPickTolerancePixels * kPickTolerancePixels;
    std::optional<std::size_t> pickedPointIndex;
    float bestDistanceSquared = toleranceSquared;
    float bestDepth = std::numeric_limits<float>::max();

    for (std::size_t pointIndex = 0; pointIndex < pointCloud_->pointCount(); ++pointIndex) {
        const std::optional<QPointF> screenPoint = projectedScreenPoint(pointCloud_->points()[pointIndex]);
        if (!screenPoint) {
            continue;
        }

        QVector4D clipPoint = projectPoint(pointCloud_->points()[pointIndex]);
        if (qFuzzyIsNull(clipPoint.w())) {
            continue;
        }

        clipPoint /= clipPoint.w();

        const float deltaX = static_cast<float>(screenPoint->x()) - static_cast<float>(screenPosition.x());
        const float deltaY = static_cast<float>(screenPoint->y()) - static_cast<float>(screenPosition.y());
        const float distanceSquared = deltaX * deltaX + deltaY * deltaY;

        if (distanceSquared > toleranceSquared) {
            continue;
        }

        if (distanceSquared < bestDistanceSquared ||
            (qFuzzyCompare(distanceSquared, bestDistanceSquared) && clipPoint.z() < bestDepth)) {
            bestDistanceSquared = distanceSquared;
            bestDepth = clipPoint.z();
            pickedPointIndex = pointIndex;
        }
    }

    return pickedPointIndex;
}
