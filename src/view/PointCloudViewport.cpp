#include "view/PointCloudViewport.h"

#include "model/geometry/BoundingBox3d.h"
#include "model/geometry/PointCloud.h"

#include <QColor>
#include <QMouseEvent>
#include <QOpenGLExtraFunctions>
#include <QWheelEvent>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <numbers>
#include <optional>

namespace
{
constexpr float kDefaultPointSize = 5.0f;
constexpr float kAxisLineWidth = 1.0f;
constexpr float kMinZoomDistance = 0.05f;
constexpr float kRotationDirection = -1.0f;
constexpr float kRotationSpeed = 0.5f;
constexpr float kPanDirection = 1.0f;
constexpr float kZoomFactor = 0.1f;

const QColor kViewportBackgroundColor("#F8F8F8");
const QVector3D kPointCloudColor(0.235f, 0.255f, 0.275f); // #3C4146

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
    axisVertexArrayObject_.destroy();
    axisVertexBuffer_.destroy();
    doneCurrent();
}

void PointCloudViewport::setPointCloud(const PointCloud *pointCloud)
{
    pointCloud_ = pointCloud;
    rebuildGeometry(pointCloud);
    fitViewToPointCloud();
    rebuildAxisGeometry();
    update();
}

void PointCloudViewport::clearPointCloud()
{
    pointCloud_ = nullptr;
    pointData_.clear();
    pointCount_ = 0;
    pointDataDirty_ = true;
    cloudCenter_ = QVector3D(0.0f, 0.0f, 0.0f);
    cloudRadius_ = 1.0f;
    yawDegrees_ = kDefaultYawDegrees;
    pitchDegrees_ = kDefaultPitchDegrees;
    zoomDistance_ = 3.0f;
    panOffset_ = QVector3D(0.0f, 0.0f, 0.0f);
    rebuildAxisGeometry();
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

    vertexArrayObject_.create();
    vertexBuffer_.create();
    axisVertexArrayObject_.create();
    axisVertexBuffer_.create();

    glInitialized_ = true;
    pointDataDirty_ = true;
    axisDataDirty_ = true;
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
    uploadPendingAxisGeometry();

    shaderProgram_.bind();
    shaderProgram_.setUniformValue("u_mvp", modelViewProjectionMatrix());
    drawAxis();
    shaderProgram_.release();

    if (pointCount_ == 0) {
        return;
    }

    shaderProgram_.bind();
    shaderProgram_.setUniformValue("u_mvp", modelViewProjectionMatrix());
    shaderProgram_.setUniformValue("u_pointSize", kDefaultPointSize);
    shaderProgram_.setUniformValue("u_color", kPointCloudColor);

    vertexArrayObject_.bind();
    gl->glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(pointCount_));
    vertexArrayObject_.release();
    shaderProgram_.release();
}

void PointCloudViewport::mousePressEvent(QMouseEvent *event)
{
    lastMousePosition_ = event->pos();
}

void PointCloudViewport::mouseMoveEvent(QMouseEvent *event)
{
    const QPoint delta = event->pos() - lastMousePosition_;
    lastMousePosition_ = event->pos();

    if (event->buttons().testFlag(Qt::LeftButton)) {
        yawDegrees_ += static_cast<float>(delta.x()) * kRotationSpeed * kRotationDirection;
        pitchDegrees_ += static_cast<float>(delta.y()) * kRotationSpeed * kRotationDirection;
        pitchDegrees_ = std::clamp(pitchDegrees_, -89.0f, 89.0f);
        update();
        return;
    }

    if (event->buttons().testFlag(Qt::RightButton) || event->buttons().testFlag(Qt::MiddleButton)) {
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
}

void PointCloudViewport::wheelEvent(QWheelEvent *event)
{
    const QPoint angleDelta = event->angleDelta();
    if (angleDelta.y() == 0) {
        return;
    }

    const float zoomStep = 1.0f - static_cast<float>(angleDelta.y()) / 120.0f * kZoomFactor;
    zoomDistance_ = std::max(kMinZoomDistance, zoomDistance_ * zoomStep);
    rebuildAxisGeometry();
    update();
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

QMatrix4x4 PointCloudViewport::modelViewProjectionMatrix() const
{
    QMatrix4x4 projection;
    const float aspectRatio = height() > 0 ? static_cast<float>(width()) / static_cast<float>(height()) : 1.0f;
    projection.perspective(
        kVerticalFieldOfViewDegrees,
        aspectRatio,
        0.01f,
        std::max(1000.0f, zoomDistance_ + cloudRadius_ * 10.0f));

    const QVector3D forward = computeForwardVector(yawDegrees_, pitchDegrees_);

    const QVector3D target = cloudCenter_ + panOffset_;
    const QVector3D eye = target - forward * zoomDistance_;

    QMatrix4x4 view;
    view.lookAt(eye, target, QVector3D(0.0f, 0.0f, 1.0f));

    return projection * view;
}
