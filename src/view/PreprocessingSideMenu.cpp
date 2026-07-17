#include "view/FormattedDoubleSpinBox.h"
#include "view/MethodSelectionDialog.h"
#include "view/PreprocessingSideMenu.h"
#include "view/ProcessingProgressWidget.h"

#include "model/processing/normals/NormalMethodRegistry.h"
#include "model/processing/reconstruction/SurfaceReconstructorRegistry.h"

#include <QAbstractSpinBox>
#include <QCheckBox>
#include <QComboBox>
#include <QCursor>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPoint>
#include <QPushButton>
#include <QSpinBox>
#include <QStackedLayout>
#include <QStyle>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <initializer_list>
#include <optional>
#include <utility>

namespace
{
constexpr int kAlignedControlWidth = 84;
constexpr int kMenuWidth = 253;
constexpr int kContextHelpPopupWidth = 240;
constexpr int kContextHelpPopupOffset = 12;
constexpr int kContextHelpDelayMs = 1000;
constexpr int kContextHelpStillThresholdPixels = 4;
constexpr auto kManualSelectionHelpText =
    "Left-drag to select points with a box. Command-left-drag to deselect points with a box. "
    "Click a point to toggle it.";
constexpr auto kRadiusHelpText =
    "Maximum search radius used to count neighbouring points.";
constexpr auto kNeighbourHelpText =
    "Minimum number of neighbours required inside the radius. Points below this count are selected.";
constexpr auto kPerfectDuplicatesHelpText =
    "Selects points with exactly identical coordinates, keeping one representative point.";
constexpr auto kDistanceThresholdHelpText =
    "Groups nearby points within the threshold and keeps one representative point.";
constexpr auto kNormalEstimationHelpText =
    "Estimates normals directly from the point cloud using the selected method.";
constexpr auto kSurfaceSearchRadiusHelpText =
    "Defines the neighbourhood radius used to analyse the local surface.";
constexpr auto kSurfaceNeighbourHelpText =
    "Defines how many neighbouring points are used for local surface analysis.";
constexpr auto kBallRadiusHelpText =
    "Defines the virtual ball radius used to seed and grow Ball Pivoting triangles.";
constexpr auto kSurfaceMethodHelpText =
    "Selects the reconstruction approach used to create a surface representation from the point cloud.";
constexpr auto kSurfaceConversionHelpText =
    "Creates an intermediate surface representation using the selected reconstruction method.";
constexpr auto kSurfaceApplyHelpText =
    "Confirms the temporary converted surface when one is available.";

MethodSelectionOption methodOption(
    const QString &value,
    const QString &description,
    QStringList advantages,
    QStringList disadvantages,
    QStringList useCases,
    bool enabled = true)
{
    return MethodSelectionOption{
        value,
        description,
        std::move(advantages),
        std::move(disadvantages),
        std::move(useCases),
        enabled,
        {},
    };
}

QVector<MethodSelectionOption> surfaceMethodOptions()
{
    QVector<MethodSelectionOption> options{
        methodOption(
            "Delaunay 2D / 2.5D Triangulation",
            "Projects samples onto a plane or height field, then creates a direct triangle mesh. "
            "It suits data without strong vertical overlap.",
            {"Fast direct triangulation", "Good for near-planar data"},
            {"Cannot model overhangs", "Needs a projection direction"},
            {"Terrain and height fields", "Near-planar scans"}),
        methodOption(
            "Alpha Shapes",
            "Filters a Delaunay complex with an alpha radius to recover a selected shape scale. "
            "It builds a direct surface that can preserve concavities.",
            {"Captures concavities", "Explicit scale control"},
            {"Sensitive to alpha", "Noise can leave holes"},
            {"Bounded point sets", "Concave scan envelopes"}),
        methodOption(
            "Ball Pivoting Algorithm",
            "Rolls a virtual ball across oriented samples and joins contacts into triangles. "
            "It produces a direct mesh when sampling and ball radius are coherent.",
            {"Preserves local detail", "Direct triangle mesh"},
            {"Needs oriented normals", "Sensitive to ball radius"},
            {"Dense object scans", "Uniformly sampled surfaces"}),
        methodOption(
            "Screened Poisson Reconstruction",
            "Solves a global implicit field from oriented samples, then extracts an isosurface. "
            "It produces a smooth volumetric reconstruction rather than connecting samples directly.",
            {"Robust to moderate noise", "Usually watertight"},
            {"Needs oriented normals", "May smooth fine detail"},
            {"Complete noisy scans", "Watertight reconstruction"}),
        methodOption(
            "RBF Reconstruction",
            "Fits a continuous implicit surface with radial basis functions centred on samples. "
            "An isosurface is then extracted from the fitted field.",
            {"Smooth interpolation", "Flexible field model"},
            {"Costly for large data", "Sensitive to kernel settings"},
            {"Moderate scattered datasets", "Smooth free-form geometry"}),
        methodOption(
            "Marching Cubes on Implicit Field",
            "Extracts an isosurface from an existing scalar or distance field on a 3D grid. "
            "It is an implicit-field extraction step, not direct point-cloud triangulation.",
            {"Standard grid extraction", "Predictable topology"},
            {"Requires an existing field", "Resolution limits detail"},
            {"Distance fields", "Implicit reconstruction pipelines"}),
        methodOption(
            "Voxel-based Reconstruction",
            "Aggregates samples into a voxel occupancy or distance field before deriving a surface. "
            "This volumetric approach trades fine detail for a stable representation.",
            {"Handles noisy scans", "Supports field operations"},
            {"Grid resolution limits detail", "Memory grows with resolution"},
            {"Raw scan consolidation", "Coarse volumetric surfaces"}),
        methodOption(
            "Greedy Projection Triangulation",
            "Projects local neighbourhoods onto tangent planes and greedily joins nearby samples. "
            "It creates an explicit mesh incrementally from local support.",
            {"Fast local construction", "Retains open boundaries"},
            {"Needs reliable normals", "Sensitive to uneven sampling"},
            {"Quick scan meshing", "Open-surface reconstruction"}),
        methodOption(
            "NURBS / B-Spline Fitting",
            "Fits smooth parametric spline patches to the sampled geometry. "
            "It targets editable CAD-oriented surfaces rather than a mesh or field.",
            {"Compact editable representation", "Smooth analytic surface"},
            {"Needs patch layout", "May suppress local detail"},
            {"Smooth engineered parts", "CAD reconstruction"}),
    };

    const SurfaceReconstructorRegistry registry;
    for (MethodSelectionOption &option : options) {
        if (option.value == "NURBS / B-Spline Fitting") {
            option.enabled = false;
            option.unavailableReason = "NURBS / B-Spline Fitting belongs to the later Parametric Fitting stage.";
            continue;
        }
        const std::optional<ReconstructionMethod> method =
            SurfaceReconstructorRegistry::methodFromDisplayName(option.value.toStdString());
        if (!method) {
            option.enabled = false;
            option.unavailableReason = "No reconstruction backend is registered for this method.";
            continue;
        }
        const ReconstructionAvailability availability = registry.availability(*method);
        option.enabled = availability.available;
        option.unavailableReason = QString::fromStdString(availability.reason);
        if (*method == ReconstructionMethod::MarchingCubes) {
            option.enabled = false;
            option.unavailableReason =
                "Marching Cubes requires an existing scalar field; use Voxel reconstruction "
                "in the current workflow.";
        } else if (*method == ReconstructionMethod::Rbf) {
            option.enabled = false;
            option.unavailableReason =
                "RBF reconstruction requires consistently oriented normals, which the current "
                "point-cloud workflow does not produce.";
        }
    }
    return options;
}

NormalRequirement normalRequirementForSurfaceMethod(const QString &surfaceMethod)
{
    if (surfaceMethod == "Delaunay 2D / 2.5D Triangulation" ||
        surfaceMethod == "Alpha Shapes" ||
        surfaceMethod == "Marching Cubes on Implicit Field" ||
        surfaceMethod == "Voxel-based Reconstruction") {
        return NormalRequirement::NotUsed;
    }
    if (surfaceMethod == "Screened Poisson Reconstruction" ||
        surfaceMethod == "RBF Reconstruction") {
        return NormalRequirement::RequiredAndOriented;
    }
    if (surfaceMethod == "Ball Pivoting Algorithm" ||
        surfaceMethod == "Greedy Projection Triangulation") {
        return NormalRequirement::Required;
    }
    if (surfaceMethod == "NURBS / B-Spline Fitting") {
        return NormalRequirement::Optional;
    }

    return NormalRequirement::NotUsed;
}

QStringList compatibleNormalMethods(const QString &surfaceMethod)
{
    const QStringList estimators{
        QString::fromStdString(NormalMethodRegistry::displayName(NormalMethod::PcaKNearest)),
        QString::fromStdString(NormalMethodRegistry::displayName(NormalMethod::PcaFixedRadius)),
        QString::fromStdString(NormalMethodRegistry::displayName(NormalMethod::PcaMultiScale)),
        QString::fromStdString(NormalMethodRegistry::displayName(NormalMethod::QuadraticSurfaceFit)),
    };

    switch (normalRequirementForSurfaceMethod(surfaceMethod)) {
    case NormalRequirement::NotUsed:
        return {};
    case NormalRequirement::Required:
    case NormalRequirement::RequiredAndOriented:
        return estimators;
    case NormalRequirement::Optional:
        return estimators;
    }

    return {};
}

QVector<MethodSelectionOption> normalMethodOptions(const QStringList &compatibleMethods)
{
    const QString pcaNearest = QString::fromUtf8("PCA local plane \xE2\x80\x94 k-nearest neighbours");
    const QString pcaRadius = QString::fromUtf8("PCA local plane \xE2\x80\x94 fixed radius");
    const auto isEnabled = [&compatibleMethods](const QString &method) {
        return compatibleMethods.contains(method);
    };

    QVector<MethodSelectionOption> options{
        methodOption(
            pcaNearest,
            "Fits a local plane to a fixed number of nearest neighbours. "
            "Its normal is the plane's least-variance direction.",
            {"Adapts to sampling density", "Simple local estimate"},
            {"Neighbour count sets scale", "Can blur sharp edges"},
            {"Unevenly sampled scans", "Initial normal estimation"},
            isEnabled(pcaNearest)),
        methodOption(
            pcaRadius,
            "Fits a local plane to samples inside a fixed physical radius. "
            "The radius defines the analysis scale.",
            {"Explicit spatial scale", "Stable on uniform scans"},
            {"Sparse areas may lack neighbours", "One radius may not fit all features"},
            {"Known feature scale", "Uniform scans"},
            isEnabled(pcaRadius)),
        methodOption(
            "PCA multi-scale",
            "Compares PCA estimates across several neighbourhood scales. "
            "It selects a stable scale to balance noise suppression and feature preservation.",
            {"More robust to noise", "Adapts to feature scale"},
            {"Slower than single-scale PCA", "Requires scale choices"},
            {"Noisy scans", "Variable-density data"},
            isEnabled("PCA multi-scale")),
        methodOption(
            "Quadratic local surface fitting",
            "Fits a quadratic patch around each point and derives its normal from the surface. "
            "It represents local curvature beyond a planar estimate.",
            {"Captures curved patches", "Smooth local normals"},
            {"Costlier than PCA", "Sensitive to noise"},
            {"Smooth curved scans", "Curvature-aware analysis"},
            isEnabled("Quadratic local surface fitting")),
    };

    for (MethodSelectionOption &option : options) {
        const std::optional<NormalMethod> method =
            NormalMethodRegistry::methodFromDisplayName(option.value.toStdString());
        if (!method) {
            option.enabled = false;
            option.unavailableReason = "No normal-processing backend is registered.";
            continue;
        }
        option.enabled = compatibleMethods.contains(option.value);
        if (!option.enabled) {
            option.unavailableReason =
                "This method is not a valid first normal stage for the selected reconstruction method.";
        }
    }
    return options;
}

constexpr auto kContainerStyle = R"(
    PreprocessingSideMenu {
        background-color: #F8F8F6;
        border: 1px solid rgba(60, 65, 70, 22);
        border-radius: 16px;
    }
)";

constexpr auto kButtonStyle = R"(
    QPushButton {
        color: #3C4146;
        background-color: transparent;
        border: none;
        border-radius: 8px;
        padding: 8px;
        font-weight: 600;
        text-align: left;
    }

    QPushButton[dialogAction="true"] {
        text-align: center;
    }

    QPushButton[actionButton="true"] {
        font-weight: 400;
    }

    QPushButton[fieldLike="true"] {
        background-color: white;
        border: 1px solid rgba(60, 65, 70, 22);
        text-align: center;
        font-weight: 400;
    }

    QPushButton[selectorField="true"] {
        text-align: left;
        padding: 4px 8px;
    }

    QPushButton[fieldLike="true"]:hover {
        background-color: rgba(60, 65, 70, 8);
    }

    QPushButton[fieldLike="true"]:pressed {
        background-color: rgba(60, 65, 70, 14);
    }

    QPushButton[fieldLike="true"][normalSelectorInactive="true"] {
        color: #9A9EA2;
        background-color: #F2F2F0;
    }

    QPushButton[fieldLike="true"][normalSelectorInactive="true"]:hover,
    QPushButton[fieldLike="true"][normalSelectorInactive="true"]:pressed {
        background-color: #F2F2F0;
    }

    QPushButton[selectionToggle="true"]:checked {
        background-color: #E7FFFF;
        border: 1px solid rgba(60, 65, 70, 22);
    }

    QPushButton:hover {
        background-color: rgba(60, 65, 70, 16);
    }

    QPushButton[selectionToggle="true"]:checked:hover {
        background-color: #E7FFFF;
    }

    QPushButton:pressed {
        background-color: rgba(60, 65, 70, 26);
    }

    QLabel {
        color: #3C4146;
        font-weight: 600;
    }

    QLabel[statusText="true"] {
        font-size: 11px;
        font-weight: 400;
    }

    QLabel[planarityValid="false"] {
        color: #A33B32;
    }

    QFrame[menuSeparator="true"] {
        background-color: rgba(60, 65, 70, 22);
        min-height: 1px;
        max-height: 1px;
        border: none;
    }

    QDoubleSpinBox,
    QSpinBox,
    QComboBox {
        border: 1px solid rgba(60, 65, 70, 22);
        border-radius: 8px;
        background-color: white;
        color: #3C4146;
        padding: 4px 8px;
        font-weight: 400;
    }

    QCheckBox {
        color: #3C4146;
        font-weight: 400;
        spacing: 6px;
    }

)";

constexpr auto kContextHelpPopupStyle = R"(
    QLabel {
        background-color: #F8F8F6;
        color: #3C4146;
        border: 1px solid rgba(60, 65, 70, 22);
        border-radius: 12px;
        padding: 8px 10px;
        font-size: 11px;
        font-weight: 400;
    }
)";

}

PreprocessingSideMenu::PreprocessingSideMenu(QWidget *parent)
    : QWidget(parent)
{
    setObjectName("PreprocessingSideMenu");
    setAttribute(Qt::WA_StyledBackground, true);
    setStyleSheet(QString::fromUtf8(kContainerStyle) + QString::fromUtf8(kButtonStyle));
    setFixedWidth(kMenuWidth);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Minimum);

    stackedLayout_ = new QStackedLayout(this);
    stackedLayout_->setContentsMargins(0, 0, 0, 0);

    operationListPage_ = new QWidget(this);
    auto *operationLayout = new QVBoxLayout(operationListPage_);
    operationLayout->setContentsMargins(10, 10, 10, 10);
    operationLayout->setSpacing(4);

    QPushButton *removeInvalidPointsButton =
        createMenuButton("Remove invalid points", operationListPage_);
    operationLayout->addWidget(removeInvalidPointsButton);
    connect(removeInvalidPointsButton, &QPushButton::clicked, this, [this] {
        showRemoveInvalidPointsMenu();
    });

    QPushButton *downsamplingButton =
        createMenuButton("Downsampling", operationListPage_);
    operationLayout->addWidget(downsamplingButton);
    connect(downsamplingButton, &QPushButton::clicked, this, [this] {
        showRemoveDuplicatesMenu();
    });

    QPushButton *convertToSurfaceButton =
        createMenuButton("Convert to surface", operationListPage_);
    operationLayout->addWidget(convertToSurfaceButton);
    connect(convertToSurfaceButton, &QPushButton::clicked, this, [this] {
        showConvertToSurfaceMenu();
    });

    removeInvalidPointsPage_ = new QWidget(this);
    auto *removeInvalidLayout = new QVBoxLayout(removeInvalidPointsPage_);
    removeInvalidLayout->setContentsMargins(10, 10, 10, 10);
    removeInvalidLayout->setSpacing(4);

    manualSelectionRowWidget_ = new QWidget(removeInvalidPointsPage_);
    auto *manualSelectionRow = new QHBoxLayout(manualSelectionRowWidget_);
    manualSelectionRow->setContentsMargins(0, 0, 0, 0);
    manualSelectionRow->setSpacing(8);
    manualSelectionLabel_ = new QLabel("Manual selection", manualSelectionRowWidget_);
    manualSelectionRow->addWidget(manualSelectionLabel_);
    manualSelectionRow->addStretch(1);

    manualSelectionButton_ = createMenuButton("Select", manualSelectionRowWidget_);
    manualSelectionButton_->setCheckable(true);
    manualSelectionButton_->setFixedWidth(kAlignedControlWidth);
    manualSelectionButton_->setProperty("selectionToggle", true);
    manualSelectionButton_->setProperty("dialogAction", true);
    manualSelectionButton_->setProperty("fieldLike", true);
    manualSelectionRow->addWidget(manualSelectionButton_);
    removeInvalidLayout->addWidget(manualSelectionRowWidget_);
    connect(manualSelectionButton_, &QPushButton::toggled, this, [this](bool enabled) {
        emit manualSelectionToggled(enabled);
    });
    registerHelpTrigger(manualSelectionRowWidget_, manualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText));
    registerHelpTrigger(manualSelectionLabel_, manualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText));
    registerHelpTrigger(manualSelectionButton_, manualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText));

    removeInvalidLayout->addWidget(createSeparator(removeInvalidPointsPage_));

    radiusRowWidget_ = new QWidget(removeInvalidPointsPage_);
    auto *radiusRow = new QHBoxLayout(radiusRowWidget_);
    radiusRow->setContentsMargins(0, 0, 0, 0);
    radiusRow->setSpacing(8);
    radiusLabel_ = new QLabel("Radius max", radiusRowWidget_);
    radiusRow->addWidget(radiusLabel_);
    radiusRow->addStretch(1);

    radiusMaxSpinBox_ = new FormattedDoubleSpinBox(radiusRowWidget_);
    radiusMaxSpinBox_->setRange(0.0, 1.0e9);
    radiusMaxSpinBox_->setValue(1.0);
    radiusMaxSpinBox_->setSingleStep(0.1);
    radiusMaxSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    radiusMaxSpinBox_->setAlignment(Qt::AlignRight);
    radiusMaxSpinBox_->setFixedWidth(kAlignedControlWidth);
    radiusRow->addWidget(radiusMaxSpinBox_);
    removeInvalidLayout->addWidget(radiusRowWidget_);
    registerHelpTrigger(radiusRowWidget_, radiusRowWidget_, QString::fromUtf8(kRadiusHelpText));
    registerHelpTrigger(radiusLabel_, radiusRowWidget_, QString::fromUtf8(kRadiusHelpText));
    registerHelpTrigger(radiusMaxSpinBox_, radiusRowWidget_, QString::fromUtf8(kRadiusHelpText));

    neighboursRowWidget_ = new QWidget(removeInvalidPointsPage_);
    auto *neighboursRow = new QHBoxLayout(neighboursRowWidget_);
    neighboursRow->setContentsMargins(0, 0, 0, 0);
    neighboursRow->setSpacing(8);
    neighboursLabel_ = new QLabel("Number of neighbours", neighboursRowWidget_);
    neighboursRow->addWidget(neighboursLabel_);
    neighboursRow->addStretch(1);

    minimumNeighboursSpinBox_ = new QSpinBox(neighboursRowWidget_);
    minimumNeighboursSpinBox_->setRange(0, 1000000);
    minimumNeighboursSpinBox_->setValue(4);
    minimumNeighboursSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    minimumNeighboursSpinBox_->setAlignment(Qt::AlignRight);
    minimumNeighboursSpinBox_->setFixedWidth(kAlignedControlWidth);
    neighboursRow->addWidget(minimumNeighboursSpinBox_);
    removeInvalidLayout->addWidget(neighboursRowWidget_);
    registerHelpTrigger(neighboursRowWidget_, neighboursRowWidget_, QString::fromUtf8(kNeighbourHelpText));
    registerHelpTrigger(neighboursLabel_, neighboursRowWidget_, QString::fromUtf8(kNeighbourHelpText));
    registerHelpTrigger(minimumNeighboursSpinBox_, neighboursRowWidget_, QString::fromUtf8(kNeighbourHelpText));

    autoSelectButton_ = createMenuButton("Auto select", removeInvalidPointsPage_);
    autoSelectButton_->setProperty("dialogAction", true);
    autoSelectButton_->setProperty("actionButton", true);
    autoSelectButton_->setProperty("fieldLike", true);
    removeInvalidLayout->addWidget(autoSelectButton_);
    connect(autoSelectButton_, &QPushButton::clicked, this, [this] {
        emit autoSelectSparsePointsRequested(
            radiusMaxSpinBox_->value(),
            minimumNeighboursSpinBox_->value());
    });

    removeInvalidProgress_ = new ProcessingProgressWidget(removeInvalidPointsPage_);
    removeInvalidLayout->addWidget(removeInvalidProgress_);

    removeInvalidLayout->addWidget(createSeparator(removeInvalidPointsPage_));

    applyButton_ = createMenuButton("Apply Removal", removeInvalidPointsPage_);
    applyButton_->setProperty("dialogAction", true);
    applyButton_->setProperty("actionButton", true);
    applyButton_->setEnabled(false);
    removeInvalidLayout->addWidget(applyButton_);
    connect(applyButton_, &QPushButton::clicked, this, [this] {
        emit applyRemoveInvalidPointsRequested();
    });

    removeInvalidLayout->addWidget(createSeparator(removeInvalidPointsPage_));

    auto *removeActionLayout = new QHBoxLayout();
    removeActionLayout->setContentsMargins(0, 0, 0, 0);
    removeActionLayout->setSpacing(6);

    okButton_ = createMenuButton("OK", removeInvalidPointsPage_);
    cancelButton_ = createMenuButton("Cancel", removeInvalidPointsPage_);
    okButton_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    cancelButton_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    okButton_->setProperty("dialogAction", true);
    cancelButton_->setProperty("dialogAction", true);
    okButton_->setProperty("actionButton", true);
    cancelButton_->setProperty("actionButton", true);
    removeActionLayout->addWidget(okButton_);
    removeActionLayout->addWidget(cancelButton_);
    removeInvalidLayout->addLayout(removeActionLayout);

    connect(okButton_, &QPushButton::clicked, this, [this] {
        emit confirmRemoveInvalidPointsRequested();
    });
    connect(cancelButton_, &QPushButton::clicked, this, [this] {
        emit cancelRemoveInvalidPointsRequested();
    });

    removeDuplicatesPage_ = new QWidget(this);
    auto *removeDuplicatesLayout = new QVBoxLayout(removeDuplicatesPage_);
    removeDuplicatesLayout->setContentsMargins(10, 10, 10, 10);
    removeDuplicatesLayout->setSpacing(4);

    duplicateManualSelectionRowWidget_ = new QWidget(removeDuplicatesPage_);
    auto *duplicateManualSelectionRow = new QHBoxLayout(duplicateManualSelectionRowWidget_);
    duplicateManualSelectionRow->setContentsMargins(0, 0, 0, 0);
    duplicateManualSelectionRow->setSpacing(8);
    duplicateManualSelectionLabel_ = new QLabel("Manual selection", duplicateManualSelectionRowWidget_);
    duplicateManualSelectionRow->addWidget(duplicateManualSelectionLabel_);
    duplicateManualSelectionRow->addStretch(1);

    duplicateManualSelectionButton_ = createMenuButton("Select", duplicateManualSelectionRowWidget_);
    duplicateManualSelectionButton_->setCheckable(true);
    duplicateManualSelectionButton_->setFixedWidth(kAlignedControlWidth);
    duplicateManualSelectionButton_->setProperty("selectionToggle", true);
    duplicateManualSelectionButton_->setProperty("dialogAction", true);
    duplicateManualSelectionButton_->setProperty("fieldLike", true);
    duplicateManualSelectionRow->addWidget(duplicateManualSelectionButton_);
    removeDuplicatesLayout->addWidget(duplicateManualSelectionRowWidget_);
    connect(duplicateManualSelectionButton_, &QPushButton::toggled, this, [this](bool enabled) {
        emit manualSelectionToggled(enabled);
    });
    registerHelpTrigger(
        duplicateManualSelectionRowWidget_,
        duplicateManualSelectionRowWidget_,
        QString::fromUtf8(kManualSelectionHelpText));
    registerHelpTrigger(
        duplicateManualSelectionLabel_,
        duplicateManualSelectionRowWidget_,
        QString::fromUtf8(kManualSelectionHelpText));
    registerHelpTrigger(
        duplicateManualSelectionButton_,
        duplicateManualSelectionRowWidget_,
        QString::fromUtf8(kManualSelectionHelpText));

    removeDuplicatesLayout->addWidget(createSeparator(removeDuplicatesPage_));

    perfectDuplicatesRowWidget_ = new QWidget(removeDuplicatesPage_);
    auto *perfectDuplicatesRow = new QHBoxLayout(perfectDuplicatesRowWidget_);
    perfectDuplicatesRow->setContentsMargins(0, 0, 0, 0);
    perfectDuplicatesRow->setSpacing(8);
    perfectDuplicatesLabel_ = new QLabel("Perfect duplicates", perfectDuplicatesRowWidget_);
    perfectDuplicatesRow->addWidget(perfectDuplicatesLabel_);
    perfectDuplicatesRow->addStretch(1);

    perfectDuplicatesAutoSelectButton_ = createMenuButton("Auto select", perfectDuplicatesRowWidget_);
    perfectDuplicatesAutoSelectButton_->setFixedWidth(kAlignedControlWidth);
    perfectDuplicatesAutoSelectButton_->setProperty("dialogAction", true);
    perfectDuplicatesAutoSelectButton_->setProperty("actionButton", true);
    perfectDuplicatesAutoSelectButton_->setProperty("fieldLike", true);
    perfectDuplicatesAutoSelectButton_->style()->unpolish(perfectDuplicatesAutoSelectButton_);
    perfectDuplicatesAutoSelectButton_->style()->polish(perfectDuplicatesAutoSelectButton_);
    perfectDuplicatesRow->addWidget(perfectDuplicatesAutoSelectButton_);
    removeDuplicatesLayout->addWidget(perfectDuplicatesRowWidget_);
    connect(perfectDuplicatesAutoSelectButton_, &QPushButton::clicked, this, [this] {
        emit autoSelectPerfectDuplicatesRequested();
    });
    registerHelpTrigger(
        perfectDuplicatesRowWidget_,
        perfectDuplicatesRowWidget_,
        QString::fromUtf8(kPerfectDuplicatesHelpText));
    registerHelpTrigger(
        perfectDuplicatesLabel_,
        perfectDuplicatesRowWidget_,
        QString::fromUtf8(kPerfectDuplicatesHelpText));
    registerHelpTrigger(
        perfectDuplicatesAutoSelectButton_,
        perfectDuplicatesRowWidget_,
        QString::fromUtf8(kPerfectDuplicatesHelpText));

    removeDuplicatesLayout->addWidget(createSeparator(removeDuplicatesPage_));

    distanceThresholdRowWidget_ = new QWidget(removeDuplicatesPage_);
    auto *distanceThresholdRow = new QHBoxLayout(distanceThresholdRowWidget_);
    distanceThresholdRow->setContentsMargins(0, 0, 0, 0);
    distanceThresholdRow->setSpacing(8);
    distanceThresholdLabel_ = new QLabel("Distance threshold", distanceThresholdRowWidget_);
    distanceThresholdRow->addWidget(distanceThresholdLabel_);
    distanceThresholdRow->addStretch(1);

    distanceThresholdSpinBox_ = new FormattedDoubleSpinBox(distanceThresholdRowWidget_);
    distanceThresholdSpinBox_->setRange(0.0, 1.0e9);
    distanceThresholdSpinBox_->setValue(0.1);
    distanceThresholdSpinBox_->setSingleStep(0.01);
    distanceThresholdSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    distanceThresholdSpinBox_->setAlignment(Qt::AlignRight);
    distanceThresholdSpinBox_->setFixedWidth(kAlignedControlWidth);
    distanceThresholdRow->addWidget(distanceThresholdSpinBox_);
    removeDuplicatesLayout->addWidget(distanceThresholdRowWidget_);
    registerHelpTrigger(
        distanceThresholdRowWidget_,
        distanceThresholdRowWidget_,
        QString::fromUtf8(kDistanceThresholdHelpText));
    registerHelpTrigger(
        distanceThresholdLabel_,
        distanceThresholdRowWidget_,
        QString::fromUtf8(kDistanceThresholdHelpText));
    registerHelpTrigger(
        distanceThresholdSpinBox_,
        distanceThresholdRowWidget_,
        QString::fromUtf8(kDistanceThresholdHelpText));

    nearDuplicatesAutoSelectButton_ = createMenuButton("Auto select", removeDuplicatesPage_);
    nearDuplicatesAutoSelectButton_->setProperty("dialogAction", true);
    nearDuplicatesAutoSelectButton_->setProperty("actionButton", true);
    nearDuplicatesAutoSelectButton_->setProperty("fieldLike", true);
    removeDuplicatesLayout->addWidget(nearDuplicatesAutoSelectButton_);
    connect(nearDuplicatesAutoSelectButton_, &QPushButton::clicked, this, [this] {
        emit autoSelectNearDuplicatesRequested(distanceThresholdSpinBox_->value());
    });
    registerHelpTrigger(
        nearDuplicatesAutoSelectButton_,
        distanceThresholdRowWidget_,
        QString::fromUtf8(kDistanceThresholdHelpText));

    downsamplingProgress_ = new ProcessingProgressWidget(removeDuplicatesPage_);
    removeDuplicatesLayout->addWidget(downsamplingProgress_);

    removeDuplicatesLayout->addWidget(createSeparator(removeDuplicatesPage_));

    duplicateApplyButton_ = createMenuButton("Apply Removal", removeDuplicatesPage_);
    duplicateApplyButton_->setProperty("dialogAction", true);
    duplicateApplyButton_->setProperty("actionButton", true);
    duplicateApplyButton_->setEnabled(false);
    removeDuplicatesLayout->addWidget(duplicateApplyButton_);
    connect(duplicateApplyButton_, &QPushButton::clicked, this, [this] {
        emit applyRemoveInvalidPointsRequested();
    });

    removeDuplicatesLayout->addWidget(createSeparator(removeDuplicatesPage_));

    auto *removeDuplicatesActionLayout = new QHBoxLayout();
    removeDuplicatesActionLayout->setContentsMargins(0, 0, 0, 0);
    removeDuplicatesActionLayout->setSpacing(6);

    duplicateOkButton_ = createMenuButton("OK", removeDuplicatesPage_);
    duplicateCancelButton_ = createMenuButton("Cancel", removeDuplicatesPage_);
    duplicateOkButton_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    duplicateCancelButton_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    duplicateOkButton_->setProperty("dialogAction", true);
    duplicateCancelButton_->setProperty("dialogAction", true);
    duplicateOkButton_->setProperty("actionButton", true);
    duplicateCancelButton_->setProperty("actionButton", true);
    removeDuplicatesActionLayout->addWidget(duplicateOkButton_);
    removeDuplicatesActionLayout->addWidget(duplicateCancelButton_);
    removeDuplicatesLayout->addLayout(removeDuplicatesActionLayout);

    connect(duplicateOkButton_, &QPushButton::clicked, this, [this] {
        emit confirmRemoveInvalidPointsRequested();
    });
    connect(duplicateCancelButton_, &QPushButton::clicked, this, [this] {
        emit cancelRemoveInvalidPointsRequested();
    });

    convertToSurfacePage_ = new QWidget(this);
    auto *convertToSurfaceLayout = new QVBoxLayout(convertToSurfacePage_);
    convertToSurfaceLayout->setContentsMargins(10, 10, 10, 10);
    convertToSurfaceLayout->setSpacing(4);

    surfaceConversionPanel_ = new QWidget(convertToSurfacePage_);
    auto *surfaceConversionLayout = new QVBoxLayout(surfaceConversionPanel_);
    surfaceConversionLayout->setContentsMargins(0, 0, 0, 0);
    surfaceConversionLayout->setSpacing(4);

    surfaceSearchRadiusRowWidget_ = new QWidget(convertToSurfacePage_);
    auto *surfaceSearchRadiusRow = new QHBoxLayout(surfaceSearchRadiusRowWidget_);
    surfaceSearchRadiusRow->setContentsMargins(0, 0, 0, 0);
    surfaceSearchRadiusRow->setSpacing(8);
    surfaceSearchRadiusLabel_ = new QLabel("Search radius", surfaceSearchRadiusRowWidget_);
    surfaceSearchRadiusRow->addWidget(surfaceSearchRadiusLabel_);
    surfaceSearchRadiusRow->addStretch(1);

    surfaceSearchRadiusSpinBox_ = new FormattedDoubleSpinBox(surfaceSearchRadiusRowWidget_);
    surfaceSearchRadiusSpinBox_->setRange(0.000001, 1.0e9);
    surfaceSearchRadiusSpinBox_->setValue(1.0);
    surfaceSearchRadiusSpinBox_->setSingleStep(0.01);
    surfaceSearchRadiusSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    surfaceSearchRadiusSpinBox_->setAlignment(Qt::AlignRight);
    surfaceSearchRadiusSpinBox_->setFixedWidth(kAlignedControlWidth);
    surfaceSearchRadiusRow->addWidget(surfaceSearchRadiusSpinBox_);
    surfaceConversionLayout->addWidget(surfaceSearchRadiusRowWidget_);
    registerHelpTrigger(
        surfaceSearchRadiusRowWidget_,
        surfaceSearchRadiusRowWidget_,
        QString::fromUtf8(kSurfaceSearchRadiusHelpText));
    registerHelpTrigger(
        surfaceSearchRadiusLabel_,
        surfaceSearchRadiusRowWidget_,
        QString::fromUtf8(kSurfaceSearchRadiusHelpText));
    registerHelpTrigger(
        surfaceSearchRadiusSpinBox_,
        surfaceSearchRadiusRowWidget_,
        QString::fromUtf8(kSurfaceSearchRadiusHelpText));

    surfaceNeighboursRowWidget_ = new QWidget(convertToSurfacePage_);
    auto *surfaceNeighboursRow = new QHBoxLayout(surfaceNeighboursRowWidget_);
    surfaceNeighboursRow->setContentsMargins(0, 0, 0, 0);
    surfaceNeighboursRow->setSpacing(8);
    surfaceNeighboursLabel_ = new QLabel("Number of neighbours", surfaceNeighboursRowWidget_);
    surfaceNeighboursRow->addWidget(surfaceNeighboursLabel_);
    surfaceNeighboursRow->addStretch(1);

    surfaceNeighboursSpinBox_ = new QSpinBox(surfaceNeighboursRowWidget_);
    surfaceNeighboursSpinBox_->setRange(1, 1000000);
    surfaceNeighboursSpinBox_->setValue(12);
    surfaceNeighboursSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    surfaceNeighboursSpinBox_->setAlignment(Qt::AlignRight);
    surfaceNeighboursSpinBox_->setFixedWidth(kAlignedControlWidth);
    surfaceNeighboursRow->addWidget(surfaceNeighboursSpinBox_);
    surfaceConversionLayout->addWidget(surfaceNeighboursRowWidget_);
    registerHelpTrigger(
        surfaceNeighboursRowWidget_,
        surfaceNeighboursRowWidget_,
        QString::fromUtf8(kSurfaceNeighbourHelpText));
    registerHelpTrigger(
        surfaceNeighboursLabel_,
        surfaceNeighboursRowWidget_,
        QString::fromUtf8(kSurfaceNeighbourHelpText));
    registerHelpTrigger(
        surfaceNeighboursSpinBox_,
        surfaceNeighboursRowWidget_,
        QString::fromUtf8(kSurfaceNeighbourHelpText));

    ballRadiusRowWidget_ = new QWidget(convertToSurfacePage_);
    auto *ballRadiusRow = new QHBoxLayout(ballRadiusRowWidget_);
    ballRadiusRow->setContentsMargins(0, 0, 0, 0);
    ballRadiusRow->setSpacing(8);
    ballRadiusLabel_ = new QLabel("Ball radius", ballRadiusRowWidget_);
    ballRadiusRow->addWidget(ballRadiusLabel_);
    ballRadiusRow->addStretch(1);

    ballRadiusSpinBox_ = new FormattedDoubleSpinBox(ballRadiusRowWidget_);
    ballRadiusSpinBox_->setRange(0.000001, 1.0e9);
    ballRadiusSpinBox_->setValue(0.5);
    ballRadiusSpinBox_->setSingleStep(0.05);
    ballRadiusSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    ballRadiusSpinBox_->setAlignment(Qt::AlignRight);
    ballRadiusSpinBox_->setFixedWidth(kAlignedControlWidth);
    ballRadiusRow->addWidget(ballRadiusSpinBox_);
    surfaceConversionLayout->addWidget(ballRadiusRowWidget_);
    registerHelpTrigger(
        ballRadiusRowWidget_,
        ballRadiusRowWidget_,
        QString::fromUtf8(kBallRadiusHelpText));
    registerHelpTrigger(
        ballRadiusLabel_,
        ballRadiusRowWidget_,
        QString::fromUtf8(kBallRadiusHelpText));
    registerHelpTrigger(
        ballRadiusSpinBox_,
        ballRadiusRowWidget_,
        QString::fromUtf8(kBallRadiusHelpText));

    planarModeRowWidget_ = new QWidget(surfaceConversionPanel_);
    auto *planarModeRow = new QHBoxLayout(planarModeRowWidget_);
    planarModeRow->setContentsMargins(0, 0, 0, 0);
    planarModeRow->setSpacing(8);
    planarModeRow->addWidget(new QLabel("Surface mode", planarModeRowWidget_));
    planarModeRow->addStretch(1);
    planarModeComboBox_ = new QComboBox(planarModeRowWidget_);
    planarModeComboBox_->addItems({"2D", "2.5D"});
    planarModeComboBox_->setCurrentIndex(1);
    planarModeComboBox_->setFixedWidth(kAlignedControlWidth);
    planarModeRow->addWidget(planarModeComboBox_);
    surfaceConversionLayout->addWidget(planarModeRowWidget_);

    planarityToleranceRowWidget_ = new QWidget(surfaceConversionPanel_);
    auto *planarityToleranceRow = new QHBoxLayout(planarityToleranceRowWidget_);
    planarityToleranceRow->setContentsMargins(0, 0, 0, 0);
    planarityToleranceRow->setSpacing(8);
    planarityToleranceRow->addWidget(new QLabel("Planarity tolerance", planarityToleranceRowWidget_));
    planarityToleranceRow->addStretch(1);
    planarityToleranceSpinBox_ = new FormattedDoubleSpinBox(planarityToleranceRowWidget_);
    planarityToleranceSpinBox_->setRange(0.0, 0.999999);
    planarityToleranceSpinBox_->setDecimals(6);
    planarityToleranceSpinBox_->setValue(0.02);
    planarityToleranceSpinBox_->setSingleStep(0.005);
    planarityToleranceSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    planarityToleranceSpinBox_->setAlignment(Qt::AlignRight);
    planarityToleranceSpinBox_->setFixedWidth(kAlignedControlWidth);
    planarityToleranceRow->addWidget(planarityToleranceSpinBox_);
    surfaceConversionLayout->addWidget(planarityToleranceRowWidget_);

    planarityStatusLabel_ = new QLabel(surfaceConversionPanel_);
    planarityStatusLabel_->setWordWrap(true);
    planarityStatusLabel_->setProperty("statusText", true);
    surfaceConversionLayout->addWidget(planarityStatusLabel_);

    alphaValueRowWidget_ = new QWidget(surfaceConversionPanel_);
    auto *alphaValueRow = new QHBoxLayout(alphaValueRowWidget_);
    alphaValueRow->setContentsMargins(0, 0, 0, 0);
    alphaValueRow->setSpacing(8);
    alphaValueRow->addWidget(new QLabel("Alpha value", alphaValueRowWidget_));
    alphaValueRow->addStretch(1);
    alphaValueSpinBox_ = new FormattedDoubleSpinBox(alphaValueRowWidget_);
    alphaValueSpinBox_->setRange(0.000001, 1.0e12);
    alphaValueSpinBox_->setValue(1.0);
    alphaValueSpinBox_->setSingleStep(0.1);
    alphaValueSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    alphaValueSpinBox_->setAlignment(Qt::AlignRight);
    alphaValueSpinBox_->setFixedWidth(kAlignedControlWidth);
    alphaValueRow->addWidget(alphaValueSpinBox_);
    surfaceConversionLayout->addWidget(alphaValueRowWidget_);

    automaticAlphaRowWidget_ = new QWidget(surfaceConversionPanel_);
    auto *automaticAlphaRow = new QHBoxLayout(automaticAlphaRowWidget_);
    automaticAlphaRow->setContentsMargins(0, 0, 0, 0);
    automaticAlphaCheckBox_ = new QCheckBox("Automatic alpha", automaticAlphaRowWidget_);
    automaticAlphaCheckBox_->setChecked(true);
    automaticAlphaRow->addWidget(automaticAlphaCheckBox_);
    surfaceConversionLayout->addWidget(automaticAlphaRowWidget_);

    automaticAlphaFactorRowWidget_ = new QWidget(surfaceConversionPanel_);
    auto *automaticAlphaFactorRow = new QHBoxLayout(automaticAlphaFactorRowWidget_);
    automaticAlphaFactorRow->setContentsMargins(0, 0, 0, 0);
    automaticAlphaFactorRow->setSpacing(8);
    automaticAlphaFactorRow->addWidget(new QLabel("Automatic factor", automaticAlphaFactorRowWidget_));
    automaticAlphaFactorRow->addStretch(1);
    automaticAlphaFactorSpinBox_ = new FormattedDoubleSpinBox(automaticAlphaFactorRowWidget_);
    automaticAlphaFactorSpinBox_->setRange(0.000001, 1.0e6);
    automaticAlphaFactorSpinBox_->setValue(4.0);
    automaticAlphaFactorSpinBox_->setSingleStep(0.25);
    automaticAlphaFactorSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    automaticAlphaFactorSpinBox_->setAlignment(Qt::AlignRight);
    automaticAlphaFactorSpinBox_->setFixedWidth(kAlignedControlWidth);
    automaticAlphaFactorRow->addWidget(automaticAlphaFactorSpinBox_);
    surfaceConversionLayout->addWidget(automaticAlphaFactorRowWidget_);

    largestComponentRowWidget_ = new QWidget(surfaceConversionPanel_);
    auto *largestComponentRow = new QHBoxLayout(largestComponentRowWidget_);
    largestComponentRow->setContentsMargins(0, 0, 0, 0);
    largestComponentCheckBox_ = new QCheckBox("Keep largest component only", largestComponentRowWidget_);
    largestComponentRow->addWidget(largestComponentCheckBox_);
    surfaceConversionLayout->addWidget(largestComponentRowWidget_);

    minimumComponentAreaRowWidget_ = new QWidget(surfaceConversionPanel_);
    auto *minimumComponentAreaRow = new QHBoxLayout(minimumComponentAreaRowWidget_);
    minimumComponentAreaRow->setContentsMargins(0, 0, 0, 0);
    minimumComponentAreaRow->setSpacing(8);
    minimumComponentAreaRow->addWidget(new QLabel("Minimum component area", minimumComponentAreaRowWidget_));
    minimumComponentAreaRow->addStretch(1);
    minimumComponentAreaSpinBox_ = new FormattedDoubleSpinBox(minimumComponentAreaRowWidget_);
    minimumComponentAreaSpinBox_->setRange(0.0, 1.0e18);
    minimumComponentAreaSpinBox_->setValue(0.0);
    minimumComponentAreaSpinBox_->setSingleStep(0.1);
    minimumComponentAreaSpinBox_->setButtonSymbols(QAbstractSpinBox::NoButtons);
    minimumComponentAreaSpinBox_->setAlignment(Qt::AlignRight);
    minimumComponentAreaSpinBox_->setFixedWidth(kAlignedControlWidth);
    minimumComponentAreaRow->addWidget(minimumComponentAreaSpinBox_);
    surfaceConversionLayout->addWidget(minimumComponentAreaRowWidget_);

    surfaceConversionLayout->addWidget(createSeparator(surfaceConversionPanel_));

    surfaceMethodRowWidget_ = new QWidget(convertToSurfacePage_);
    auto *surfaceMethodLayout = new QVBoxLayout(surfaceMethodRowWidget_);
    surfaceMethodLayout->setContentsMargins(0, 0, 0, 0);
    surfaceMethodLayout->setSpacing(4);
    surfaceMethodLabel_ = new QLabel("Surface reconstruction", surfaceMethodRowWidget_);
    surfaceMethodLayout->addWidget(surfaceMethodLabel_);

    surfaceMethodSelectorButton_ = createMenuButton("None", surfaceMethodRowWidget_);
    surfaceMethodSelectorButton_->setProperty("actionButton", true);
    surfaceMethodSelectorButton_->setProperty("fieldLike", true);
    surfaceMethodSelectorButton_->setProperty("selectorField", true);
    surfaceMethodSelectorButton_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    surfaceMethodLayout->addWidget(surfaceMethodSelectorButton_);
    surfaceConversionLayout->insertWidget(0, surfaceMethodRowWidget_);
    convertToSurfaceLayout->addWidget(surfaceConversionPanel_);
    connect(
        surfaceMethodSelectorButton_,
        &QPushButton::clicked,
        this,
        &PreprocessingSideMenu::openSurfaceMethodSelectionDialog);
    registerHelpTrigger(
        surfaceMethodRowWidget_,
        surfaceMethodRowWidget_,
        QString::fromUtf8(kSurfaceMethodHelpText));
    registerHelpTrigger(
        surfaceMethodLabel_,
        surfaceMethodRowWidget_,
        QString::fromUtf8(kSurfaceMethodHelpText));
    registerHelpTrigger(
        surfaceMethodSelectorButton_,
        surfaceMethodRowWidget_,
        QString::fromUtf8(kSurfaceMethodHelpText));

    normalEstimationRowWidget_ = new QWidget(convertToSurfacePage_);
    auto *normalEstimationLayout = new QVBoxLayout(normalEstimationRowWidget_);
    normalEstimationLayout->setContentsMargins(0, 0, 0, 0);
    normalEstimationLayout->setSpacing(4);
    normalEstimationLabel_ = new QLabel("Normal estimation", normalEstimationRowWidget_);
    normalEstimationLayout->addWidget(normalEstimationLabel_);

    normalMethodSelectorButton_ = createMenuButton("Select method", normalEstimationRowWidget_);
    normalMethodSelectorButton_->setProperty("actionButton", true);
    normalMethodSelectorButton_->setProperty("fieldLike", true);
    normalMethodSelectorButton_->setProperty("selectorField", true);
    normalMethodSelectorButton_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    normalEstimationLayout->addWidget(normalMethodSelectorButton_);
    surfaceConversionLayout->addWidget(normalEstimationRowWidget_);
    connect(
        normalMethodSelectorButton_,
        &QPushButton::clicked,
        this,
        &PreprocessingSideMenu::openNormalMethodSelectionDialog);
    registerHelpTrigger(
        normalEstimationRowWidget_,
        normalEstimationRowWidget_,
        QString::fromUtf8(kNormalEstimationHelpText));
    registerHelpTrigger(
        normalEstimationLabel_,
        normalEstimationRowWidget_,
        QString::fromUtf8(kNormalEstimationHelpText));
    registerHelpTrigger(
        normalMethodSelectorButton_,
        normalEstimationRowWidget_,
        QString::fromUtf8(kNormalEstimationHelpText));
    surfaceConversionButton_ = createMenuButton("Convert", convertToSurfacePage_);
    surfaceConversionButton_->setProperty("dialogAction", true);
    surfaceConversionButton_->setProperty("actionButton", true);
    surfaceConversionButton_->setProperty("fieldLike", true);
    surfaceConversionRowWidget_ = surfaceConversionButton_;
    surfaceConversionLayout->addWidget(surfaceConversionButton_);
    connect(surfaceConversionButton_, &QPushButton::clicked, this, [this] {
        const QString surfaceMethod = selectedSurfaceMethod_;
        const QString normalMethod = selectedNormalMethod_;
        if (surfaceMethod.isEmpty()) {
            emit placeholderMessageRequested(
                "Convert to surface",
                "Select a surface reconstruction method first.");
            return;
        }

        const std::optional<ReconstructionMethod> method =
            SurfaceReconstructorRegistry::methodFromDisplayName(surfaceMethod.toStdString());
        if (!method) {
            emit placeholderMessageRequested(
                "Convert to surface",
                "No reconstruction backend is registered for the selected method.");
            return;
        }
        const ReconstructionAvailability availability =
            SurfaceReconstructorRegistry().availability(*method);
        if (!availability.available) {
            emit placeholderMessageRequested(
                "Convert to surface",
                QString::fromStdString(availability.reason));
            return;
        }

        const NormalRequirement normalRequirement = normalRequirementForSurfaceMethod(surfaceMethod);
        const bool compatibleNormalSelected =
            compatibleNormalMethods(surfaceMethod).contains(normalMethod);
        if ((normalRequirement == NormalRequirement::Required ||
             normalRequirement == NormalRequirement::RequiredAndOriented) &&
            !compatibleNormalSelected) {
            emit placeholderMessageRequested(
                "Convert to surface",
                "Select a compatible normal estimation method first.");
            return;
        }

        SurfaceConversionSettings settings;
        settings.searchRadius = surfaceSearchRadiusSpinBox_->value();
        settings.neighbourCount = surfaceNeighboursSpinBox_->value();
        settings.scaleParameter = ballRadiusSpinBox_->value();
        settings.surfaceMethod = surfaceMethod;
        settings.normalMethod = normalMethod;
        settings.preservePlanarHeight = planarModeComboBox_->currentIndex() == 1;
        settings.planarityTolerance = planarityToleranceSpinBox_->value();
        settings.alphaValue = alphaValueSpinBox_->value();
        settings.automaticAlpha = automaticAlphaCheckBox_->isChecked();
        settings.automaticAlphaFactor = automaticAlphaFactorSpinBox_->value();
        settings.keepLargestComponentOnly = largestComponentCheckBox_->isChecked();
        settings.minimumComponentArea = minimumComponentAreaSpinBox_->value();
        if (*method == ReconstructionMethod::BallPivoting ||
            *method == ReconstructionMethod::GreedyProjection ||
            *method == ReconstructionMethod::Voxel ||
            *method == ReconstructionMethod::Rbf) {
        } else if (*method != ReconstructionMethod::Delaunay25D &&
                   *method != ReconstructionMethod::AlphaShapes) {
            emit placeholderMessageRequested(
                "Convert to surface",
                "The selected backend is not available from the point-cloud conversion workflow.");
            return;
        }

        emit surfaceReconstructionRequested(settings);
    });
    registerHelpTrigger(
        surfaceConversionButton_,
        surfaceConversionButton_,
        QString::fromUtf8(kSurfaceConversionHelpText));

    surfaceProgress_ = new ProcessingProgressWidget(convertToSurfacePage_);
    surfaceConversionLayout->addWidget(surfaceProgress_);

    surfaceConversionLayout->addWidget(createSeparator(surfaceConversionPanel_));

    surfaceApplyButton_ = createMenuButton("Apply", convertToSurfacePage_);
    surfaceApplyButton_->setProperty("dialogAction", true);
    surfaceApplyButton_->setProperty("actionButton", true);
    surfaceApplyButton_->setEnabled(false);
    convertToSurfaceLayout->addWidget(surfaceApplyButton_);
    connect(surfaceApplyButton_, &QPushButton::clicked, this, [this] {
        emit applySurfaceReconstructionRequested();
    });
    registerHelpTrigger(
        surfaceApplyButton_,
        surfaceApplyButton_,
        QString::fromUtf8(kSurfaceApplyHelpText));

    convertToSurfaceLayout->addWidget(createSeparator(convertToSurfacePage_));

    auto *surfaceActionLayout = new QHBoxLayout();
    surfaceActionLayout->setContentsMargins(0, 0, 0, 0);
    surfaceActionLayout->setSpacing(6);

    surfaceOkButton_ = createMenuButton("OK", convertToSurfacePage_);
    surfaceCancelButton_ = createMenuButton("Cancel", convertToSurfacePage_);
    surfaceOkButton_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    surfaceCancelButton_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    surfaceOkButton_->setProperty("dialogAction", true);
    surfaceCancelButton_->setProperty("dialogAction", true);
    surfaceOkButton_->setProperty("actionButton", true);
    surfaceCancelButton_->setProperty("actionButton", true);
    surfaceActionLayout->addWidget(surfaceOkButton_);
    surfaceActionLayout->addWidget(surfaceCancelButton_);
    convertToSurfaceLayout->addLayout(surfaceActionLayout);

    connect(surfaceOkButton_, &QPushButton::clicked, this, [this] {
        emit closeSurfaceReconstructionRequested();
        resetSurfaceConversionControls();
        showOperationList();
    });
    connect(surfaceCancelButton_, &QPushButton::clicked, this, [this] {
        emit cancelSurfaceReconstructionRequested();
        resetSurfaceConversionControls();
        showOperationList();
    });

    connect(planarModeComboBox_, &QComboBox::currentIndexChanged, this, [this](int) {
        updateSurfaceConversionEligibility();
    });
    connect(planarityToleranceSpinBox_, &QDoubleSpinBox::valueChanged, this, [this](double) {
        updateSurfaceConversionEligibility();
    });
    connect(alphaValueSpinBox_, &QDoubleSpinBox::valueChanged, this, [this](double) {
        updateSurfaceConversionEligibility();
    });
    connect(automaticAlphaCheckBox_, &QCheckBox::toggled, this, [this](bool) {
        updateSurfaceMethodControls();
    });
    connect(automaticAlphaFactorSpinBox_, &QDoubleSpinBox::valueChanged, this, [this](double) {
        updateSurfaceConversionEligibility();
    });
    connect(largestComponentCheckBox_, &QCheckBox::toggled, this, [this](bool) {
        updateSurfaceConversionEligibility();
    });
    connect(minimumComponentAreaSpinBox_, &QDoubleSpinBox::valueChanged, this, [this](double) {
        updateSurfaceConversionEligibility();
    });

    updateNormalMethodCompatibility();
    updateSurfaceMethodControls();

    parameterPlaceholderPage_ = new QWidget(this);
    auto *parameterLayout = new QVBoxLayout(parameterPlaceholderPage_);
    parameterLayout->setContentsMargins(10, 10, 10, 10);
    parameterLayout->setSpacing(4);

    for (const QString &label : {QString("test1"), QString("test2"), QString("test3")}) {
        parameterLayout->addWidget(createMenuButton(label, parameterPlaceholderPage_));
    }

    auto *actionLayout = new QHBoxLayout();
    actionLayout->setContentsMargins(0, 2, 0, 0);
    actionLayout->setSpacing(6);

    QPushButton *placeholderOkButton = createMenuButton("OK", parameterPlaceholderPage_);
    QPushButton *placeholderCancelButton = createMenuButton("Cancel", parameterPlaceholderPage_);
    placeholderOkButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    placeholderCancelButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    placeholderOkButton->setProperty("dialogAction", true);
    placeholderCancelButton->setProperty("dialogAction", true);

    actionLayout->addWidget(placeholderOkButton);
    actionLayout->addWidget(placeholderCancelButton);
    parameterLayout->addLayout(actionLayout);

    connect(
        placeholderOkButton,
        &QPushButton::clicked,
        this,
        &PreprocessingSideMenu::showOperationList);
    connect(
        placeholderCancelButton,
        &QPushButton::clicked,
        this,
        &PreprocessingSideMenu::showOperationList);

    stackedLayout_->addWidget(operationListPage_);
    stackedLayout_->addWidget(removeInvalidPointsPage_);
    stackedLayout_->addWidget(removeDuplicatesPage_);
    stackedLayout_->addWidget(convertToSurfacePage_);
    stackedLayout_->addWidget(parameterPlaceholderPage_);
    stackedLayout_->setCurrentWidget(operationListPage_);

    contextHelpPopup_ = new QLabel(parentWidget() ? parentWidget() : this);
    contextHelpPopup_->setObjectName("ContextHelpPopup");
    contextHelpPopup_->setAttribute(Qt::WA_TransparentForMouseEvents, true);
    contextHelpPopup_->setWordWrap(true);
    contextHelpPopup_->setFixedWidth(kContextHelpPopupWidth);
    contextHelpPopup_->setStyleSheet(QString::fromUtf8(kContextHelpPopupStyle));
    contextHelpPopup_->hide();

    contextHelpTimer_ = new QTimer(this);
    contextHelpTimer_->setSingleShot(true);
    connect(contextHelpTimer_, &QTimer::timeout, this, [this] {
        if (!pendingContextHelpAnchor_) {
            return;
        }

        showContextHelpPopup(pendingContextHelpAnchor_, pendingContextHelpText_);
    });

    hide();
}

QSize PreprocessingSideMenu::sizeHint() const
{
    if (!stackedLayout_ || stackedLayout_->count() == 0) {
        return QWidget::sizeHint();
    }

    if (QWidget *currentPage = stackedLayout_->currentWidget()) {
        return QSize(kMenuWidth, currentPage->sizeHint().height());
    }

    return QWidget::sizeHint();
}

QSize PreprocessingSideMenu::minimumSizeHint() const
{
    return sizeHint();
}

void PreprocessingSideMenu::showOperationList()
{
    if (autoSelectInProgress_) {
        return;
    }

    setManualSelectionEnabled(false);
    selectedOperation_.clear();
    stackedLayout_->setCurrentWidget(operationListPage_);
    updateGeometry();
    adjustSize();
    show();
    raise();
}

void PreprocessingSideMenu::hideMenu()
{
    if (autoSelectInProgress_) {
        return;
    }

    setManualSelectionEnabled(false);
    selectedOperation_.clear();
    stackedLayout_->setCurrentWidget(operationListPage_);
    updateGeometry();
    adjustSize();
    hide();
}

void PreprocessingSideMenu::showRemoveInvalidPointsMenu()
{
    if (autoSelectInProgress_) {
        return;
    }

    selectedOperation_ = "Remove invalid points";
    setManualSelectionEnabled(false);
    stackedLayout_->setCurrentWidget(removeInvalidPointsPage_);
    updateGeometry();
    adjustSize();
    show();
    raise();
    emit removeInvalidPointsMenuOpened();
}

void PreprocessingSideMenu::showRemoveDuplicatesMenu()
{
    if (autoSelectInProgress_) {
        return;
    }

    selectedOperation_ = "Downsampling";
    setManualSelectionEnabled(false);
    stackedLayout_->setCurrentWidget(removeDuplicatesPage_);
    updateGeometry();
    adjustSize();
    show();
    raise();
    emit removeDuplicatesMenuOpened();
}

void PreprocessingSideMenu::showConvertToSurfaceMenu()
{
    if (autoSelectInProgress_) {
        return;
    }

    selectedOperation_ = "Convert to surface";
    setManualSelectionEnabled(false);
    resetSurfaceConversionControls();
    stackedLayout_->setCurrentWidget(convertToSurfacePage_);
    updateGeometry();
    adjustSize();
    show();
    raise();
}

void PreprocessingSideMenu::setManualSelectionEnabled(bool enabled)
{
    if (!manualSelectionButton_ || !duplicateManualSelectionButton_) {
        return;
    }

    if (autoSelectInProgress_) {
        enabled = false;
    }

    manualSelectionButton_->blockSignals(true);
    manualSelectionButton_->setChecked(enabled);
    manualSelectionButton_->blockSignals(false);
    manualSelectionButton_->style()->unpolish(manualSelectionButton_);
    manualSelectionButton_->style()->polish(manualSelectionButton_);

    duplicateManualSelectionButton_->blockSignals(true);
    duplicateManualSelectionButton_->setChecked(enabled);
    duplicateManualSelectionButton_->blockSignals(false);
    duplicateManualSelectionButton_->style()->unpolish(duplicateManualSelectionButton_);
    duplicateManualSelectionButton_->style()->polish(duplicateManualSelectionButton_);
}

void PreprocessingSideMenu::setPointSelectionResultAvailable(bool available)
{
    pointSelectionResultAvailable_ = available;
    applyButton_->setEnabled(!autoSelectInProgress_ && available);
    duplicateApplyButton_->setEnabled(!autoSelectInProgress_ && available);
}

void PreprocessingSideMenu::setSurfaceInputState(
    bool available,
    std::optional<double> planarityIndicator,
    const QString &planarityError)
{
    surfaceInputAvailable_ = available;
    surfacePlanarityIndicator_ = planarityIndicator;
    surfacePlanarityError_ = planarityError;
    updateSurfaceConversionEligibility();
}

void PreprocessingSideMenu::setAutoSelectInProgress(bool inProgress)
{
    autoSelectInProgress_ = inProgress;

    manualSelectionButton_->setEnabled(!inProgress);
    duplicateManualSelectionButton_->setEnabled(!inProgress);
    radiusMaxSpinBox_->setEnabled(!inProgress);
    minimumNeighboursSpinBox_->setEnabled(!inProgress);
    autoSelectButton_->setEnabled(!inProgress);
    perfectDuplicatesAutoSelectButton_->setEnabled(!inProgress);
    distanceThresholdSpinBox_->setEnabled(!inProgress);
    nearDuplicatesAutoSelectButton_->setEnabled(!inProgress);
    applyButton_->setEnabled(!inProgress && pointSelectionResultAvailable_);
    duplicateApplyButton_->setEnabled(!inProgress && pointSelectionResultAvailable_);
    okButton_->setEnabled(!inProgress);
    duplicateOkButton_->setEnabled(!inProgress);
    cancelButton_->setEnabled(!inProgress);
    duplicateCancelButton_->setEnabled(!inProgress);
    surfaceSearchRadiusSpinBox_->setEnabled(!inProgress);
    surfaceNeighboursSpinBox_->setEnabled(!inProgress);
    ballRadiusSpinBox_->setEnabled(!inProgress);
    planarModeComboBox_->setEnabled(!inProgress);
    planarityToleranceSpinBox_->setEnabled(!inProgress);
    alphaValueSpinBox_->setEnabled(!inProgress);
    automaticAlphaCheckBox_->setEnabled(!inProgress);
    automaticAlphaFactorSpinBox_->setEnabled(!inProgress);
    largestComponentCheckBox_->setEnabled(!inProgress);
    minimumComponentAreaSpinBox_->setEnabled(!inProgress);
    surfaceMethodSelectorButton_->setEnabled(!inProgress);
    normalMethodSelectorButton_->setEnabled(!inProgress && normalEstimationRowWidget_->isVisible());
    surfaceApplyButton_->setEnabled(!inProgress && surfaceResultAvailable_);
    surfaceOkButton_->setEnabled(!inProgress);
    surfaceCancelButton_->setEnabled(!inProgress);
    if (inProgress) {
        setManualSelectionEnabled(false);
    }

    updateSurfaceConversionEligibility();

    updateGeometry();
    adjustSize();
}

bool PreprocessingSideMenu::isAutoSelectInProgress() const
{
    return autoSelectInProgress_;
}

void PreprocessingSideMenu::beginProcessingProgress()
{
    if (ProcessingProgressWidget *progressWidget = currentProgressWidget()) {
        if (progressWidget == surfaceProgress_) {
            surfaceResultAvailable_ = false;
            surfaceApplyButton_->setEnabled(false);
        }
        progressWidget->begin();
        updateGeometry();
        adjustSize();
    }
}

void PreprocessingSideMenu::updateProcessingProgress(const ProcessingProgress &progress)
{
    if (ProcessingProgressWidget *progressWidget = currentProgressWidget()) {
        progressWidget->setProgress(progress);
    }
}

void PreprocessingSideMenu::completeProcessingProgress()
{
    if (ProcessingProgressWidget *progressWidget = currentProgressWidget()) {
        progressWidget->complete();
        if (progressWidget == surfaceProgress_) {
            surfaceResultAvailable_ = true;
            surfaceApplyButton_->setEnabled(true);
        }
    }
}

void PreprocessingSideMenu::clearProcessingProgress()
{
    if (ProcessingProgressWidget *progressWidget = currentProgressWidget()) {
        progressWidget->clear();
        if (progressWidget == surfaceProgress_) {
            surfaceResultAvailable_ = false;
            surfaceApplyButton_->setEnabled(false);
        }
        updateGeometry();
        adjustSize();
    }
}

ProcessingProgressWidget *PreprocessingSideMenu::currentProgressWidget() const
{
    if (!stackedLayout_) {
        return nullptr;
    }
    if (stackedLayout_->currentWidget() == removeInvalidPointsPage_) {
        return removeInvalidProgress_;
    }
    if (stackedLayout_->currentWidget() == removeDuplicatesPage_) {
        return downsamplingProgress_;
    }
    if (stackedLayout_->currentWidget() == convertToSurfacePage_) {
        return surfaceProgress_;
    }
    return nullptr;
}

bool PreprocessingSideMenu::eventFilter(QObject *watched, QEvent *event)
{
    auto scheduleHelp = [this](QWidget *anchor, const QString &text) {
        scheduleContextHelpPopup(anchor, text, QCursor::pos());
    };

    auto maybeResetHelpTimer = [this](QWidget *anchor, const QString &text, QEvent *currentEvent) {
        if (!contextHelpTimer_ || !contextHelpTimer_->isActive() ||
            currentEvent->type() != QEvent::MouseMove) {
            return;
        }

        const auto *mouseEvent = static_cast<QMouseEvent *>(currentEvent);
        const QPoint currentGlobalPosition = mouseEvent->globalPosition().toPoint();
        if ((currentGlobalPosition - pendingContextHelpGlobalPosition_).manhattanLength() >
            kContextHelpStillThresholdPixels) {
            scheduleContextHelpPopup(anchor, text, currentGlobalPosition);
        }
    };

    auto handleHelpEvent = [this, watched, event, &scheduleHelp, &maybeResetHelpTimer](
                               QWidget *anchor,
                               const QString &text,
                               std::initializer_list<QObject *> triggers) {
        if (std::find(triggers.begin(), triggers.end(), watched) == triggers.end()) {
            return false;
        }

        if (event->type() == QEvent::Enter) {
            scheduleHelp(anchor, text);
        } else if (event->type() == QEvent::MouseMove) {
            maybeResetHelpTimer(anchor, text, event);
        } else if (event->type() == QEvent::MouseButtonPress) {
            hideContextHelpPopup();
        } else if (event->type() == QEvent::Leave && !cursorInsideRegisteredHelpTrigger()) {
            hideContextHelpPopup();
        }

        return true;
    };

    if (handleHelpEvent(
            normalEstimationRowWidget_,
            QString::fromUtf8(kNormalEstimationHelpText),
            {
                normalEstimationRowWidget_,
                normalEstimationLabel_,
                normalMethodSelectorButton_,
            }) ||
        handleHelpEvent(
            surfaceSearchRadiusRowWidget_,
            QString::fromUtf8(kSurfaceSearchRadiusHelpText),
            {surfaceSearchRadiusRowWidget_, surfaceSearchRadiusLabel_, surfaceSearchRadiusSpinBox_}) ||
        handleHelpEvent(
            surfaceNeighboursRowWidget_,
            QString::fromUtf8(kSurfaceNeighbourHelpText),
            {surfaceNeighboursRowWidget_, surfaceNeighboursLabel_, surfaceNeighboursSpinBox_}) ||
        handleHelpEvent(
            ballRadiusRowWidget_,
            QString::fromUtf8(kBallRadiusHelpText),
            {ballRadiusRowWidget_, ballRadiusLabel_, ballRadiusSpinBox_}) ||
        handleHelpEvent(
            surfaceMethodRowWidget_,
            QString::fromUtf8(kSurfaceMethodHelpText),
            {surfaceMethodRowWidget_, surfaceMethodLabel_, surfaceMethodSelectorButton_}) ||
        handleHelpEvent(
            surfaceConversionRowWidget_,
            QString::fromUtf8(kSurfaceConversionHelpText),
            {surfaceConversionButton_}) ||
        handleHelpEvent(
            surfaceApplyButton_,
            QString::fromUtf8(kSurfaceApplyHelpText),
            {surfaceApplyButton_})) {
        return QWidget::eventFilter(watched, event);
    }

    if (watched == manualSelectionRowWidget_ || watched == manualSelectionLabel_ || watched == manualSelectionButton_) {
        if (event->type() == QEvent::Enter) {
            scheduleHelp(manualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText));
        } else if (event->type() == QEvent::MouseMove) {
            maybeResetHelpTimer(manualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText), event);
        } else if (event->type() == QEvent::MouseButtonPress) {
            if (contextHelpTimer_) {
                contextHelpTimer_->stop();
            }
            showContextHelpPopup(manualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText));
        } else if (event->type() == QEvent::Leave && !cursorInsideRegisteredHelpTrigger()) {
            hideContextHelpPopup();
        }
    } else if (
        watched == duplicateManualSelectionRowWidget_ ||
        watched == duplicateManualSelectionLabel_ ||
        watched == duplicateManualSelectionButton_) {
        if (event->type() == QEvent::Enter) {
            scheduleHelp(duplicateManualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText));
        } else if (event->type() == QEvent::MouseMove) {
            maybeResetHelpTimer(duplicateManualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText), event);
        } else if (event->type() == QEvent::MouseButtonPress) {
            if (contextHelpTimer_) {
                contextHelpTimer_->stop();
            }
            showContextHelpPopup(duplicateManualSelectionRowWidget_, QString::fromUtf8(kManualSelectionHelpText));
        } else if (event->type() == QEvent::Leave && !cursorInsideRegisteredHelpTrigger()) {
            hideContextHelpPopup();
        }
    } else if (watched == radiusRowWidget_ || watched == radiusLabel_ || watched == radiusMaxSpinBox_) {
        if (event->type() == QEvent::Enter) {
            scheduleHelp(radiusRowWidget_, QString::fromUtf8(kRadiusHelpText));
        } else if (event->type() == QEvent::MouseMove) {
            maybeResetHelpTimer(radiusRowWidget_, QString::fromUtf8(kRadiusHelpText), event);
        } else if (event->type() == QEvent::MouseButtonPress) {
            if (contextHelpTimer_) {
                contextHelpTimer_->stop();
            }
            showContextHelpPopup(radiusRowWidget_, QString::fromUtf8(kRadiusHelpText));
        } else if (event->type() == QEvent::Leave && !cursorInsideRegisteredHelpTrigger()) {
            hideContextHelpPopup();
        }
    } else if (
        watched == neighboursRowWidget_ || watched == neighboursLabel_ || watched == minimumNeighboursSpinBox_) {
        if (event->type() == QEvent::Enter) {
            scheduleHelp(neighboursRowWidget_, QString::fromUtf8(kNeighbourHelpText));
        } else if (event->type() == QEvent::MouseMove) {
            maybeResetHelpTimer(neighboursRowWidget_, QString::fromUtf8(kNeighbourHelpText), event);
        } else if (event->type() == QEvent::MouseButtonPress) {
            if (contextHelpTimer_) {
                contextHelpTimer_->stop();
            }
            showContextHelpPopup(neighboursRowWidget_, QString::fromUtf8(kNeighbourHelpText));
        } else if (event->type() == QEvent::Leave && !cursorInsideRegisteredHelpTrigger()) {
            hideContextHelpPopup();
        }
    } else if (
        watched == perfectDuplicatesRowWidget_ ||
        watched == perfectDuplicatesLabel_ ||
        watched == perfectDuplicatesAutoSelectButton_) {
        if (event->type() == QEvent::Enter) {
            scheduleHelp(perfectDuplicatesRowWidget_, QString::fromUtf8(kPerfectDuplicatesHelpText));
        } else if (event->type() == QEvent::MouseMove) {
            maybeResetHelpTimer(perfectDuplicatesRowWidget_, QString::fromUtf8(kPerfectDuplicatesHelpText), event);
        } else if (event->type() == QEvent::MouseButtonPress) {
            if (contextHelpTimer_) {
                contextHelpTimer_->stop();
            }
            showContextHelpPopup(perfectDuplicatesRowWidget_, QString::fromUtf8(kPerfectDuplicatesHelpText));
        } else if (event->type() == QEvent::Leave && !cursorInsideRegisteredHelpTrigger()) {
            hideContextHelpPopup();
        }
    } else if (
        watched == distanceThresholdRowWidget_ ||
        watched == distanceThresholdLabel_ ||
        watched == distanceThresholdSpinBox_ ||
        watched == nearDuplicatesAutoSelectButton_) {
        if (event->type() == QEvent::Enter) {
            scheduleHelp(distanceThresholdRowWidget_, QString::fromUtf8(kDistanceThresholdHelpText));
        } else if (event->type() == QEvent::MouseMove) {
            maybeResetHelpTimer(distanceThresholdRowWidget_, QString::fromUtf8(kDistanceThresholdHelpText), event);
        } else if (event->type() == QEvent::MouseButtonPress) {
            if (contextHelpTimer_) {
                contextHelpTimer_->stop();
            }
            showContextHelpPopup(distanceThresholdRowWidget_, QString::fromUtf8(kDistanceThresholdHelpText));
        } else if (event->type() == QEvent::Leave && !cursorInsideRegisteredHelpTrigger()) {
            hideContextHelpPopup();
        }
    }

    return QWidget::eventFilter(watched, event);
}

void PreprocessingSideMenu::leaveEvent(QEvent *event)
{
    QWidget::leaveEvent(event);

    if (!cursorInsideRegisteredHelpTrigger()) {
        hideContextHelpPopup();
    }
}

void PreprocessingSideMenu::showParameterPlaceholder(const QString &operationName)
{
    if (autoSelectInProgress_) {
        return;
    }

    selectedOperation_ = operationName;
    show();
    raise();
    emit placeholderOperationRequested(operationName);
}

void PreprocessingSideMenu::openSurfaceMethodSelectionDialog()
{
    QWidget *centerTarget = parentWidget() ? parentWidget() : this;
    MethodSelectionDialog dialog(
        "Surface Reconstruction Method",
        surfaceMethodOptions(),
        selectedSurfaceMethod_,
        centerTarget);

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    selectedSurfaceMethod_ = dialog.selectedValue();
    surfaceMethodSelectorButton_->setText(selectedSurfaceMethod_);
    updateNormalMethodCompatibility();
    updateSurfaceMethodControls();
}

void PreprocessingSideMenu::updateSurfaceMethodControls()
{
    if (!surfaceSearchRadiusRowWidget_ || !surfaceNeighboursRowWidget_ ||
        !ballRadiusRowWidget_ || !surfaceConversionButton_) {
        return;
    }

    const bool ballPivoting = selectedSurfaceMethod_ == "Ball Pivoting Algorithm";
    const bool greedyProjection = selectedSurfaceMethod_ == "Greedy Projection Triangulation";
    const bool voxel = selectedSurfaceMethod_ == "Voxel-based Reconstruction";
    const bool rbf = selectedSurfaceMethod_ == "RBF Reconstruction";
    const bool delaunay = selectedSurfaceMethod_ == "Delaunay 2D / 2.5D Triangulation";
    const bool alphaShapes = selectedSurfaceMethod_ == "Alpha Shapes";
    const bool planarMethod = delaunay || alphaShapes;
    surfaceConversionPanel_->setVisible(true);
    surfaceSearchRadiusRowWidget_->setVisible(ballPivoting || greedyProjection || rbf);
    surfaceNeighboursRowWidget_->setVisible(ballPivoting || greedyProjection || rbf);
    ballRadiusRowWidget_->setVisible(ballPivoting || voxel);
    planarModeRowWidget_->setVisible(planarMethod);
    planarityToleranceRowWidget_->setVisible(planarMethod);
    planarityStatusLabel_->setVisible(planarMethod);
    automaticAlphaRowWidget_->setVisible(alphaShapes);
    alphaValueRowWidget_->setVisible(alphaShapes && !automaticAlphaCheckBox_->isChecked());
    automaticAlphaFactorRowWidget_->setVisible(alphaShapes && automaticAlphaCheckBox_->isChecked());
    largestComponentRowWidget_->setVisible(alphaShapes);
    minimumComponentAreaRowWidget_->setVisible(alphaShapes);
    surfaceSearchRadiusLabel_->setText("Search radius");
    surfaceNeighboursLabel_->setText(greedyProjection ? "Maximum neighbours" : "Number of neighbours");
    ballRadiusLabel_->setText(voxel ? "Voxel size" : "Ball radius");
    normalEstimationRowWidget_->setVisible(ballPivoting || greedyProjection || rbf);
    updateSurfaceConversionEligibility();
    updateGeometry();
    adjustSize();
}

void PreprocessingSideMenu::updateSurfaceConversionEligibility()
{
    if (!surfaceConversionButton_) {
        return;
    }

    QString unavailableReason;
    const std::optional<ReconstructionMethod> method =
        SurfaceReconstructorRegistry::methodFromDisplayName(selectedSurfaceMethod_.toStdString());
    if (selectedSurfaceMethod_.isEmpty()) {
        unavailableReason = "Select a surface reconstruction method first.";
    } else if (!method) {
        unavailableReason = "No reconstruction backend is registered for the selected method.";
    } else {
        const ReconstructionAvailability availability = SurfaceReconstructorRegistry().availability(*method);
        if (!availability.available) {
            unavailableReason = QString::fromStdString(availability.reason);
        }
    }

    if (unavailableReason.isEmpty() && !surfaceInputAvailable_) {
        unavailableReason = "No valid point cloud is selected.";
    }

    if (unavailableReason.isEmpty() && method) {
        const NormalRequirement normalRequirement = normalRequirementForSurfaceMethod(selectedSurfaceMethod_);
        if ((normalRequirement == NormalRequirement::Required ||
             normalRequirement == NormalRequirement::RequiredAndOriented) &&
            !compatibleNormalMethods(selectedSurfaceMethod_).contains(selectedNormalMethod_)) {
            unavailableReason = "Select a compatible normal estimation method first.";
        }

        if (*method == ReconstructionMethod::Delaunay25D ||
            *method == ReconstructionMethod::AlphaShapes) {
            const double tolerance = planarityToleranceSpinBox_->value();
            const bool planarEnough = surfacePlanarityIndicator_.has_value() &&
                *surfacePlanarityIndicator_ <= tolerance;
            planarityStatusLabel_->setProperty("planarityValid", planarEnough);
            if (surfacePlanarityIndicator_) {
                planarityStatusLabel_->setText(
                    QString("Planarity: %1 (limit %2)")
                        .arg(*surfacePlanarityIndicator_, 0, 'g', 4)
                        .arg(tolerance, 0, 'g', 4));
            } else {
                planarityStatusLabel_->setText(
                    surfacePlanarityError_.isEmpty()
                        ? "Planarity cannot be evaluated for this point cloud."
                        : surfacePlanarityError_);
            }
            planarityStatusLabel_->style()->unpolish(planarityStatusLabel_);
            planarityStatusLabel_->style()->polish(planarityStatusLabel_);
            if (!planarEnough && unavailableReason.isEmpty()) {
                unavailableReason = surfacePlanarityError_.isEmpty()
                    ? "The point cloud exceeds the selected planarity tolerance."
                    : surfacePlanarityError_;
            }
        }
    }

    surfaceConversionButton_->setEnabled(unavailableReason.isEmpty() && !autoSelectInProgress_);
    surfaceConversionButton_->setToolTip(unavailableReason);
}

void PreprocessingSideMenu::openNormalMethodSelectionDialog()
{
    QWidget *centerTarget = parentWidget() ? parentWidget() : this;
    MethodSelectionDialog dialog(
        "Normal Selection Method",
        normalMethodOptions(
            compatibleNormalMethods(selectedSurfaceMethod_)),
        selectedNormalMethod_,
        centerTarget);

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    selectedNormalMethod_ = dialog.selectedValue();
    normalMethodSelectorButton_->setText(selectedNormalMethod_);
    updateSurfaceConversionEligibility();
}

void PreprocessingSideMenu::updateNormalMethodCompatibility()
{
    if (!surfaceMethodSelectorButton_ || !normalMethodSelectorButton_) {
        return;
    }

    const NormalRequirement normalRequirement = normalRequirementForSurfaceMethod(selectedSurfaceMethod_);
    const QStringList compatibleMethods = compatibleNormalMethods(selectedSurfaceMethod_);
    if (normalRequirement == NormalRequirement::NotUsed ||
        (!selectedNormalMethod_.isEmpty() && !compatibleMethods.contains(selectedNormalMethod_))) {
        selectedNormalMethod_.clear();
    }

    normalMethodSelectorButton_->setText(
        selectedNormalMethod_.isEmpty()
            ? (normalRequirement == NormalRequirement::NotUsed ? "Not required" : "Select method")
            : selectedNormalMethod_);
    const bool inactive = normalRequirement == NormalRequirement::NotUsed;
    normalMethodSelectorButton_->setProperty("normalSelectorInactive", inactive);
    normalMethodSelectorButton_->setCursor(inactive ? Qt::ArrowCursor : Qt::PointingHandCursor);
    normalMethodSelectorButton_->style()->unpolish(normalMethodSelectorButton_);
    normalMethodSelectorButton_->style()->polish(normalMethodSelectorButton_);
    updateSurfaceConversionEligibility();
}

void PreprocessingSideMenu::resetSurfaceConversionControls()
{
    if (!surfaceMethodSelectorButton_ || !normalMethodSelectorButton_) {
        return;
    }

    selectedSurfaceMethod_.clear();
    selectedNormalMethod_.clear();
    surfaceResultAvailable_ = false;
    if (surfaceApplyButton_) {
        surfaceApplyButton_->setEnabled(false);
    }
    surfaceMethodSelectorButton_->setText("None");
    updateNormalMethodCompatibility();
    updateSurfaceMethodControls();
}

QPushButton *PreprocessingSideMenu::createMenuButton(const QString &label, QWidget *parent) const
{
    auto *button = new QPushButton(label, parent);
    button->setFlat(true);
    button->setCursor(Qt::PointingHandCursor);
    return button;
}

QFrame *PreprocessingSideMenu::createSeparator(QWidget *parent) const
{
    auto *separator = new QFrame(parent);
    separator->setProperty("menuSeparator", true);
    separator->setFrameShape(QFrame::HLine);
    separator->setFrameShadow(QFrame::Plain);
    return separator;
}

void PreprocessingSideMenu::registerHelpTrigger(QWidget *trigger, QWidget *anchor, const QString &text)
{
    if (!trigger || !anchor) {
        return;
    }

    trigger->setToolTip(QString());
    trigger->setMouseTracking(true);
    trigger->installEventFilter(this);
}

void PreprocessingSideMenu::scheduleContextHelpPopup(
    QWidget *anchor,
    const QString &text,
    const QPoint &globalPosition)
{
    if (!contextHelpTimer_ || !anchor) {
        return;
    }

    pendingContextHelpAnchor_ = anchor;
    pendingContextHelpText_ = text;
    pendingContextHelpGlobalPosition_ = globalPosition;
    contextHelpTimer_->start(kContextHelpDelayMs);
}

void PreprocessingSideMenu::showContextHelpPopup(QWidget *anchor, const QString &text)
{
    if (!contextHelpPopup_ || !anchor) {
        return;
    }

    QWidget *popupParent = contextHelpPopup_->parentWidget();
    if (!popupParent) {
        popupParent = this;
    }

    contextHelpPopup_->setText(text);
    contextHelpPopup_->adjustSize();

    const QPoint anchorTopLeft = anchor->mapTo(popupParent, QPoint(0, 0));
    const int popupX = x() - contextHelpPopup_->width() - kContextHelpPopupOffset;
    const int popupY =
        anchorTopLeft.y() + (anchor->height() - contextHelpPopup_->height()) / 2;

    contextHelpPopup_->move(std::max(0, popupX), std::max(0, popupY));
    contextHelpPopup_->raise();
    contextHelpPopup_->show();
}

void PreprocessingSideMenu::hideContextHelpPopup()
{
    if (contextHelpTimer_) {
        contextHelpTimer_->stop();
    }

    pendingContextHelpAnchor_ = nullptr;
    pendingContextHelpText_.clear();

    if (contextHelpPopup_) {
        contextHelpPopup_->hide();
    }
}

bool PreprocessingSideMenu::cursorInsideRegisteredHelpTrigger() const
{
    const QPoint globalCursorPosition = QCursor::pos();
    const QWidget *widgets[] = {
        manualSelectionRowWidget_,
        manualSelectionLabel_,
        manualSelectionButton_,
        duplicateManualSelectionRowWidget_,
        duplicateManualSelectionLabel_,
        duplicateManualSelectionButton_,
        radiusRowWidget_,
        radiusLabel_,
        radiusMaxSpinBox_,
        neighboursRowWidget_,
        neighboursLabel_,
        minimumNeighboursSpinBox_,
        perfectDuplicatesRowWidget_,
        perfectDuplicatesLabel_,
        perfectDuplicatesAutoSelectButton_,
        distanceThresholdRowWidget_,
        distanceThresholdLabel_,
        distanceThresholdSpinBox_,
        nearDuplicatesAutoSelectButton_,
        normalEstimationRowWidget_,
        normalEstimationLabel_,
        normalMethodSelectorButton_,
        surfaceSearchRadiusRowWidget_,
        surfaceSearchRadiusLabel_,
        surfaceSearchRadiusSpinBox_,
        surfaceNeighboursRowWidget_,
        surfaceNeighboursLabel_,
        surfaceNeighboursSpinBox_,
        ballRadiusRowWidget_,
        ballRadiusLabel_,
        ballRadiusSpinBox_,
        surfaceMethodRowWidget_,
        surfaceMethodLabel_,
        surfaceMethodSelectorButton_,
        surfaceConversionRowWidget_,
        surfaceConversionButton_,
        surfaceApplyButton_,
    };

    for (const QWidget *widget : widgets) {
        if (!widget || !widget->isVisible()) {
            continue;
        }

        const QRect globalGeometry(
            widget->mapToGlobal(QPoint(0, 0)),
            widget->size());
        if (globalGeometry.contains(globalCursorPosition)) {
            return true;
        }
    }

    return false;
}
