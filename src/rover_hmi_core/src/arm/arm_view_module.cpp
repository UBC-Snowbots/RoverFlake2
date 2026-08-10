// arm_view_module.cpp
// See arm_view_module.h for the panel overview.
//
// The FK chain below is transcribed from dev_arm_description_v2/urdf/
// dev_arm.urdf (joint <origin> xyz/rpy and <axis>). A painted projection was
// chosen over embedding RViz because it needs no subprocess, no xdotool, and
// draws fine at panel sizes; RvizModule stays the full-3D option.

#include "arm_view_module.h"
#include <rover_hmi_core/catppuccin.h>

#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QTimer>

#include <cmath>
#include <cstring>

#include <pluginlib/class_list_macros.hpp>

namespace {

// ---- tiny 3D affine math ---------------------------------------------------
struct Mat4 {
    double m[16];  // row-major
    static Mat4 identity() {
        Mat4 r{}; r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0; return r;
    }
    Mat4 operator*(const Mat4& o) const {
        Mat4 r{};
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                double s = 0;
                for (int k = 0; k < 4; k++) s += m[i*4+k] * o.m[k*4+j];
                r.m[i*4+j] = s;
            }
        return r;
    }
    void origin(double& x, double& y, double& z) const { x = m[3]; y = m[7]; z = m[11]; }
};

Mat4 xlate(double x, double y, double z) {
    Mat4 r = Mat4::identity(); r.m[3] = x; r.m[7] = y; r.m[11] = z; return r;
}

// URDF rpy: R = Rz(yaw) * Ry(pitch) * Rx(roll)
Mat4 rpy(double roll, double pitch, double yaw) {
    const double cr = std::cos(roll),  sr = std::sin(roll);
    const double cp = std::cos(pitch), sp = std::sin(pitch);
    const double cy = std::cos(yaw),   sy = std::sin(yaw);
    Mat4 r = Mat4::identity();
    r.m[0] = cy*cp;  r.m[1] = cy*sp*sr - sy*cr;  r.m[2]  = cy*sp*cr + sy*sr;
    r.m[4] = sy*cp;  r.m[5] = sy*sp*sr + cy*cr;  r.m[6]  = sy*sp*cr - cy*sr;
    r.m[8] = -sp;    r.m[9] = cp*sr;             r.m[10] = cp*cr;
    return r;
}

// Rodrigues rotation about a unit axis.
Mat4 axisAngle(double ux, double uy, double uz, double th) {
    const double c = std::cos(th), s = std::sin(th), t = 1.0 - c;
    Mat4 r = Mat4::identity();
    r.m[0] = t*ux*ux + c;     r.m[1] = t*ux*uy - s*uz;  r.m[2]  = t*ux*uz + s*uy;
    r.m[4] = t*ux*uy + s*uz;  r.m[5] = t*uy*uy + c;     r.m[6]  = t*uy*uz - s*ux;
    r.m[8] = t*ux*uz - s*uy;  r.m[9] = t*uy*uz + s*ux;  r.m[10] = t*uz*uz + c;
    return r;
}

// One movable joint of the chain: fixed origin transform, then rotation about
// `axis` by the joint angle.
struct ChainLink {
    double xyz[3];
    double rpy_[3];
    double axis[3];
};

// dev_arm_description_v2/urdf/dev_arm.urdf, in chain order.
// Indices line up with AxisIndex 0..5; joint_ee (fixed) is appended last.
constexpr ChainLink CHAIN[6] = {
    /*shoulder_joint*/ {{0.0875, 0.17, 0.0381},              {0, 0, -3.1416},                          {0, 0, 1}},
    /*link_1_joint*/   {{0.002825, -0.045, 0.075},           {1.5707963, 0, -1.5707963},               {0, 0, 1}},
    /*link1_link2*/    {{-0.5, 0, -0.06304},                 {0, 1.5707963, 0},                        {-1, 0, 0}},
    /*a4_rotation*/    {{0.0497861, 0.1485064, 0.1867034},   {-1.5707963, -0.8988565, 1.5707963},      {-1, 0, 0}},
    /*a5_rotation*/    {{0.241737, 0, 0.0240007},            {0.6719399, 1.5707963, 0},                {1, 0, 0}},
    /*a6_rotation*/    {{0.025, 0.0872905, 0.0572248},       {-1.5707963, -0.580279, 1.5707963},       {-1, 0, 0}},
};
constexpr double EE_XYZ[3] = {0.1, 0.0, 0.0};
constexpr double EE_RPY[3] = {0, 1.571, 0};

// Forward kinematics: returns the 8 chain points (base + 6 joints + EE tip)
// in base_link coordinates for the given joint angles (radians).
void forwardKinematics(const std::array<double, 6>& q, QPointF top[8], QPointF side[8],
                       double pts3d[8][3]) {
    Mat4 T = Mat4::identity();
    double x, y, z;
    T.origin(x, y, z);
    pts3d[0][0] = x; pts3d[0][1] = y; pts3d[0][2] = z;
    for (int i = 0; i < 6; i++) {
        T = T * xlate(CHAIN[i].xyz[0], CHAIN[i].xyz[1], CHAIN[i].xyz[2])
              * rpy(CHAIN[i].rpy_[0], CHAIN[i].rpy_[1], CHAIN[i].rpy_[2])
              * axisAngle(CHAIN[i].axis[0], CHAIN[i].axis[1], CHAIN[i].axis[2], q[i]);
        T.origin(x, y, z);
        pts3d[i+1][0] = x; pts3d[i+1][1] = y; pts3d[i+1][2] = z;
    }
    T = T * xlate(EE_XYZ[0], EE_XYZ[1], EE_XYZ[2]) * rpy(EE_RPY[0], EE_RPY[1], EE_RPY[2]);
    T.origin(x, y, z);
    pts3d[7][0] = x; pts3d[7][1] = y; pts3d[7][2] = z;

    for (int i = 0; i < 8; i++) {
        top[i]  = QPointF(pts3d[i][0], pts3d[i][1]);   // X-Y plane (from above)
        side[i] = QPointF(pts3d[i][0], pts3d[i][2]);   // X-Z plane (from the side)
    }
}

// The joint angles the URDF calls zero produce the pose RViz shows at zero;
// angles arrive already in URDF radians from either source.

const char* AXIS_SHORT[6] = {"A1", "A2", "A3", "A4", "A5", "A6"};

}  // namespace

// ---- painted widget ---------------------------------------------------------

class ArmViewWidget : public QWidget {
public:
    explicit ArmViewWidget(QWidget* parent = nullptr) : QWidget(parent) {
        setMinimumSize(260, 220);
        angles_.fill(0.0);
    }

    void setAngles(const std::array<double, 6>& rad, bool fresh) {
        angles_ = rad;
        fresh_ = fresh;
        have_data_ = true;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);
        p.fillRect(rect(), QColor(theme::BgPanel));

        // Two viewports stacked: top view above, side view below.
        const int gap = 8;
        QRectF topR (gap, gap, width() - 2*gap, height()/2.0 - 1.5*gap);
        QRectF sideR(gap, height()/2.0 + 0.5*gap, width() - 2*gap, height()/2.0 - 1.5*gap);

        QPointF top[8], side[8];
        double pts[8][3];
        forwardKinematics(angles_, top, side, pts);

        drawProjection(p, topR,  top,  "TOP (X-Y)");
        drawProjection(p, sideR, side, "SIDE (X-Z)");

        if (!have_data_) {
            p.setPen(QColor(theme::TextDim));
            p.drawText(rect(), Qt::AlignCenter, "no data");
        }
    }

private:
    void drawProjection(QPainter& p, const QRectF& r, const QPointF pts[8],
                        const char* label) {
        p.save();
        p.setPen(QPen(QColor(theme::BorderDim), 1));
        p.drawRect(r);
        p.setPen(QColor(theme::TextDim));
        p.setFont(QFont("monospace", theme::px(theme::FontSizeSm) * 0.75));
        p.drawText(r.adjusted(4, 2, -4, -2), Qt::AlignTop | Qt::AlignLeft, label);

        // Fit ±reach into the viewport (arm reach ~0.9 m; keep margins).
        const double reach = 1.0;
        const double s = std::min(r.width(), r.height()) / (2.0 * reach);
        QPointF c = r.center();
        auto map = [&](const QPointF& q) {
            return QPointF(c.x() + q.x() * s, c.y() - q.y() * s);
        };

        // faint origin crosshair
        p.setPen(QPen(QColor(theme::BorderDim), 1, Qt::DotLine));
        p.drawLine(QPointF(r.left(), c.y()), QPointF(r.right(), c.y()));
        p.drawLine(QPointF(c.x(), r.top()), QPointF(c.x(), r.bottom()));

        // links, colored per motor
        for (int i = 0; i < 7; i++) {
            const char* col = (i < 6) ? theme::MotorColors[i] : theme::Text;
            QPen pen(QColor(fresh_ ? col : theme::TextDim), 3, Qt::SolidLine, Qt::RoundCap);
            p.setPen(pen);
            p.drawLine(map(pts[i]), map(pts[i+1]));
        }
        // joints
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(fresh_ ? theme::Text : theme::TextDim));
        for (int i = 0; i < 8; i++)
            p.drawEllipse(map(pts[i]), 3, 3);
        // EE marker
        p.setBrush(QColor(fresh_ ? theme::Green : theme::TextDim));
        p.drawEllipse(map(pts[7]), 5, 5);
        p.restore();
    }

    std::array<double, 6> angles_{};
    bool have_data_ = false;
    bool fresh_ = false;
};

// ---- shared module scaffolding ----------------------------------------------

QWidget* ArmViewModuleBase::createWidget(QWidget* parent) {
    auto* w = new QWidget(parent);
    auto* layout = new QVBoxLayout(w);
    layout->setSpacing(6);

    status_ = new QLabel(statusHint());
    status_->setFont(QFont("monospace", theme::FontSizeSm));
    status_->setStyleSheet(QString("color: %1;").arg(theme::TextDim));
    status_->setWordWrap(true);
    layout->addWidget(status_);

    view_ = new ArmViewWidget();
    layout->addWidget(view_, 1);

    auto* grid = new QGridLayout();
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(2);
    for (int i = 0; i < 6; i++) {
        auto* name = new QLabel(AXIS_SHORT[i]);
        name->setFont(QFont("monospace", theme::FontSizeSm, QFont::Bold));
        name->setStyleSheet(QString("color: %1;").arg(theme::MotorColors[i]));
        grid->addWidget(name, i / 3, (i % 3) * 2);
        angle_labels_[i] = new QLabel("--");
        angle_labels_[i]->setFont(QFont("monospace", theme::FontSizeSm));
        grid->addWidget(angle_labels_[i], i / 3, (i % 3) * 2 + 1);
    }
    layout->addLayout(grid);
    return w;
}

void ArmViewModuleBase::applyAngles(const std::array<double, 6>& rad, bool fresh) {
    if (view_) view_->setAngles(rad, fresh);
    for (int i = 0; i < 6; i++)
        if (angle_labels_[i])
            angle_labels_[i]->setText(QString::number(rad[i], 'f', 2));
}

void ArmViewModuleBase::setStatus(const QString& text, const char* color) {
    if (!status_) return;
    status_->setText(text);
    status_->setStyleSheet(QString("color: %1;").arg(color));
}

// ---- Digital Twin: telemetry → FK view --------------------------------------

void ArmTwinModule::setNode(rclcpp::Node::SharedPtr node) {
    node_ = node;
    auto qos = rclcpp::QoS(1).reliable().durability_volatile();

    // /arm/feedback: axis-space output revs decoded from the CAN replies each
    // driver cycle (wrist differential already inverted) — the ground truth of
    // what the arm is DOING.
    feedback_sub_ = node->create_subscription<rover_msgs::msg::ArmCommand>(
        "/arm/feedback", qos,
        [this](rover_msgs::msg::ArmCommand::SharedPtr msg) {
            std::array<double, 6> rad{};
            for (int a = 0; a < 6; a++) {
                const double rev = (a < (int)msg->positions.size()) ? msg->positions[a] : NAN;
                rad[a] = std::isnan(rev)
                    ? ARM_JOINTS[a].initial_pos_rad     // not replying → draw homed
                    : motorRevToJointRad(a, rev);
            }
            last_msg_ = node_->now();
            applyAngles(rad, true);
            setStatus("LIVE — motor telemetry", theme::Green);
        });

    // Connection staleness from the moteus status stream (drives the dim-out).
    status_sub_ = node->create_subscription<rover_msgs::msg::MoteusArmStatus>(
        "/arm/moteus_feedback", qos,
        [this](rover_msgs::msg::MoteusArmStatus::SharedPtr msg) {
            if (msg->motors_replying == 0)
                setStatus("driver up, no motors replying", theme::Yellow);
        });
}

// ---- Sim Arm: /joint_states → FK view ---------------------------------------

void ArmSimModule::setNode(rclcpp::Node::SharedPtr node) {
    js_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
            std::array<double, 6> rad{};
            bool any = false;
            for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); i++) {
                for (int a = 0; a < 6; a++) {
                    if (msg->name[i] == ARM_JOINTS[a].urdf_joint_name) {
                        rad[a] = msg->position[i];
                        any = true;
                    }
                }
            }
            if (!any) return;   // not an arm message (e.g. gripper only)
            applyAngles(rad, true);
            setStatus("LIVE — /joint_states", theme::Cyan);
        });
}

PLUGINLIB_EXPORT_CLASS(ArmTwinModule, rover_hmi_core::GuiModule)
PLUGINLIB_EXPORT_CLASS(ArmSimModule, rover_hmi_core::GuiModule)
