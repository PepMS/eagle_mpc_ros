///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020, University of Edinburgh, Istituto Italiano di Tecnologia
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "eagle_mpc_rviz_plugins/WholeBodyTrajectoryDisplay.h"
#include "eagle_mpc_rviz_plugins/PinocchioLinkUpdater.h"
#include <Eigen/Dense>
#include <OgreManualObject.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <QTimer>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/parsers/urdf.hpp>

using namespace rviz;

namespace eagle_mpc_rviz_plugins {

void linkUpdaterStatusFunction(rviz::StatusLevel level, const std::string &link_name, const std::string &text,
                               WholeBodyTrajectoryDisplay *display) {
  display->setStatus(level, QString::fromStdString(link_name), QString::fromStdString(text));
}

WholeBodyTrajectoryDisplay::WholeBodyTrajectoryDisplay()
    : is_info_(false),
      initialized_model_(false),
      weight_(0.),
      gravity_(9.81),
      target_enable_(true),
      com_enable_(true),
      com_axes_enable_(true),
      placement_enable_(true) {
  // Category Groups
  target_category_ = new rviz::Property("Robot", QVariant(), "", this);
  com_category_ = new rviz::Property("Center of Mass", QVariant(), "", this);
  placement_category_ = new rviz::Property("Placements", QVariant(), "", this);

  // Robot properties
  target_enable_property_ = new BoolProperty("Enable", true, "Enable/disable the Target display", target_category_,
                                             SLOT(updateTargetEnable()), this);
  robot_model_property_ = new StringProperty("Robot Description", "robot_description",
                                             "Name of the parameter to search for to load the robot description.",
                                             target_category_, SLOT(updateRobotModel()), this);
  robot_visual_enabled_property_ =
      new Property("Robot Visual", true, "Whether to display the visual representation of the robot.",
                   target_category_, SLOT(updateRobotVisualVisible()), this);
  robot_collision_enabled_property_ =
      new Property("Robot Collision", false, "Whether to display the collision representation of the robot.",
                   target_category_, SLOT(updateRobotCollisionVisible()), this);
  robot_alpha_property_ = new FloatProperty("Robot Alpha", 1., "Amount of transparency to apply to the links.",
                                            target_category_, SLOT(updateRobotAlpha()), this);
  robot_alpha_property_->setMin(0.0);
  robot_alpha_property_->setMax(1.0);

  // Center of Mass trajectory properties
  com_enable_property_ =
      new BoolProperty("Enable", true, "Enable/disable the CoM display", com_category_, SLOT(updateCoMEnable()), this);
  com_style_property_ =
      new EnumProperty("Line Style", "Billboards", "The rendering operation to use to draw the grid lines.",
                       com_category_, SLOT(updateCoMStyle()), this);
  com_style_property_->addOption("Billboards", BILLBOARDS);
  com_style_property_->addOption("Lines", LINES);
  com_style_property_->addOption("Points", POINTS);
  com_line_width_property_ = new FloatProperty("Line Width", 0.01,
                                               "The width, in meters, of each path line. "
                                               "Only works with the 'Billboards' and 'Points' style.",
                                               com_category_, SLOT(updateCoMLineProperties()), this);
  com_line_width_property_->setMin(0.001);
  com_line_width_property_->show();
  com_color_property_ = new ColorProperty("Line Color", QColor(0, 85, 255), "Color to draw the path.", com_category_,
                                          SLOT(updateCoMLineProperties()), this);
  com_scale_property_ = new FloatProperty("Axes Scale", 1.0, "The scale of the axes that describe the orientation.",
                                          com_category_, SLOT(updateCoMLineProperties()), this);
  com_alpha_property_ = new FloatProperty("Alpha", 1.0, "Amount of transparency to apply to the trajectory.",
                                          com_category_, SLOT(updateCoMLineProperties()), this);
  com_alpha_property_->setMin(0);
  com_alpha_property_->setMax(1);

  // Placement costs
  placement_enable_property_ = new BoolProperty("Enable", true, "Enable/disable the WP display", placement_category_,
                                                SLOT(updatePlacementEnable()), this);
  placement_alpha_property_ = new FloatProperty("Alpha", 1.0, "Amount of transparency to apply to the waypoints.",
                                                placement_category_, SLOT(updatePlacementProperties()), this);
  placement_alpha_property_->setMin(0.0);
  placement_alpha_property_->setMax(1.0);
}

WholeBodyTrajectoryDisplay::~WholeBodyTrajectoryDisplay() {
  clearRobotModel();
  destroyObjects();
}

void WholeBodyTrajectoryDisplay::onInitialize() {
  MFDClass::onInitialize();
  robot_.reset(new rviz::Robot(scene_node_, context_, "Robot: " + getName().toStdString(), this));
  updateRobotVisualVisible();
  updateRobotCollisionVisible();
  updateRobotAlpha();
  updateCoMStyle();
  // updateCoMLineProperties();
  updatePlacementProperties();
}

void WholeBodyTrajectoryDisplay::onEnable() {
  MFDClass::onEnable();
  loadRobotModel();
  updateTargetEnable();
  updateCoMEnable();
  updatePlacementEnable();
}

void WholeBodyTrajectoryDisplay::onDisable() {
  MFDClass::onDisable();
  robot_->setVisible(false);
  clearRobotModel();
}

void WholeBodyTrajectoryDisplay::fixedFrameChanged() {
  if (is_info_) {
    processTargetPosture();
    processCoMTrajectory();
  }
}

void WholeBodyTrajectoryDisplay::reset() { MFDClass::reset(); }

void WholeBodyTrajectoryDisplay::loadRobotModel() {
  std::string content;
  if (!update_nh_.getParam(robot_model_property_->getStdString(), content)) {
    std::string loc;
    if (update_nh_.searchParam(robot_model_property_->getStdString(), loc)) {
      update_nh_.getParam(loc, content);
    } else {
      clearRobotModel();
      setStatus(
          StatusProperty::Error, "URDF",
          "Parameter [" + robot_model_property_->getString() + "] does not exist, and was not found by searchParam()");
      // try again in a second
      QTimer::singleShot(1000, this, SLOT(updateRobotModel()));
      return;
    }
  }
  if (content.empty()) {
    clearRobotModel();
    setStatus(StatusProperty::Error, "URDF", "URDF is empty");
    return;
  }
  if (content == robot_model_) {
    return;
  }
  robot_model_ = content;
  urdf::Model descr;
  if (!descr.initString(robot_model_)) {
    clearRobotModel();
    setStatus(StatusProperty::Error, "URDF", "Failed to parse URDF model");
    return;
  }

  // Initializing the dynamics from the URDF model
  pinocchio::urdf::buildModelFromXML(robot_model_, pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);
  gravity_ = model_.gravity.linear().norm();
  weight_ = pinocchio::computeTotalMass(model_) * gravity_;
  initialized_model_ = true;
  robot_->load(descr);
  updateTargetEnable();
  setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
}

void WholeBodyTrajectoryDisplay::clearRobotModel() {
  clearStatuses();
  robot_model_.clear();
  model_ = pinocchio::Model();
  data_ = pinocchio::Data();
  initialized_model_ = false;
}

void WholeBodyTrajectoryDisplay::destroyObjects() {
  com_manual_object_.reset();
  com_billboard_line_.reset();
  com_points_.clear();
}

void WholeBodyTrajectoryDisplay::updateTargetEnable() {
  target_enable_ = target_enable_property_->getBool();
  if (target_enable_) {
    robot_->setVisible(true);
  } else {
    robot_->setVisible(false);
  }
}

void WholeBodyTrajectoryDisplay::updateRobotModel() {
  if (isEnabled()) {
    loadRobotModel();
    context_->queueRender();
  }
}

void WholeBodyTrajectoryDisplay::updateRobotVisualVisible() {
  robot_->setVisualVisible(robot_visual_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateRobotCollisionVisible() {
  robot_->setCollisionVisible(robot_collision_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateRobotAlpha() {
  robot_->setAlpha(robot_alpha_property_->getFloat());
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateCoMEnable() {
  com_enable_ = com_enable_property_->getBool();
  if (!com_enable_) {
    com_billboard_line_.reset();
    com_manual_object_.reset();
    com_points_.clear();
  }
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updateCoMStyle() {
  LineStyle style = (LineStyle)com_style_property_->getOptionInt();
  switch (style) {
    case BILLBOARDS:
      com_line_width_property_->show();
      com_manual_object_.reset();
      com_points_.clear();
      break;
    case LINES:
      com_line_width_property_->hide();
      com_billboard_line_.reset();
      com_points_.clear();
      break;
    case POINTS:
      com_line_width_property_->show();
      com_manual_object_.reset();
      com_billboard_line_.reset();
      break;
  }
  if (is_info_) {
    processCoMTrajectory();
  }
}

void WholeBodyTrajectoryDisplay::updateCoMLineProperties() {
  LineStyle style = (LineStyle)com_style_property_->getOptionInt();
  float line_width = com_line_width_property_->getFloat();
  float scale = com_scale_property_->getFloat();
  com_axes_enable_ = true;
  if (scale == 0) {
    com_axes_enable_ = false;
    com_axes_.clear();
  }
  Ogre::ColourValue color = com_color_property_->getOgreColor();
  color.a = com_alpha_property_->getFloat();
  if (style == BILLBOARDS) {
    if (com_billboard_line_ != nullptr) {
      com_billboard_line_->setLineWidth(line_width);
      com_billboard_line_->setColor(color.r, color.g, color.b, color.a);
    }
    if (com_axes_enable_) {
      std::size_t num_axes = com_axes_.size();
      for (std::size_t i = 0; i < num_axes; ++i) {
        Ogre::ColourValue x_color = com_axes_[i]->getDefaultXColor();
        Ogre::ColourValue y_color = com_axes_[i]->getDefaultYColor();
        Ogre::ColourValue z_color = com_axes_[i]->getDefaultZColor();
        x_color.a = com_alpha_property_->getFloat();
        y_color.a = com_alpha_property_->getFloat();
        z_color.a = com_alpha_property_->getFloat();
        com_axes_[i]->setXColor(x_color);
        com_axes_[i]->setYColor(y_color);
        com_axes_[i]->setZColor(z_color);
        com_axes_[i]->getSceneNode()->setVisible(true);
        com_axes_[i]->setScale(Ogre::Vector3(scale, scale, scale));
      }
    }
  } else if (style == LINES) {
    // we have to process again the base trajectory
    if (is_info_) processCoMTrajectory();
  } else {
    std::size_t n_points = com_points_.size();
    for (std::size_t i = 0; i < n_points; ++i) {
      com_points_[i]->setColor(color.r, color.g, color.b, color.a);
      com_points_[i]->setRadius(line_width);
    }
    if (com_axes_enable_) {
      std::size_t num_axes = com_axes_.size();
      for (std::size_t i = 0; i < num_axes; ++i) {
        Ogre::ColourValue x_color = com_axes_[i]->getDefaultXColor();
        Ogre::ColourValue y_color = com_axes_[i]->getDefaultYColor();
        Ogre::ColourValue z_color = com_axes_[i]->getDefaultZColor();
        x_color.a = com_alpha_property_->getFloat();
        y_color.a = com_alpha_property_->getFloat();
        z_color.a = com_alpha_property_->getFloat();
        com_axes_[i]->setXColor(x_color);
        com_axes_[i]->setYColor(y_color);
        com_axes_[i]->setZColor(z_color);
        com_axes_[i]->getSceneNode()->setVisible(true);
        com_axes_[i]->setScale(Ogre::Vector3(scale, scale, scale));
      }
    }
  }
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updatePlacementEnable() {
  placement_enable_ = placement_enable_property_->getBool();
  if (!placement_enable_) {
    placement_axes_.clear();
  }
  context_->queueRender();
}

void WholeBodyTrajectoryDisplay::updatePlacementProperties() {
  double alpha = placement_alpha_property_->getFloat();
  std::size_t num_wp = placement_axes_.size();
  for (std::size_t i = 0; i < num_wp; i++) {
    Ogre::ColourValue x_color = placement_axes_[i]->getDefaultXColor();
    Ogre::ColourValue y_color = placement_axes_[i]->getDefaultYColor();
    Ogre::ColourValue z_color = placement_axes_[i]->getDefaultZColor();
    x_color.a = alpha;
    y_color.a = alpha;
    z_color.a = alpha;
    placement_axes_[i]->setXColor(x_color);
    placement_axes_[i]->setYColor(y_color);
    placement_axes_[i]->setZColor(z_color);
    placement_axes_[i]->getSceneNode()->setVisible(true);
    // wp_axes_[i]->setScale(Ogre::Vector3(scale, scale, scale));
  }
}

void WholeBodyTrajectoryDisplay::processMessage(const eagle_mpc_msgs::WholeBodyTrajectory::ConstPtr &msg) {
  msg_ = msg;
  is_info_ = true;
  processTargetPosture();
  processCoMTrajectory();
  processTrajectory();
}

void WholeBodyTrajectoryDisplay::processTargetPosture() {
  // Checking if the urdf model was initialized
  if (!initialized_model_) return;

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Point message.  If
  // it fails, we can't do anything else so we return.
  if (target_enable_) {
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if (!context_->getFrameManager()->getTransform(msg_->header.frame_id, msg_->header.stamp, position, orientation)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg_->header.frame_id.c_str(),
                qPrintable(fixed_frame_));
      return;
    }

    // Display the robot
    const eagle_mpc_msgs::WholeBodyState &state = msg_->robot_state_trajectory.back();
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
    q(0) = state.floating_base.pose.position.x;
    q(1) = state.floating_base.pose.position.y;
    q(2) = state.floating_base.pose.position.z;
    q(3) = state.floating_base.pose.orientation.x;
    q(4) = state.floating_base.pose.orientation.y;
    q(5) = state.floating_base.pose.orientation.z;
    q(6) = state.floating_base.pose.orientation.w;
    std::size_t n_joints = state.joints.size();
    for (std::size_t j = 0; j < n_joints; ++j) {
      pinocchio::JointIndex jointId = model_.getJointId(state.joints[j].name) - 2;
      q(jointId + 7) = state.joints[j].position;
    }
    robot_->update(PinocchioLinkUpdater(model_, data_, q, boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this)));
  }
}

void WholeBodyTrajectoryDisplay::processCoMTrajectory() {
  // Lookup transform into fixed frame
  if (com_enable_) {
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if (!context_->getFrameManager()->getTransform(msg_->header, position, orientation)) {
      ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg_->header.frame_id.c_str(),
                qPrintable(fixed_frame_));
    }
    Ogre::Matrix4 transform(orientation);
    transform.setTrans(position);

    // Visualization of the base trajectory
    // Getting the base trajectory style
    LineStyle base_style = (LineStyle)com_style_property_->getOptionInt();

    // Getting the base trajectory color
    Ogre::ColourValue base_color = com_color_property_->getOgreColor();
    base_color.a = com_alpha_property_->getFloat();

    // Visualization of the base trajectory
    std::size_t n_points = msg_->robot_state_trajectory.size();
    com_axes_.clear();
    for (std::size_t i = 0; i < n_points; ++i) {
      const eagle_mpc_msgs::WholeBodyState &state = msg_->robot_state_trajectory[i];
      // Obtaining the CoM position and the base orientation
      Ogre::Vector3 com_position;
      Ogre::Quaternion base_orientation;
      com_position.x = state.floating_base.pose.position.x;
      com_position.y = state.floating_base.pose.position.y;
      com_position.z = state.floating_base.pose.position.z;
      base_orientation.x = state.floating_base.pose.orientation.x;
      base_orientation.y = state.floating_base.pose.orientation.y;
      base_orientation.z = state.floating_base.pose.orientation.z;
      base_orientation.w = state.floating_base.pose.orientation.w;
      // sanity checks
      if (!(std::isfinite(com_position.x) && std::isfinite(com_position.y) && std::isfinite(com_position.z))) {
        std::cerr << "CoM position is not finite, resetting to zero" << std::endl;
        com_position.x = 0.0;
        com_position.y = 0.0;
        com_position.z = 0.0;
      }
      if (!(std::isfinite(base_orientation.x) && std::isfinite(base_orientation.y) &&
            std::isfinite(base_orientation.z) && std::isfinite(base_orientation.w))) {
        std::cerr << "Body orientation is not finite, resetting to [0 0 0 1]" << std::endl;
        base_orientation.x = 0.;
        base_orientation.y = 0.;
        base_orientation.z = 0.;
        base_orientation.w = 1.;
      }

      Ogre::Vector3 point_position = transform * com_position;
      if (com_axes_enable_) {
        pushBackCoMAxes(point_position, base_orientation * orientation);
      }
      switch (base_style) {
        case BILLBOARDS: {
          // Getting the base line width
          if (i == 0) {
            float base_line_width = com_line_width_property_->getFloat();
            com_billboard_line_.reset(new rviz::BillboardLine(scene_manager_, scene_node_));
            com_billboard_line_->setNumLines(1);
            com_billboard_line_->setMaxPointsPerLine(n_points);
            com_billboard_line_->setLineWidth(base_line_width);
          }
          com_billboard_line_->addPoint(point_position, base_color);
        } break;
        case LINES: {
          if (i == 0) {
            com_manual_object_.reset(scene_manager_->createManualObject());
            com_manual_object_->setDynamic(true);
            scene_node_->attachObject(com_manual_object_.get());
            com_manual_object_->estimateVertexCount(n_points);
            com_manual_object_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_STRIP);
          }
          com_manual_object_->position(point_position.x, point_position.y, point_position.z);
          com_manual_object_->colour(base_color);
        } break;
        case POINTS: {
          if (i == 0) {
            com_points_.clear();
          }
          float base_line_width = com_line_width_property_->getFloat();
          // We are keeping a vector of CoM visual pointers. This creates the next
          // one and stores it in the vector
          boost::shared_ptr<PointVisual> point_visual;
          point_visual.reset(new PointVisual(context_->getSceneManager(), scene_node_));
          point_visual->setColor(base_color.r, base_color.g, base_color.b, base_color.a);
          point_visual->setRadius(base_line_width);
          point_visual->setPoint(com_position);
          point_visual->setFramePosition(position);
          point_visual->setFrameOrientation(orientation);
          // And send it to the end of the vector
          com_points_.push_back(point_visual);
        } break;
      }
    }
    if (base_style == LINES) {
      com_manual_object_->end();
    }
  }
}

void WholeBodyTrajectoryDisplay::processTrajectory() {
  if (placement_enable_) {
    std::size_t wp_num = msg_->trajectory.placements.size();
    placement_axes_.clear();
    for (std::size_t i = 0; i < wp_num; ++i) {
      boost::shared_ptr<rviz::Axes> axes;
      axes.reset(new Axes(scene_manager_, scene_node_, 0.1, 0.02));

      Ogre::Vector3 position;
      position.x = msg_->trajectory.placements[i].pose.position.x;
      position.y = msg_->trajectory.placements[i].pose.position.y;
      position.z = msg_->trajectory.placements[i].pose.position.z;
      Ogre::Quaternion orientation;
      orientation.x = msg_->trajectory.placements[i].pose.orientation.x;
      orientation.y = msg_->trajectory.placements[i].pose.orientation.y;
      orientation.z = msg_->trajectory.placements[i].pose.orientation.z;
      orientation.w = msg_->trajectory.placements[i].pose.orientation.w;
      axes->setPosition(position);
      axes->setOrientation(orientation);

      Ogre::ColourValue x_color = axes->getDefaultXColor();
      Ogre::ColourValue y_color = axes->getDefaultYColor();
      Ogre::ColourValue z_color = axes->getDefaultZColor();
      x_color.a = placement_alpha_property_->getFloat();
      y_color.a = placement_alpha_property_->getFloat();
      z_color.a = placement_alpha_property_->getFloat();
      axes->setXColor(x_color);
      axes->setYColor(y_color);
      axes->setZColor(z_color);
      axes->getSceneNode()->setVisible(true);
      // axes->setScale(Ogre::Vector3(scale, scale, scale));
      placement_axes_.push_back(axes);
    }
  }
}

void WholeBodyTrajectoryDisplay::pushBackCoMAxes(const Ogre::Vector3 &axes_position,
                                                 const Ogre::Quaternion &axes_orientation) {
  // We are keeping a vector of CoM frame pointers. This creates the next
  // one and stores it in the vector
  float scale = com_scale_property_->getFloat();
  // Adding the frame with a distant from the last one
  float sq_distant = axes_position.squaredDistance(last_point_position_);
  if (sq_distant >= scale * scale * 0.0032) {
    boost::shared_ptr<rviz::Axes> axes;
    axes.reset(new Axes(scene_manager_, scene_node_, 0.04, 0.008));
    axes->setPosition(axes_position);
    axes->setOrientation(axes_orientation);
    Ogre::ColourValue x_color = axes->getDefaultXColor();
    Ogre::ColourValue y_color = axes->getDefaultYColor();
    Ogre::ColourValue z_color = axes->getDefaultZColor();
    x_color.a = com_alpha_property_->getFloat();
    y_color.a = com_alpha_property_->getFloat();
    z_color.a = com_alpha_property_->getFloat();
    axes->setXColor(x_color);
    axes->setYColor(y_color);
    axes->setZColor(z_color);
    axes->getSceneNode()->setVisible(true);
    axes->setScale(Ogre::Vector3(scale, scale, scale));
    com_axes_.push_back(axes);
    last_point_position_ = axes_position;
  }
}

}  // namespace eagle_mpc_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_mpc_rviz_plugins::WholeBodyTrajectoryDisplay, rviz::Display)
