///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2021, University of Edinburgh, Istituto Italiano di Tecnologia
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "eagle_mpc_rviz_plugins/WholeBodyStateDisplay.h"
#include "eagle_mpc_rviz_plugins/PinocchioLinkUpdater.h"
#include <Eigen/Dense>
#include <QTimer>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/parsers/urdf.hpp>

using namespace rviz;

namespace eagle_mpc_rviz_plugins {

void linkUpdaterStatusFunction(rviz::StatusLevel level, const std::string &link_name, const std::string &text,
                               WholeBodyStateDisplay *display) {
  display->setStatus(level, QString::fromStdString(link_name), QString::fromStdString(text));
}

WholeBodyStateDisplay::WholeBodyStateDisplay()
    : is_info_(false), initialized_model_(false), weight_(0.), gravity_(9.81), thrusts_enable_(true) {
  // Category Groups
  robot_category_ = new rviz::Property("Robot", QVariant(), "", this);
  thrusts_category_ = new rviz::Property("Thrusts", QVariant(), "", this);

  // Robot properties
  robot_enable_property_ = new BoolProperty("Enable", true, "Enable/disable the robot display", robot_category_,
                                            SLOT(updateRobotEnable()), this);
  robot_model_property_ = new StringProperty("Robot Description", "robot_description",
                                             "Name of the parameter to search for to load the robot description.",
                                             robot_category_, SLOT(updateRobotModel()), this);
  robot_visual_enabled_property_ =
      new Property("Robot Visual", true, "Whether to display the visual representation of the robot.", robot_category_,
                   SLOT(updateRobotVisualVisible()), this);
  robot_collision_enabled_property_ =
      new Property("Robot Collision", false, "Whether to display the collision representation of the robot.",
                   robot_category_, SLOT(updateRobotCollisionVisible()), this);
  robot_alpha_property_ = new FloatProperty("Robot Alpha", 1., "Amount of transparency to apply to the links.",
                                            robot_category_, SLOT(updateRobotAlpha()), this);
  robot_alpha_property_->setMin(0.0);
  robot_alpha_property_->setMax(1.0);

  // Thrusts properties
  thrusts_enable_property_ = new BoolProperty("Enable", true, "Enable/disable the thrusts arrows display",
                                              thrusts_category_, SLOT(updateThrustsEnable()), this);
  thrusts_color_property_ = new ColorProperty("Color", QColor(85, 0, 255), "Color of the thrusts arrow.",
                                              thrusts_category_, SLOT(updateThrustsColorAndAlpha()), this);
  thrusts_alpha_property_ = new FloatProperty("Alpha", 1.0, "Transparency of the arrow", thrusts_category_,
                                              SLOT(updateThrustsColorAndAlpha()), this);
  thrusts_alpha_property_->setMin(0.0);
  thrusts_alpha_property_->setMax(1.0);
  thrusts_shaft_length_property_ = new FloatProperty("Shaft Length", 0.8, "Length of the arrow's shaft, in meters.",
                                                     thrusts_category_, SLOT(updateThrustsArrowGeometry()), this);
  thrusts_shaft_radius_property_ = new FloatProperty("Shaft Radius", 0.01, "Radius of the arrow's shaft, in meters.",
                                                     thrusts_category_, SLOT(updateThrustsArrowGeometry()), this);
  thrusts_head_length_property_ = new FloatProperty("Head Length", 0.08, "Length of the arrow's head, in meters.",
                                                    thrusts_category_, SLOT(updateThrustsArrowGeometry()), this);
  thrusts_head_radius_property_ = new FloatProperty("Head Radius", 0.03, "Radius of the arrow's head, in meters.",
                                                    thrusts_category_, SLOT(updateThrustsArrowGeometry()), this);
}

WholeBodyStateDisplay::~WholeBodyStateDisplay() {}

void WholeBodyStateDisplay::onInitialize() {
  MFDClass::onInitialize();
  robot_.reset(new rviz::Robot(scene_node_, context_, "Robot: " + getName().toStdString(), this));
  updateRobotVisualVisible();
  updateRobotCollisionVisible();
  updateRobotAlpha();
  updateThrustsColorAndAlpha();
}

void WholeBodyStateDisplay::onEnable() {
  MFDClass::onEnable();
  loadRobotModel();
  updateRobotEnable();
  updateThrustsEnable();
}

void WholeBodyStateDisplay::onDisable() {
  MFDClass::onDisable();
  robot_->setVisible(false);
  clearRobotModel();
}

void WholeBodyStateDisplay::fixedFrameChanged() {
  if (is_info_) {
    processWholeBodyState();
  }
}

void WholeBodyStateDisplay::reset() {
  MFDClass::reset();
  thrusts_visual_.clear();
}

void WholeBodyStateDisplay::loadRobotModel() {
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
  updateRobotEnable();
  setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
}

void WholeBodyStateDisplay::clearRobotModel() {
  clearStatuses();
  robot_model_.clear();
  model_ = pinocchio::Model();
  data_ = pinocchio::Data();
  initialized_model_ = false;
}

void WholeBodyStateDisplay::updateRobotEnable() {
  robot_enable_ = robot_enable_property_->getBool();
  if (robot_enable_) {
    robot_->setVisible(true);
  } else {
    robot_->setVisible(false);
  }
}

void WholeBodyStateDisplay::updateRobotModel() {
  if (isEnabled()) {
    loadRobotModel();
    context_->queueRender();
  }
}

void WholeBodyStateDisplay::updateRobotVisualVisible() {
  robot_->setVisualVisible(robot_visual_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void WholeBodyStateDisplay::updateRobotCollisionVisible() {
  robot_->setCollisionVisible(robot_collision_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void WholeBodyStateDisplay::updateRobotAlpha() {
  robot_->setAlpha(robot_alpha_property_->getFloat());
  context_->queueRender();
}

void WholeBodyStateDisplay::updateThrustsEnable() {
  thrusts_enable_ = thrusts_enable_property_->getBool();
  if (thrusts_visual_.size() != 0 && !thrusts_enable_) {
    thrusts_visual_.clear();
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::updateThrustsColorAndAlpha() {

  Ogre::ColourValue color = thrusts_color_property_->getOgreColor();
  color.a = thrusts_alpha_property_->getFloat();
  for (size_t i = 0; i < thrusts_visual_.size(); ++i) {
    thrusts_visual_[i]->setColor(color.r, color.g, color.b, color.a);
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::updateThrustsArrowGeometry() {
  const float &shaft_length = thrusts_shaft_length_property_->getFloat();
  const float &shaft_radius = thrusts_shaft_radius_property_->getFloat();
  const float &head_length = thrusts_head_length_property_->getFloat();
  const float &head_radius = thrusts_head_radius_property_->getFloat();
  for (size_t i = 0; i < thrusts_visual_.size(); ++i) {
    thrusts_visual_[i]->setProperties(shaft_length, shaft_radius, head_length, head_radius);
  }
  context_->queueRender();
}

void WholeBodyStateDisplay::processMessage(const eagle_mpc_msgs::WholeBodyState::ConstPtr &msg) {
  msg_ = msg;
  is_info_ = true;
  processWholeBodyState();
}

void WholeBodyStateDisplay::processWholeBodyState() {
  // Checking if the urdf model was initialized
  if (!initialized_model_) return;

  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Point message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(msg_->header.frame_id, msg_->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg_->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }

  // Display the robot
  if (robot_enable_) {
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nq);
    q(0) = msg_->floating_base.pose.position.x;
    q(1) = msg_->floating_base.pose.position.y;
    q(2) = msg_->floating_base.pose.position.z;
    q(3) = msg_->floating_base.pose.orientation.x;
    q(4) = msg_->floating_base.pose.orientation.y;
    q(5) = msg_->floating_base.pose.orientation.z;
    q(6) = msg_->floating_base.pose.orientation.w;

    std::size_t n_joints = msg_->joints.size();
    for (std::size_t j = 0; j < n_joints; ++j) {
      pinocchio::JointIndex jointId =
          model_.getJointId(msg_->joints[j].name) - 2;
      q(jointId + 7) = msg_->joints[j].position;
    }    
    robot_->update(PinocchioLinkUpdater(model_, data_, q, boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this)));

    if (thrusts_enable_) {
      std::size_t n_rotors = msg_->thrusts.size();
      thrusts_visual_.clear();
      for (std::size_t i = 0; i < n_rotors; ++i) {
        boost::shared_ptr<ArrowVisual> arrow;
        arrow.reset(new ArrowVisual(context_->getSceneManager(), scene_node_));
        Ogre::Vector3 thrust_pos(msg_->thrusts[i].pose.position.x, msg_->thrusts[i].pose.position.y,
                                 msg_->thrusts[i].pose.position.z);
        Ogre::Quaternion thrust_orientation(
            msg_->thrusts[i].pose.orientation.w, msg_->thrusts[i].pose.orientation.x,
            msg_->thrusts[i].pose.orientation.y, msg_->thrusts[i].pose.orientation.z);

        arrow->setArrow(thrust_pos, thrust_orientation);

        // Setting the arrow color and properties
        Ogre::ColourValue color = thrusts_color_property_->getOgreColor();
        color.a = thrusts_alpha_property_->getFloat();
        arrow->setColor(color.r, color.g, color.b, color.a);
        const float &shaft_length = -thrusts_shaft_length_property_->getFloat() *
                                    msg_->thrusts[i].thrust_command / msg_->thrusts[i].thrust_max;
        const float &shaft_radius = thrusts_shaft_radius_property_->getFloat();
        const float &head_length = -thrusts_head_length_property_->getFloat();
        const float &head_radius = thrusts_head_radius_property_->getFloat();
        arrow->setProperties(shaft_length, shaft_radius, head_length, head_radius);

        // And send it to the end of the vector
        if (std::isfinite(shaft_length) && std::isfinite(shaft_radius) && std::isfinite(head_length) &&
            std::isfinite(head_radius)) {
          thrusts_visual_.push_back(arrow);
        }
      }
    }
  }
}

}  // namespace eagle_mpc_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_mpc_rviz_plugins::WholeBodyStateDisplay, rviz::Display)
