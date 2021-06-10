///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020, University of Edinburgh, Istituto Italiano di Tecnologia
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef MULTICOPTER_MPC_RVIZ_WHOLE_BODY_STATE_DISPLAY_H
#define MULTICOPTER_MPC_RVIZ_WHOLE_BODY_STATE_DISPLAY_H

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rviz/message_filter_display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/robot/robot.h>
#include <eagle_mpc_msgs/WholeBodyState.h>
#include "eagle_mpc_rviz_plugins/ArrowVisual.h"

namespace Ogre {
class SceneNode;
}

namespace rviz {
class BoolProperty;
class EnumProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class Shape;
}  // namespace rviz

namespace eagle_mpc_rviz_plugins {

/**
 * @class WholeBodyStateDisplay
 * @brief Displays a eagle_mpc_msgs::WholeBodyState message
 */
class WholeBodyStateDisplay : public rviz::MessageFilterDisplay<eagle_mpc_msgs::WholeBodyState> {
  Q_OBJECT
 public:
  /** @brief Constructor function */
  WholeBodyStateDisplay();

  /** @brief Destructor function */
  ~WholeBodyStateDisplay();

  /** @brief Initialization procedure of the plugin. */
  void onInitialize() override;

  /** @brief Enable procedure of the plugin. */
  void onEnable() override;

  /** @brief Disable procedure of the plugin. */
  void onDisable() override;

  /** @brief Called when the fixed frame changed */
  void fixedFrameChanged() override;

  /** @brief Clear the visuals by deleting their objects */
  void reset() override;

  /**
   * @brief Function to handle an incoming ROS message
   * This is our callback to handle an incoming message
   * @param const eagle_mpc_msgs::WholeBodyState::ConstPtr& Whole-body state msg
   */
  void processMessage(const eagle_mpc_msgs::WholeBodyState::ConstPtr &msg) override;

 private Q_SLOTS:
  /**@{*/
  /** Helper function to apply color and alpha to all visuals.
   * Set the current color and alpha values for each visual */
  void updateRobotEnable();
  void updateRobotModel();
  void updateRobotVisualVisible();
  void updateRobotCollisionVisible();
  void updateRobotAlpha();
  void updateThrustsEnable();
  void updateThrustsColorAndAlpha();
  void updateThrustsArrowGeometry();

 private:
  void processWholeBodyState();

  /** @brief Loads a URDF from the ros-param named by our
   * "Robot Description" property, iterates through the links, and
   * loads any necessary models.
   */
  void loadRobotModel();

  /** @brief Clear the robot model */
  void clearRobotModel();

  /** @brief Whole-body state message */
  eagle_mpc_msgs::WholeBodyState::ConstPtr msg_;
  bool is_info_;

  /**@{*/
  /** Properties to show on side panel */
  rviz::Property *robot_category_;
  rviz::Property *thrusts_category_;
  /**@}*/

  /**@{*/
  /** Object for visualization of the data */
  boost::shared_ptr<rviz::Robot> robot_;
  std::vector<boost::shared_ptr<ArrowVisual>> thrusts_visual_;
  /**@}*/

  /**@{*/
  /** Property objects for user-editable properties */
  rviz::BoolProperty *robot_enable_property_;
  rviz::StringProperty *robot_model_property_;
  rviz::Property *robot_visual_enabled_property_;
  rviz::Property *robot_collision_enabled_property_;
  rviz::FloatProperty *robot_alpha_property_;
  rviz::BoolProperty *thrusts_enable_property_;
  rviz::ColorProperty *thrusts_color_property_;
  rviz::FloatProperty *thrusts_alpha_property_;
  rviz::FloatProperty *thrusts_head_radius_property_;
  rviz::FloatProperty *thrusts_head_length_property_;
  rviz::FloatProperty *thrusts_shaft_radius_property_;
  rviz::FloatProperty *thrusts_shaft_length_property_;
  /**@}*/

  /**@{*/
  /** @brief Robot and whole-body variables */
  std::string robot_model_;
  bool initialized_model_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  double weight_;
  double gravity_;
  double friction_mu_;
  /**@}*/

  /**@{*/
  /** Flag that indicates if the category are enable */
  bool robot_enable_;
  bool thrusts_enable_;
  /**@}*/
};

}  // namespace eagle_mpc_rviz_plugins

#endif  // eagle_mpc_rviz_WHOLE_BODY_STATE_DISPLAY_H