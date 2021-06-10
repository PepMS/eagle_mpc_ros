///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2021, University of Edinburgh, Istituto Italiano di Tecnologia
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef MULTICOPTER_MPC_RVIZ_WHOLE_BODY_TRAJECTORY_DISPLAY_H
#define MULTICOPTER_MPC_RVIZ_WHOLE_BODY_TRAJECTORY_DISPLAY_H

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rviz/message_filter_display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/robot/robot.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/ogre_helpers/billboard_line.h>

#include <eagle_mpc_msgs/WholeBodyTrajectory.h>

#include "eagle_mpc_rviz_plugins/PointVisual.h"
#include "eagle_mpc_rviz_plugins/ArrowVisual.h"

namespace Ogre {
class ManualObject;
}

namespace rviz {
class BoolProperty;
class EnumProperty;
class ColorProperty;
class FloatProperty;
class IntProperty;
class BillboardLine;
class VectorProperty;
class Axes;
}  // namespace rviz

namespace eagle_mpc_rviz_plugins {

/**
 * @class WholeBodyTrajectoryDisplay
 * @brief Displays a eagle_mpc_msgs::WholeBodyTrajectory message
 */
class WholeBodyTrajectoryDisplay : public rviz::MessageFilterDisplay<eagle_mpc_msgs::WholeBodyTrajectory> {
  Q_OBJECT
 public:
  /** @brief Constructor function */
  WholeBodyTrajectoryDisplay();

  /** @brief Destructor function */
  ~WholeBodyTrajectoryDisplay();

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
   * @param const eagle_mpc_msgs::WholeBodyTrajectory::ConstPtr& Whole-body state msg
   */
  void processMessage(const eagle_mpc_msgs::WholeBodyTrajectory::ConstPtr &msg) override;

 private Q_SLOTS:
  /**@{*/
  /** Helper function to apply color and alpha to all visuals.
   * Set the current color and alpha values for each visual */
  void updateTargetEnable();
  void updateRobotModel();
  void updateRobotVisualVisible();
  void updateRobotCollisionVisible();
  void updateRobotAlpha();
  void updateCoMEnable();
  void updateCoMStyle();
  void updateCoMLineProperties();
  void updatePlacementEnable();
  void updatePlacementProperties();

  void pushBackCoMAxes(const Ogre::Vector3 &axes_position, const Ogre::Quaternion &axes_orientation);

 private:
  void processTargetPosture();
  void processCoMTrajectory();
  void processTrajectory();

  /** @brief Loads a URDF from the ros-param named by our
   * "Robot Description" property, iterates through the links, and
   * loads any necessary models.
   */
  void loadRobotModel();

  /** @brief Clear the robot model */
  void clearRobotModel();

  void destroyObjects();

  /** @brief Whole-body state message */
  eagle_mpc_msgs::WholeBodyTrajectory::ConstPtr msg_;
  bool is_info_;

  /**@{*/
  /** Properties to show on side panel */
  rviz::Property *target_category_;
  rviz::Property *com_category_;
  rviz::Property *placement_category_;
  /**@}*/

  /**@{*/
  /** Object for visualization of the data */
  boost::shared_ptr<rviz::Robot> robot_;
  boost::shared_ptr<Ogre::ManualObject> com_manual_object_;
  boost::shared_ptr<rviz::BillboardLine> com_billboard_line_;
  std::vector<boost::shared_ptr<PointVisual>> com_points_;
  std::vector<boost::shared_ptr<rviz::Axes>> com_axes_;
  std::vector<boost::shared_ptr<rviz::Axes>> placement_axes_;
  /**@}*/

  /**@{*/
  /** Property objects for user-editable properties */
  rviz::BoolProperty *target_enable_property_;
  rviz::StringProperty *robot_model_property_;
  rviz::Property *robot_visual_enabled_property_;
  rviz::Property *robot_collision_enabled_property_;
  rviz::FloatProperty *robot_alpha_property_;
  rviz::BoolProperty *com_enable_property_;
  rviz::EnumProperty *com_style_property_;
  rviz::ColorProperty *com_color_property_;
  rviz::FloatProperty *com_alpha_property_;
  rviz::FloatProperty *com_line_width_property_;
  rviz::FloatProperty *com_scale_property_;
  rviz::BoolProperty *placement_enable_property_;
  rviz::FloatProperty *placement_alpha_property_;
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

  Ogre::Vector3 last_point_position_;
  enum LineStyle { BILLBOARDS, LINES, POINTS };

  /**@{*/
  /** Flag that indicates if the category are enable */
  bool target_enable_;
  bool com_enable_;
  bool com_axes_enable_;
  bool placement_enable_;
  /**@}*/
};

}  // namespace eagle_mpc_rviz_plugins

#endif  // eagle_mpc_rviz_WHOLE_BODY_STATE_DISPLAY_H