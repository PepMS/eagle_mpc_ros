///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2021, University of Edinburgh, Istituto Italiano di Tecnologia
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef MULTICOPTER_MPC_RVIZs_ARROW_VISUAL_H
#define MULTICOPTER_MPC_RVIZs_ARROW_VISUAL_H

#include <rviz/properties/quaternion_property.h>

namespace Ogre {
class Vector3;
class Quaternion;
}  // namespace Ogre

namespace rviz {
class Arrow;
}

namespace eagle_mpc_rviz_plugins {

/**
 * @class ArrowVisual
 * @brief Visualizes 3d arrow
 * Each instance of ArrowVisual represents the visualization of a single arrow
 * data. Currently it just shows an arrow with the direction and magnitude of
 * the acceleration vector
 */
class ArrowVisual {
 public:
  /**
   * @brief Constructor that creates the visual stuff and puts it into the scene
   * @param scene_manager  Manager the organization and rendering of the scene
   * @param parent_node    Represent the arrow as node in the scene
   */
  ArrowVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);

  /** @brief Destructor that removes the visual stuff from the scene */
  ~ArrowVisual();

  /**
   * @brief Configure the visual to show the arrow
   * @param position     Arrow position
   * @param orientation  Arrow orientation
   */
  void setArrow(const Ogre::Vector3 &position, const Ogre::Quaternion &orientation);

  /**
   * @brief Set the position of the coordinate frame
   * @param position  Frame position
   */
  void setFramePosition(const Ogre::Vector3 &position);

  /**
   * @brief Set the orientation of the coordinate frame
   * @param orientation Frame orientation
   */
  void setFrameOrientation(const Ogre::Quaternion &orientation);

  /**
   * @brief Set the color and alpha of the visual, which are user-editable
   * @param r  Red value
   * @param g  Green value
   * @param b  Blue value
   * @param a  Alpha value
   */
  void setColor(float r, float g, float b, float a);

  /**
   * @brief Set the parameters for this arrow
   * @param shaft_length    Length of the arrow's shaft
   * @param shaft_diameter  Diameter of the arrow's shaft
   * @param head_length     Length of the arrow's head
   * @param head_diameter   Diameter of the arrow's head
   */
  void setProperties(float shaft_length, float shaft_diameter, float head_length, float head_diameter);

 private:
  /** @brief The object implementing the arrow */
  rviz::Arrow *arrow_;

  /** @brief A SceneNode whose pose is set to match the coordinate frame */
  Ogre::SceneNode *frame_node_;

  /** @brief The SceneManager, kept here only so the destructor can ask it to
   * destroy the ``frame_node_``.
   */
  Ogre::SceneManager *scene_manager_;
};

}  // namespace eagle_mpc_rviz_plugins

#endif  // eagle_mpc_rviz_ARROW_VISUAL_H
