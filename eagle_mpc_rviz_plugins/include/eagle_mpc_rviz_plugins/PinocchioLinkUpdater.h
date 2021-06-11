///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2020-2021, University of Edinburgh, Istituto Italiano di Tecnologia
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef MULTICOPTER_MPC_RVIZ_PINOCCHIO_LINK_UPDATER_H
#define MULTICOPTER_MPC_RVIZ_PINOCCHIO_LINK_UPDATER_H

#include <boost/function.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rviz/robot/link_updater.h>
#include <string>

namespace eagle_mpc_rviz_plugins {

class PinocchioLinkUpdater : public rviz::LinkUpdater {
public:
  typedef boost::function<void(rviz::StatusLevel, const std::string &,
                               const std::string &)>
      StatusCallback;

  PinocchioLinkUpdater(pinocchio::Model &model, pinocchio::Data &data,
                       const Eigen::Ref<const Eigen::VectorXd> &q,
                       const StatusCallback &status_cb = StatusCallback(),
                       const std::string &tf_prefix = std::string());

  bool getLinkTransforms(
      const std::string &link_name, Ogre::Vector3 &visual_position,
      Ogre::Quaternion &visual_orientation, Ogre::Vector3 &collision_position,
      Ogre::Quaternion &collision_orientation) const override;

  void setLinkStatus(rviz::StatusLevel level, const std::string &link_name,
                     const std::string &text) const override;

private:
  pinocchio::Model &model_;
  pinocchio::Data &data_;
  StatusCallback status_callback_;
  std::string tf_prefix_;
};

} // namespace eagle_mpc_rviz_plugins

#endif // eagle_mpc_rviz_PINOCCHIO_LINK_UPDATER_H