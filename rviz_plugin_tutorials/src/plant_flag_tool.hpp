// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Willow Garage, Inc. nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef PLANT_FLAG_TOOL_HPP_
#define PLANT_FLAG_TOOL_HPP_

#include <string>
#include <vector>

#include <Ogre.h>

#include "rviz_common/tool.hpp"

namespace Ogre
{
class SceneNode;
}

namespace rviz_common::properties
{
class VectorProperty;
}

namespace rviz_common
{
class VisualizationManager;
class ViewportMouseEvent;
}

namespace rviz_plugin_tutorials
{

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz_common::Tool.  Every tool
// which can be added to the tool bar is a subclass of
// rviz_common::Tool.
class PlantFlagTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  PlantFlagTool();
  ~PlantFlagTool();

  virtual void onInitialize();

  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz_common::ViewportMouseEvent & event);

  virtual void load(const rviz_common::Config & config);
  virtual void save(rviz_common::Config config) const;

private:
  void makeFlag(const Ogre::Vector3 & position);

  std::vector<Ogre::SceneNode *> flag_nodes_;
  Ogre::SceneNode * moving_flag_node_;
  std::string flag_resource_;
  rviz_common::properties::VectorProperty * current_flag_property_;
};
// END_TUTORIAL

}  // end namespace rviz_plugin_tutorials

#endif  // PLANT_FLAG_TOOL_HPP_
