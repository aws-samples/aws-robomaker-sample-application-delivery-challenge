/*
#
# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: MIT-0
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of this
# software and associated documentation files (the "Software"), to deal in the Software
# without restriction, including without limitation the rights to use, copy, modify,
# merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
# INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
*/
/*
This plugin moves SDF model with the poses defined by this plugin parameter.
You can define a key frame as following format.

  <key1><time>time in sec</time><pose>x y z roll pitck yaw</pose></key1>
  
Total time for whole animation move should also specified by <total_time> parameter.
As option, you also can specify if the animation should be repeated with <repeat> parameter.

Following is the example to apply this plugin to a model.

    <model name="bus2">
      <include>
        <uri>model://models/bus1</uri>
      </include>
      <plugin name="pose_animation_plugin" filename="libpose_animation_plugin.so">
        <repeat>true</repeat>
        <total_time>20</total_time>
        <key1>
          <time>0</time>
          <pose>2.7 0.053 0 0 0 0.087646</pose>
        </key1>
        <key2>
          <time>8</time>
          <pose>5.395567 0.324643 0 0 0 0.087646</pose>
        </key2>
        <key3>
          <time>16</time>
          <pose>2.7 0.053 0 0 0 0.087646</pose>
        </key3>
      </plugin> 
    </model>
*/

#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <boost/algorithm/string.hpp> 
#include <list>

namespace gazebo
{
  class MoveObject : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      if (!_sdf->HasElement("total_time")) {
        std::cout << "move_object plugin. parameter error. <total_time> element should be defined.";
        return;
      }        

      double total_time = _sdf->Get<double>("total_time");
      bool repeat = true;
      if (!_sdf->HasElement("repeat")) {
        repeat = _sdf->Get<bool>("repeat");
      }
      
      // create the animation
      gazebo::common::PoseAnimationPtr anim(new gazebo::common::PoseAnimation("animation", total_time, repeat));

      gazebo::common::PoseKeyFrame *key;

      int i = 1;
      while(true) {
        std::string keyTag = "key" + std::to_string(i);

        if (!_sdf->HasElement(keyTag)) {
          break;
        }        
        
        sdf::ElementPtr keyN = _sdf->GetElement(keyTag);
        
        double time = keyN->Get<double>("time");
        std::string pose = keyN->Get<std::string>("pose");

        std::list<std::string> list_string;
        
        boost::split(list_string, pose, boost::is_space(), boost::algorithm::token_compress_on);
 
        if (list_string.size() != 6) {
          std::cout << "move_object plugin. parameter error. number of elements in <pose> is wrong.";
          break;
        }
 
        auto it = list_string.begin();
        
        double x = std::stof(*it);
        ++it;
        double y = std::stof(*it);
        ++it;
        double z = std::stof(*it);
        ++it;
        double roll = std::stof(*it);
        ++it;
        double pitch = std::stof(*it);
        ++it;
        double yaw = std::stof(*it);
        
//        std::cout << "time:" << time << ":" << x << "," << y << "," << z << "," << roll << "," << pitch << "," << yaw;

        key = anim->CreateKeyFrame(time);
        key->Translation(ignition::math::Vector3d(x,y,z));
        key->Rotation(ignition::math::Quaterniond(roll, pitch, yaw));

        i++;
      }

      // set the animation
      _parent->SetAnimation(anim);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MoveObject)
}