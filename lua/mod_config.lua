--[[
Copyright (c) 2021, TU Delft

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

author: Ting-Chia Chiang
author: G.A. vd. Hoorn
--]]

mod_config =
{
  controlled_vehicle = {
    base_link_frame_id = "case_ih_7210_pro_9"
  },
  laser_scan = {
    num_rays = 128,
    num_layers = 5,
    angle_min = -math.pi,
    angle_max = math.pi,
    range_min = 0.1,
    range_max = 30,
    ignore_terrain = true,
    inter_layer_distance = 0.1,
    -- apply a transform to first laser scan frame
    laser_transform = {
      translation = {
          x = 0.0,
          y = 0.0,
          z = 0.0
      },
      rotation = {
          x = -math.pi/6,
          y = 0.0,
          z = 0.0
      }
    }
  }
}

return mod_config
