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
  control_only_active_vehicle = false,
  vehicle = {
    case_ih_7210_pro = {
      laser_scan = {
        num_rays = 64,
        num_layers = 2,
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
              y = -1.5,
              z = 0.0
          },
          rotation = {
              -- positive rotation over X rotates laser scanner down (ie: math.pi/6)
              x = 0.0,
              y = 0.0,
              z = 0.0
          }
        },
        laser_attachments = "raycastNode(0)"
      },
      -- the collision_mask for raycasting can be customized. Both
      -- decimal and hexadecimal notations are supported (ie: 9 and 0x9).
      -- if 'nil', the default mask will be used.
      raycast = {
        collision_mask = nil
      }
    },
    diesel_locomotive = {
      laser_scan = {
        num_rays = 64,
        num_layers = 2,
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
              y = -1.5,
              z = 0.0
          },
          rotation = {
              -- positive rotation over X rotates laser scanner down (ie: math.pi/6)
              x = 0.0,
              y = 0.0,
              z = 0.0
          }
        },
        laser_attachments = "raycastNode(0)"
      },
      -- the collision_mask for raycasting can be customized. Both
      -- decimal and hexadecimal notations are supported (ie: 9 and 0x9).
      raycast = {
        collision_mask = nil
      }
    },
    fiat_1300dt = {
      laser_scan = {
        num_rays = 64,
        num_layers = 2,
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
              y = -1.5,
              z = 0.0
          },
          rotation = {
              -- positive rotation over X rotates laser scanner down (ie: math.pi/6)
              x = 0.0,
              y = 0.0,
              z = 0.0
          }
        },
        laser_attachments = "raycastNode(0)"
      },
      -- the collision_mask for raycasting can be customized. Both
      -- decimal and hexadecimal notations are supported (ie: 9 and 0x9).
      raycast = {
        collision_mask = nil
      }
    },
    new_holland_tx_32 = {
      laser_scan = {
        num_rays = 64,
        num_layers = 2,
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
              y = -3,
              z = 0.0
          },
          rotation = {
              -- positive rotation over X rotates laser scanner down (ie: math.pi/6)
              x = 0.0,
              y = 0.0,
              z = 0.0
          }
        },
        laser_attachments = "raycastNode(0)"
      },
      -- the collision_mask for raycasting can be customized. Both
      -- decimal and hexadecimal notations are supported (ie: 9 and 0x9).
      raycast = {
        collision_mask = nil
      }
    }
  }
}

return mod_config
