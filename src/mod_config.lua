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
        enabled = true,
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
              -- This lowers the origin of the scanner to be more in at an expected hight
              -- since the original attachment point is on the bonnet and it's too high to see important obstacles
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
      -- TODO: describe what the "default mask" is exactly (which classes of objects are included / have their bit set to 1).
      raycast = {
        collision_mask = nil
      },
      imu = {
        enabled = true
      }
    },
    diesel_locomotive = {
      laser_scan = {
        enabled = false,
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
              -- This lowers the origin of the scanner to be more in at an expected hight
              -- since the original attachment point is on the bonnet and it's too high to see important obstacles
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
      },
      imu = {
        enabled = false
      }
    },
    fiat_1300dt = {
      laser_scan = {
        enabled = true,
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
              -- This lowers the origin of the scanner to be more in at an expected hight
              -- since the original attachment point is on the bonnet and it's too high to see important obstacles
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
      },
      imu = {
        enabled = true
      }
    },
    new_holland_tx_32 = {
      laser_scan = {
        enabled = true,
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
              -- This lowers the origin of the scanner to be more in at an expected hight
              -- since the original attachment point is on the bonnet and it's too high to see important obstacles
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
      },
      imu = {
        enabled = true
      }
    },
    lizard_caterpillar_836k_landfill_eiffage_18= {
      laser_scan = {
        enabled = true,
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
              -- This lowers the origin of the scanner to be more in at an expected hight
              -- since the original attachment point is on the bonnet and it's too high to see important obstacles
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
      },
      imu = {
        enabled = true
      }
    },
    default_vehicle = {
      laser_scan = {
        enabled = true,
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
              -- This lowers the origin of the scanner to be more in at an expected hight
              -- since the original attachment point is on the bonnet and it's too high to see important obstacles
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
      },
      imu = {
        enabled = true
      }
    },
  }
}

return mod_config
