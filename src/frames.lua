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

frames = {}

function frames.create_attached_node(parentNodeId, transformName, tran_x, tran_y, tran_z, rot_x, rot_y, rot_z)
    -- create a node
    local node = createTransformGroup(transformName)
    -- link the node to parentNodeId
    link(parentNodeId, node)
    -- apply a transfrom to the new node
    setTranslation(node, tran_x, tran_y, tran_z)
    setRotation(node, rot_x, rot_y, rot_z)
    return node
end

return frames

