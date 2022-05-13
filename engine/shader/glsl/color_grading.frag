#version 310 es

#extension GL_GOOGLE_include_directive : enable

#include "constants.h"

layout(input_attachment_index = 0, set = 0, binding = 0) uniform highp subpassInput in_color;

layout(set = 0, binding = 1) uniform sampler2D color_grading_lut_texture_sampler;

layout(location = 0) out highp vec4 out_color;

void main()
{
    highp ivec2 lut_tex_size = textureSize(color_grading_lut_texture_sampler, 0);
    highp float _COLORS      = float(lut_tex_size.y);

    highp vec4 color       = subpassLoad(in_color).rgba;

    highp float total_grids = float(lut_tex_size.x / lut_tex_size.y);
    
    highp float grid_idx_float = color.b * total_grids;
    highp int grid_idx_floor = int(floor(grid_idx_float) + 0.5);
    highp int grid_idx_ceil = int(ceil(grid_idx_float) + 0.5);

    highp vec2 uv_floor = vec2(min(1.0, (color.r + float(grid_idx_floor)) / total_grids), color.g);
    highp vec2 uv_ceil = vec2(min(1.0, (color.r + float(grid_idx_ceil)) / total_grids), color.g);

    highp vec4 color_floor = texture(color_grading_lut_texture_sampler, uv_floor);
    highp vec4 color_ceil = texture(color_grading_lut_texture_sampler, uv_ceil);

    highp float grid_idx_flag = float(grid_idx_ceil - grid_idx_floor);
    highp float lerp_alpha = float(grid_idx_ceil) - grid_idx_float;
    color = grid_idx_flag * (lerp_alpha * color_floor + (1.0 - lerp_alpha) * color_ceil) + (1.0 - grid_idx_flag) * color_floor;
    
    //color = texture(color_grading_lut_texture_sampler, uv_floor);

    out_color = color;
}
