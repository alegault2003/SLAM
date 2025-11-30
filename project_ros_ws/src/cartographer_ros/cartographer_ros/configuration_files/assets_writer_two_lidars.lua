include "transform.lua"

options = {
  tracking_frame = "base_link",

  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 0.5,
      max_range = 30.0,
    },
    {
      action = "write_ply",
      filename = "point_cloud.ply",
    },
  }
}

MAP_BUILDER.use_trajectory_builder_3d = false

return options
