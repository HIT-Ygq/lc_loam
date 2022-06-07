options = {
     max_distance = 90.0,
     min_distance = 3,
     scan_lines = 16,
     scan_period = 0.1,
     vertical_angle = 2.0,

     map_resolution_edge = 0.4,
     map_resolution_surf = 0.8,
     num_range_data = 40,
     num_thread_pool = 4,
     speed_filter = 0.001,

     -- 后端
     distance_threshold = 5,   --  50
     loop_closure_rotation_weight = 1000.0,
     loop_closure_translation_weight = 1000.0,
     submap_threshold = 47,  -- 47
     node_id_threshold = 7000, -- 7000
     scan_to_scan_min = 5,
     num_of_submap_threshold = 30,  --  100


     matcher_rotation_weight = 2000.0;
     matcher_translation_weight = 500.0,
     optimize_every_n_nodes = 60,
     fix_z_in_3d = true
}

return options
