import dynosam_utils.evaluation.evaluation_lib as eval

from evo.core import lie_algebra, trajectory, metrics, transformations
import evo.tools.plot as evo_plot
import evo.core.units as evo_units
import numpy as np

import matplotlib.pyplot as plt

plt.rcdefaults()

def make_plot(results_folder_path, prefix):
    dataset_eval = eval.DatasetEvaluator(results_folder_path)
    data_files = dataset_eval.make_data_files(prefix)

    map_points_log_path = dataset_eval.create_existing_file_path(data_files.map_point_log)
    if map_points_log_path is None:
        print("Cannot find map points file")
        return

    camera_pose_eval = dataset_eval.create_camera_pose_evaluator(data_files)
    motion_eval = dataset_eval.create_motion_error_evaluator(data_files)

    plotter = eval.MapPlotter3D(map_points_log_path, camera_pose_eval, motion_eval)

    plot_collection = evo_plot.PlotCollection("Map")
    results = {}

    plotter.process(plot_collection, results)
    plot_collection.show()

    # MapPlotter3D,
    #             map_points_log_path,
    #             camera_pose_eval,
    #             motion_eval


# make_plot("/root/results/DynoSAM/acfr_1_moving_small", "rgbd_motion_world_backend")
# make_plot("/root/results/Dynosam_tro2024/kitti_0000", "rgbd_motion_world_backend")
# make_plot("/root/results/Dynosam_tro2024/carla_l2", "rgbd_motion_world_backend")
# make_plot("/root/results/Dynosam_tro2024/omd_swinging_4_unconstrained_sliding", "rgbd_motion_world_backend")
# make_plot("/root/results/DynoSAM/test_carla_l1/", "rgbd_motion_world_backend")
# make_plot("/root/results/DynoSAM/test_kitti_main/", "rgbd_motion_world_backend")
# make_plot("/root/results/DynoSAM/test_kitti_vo_0004/", "rgbd_motion_world_backend")
# make_plot("/root/results/DynoSAM/omd_vo_test/", "rgbd_motion_world_backend")
make_plot("/root/results/DynoSAM/test_kitti_main/", "rgbd_motion_world_backend")
# make_plot("/root/results/Dynosam_tro2024/kitti_0000/", "rgbd_motion_world_backend")
