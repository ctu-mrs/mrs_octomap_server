# MRS OctoMap Server

## Updates of this branch
1. Added call back function for the custom livox message: ``` callbackLivoxCloud ```.
2. Added Livox parameters (to be tuned).
3. To use the livox point cloud call back set the parameter ``` sensor_params/3d_lidar/livox/is_livox := true ```.
4. Some strange boxes appear around the drone during the take-off.

## Dependencies

* [octomap](https://github.com/ctu-mrs/octomap.git)
* [octomap_ros](https://github.com/OctoMap/octomap_ros)
