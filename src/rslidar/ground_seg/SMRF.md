## ground seg with SMRF algorithm
what i feel like is the most simplified version of ground segmentation that isn't just assuming the ground is a flat plane while being a first step to loading in some random model

*1. Creating Minimum Elevation Surface Map*
1) Divide the point cloud data into grids along the xy-plane, occupancy grid first?
2) Find the lowest elevation value for each grid element.
3) Combine all the Z_min values into a 2d matrix to create a minimum elevation surface map.

*2. Segment Surface Map*
Segments point cloud data into ground and unground points. 
1) Apply morphological opening operation on the minimum surface map. Applies an erosion filter followed by a dilation filter. 
2) Shape of the structuring element and its window radius define the search neighborhood for morphological opening. Start with a window radius of 1 pixel.
3) Calculate slope between minimum surface and opened surface maps at each grid element.
    - If the difference is greater than the elevation threshold, classify the pixel as nonground.
4) Execute steps 1 through 3 iteratively. Increasing the window radius by 1 pixel in each iteration until max_window_radius.
5) End result of iteration is a binary mask which each pixel of the point cloud is ground or nonground.

*3. Segment Original Point Cloud*
1) Apply the binary mask to the original minimum surface map to remove nonground grids. 
2) Fill the unfilled grids using image interpolation techniques to create an estimated elevation model. 
3) Calculate the elevation difference between each point in the original point cloud and the corresponding point in the estimated elevation model. If difference is greater than elevation_threshold, classify the pixel as nonground.
4) Multiply the slope of the elevation model at each point by elevation_scale, add result to the elevation_threshold value to identify ground points on steep slopes.