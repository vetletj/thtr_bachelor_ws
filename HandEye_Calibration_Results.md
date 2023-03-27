# Hand to Eye Calibration

- Marker size: Test with a few different marker sizes to find the one that provides the best detection and pose estimation performance in your setup. Typically, larger markers are easier to detect but may require the camera to be further away, while smaller markers allow for closer camera placement but can be more challenging to detect and estimate their pose accurately.

- Dictionary: ArUco markers are available in several predefined dictionaries, each with different marker counts and error correction capabilities. You can test a few dictionaries with varying error correction levels, such as the 4x4, 5x5, 6x6, or 7x7 dictionaries. For CharUco, you can use the same ArUco dictionaries for the individual markers.

- Board dimensions: For CharUco boards, you can test a few different combinations of the number of squares (checkerboard pattern) in the X and Y dimensions. A larger board provides more information for pose estimation but may require a greater camera distance or a wider field of view.

- Square length and marker separation: The distance between markers and the size of the squares in a CharUco board can affect the calibration accuracy. You can test a few different combinations of square length and marker separation to find the optimal configuration.


## Results

| Algorithm | Marker   | Sample Set | Trans. Error (RMSE) | Rot. Error (RMSE) | Grasp Error (RMSE) | Avg. Trans. Error | Avg. Rot. Error |
|-----------|----------|------------|---------------------|-------------------|--------------------|------------------|----------------|
| Tsai      | ArUco    | Sample 1   |                     |                   |                    |                  |                |
| Tsai      | ArUco    | Sample 2   |                     |                   |                    |                  |                |
| ...       | ...      | ...        | ...                 | ...               | ...                | ...              | ...            |
| Park      | CharUco  | Sample 1   |                     |                   |                    |                  |                |
| Park      | CharUco  | Sample 2   |                     |                   |                    |                  |                |
| ...       | ...      | ...        | ...                 | ...               | ...                | ...              | ...            |
| ...       | ...      | ...        | ...                 | ...               | ...                | ...              | ...            |

Column A: Algorithm (Algo.)
Column B: Marker
Column C: Sample Set
Column D: Translation Error (RMSE)
Column E: Rotation Error (RMSE)
Column F: Grasping/Pose Error (RMSE) (optional)
Column G: Average Translation Error (Avg. Trans. Error) (Calculated for each combination)
Column H: Average Rotation Error (Avg. Rot. Error) (Calculated for each combination)

## Summarize results

| Algorithm | Marker   | Avg. Trans. Error | Std. Dev. Trans. Error | Avg. Rot. Error | Std. Dev. Rot. Error | Avg. Grasp Error | Std. Dev. Grasp Error |
|-----------|----------|-------------------|------------------------|-----------------|----------------------|------------------|----------------------|
| Tsai      | ArUco    |                   |                        |                 |                      |                  |                      |
| Tsai      | CharUco  |                   |                        |                 |                      |                  |                      |
| ...       | ...      | ...               | ...                    | ...             | ...                  | ...              | ...                  |
| Park      | AprilTag |                   |                        |                 |                      |                  |                      |
| Park      | Checker  |                   |                        |                 |                      |                  |                      |
| ...       | ...      | ...               | ...                    | ...             | ...                  | ...              | ...                  |
| ...       | ...      | ...               | ...                    | ...             | ...                  | ...              | ...                  |
