# Custom Package for OAK-D Camera

## Why do we have a custom package? 
Because of a firmware bug, we can't use `depthai` package to publish rgb images because we can't set the manual focus. So, we had to change the source code to allow for this.

## How to use?
You can use any of the launch files we added in the `launch` directory. If you want to do camera calibration, you can use the `camcalib_tht_rgb_publisher` launch file by typing:
```bash
roslaunch tht_depthai camcalib_tht_rgb_publisher.launch
```
