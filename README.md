<!--
 * @Author: Aiden
 * @Date: 2021-07-07 19:29:29
 * @LastEditTime: 2021-07-12 17:33:18
-->
# Carton_expansion_detection

According to the point cloud and image to calulate the expansion of carton,used sensor devices likes **linelaser**、**percipioRGBD**、**Azure Kinect** 、**ifm O3D303**.

The essence of the algorithm is to calculate the maximum distance between all points on the target plane of the carton and the plane of bottom edge.There are two ways to achieve this. 

**Only need the point cloud of the carton**

1. Find all the segments by clustering method,
2. Find the target plane of the carton according to the normal vector of the reference box plane
3. Fitting the plane of bottom edge according to the reference ground and the points of the bottom edge

**Need image and corresponding point cloud**

1. To detect complete the carton from the background
2. According to the mapping relationship like LUT(lookup table) to find the point cloud corresponding to the image coordinates
3. Similar to step 3 above





## Prerequisties

**OpenCV3.0**+

**PCL1.9**

**Yaml-cpp**

[**camport3**](https://github.com/percipioxyz/camport3)

**[k4a-tools](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download)**

**[libk4a1.4-dev](https://docs.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download)**



## Example

**if you use vscode IDE,you can directly use launch.json for custom execution** 

### test_get_standard_normal

Calculate the standard normal of ground and the front of the carton

```
./test_get_standard_normal ${workspaceFolder}/data/percipio3D/box_test/points-0.xyz ${workspaceFolder}/config/percipioRGBD.yaml"
 0 2
```



### test_linelaser_detect

**linelaser device**

```
./test_linelaser_detect ${workspaceFolder}/data/linelaser3D/linelaser-03.pcd ${workspaceFolder}/config/linelasr3D_param.yaml
```

**percipio device**

```
./test_linelaser_detect ${workspaceFolder}/data/percipio3D/box_test/points-0.xyz ${workspaceFolder}/config/percipioRGBD.yaml
```



### test_percipio_rgbd_detect

```
./test_percipio_rgbd_detect ${workspaceFolder}/data/percipio3D/box_test/result-0.png ${workspaceFolder}/data/percipio3D/box_test/rawdepth-0.png ${workspaceFolder}/data/percipio3D/box_test/points-0.xyz ${workspaceFolder}/config/percipioRGBD.yaml
```

