# [About the analysis of high-precision map ROI filter](https://mp.weixin.qq.com/s?__biz=MzI1NjkxOTMyNQ==&mid=2247485710&idx=1&sn=07ba741effb95e10d40e175ba61cd3d1&chksm=ea1e1b7cdd69926a42ff1a809e4791810661f9a15a7c590b0924eb0a3728ba9a365622a3f068&mpshare=1&scene=23&srcid=0306ludMb9jq80wVdt7JKWxi#rd)


## 1 데이터 수신(ROS) & 변환(PCL)

- 가장 먼저 수행 된다. `The high-resolution map ROI filter is the first process of callback. `


- 배경, 도로변, 빌딩, 나무등을 제거 한다. `This process processes lidar points outside the ROI, removing background objects such as roadside buildings and trees. The remaining point clouds are reserved for subsequent processing. `

- 입력은 ROS의 포인트 클라우드 이다. `As can be seen from the initial module framework diagram, the input data type accepted by the LidarProcessSubnode subnode is ROS original point cloud data type, sensor_msgs::PointCloud2`

- 실제 작업은 PCL로 이루어 지므로 변환 작업이 필요 하다. `There is also a detail, the point cloud obtained by Lidar is ROS original sensor_msgs::PointClouds type , and the actual processing used more is the PCL library pcl::PointCloud type , you need to do a conversion in the code, use The pcl::fromROSMsg and pcl::toROSMsg functions of pcl_conversions can be easily converted to each other.` 

```python 
"""
[Sensor_msgs::PointCloud 와 Sensor_msgs::PointCloud2의 차이]

Sensor_msgs::PointCloud2 has some differences from the first version of sensor_msgs::PointCloud, supports arbitrary N-dimensional data, and supports arbitrary basic data types, as well as dense storage. 

As can be seen from the above description, PointCloud2 can support 2D data structure, each point N-dimensional, X lines per line, a total of H columns, can store image information. 

Fields store the names of the various channels, such as x, y, z, r, g, b, and so on. 
"""

Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points
```

## 2 Data conversion and ROI generation



센서 정보 저장시 중요한 작업 두가지 `The two points worthy of attention in the code of the stored procedure are`
- the conversion matrix **velodyne_trans of the sensor** to **the world coordinate system** and 
- the conversion of `sensor_msgs::PointCloud2` to `PCL::PointCloud`


```cpp
/// file in apollo/modules/perception/obstacle/onboard/lidar_process_subnode.cc
void LidarProcessSubnode::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  ...
  /// get velodyne2world transfrom
  std::shared_ptr<Matrix4d> velodyne_trans = std::make_shared<Matrix4d>();
  if (!GetVelodyneTrans(kTimeStamp, velodyne_trans.get())) {
    ...
  }
  out_sensor_objects->sensor2world_pose = *velodyne_trans;

  PointCloudPtr point_cloud(new PointCloud);
  TransPointCloudToPCL(message, &point_cloud);
}
```

다음 절차는 포인트 클라우드에서 ROI영역을 추출 하는 것이다. `the next step is to retrieve the ROI areas from the point cloud , which contain the driving areas of the road and intersection.`

```cpp
/// file in apollo/modules/perception/obstacle/onboard/lidar_process_subnode.cc
void LidarProcessSubnode::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  /// get velodyne2world transfrom
  if (!GetVelodyneTrans(kTimeStamp, velodyne_trans.get())) {
  	...
  }
  /// call hdmap to get ROI
  HdmapStructPtr hdmap = nullptr;
  if (hdmap_input_) {
    PointD velodyne_pose = {0.0, 0.0, 0.0, 0};  // (0,0,0)
    Affine3d temp_trans(*velodyne_trans);
    PointD velodyne_pose_world = pcl::transformPoint(velodyne_pose, temp_trans);
    hdmap.reset(new HdmapStruct);
    hdmap_input_->GetROI(velodyne_pose_world, FLAGS_map_radius, &hdmap);
    PERF_BLOCK_END("lidar_get_roi_from_hdmap");
  }
}
```


도로를 파악하기 위해서는 고정밀 지도가 이용된다. `The driving area of ​​the road surface and intersection needs to be inspected by high-precision map. `

이 단계에서 기준 좌표계로의 센서 좌표계 변환이 이루어 진다. `At this stage, the coordinate system is transformed by tf (the transformation matrix of the lidar coordinate system to the world coordinate system), and the velodyne_pose_world is calculated with velodyne_pose (lidar is in the world coordinate system). Coordinates).`

실질적인 ROS는 `GetROI`에서 수행된다. `The real ROI is using the GetROI function . `

**GetVelodyneTrans function**에서 좌표계 변환을 수행한다. `A simple analysis of the GetVelodyneTrans function , this function is the transformation matrix that produces the lidar coordinate system to the world coordinate system. `

The implementation process can be briefly followed by a functional analysis:

```cpp
/// file in apollo/modules/perception/onboard/transform_input.cc
bool GetVelodyneTrans(const double query_time, Eigen::Matrix4d* trans) {
  ...
  // Step1: lidar refer to novatel(GPS/IMU)
  geometry_msgs::TransformStamped transform_stamped;
  try {
    transform_stamped = tf2_buffer.lookupTransform(FLAGS_lidar_tf2_frame_id, FLAGS_lidar_tf2_child_frame_id, query_stamp);
  } 
  Eigen::Affine3d affine_lidar_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_lidar_3d);
  Eigen::Matrix4d lidar2novatel_trans = affine_lidar_3d.matrix();

  // Step2 notavel(GPS/IMU) refer to world coordinate system
  try {
    transform_stamped = tf2_buffer.lookupTransform(FLAGS_localization_tf2_frame_id, FLAGS_localization_tf2_child_frame_id, query_stamp);
  } 
  Eigen::Affine3d affine_localization_3d;
  tf::transformMsgToEigen(transform_stamped.transform, affine_localization_3d);
  Eigen::Matrix4d novatel2world_trans = affine_localization_3d.matrix();

  * trans = novatel2world_trans * lidar2novatel_trans; 
}
```

transformation matrix획득을 위한 절차는 두단계로 구분 된다. `Therefore, the acquisition transformation matrix is ​​divided into two steps.`
- The first step is to obtain the transformation matrix of the lidar coordinate system of the lidar to the IMU coordinate system of the inertial unit ; 
- the second step is to obtain the transformation matrix of the inertial unit IMU coordinate system to the world coordinate system . 

From the above code, we can clearly see that there are two parts of code with high similarity:

  
    
- Calculate the affine transformation matrix lidar2novatel_trans, the lidar lidar coordinate system to the inertial IMU coordinate system (vehicle coordinate system) transformation matrix. Although this matrix is ​​calculated by calling the lookupTransform function of the ROS tf module, it is actually determined by the foreign parameter and remains unchanged during the running process.
  
```cpp
/// file in apollo/modules/localization/msf/params/velodyne_params/velodyne64_novatel_extrinsics_example.yaml
child_frame_id: velodyne64
transform:
  translation:
    x: -0.0581372003122598
    y: 1.459274166013735
    z: 1.24965
  rotation:
    x: 0.02748694630673456
    y: -0.03223034579615043
    z: 0.7065742186090662
    w: 0.706369978261802
header:
  seq: 0
  stamp:
    secs: 1512689414
    nsecs: 0
  frame_id: novatel
```

- Calculate the affine transformation matrix novatel2world_trans, the affine transformation matrix of the inertial unit IMU coordinate system (vehicle coordinate system) relative to the world coordinate system.

- The affine transformation matrix lidar2world_trans is calculated, and finally the two matrices are multiplied to obtain the transformation matrix of the lidar lidar coordinate system to the world coordinate system.


## 3 Coordinate transformation

```cpp
/// file in apollo/modules/perception/obstacle/onboard/lidar_process_subnode.cc
void LidarProcessSubnode::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  /// get velodyne2world transfrom
  ...
  /// call hdmap to get ROI
  ...
  /// call roi_filter
  PointCloudPtr roi_cloud(new PointCloud);
  if (roi_filter_ != nullptr) {
    PointIndicesPtr roi_indices(new PointIndices);
    ROIFilterOptions roi_filter_options;
    roi_filter_options.velodyne_trans = velodyne_trans;
    roi_filter_options.hdmap = hdmap;
    if (roi_filter_->Filter(point_cloud, roi_filter_options, roi_indices.get())) {
      pcl::copyPointCloud(*point_cloud, *roi_indices, *roi_cloud);
      roi_indices_ = roi_indices;
    } else {
      ...
    }
  }
}
```

좌표계 변환과 이후 ROI LUT construction작업은 `HdmapROIFilter` 클래스에서 수행된다. `The coordinate transformation and subsequent steps of the ROI LUT construction and point query are done in the HdmapROIFilter class.`

이단계에서 사용되는 변환 메트릭스는 `lidar2world_trans matrix`이다. `The transformation matrix used in this phase is the above lidar2world_trans matrix.`

After reading the official instructions and matching the specific code, there may be some doubts. Here are some research ideas for transformation. 

The implementation of the coordinate transformation is done in the `HdmapROIFilter::Filter` function 

The specific transformation process is as follows:

```cpp
/// file in apollo/modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.cc
bool HdmapROIFilter::Filter(const pcl_util::PointCloudPtr& cloud,
                            const ROIFilterOptions& roi_filter_options,
                            pcl_util::PointIndices* roi_indices) {
  Eigen::Affine3d temp_trans(*(roi_filter_options.velodyne_trans));
  std::vector<PolygonDType> polygons;
  MergeHdmapStructToPolygons(roi_filter_options.hdmap, &polygons);
  ...
  // Transform polygon and point to local coordinates
  pcl_util::PointCloudPtr cloud_local(new pcl_util::PointCloud);
  std::vector<PolygonType> polygons_local;
  TransformFrame(cloud, temp_trans, polygons, &polygons_local, cloud_local);
  ...
}

void HdmapROIFilter::TransformFrame(
    const pcl_util::PointCloudConstPtr& cloud, const Eigen::Affine3d& vel_pose,
    const std::vector<PolygonDType>& polygons_world,
    std::vector<PolygonType>* polygons_local,
    pcl_util::PointCloudPtr cloud_local) {
  ...
}

```

Note 1: 
- `MergeHdmapStructToPolygons`함수는 교차로와 도로의 포인트 클라우드를 polygons로 포함 시킨다. `The MergeHdmapStructToPolygons function in the above code is responsible for incorporating the point cloud of the intersection and the road surface into the polygon collection polygons.`
- `roi_filter_options`에서의 데이터는 고밀도 지도에서 얻어온 교차로와 도로 데이터(기준좌표) 들이다. `Here, the data in the roi_filter_options is the intersection and road information obtained through the high-precision map query, which is based on the world coordinate system.`
- 따라서 폴리곤으로 합쳐진 결과값역시 기준 좌표계를 따른다. `So the result of the merged polygons is also the data of the world coordinate system.`


Note 2: 
- 입력된 포인트 클라우드 데이터는 센서 좌표계로 되어 있으며 cloud_local, polygons_local로 변경이 필요 하다. local좌표계는 Annotation에 기술 되어 있다. `The input cloud is point cloud data based on the lidar coordinate system, and the following code needs to be converted into cloud_local, polygons_local, and the local coordinate system is explained according to the annotation. `
- 여기서 local좌표계는 센서 좌표계라고 생각 하면 된다. `Then, what is the coordinate system of this local coordinate system? If you can understand the TransformFrame function, it is not difficult to find: this so-called "local coordinate system" is actually very similar to the lidar coordinate system.`
- He represents the ENU coordinate system with lidar as the origin . 
- This coordinate system is X (East)-Y. (North)-Z (day) is the two-dimensional projected coordinate system of the coordinate axis. 


In the TransformFrame function,
```cpp
Eigen::Vector3d vel_location = vel_pose.translation();
Eigen::Matrix3d vel_rot = vel_pose.linear();
Eigen::Vector3d x_axis = vel_rot.row(0);
Eigen::Vector3d y_axis = vel_rot.row(1);
```

- `Vel_location` is the translation component of the **lidar coordinate system** relative to the **world coordinate system**, and 
- `vel_rot` is the rotation matrix of the **lidar coordinate system** relative to the **world coordinate system**. 
- Then the coordinate transformation from the lidar coordinate system to the world coordinate system is actually very simple. 
- Assuming that there is a coordinate point P(x1, y1, z1) in the lidar coordinate system, the coordinate P_hat of the point in the world coordinate system is: P_hat = Vel_rot * P + vel_location. 


Understand this transformation, then observe the transformation code for cloud and polygons:
```cpp
polygons_local->resize(polygons_world.size());
for (size_t i = 0; i < polygons_local->size(); ++i) {
  const auto& polygon_world = polygons_world[i];
  auto& polygon_local = polygons_local->at(i);
  polygon_local.resize(polygon_world.size());
  for (size_t j = 0; j < polygon_local.size(); ++j) {
    polygon_local[j].x = polygon_world[j].x - vel_location.x();
    polygon_local[j].y = polygon_world[j].y - vel_location.y();
  }
}
```

It was also very strange at the beginning, why the final transformation form was `P_local = P_world - translation`. 

Later, after research and speculation (to be confirmed later), the intersection and road polygon information only passed the translation to reach the new local ENU coordinate system, which can be inferred to be the world. 

The coordinate system is also the ENU(east, north, up) coordinate system, so there is no rotation component between the two coordinate systems, and the translation can be directly removed from the world coordinate system to the local ENU coordinate system.

```
Note that the heights z of polygons_world and polygons_local before and after the transformation are changed, 
but since the polygons are used to construct the 2D projection grid LUT, 
the dimension of height z is not concerned, and these do not change z; It will never change.
```

`P_world = vel_rot * P_local + translation` When the `vel_rot` rotation component is 0: `P_local = P_world - translation`

```cpp
cloud_local->resize(cloud->size());
for (size_t i = 0; i < cloud_local->size(); ++i) {
  const auto& pt = cloud->points[i];
  auto& local_pt = cloud_local->points[i];
  Eigen::Vector3d e_pt(pt.x, pt.y, pt.z);
  local_pt.x = x_axis.dot(e_pt);
  local_pt.y = y_axis.dot(e_pt);
}

```


> 50% 진행 
