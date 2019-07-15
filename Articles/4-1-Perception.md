
# [Apollo 4.1 - Perception.md](https://blog.csdn.net/yuxuan20062007/article/details/81348010)

> 중국어를 구글 번역기를 이용하여 영어로 변역후 정리 한 글입니다. 내용상 오류가 있을 수 있습니다.

First of all, I am very grateful to the relevant materials shared by several big cattle and Apollo developer communities. The entire Baidu Apollo series of articles, the author here to do some information collection, I hope to learn with you. The main reference sources include:

-   Know [the world in the eyes of](https://zhuanlan.zhihu.com/yuan-ji) the column[](https://zhuanlan.zhihu.com/yuan-ji)
-   [Apollo Apollo Autopilot Developer Community](https://apolloauto.io/)
-   [Apollo Developer Online Documentation](http://www.fzb.me/apollo/)
-   CSDN author [Zhihanggeyi 2018](https://blog.csdn.net/davidhopper/article/details/79183557)
-   CSDN by [DinnerHowe](https://blog.csdn.net/dinnerhowe)
-   Baidu [Apollo GitHub](https://github.com/ApolloAuto/apollo)

The content of today's Perceptual Module is organized from the article of  
[Aoyuan](https://zhuanlan.zhihu.com/p/33416142) : [Apollo 2.0 Framework and Source Code Analysis (2) | Perception Module | Lidar](https://zhuanlan.zhihu.com/p/33416142)  
[Apollo 2.0 Framework and Source Code Analysis (3) | Perception Module | Radar & Fusion](https://zhuanlan.zhihu.com/p/33852112)

# Apollo Perception Module (Perception) implementation framework

The part of the driverless system is roughly divided into three parts: perception, decision, and control. Among them, perception is the basis of the other two, and is a very important part of driverless driving.

Sensors used in driverless driving have their own strengths and short plates. A single sensor is difficult to meet the needs of complex scenes. Therefore, it is necessary to use a variety of sensors for sensor funnation.

The following picture shows the Apollo open class on common sensors in driverless driving. It can be seen from the figure that each sensor is good at different situations, and the fusion of sensors can handle most situations.

![Write a picture description here](https://img-blog.csdn.net/20180801205554779?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
This picture of Apollo is well described and is very clear.  
Yan Yuan believes that Lidar's description of Lane Tracking is a bit inadequate. He believes that lanes can be tracked based on the difference in the intensity of the Lidar rays reflected from the road and lane lines. So Lidar's ability to track lanes should be considered Fair rather than the same Poor as Radar.

In addition, the article on Lei Feng.com also mentions the possibility of detecting lane lines based on laser radar echo signals. [Column | How to use Lidar to detect lane lines? Here are four methods of](https://www.leiphone.com/news/201712/iG2xBYren1q9faI9.html?utm_source=debugrun&utm_medium=referral)  
Apollo 2.0-aware overall framework as shown in the following figure, divided into 3D obstacle perception and traffic light perception.

![](https://i.imgur.com/jcOxF6N.png)  
In fact, the two have been clearly explained in the official Apollo documentation and lectures.
- [Traffic Signal Sensing](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/traffic_light_cn.md) and 
- [3D Obstacle Perception](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception_cn.md)  

The function of the Perceptual Part of the Apollo Official Open Class is described in the following figure.

![Write a picture description here](https://img-blog.csdn.net/20180801210654651?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

As you can see from the figure, the awareness module in Apollo has the following functions:

1.  Detecting objects (whether there are obstacles)
2.  Classify objects (what are obstacles)
3.  Semantic analysis (obstacles are segmented from the background)
4.  Object tracking (obstacle tracking)

This article will next describe how the Apollo 3D Obstacle Perception section performs the above functions.

The framework of the obstacle-aware part is shown below. Obstacle perception relies mainly on the fusion of the perceived results of both Lidar and Radar. This part of the input is the raw data of the sensor, and different processing is performed according to the source of the received data, and finally the result of the fusion is output.  
![Write a picture description here](https://img-blog.csdn.net/20180801210835150?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

From the code point of view, obstacle perception consists of three main parts: Lidar, Radar, and fusion. Next, this article will focus on the implementation of the Lidar part.

# Lidar part

The official explanation of the principles of the Lidar part is extremely clear. Combining the reference documents and PPT materials listed above, you should have a clear understanding of the overall workings of the Lidar part.  
![Write a picture description here](https://img-blog.csdn.net/20180801210925951?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  

As shown in the PPT above, the Lidar section simply works like this:

1.  Enter the point cloud data obtained by Lidar and output as the detected obstacle
2.  ROI (region of interest) is determined by HDmap, filtering points outside the ROI area
3.  Process point cloud data, detect and identify obstacles (completed by AI)
4.  Obstacle tracking

[Yan Yuan](https://zhuanlan.zhihu.com/p/33416142) also explained the code.

Entry function location: [apollo/modules/perception/obstacle/onboard/lidar_process_subnode.cc](https://github.com/ApolloAuto/apollo/blob/804cd5b25ca8be591856a5319b76206d4f2a2f4e/modules/perception/obstacle/onboard/lidar_process_subnode.cc)

```cpp
void LidarProcessSubnode::OnPointCloud(
    const sensor_msgs::PointCloud2& message);  

```

According to the code's comments, here is a 7-step description of Lidar's process.

---

## 1. coordinate and format conversion (get velodyne2world transform)

Apollo uses the open source library Eigen for efficient matrix calculations and uses the PCL point cloud library to process point clouds.

In this section, Apollo first calculates the transformation matrix velodyne_trans, which is used to convert Velodyne coordinates into world coordinates. The Velodyne point cloud is then converted to the PCL point cloud library format for later calculations.

---

## 2. Get the ROI area (call hdmap to get ROI)

Core function location: obstacle/onboard/hdmap_input.cc

```cpp
bool HDMapInput::GetROI(const PointD& pointd, const double& map_radius,
                        HdmapStructPtr* mapptr);

```

Query the HDmap to get the ROI area based on Velodyne's world coordinates and the preset radius (FLAG_map_radius).

First, the boundaries of the roads and road intersections within the specified range are obtained, and the results of the fusion are stored in the ROI polygon. All points in this area are in the world coordinate system.

---

## 3. Call the ROI filter (call roi_filter)

Core function location: obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.cc

```cpp
bool HdmapROIFilter::Filter(const pcl_util::PointCloudPtr& cloud,
                            const ROIFilterOptions& roi_filter_options,
                            pcl_util::PointIndices* roi_indices);

```

The official documentation describes this part as follows:

> The high-precision map ROI filter (hereinafter referred to as "filter") processes the lidar points outside the ROI, removing background objects such as roadside buildings and trees, and the remaining point clouds are reserved for subsequent processing.  
> In general, the Apollo high-precision map ROI filter has the following three steps:  
> 1. Coordinate transformation  
> 2. ROI LUT construction  
> 3. ROI LUT point query

![Write a picture description here](https://img-blog.csdn.net/20180801211848694?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

The blue lines mark the boundaries of the high-precision map ROI, including roads and intersections. The red bold dots indicate the original position of the local coordinate system corresponding to the position of the lidar sensor. The 2D grid consists of 8*8 green squares, the cells in the ROI are blue filled squares, and the yellow filled squares.

ROI filter portion related to the **scan line method** and the **bitmap encoding** two techniques. Specifically, this part is divided into the following steps:

### a. coordinate conversion

Converts the map ROI polygon and point cloud to the local coordinate system of the lidar sensor location.

### b. Determine the main direction of the map polygon

Compare the range of the x and y directions of all points, and take the direction with the smaller range as the main direction. Convert map polygons (map_polygons) to raw polygons.

### c. Create a bitmap

Convert raw polygons to grid points in bitmaps. Bitmaps have the following characteristics:

-   Bitmap range, within a region (-range, range)* (-range, range) with Lidar as the origin, range defaults to 70 meters
-   Bitmaps are used to store ROI information in a grid. If a grid point is true, it means that this grid point belongs to the ROI.
-   The default grid size is cell_size 0.25 meters.
-   In the column direction, 1bit represents 1grid. To speed up the operation, Apollo uses uint64_t to manipulate 64 grids at a time.

In order to draw a polygon in the bitmap, the following 3 steps need to be completed.

i. **Obtain the effective range of the main direction**

Ii. **Scanning interval required to convert a polygon to a scanning line method** : Decompose a multi-deformation into a line in the main direction (polygon->slice->line), and calculate the scanning interval of each line.

Iii. **Draw a grid point in the bitmap based on the scan interval**

![Write a picture description here](https://img-blog.csdn.net/20180801212703623?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

About scanning lines recommended here I refer to an article resolve [scan line algorithm is fully resolved](https://link.zhihu.com/?target=https://www.jianshu.com/p/d9be99077c2b)

### d. ROI point inquiry

By checking the value of the grid, determine if each grid in the bitmap is part of the ROI.

---

## 4. Call the segmenter (segmentor)

The file where the entry function is located`cnn_segmentation.cc`

```cpp
bool CNNSegmentation::Segment(const pcl_util::PointCloudPtr& pc_ptr,
                              const pcl_util::PointIndices& valid_indices,
                              const SegmentationOptions& options,
                              vector<ObjectPtr>* objects)

```

The splitter uses the **deep full convolutional neural network (FCNN)** of the **caffe frame** to segment obstacles

Simply put, there are four steps:

### a. Channel feature extraction

Calculate the 8 statistics related to the points in each cell in a range of Lidar sensors, and input them as channel features to FCNN.

> 1.  The maximum height of the point in the cell
> 2.  The strength of the highest point in the cell
> 3.  The average height of the points in the cell
> 4.  Average intensity of the points in the cell
> 5.  Number of points in the cell
> 6.  The angle of the cell center relative to the origin
> 7.  The distance between the center of the cell and the origin
> 8.  The binary value indicates whether the cell is empty or occupied.

By default, only points in the ROI area are used for the calculation, or points in the entire Lidar range are used, and the flag use_full_cloud_ is used as the switch.

### b. Obstacle prediction based on convolutional neural networks

-   The FCNN source associated with caffe looks like it is **not open source** .
-   Apllo officially describes how it works, excerpted as follows

> The complete convolutional neural network consists of three layers: a downstream coding layer (feature encoder), an upstream decoding layer (feature decoder), and an obstacle property prediction layer (predictor).
> 
> The feature encoder takes the channel feature image as input and continuously downsamples its spatial resolution as the feature extraction increases. The feature decoder then progressively  
> samples the feature image to the spatial resolution of the input 2D mesh, and the spatial detail of the feature image can be restored to facilitate obstacle location and velocity property prediction in the cell direction.  
> The activation with nonlinear (i.e. RELU) stacked convolution / distribution layers layer to achieve **downsampling** and **upsampling** operations

![Write a picture description here](https://img-blog.csdn.net/20180801213522773?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
Based on FCNN's predictions, Apollo obtained four predictions for each cell for subsequent obstacle clustering and post-processing.  
![Write a picture description here](https://img-blog.csdn.net/20180801213605676?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

### c. Obstacle cluster (Cluster2D)

Core function location `obstacle/lidar/segmentation/cnnseg/cluster2d.h`

```cpp
void Cluster(const caffe::Blob<float>& category_pt_blob,
               const caffe::Blob<float>& instance_pt_blob,
               const apollo::perception::pcl_util::PointCloudPtr& pc_ptr,
               const apollo::perception::pcl_util::PointIndices& valid_indices,
               float objectness_thresh, bool use_all_grids_for_clustering);

```

Apollo based cell **center offset predictive** construct a directed graph, using compression **joint search algorithm** (Union Find algorithm) based on **the object prediction** efficient lookup of connection assembly constructed obstacle cluster.  
![Write a picture description here](https://img-blog.csdn.net/20180801213814310?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
(a) The red arrow indicates the center offset prediction for each cell object; the blue fill corresponds to the object unit whose object probability is not less than 0.5.  
(b) The cells within the solid red polygon form a cluster of candidate objects.

### d. Post processing

The function involved is [obstacle/lidar/segmentation/cnnseg/cluster2d.h](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/lidar/segmentation/cnnseg/cluster2d.h)

```cpp
void Filter(const caffe::Blob<float>& confidence_pt_blob,
              const caffe::Blob<float>& height_pt_blob);
void Classify(const caffe::Blob<float>& classify_pt_blob);
void GetObjects(const float confidence_thresh, const float height_thresh,
                  const int min_pts_num, std::vector<ObjectPtr>* objects);

```

After clustering, Apollo obtains a set of candidate objects consisting of several cells, each of which includes several cells. The final output obstacle set/segment is determined based on the detection confidence score and object height for each candidate population. It can be seen from the code that there are three types of objects finally recognized by the CNN splitter: small motor vehicles, large motor vehicles, non-motor vehicles and pedestrians.

In `obstacle/lidar/segmentation/cnnseg/cluster2d.h`the

```cpp
enum MetaType {
  META_UNKNOWN,
  META_SMALLMOT,
  META_BIGMOT,
  META_NONMOT,
  META_PEDESTRIAN,
  MAX_META_TYPE
};

```

---

## 5. Obstacle border construction

Entry function location [obstacle/lidar/object_builder/min_box/min_box.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/lidar/object_builder/min_box/min_box.cc)

```cpp
void BuildObject(ObjectBuilderOptions options, ObjectPtr object)

```

> The main purpose of the bounding box is to predict the direction of the obstacle (for example, the vehicle). Similarly, borders are also used to visualize obstacles.  
> As shown in the figure, Apollo determines a 6-boundary border and will choose the scheme with the smallest area as the final bounding box.  
> ![Write a picture description here](https://img-blog.csdn.net/20180801214233893?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

---

## 6. Obstacle tracking

Entry function location: [obstacle/lidar/tracker/hm_tracker/hm_tracker.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.cc)

```cpp
  // @brief track detected objects over consecutive frames
  // @params[IN] objects: recently detected objects
  // @params[IN] timestamp: timestamp of recently detected objects
  // @params[IN] options: tracker options with necessary information
  // @params[OUT] tracked_objects: tracked objects with tracking information
  // @return true if track successfully, otherwise return false
  bool Track(const std::vector<ObjectPtr>& objects, double timestamp,
             const TrackerOptions& options,
             std::vector<ObjectPtr>* tracked_objects); 

```

Track obstacles can be divided into two parts, namely 
- **data association** and 
- **tracking dynamic estimates** . 

Apollo uses an object tracker called HM tracker. Implementation principle:

> In the **HM object tracker**, the **Hungarian algorithm** is used to detect the tracking association, and the Robust **Kalman Filter** is used for motion estimation.

### 6.1 Data association

The process of data association is the process of determining the correspondence between the measurement information received by the sensor and the target source, and is the core and most important process of the multi-sensor multi-target tracking system [11].

Apollo first associated distance matrix for calculating each **subject** (object), and each of the **track**between the (Track) **associated distance** . The Hungarian algorithm is then used to optimally assign objects and tracks.

When calculating the **associated distance**, Apollo considers the following five correlation features to evaluate the motion and appearance consistency of object and track, and assign different weights to them.

![Write a picture description here](https://img-blog.csdn.net/20180801214531554?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
As can be seen from the above table, Apollo considers the geometric distance and the shape similarity of the two when calculating the correlation distance. After calculating the associated distance matrix similar to the following figure, the Hungarian algorithm is used to match the Object to the Track.  
![Write a picture description here](https://img-blog.csdn.net/20180801214550966?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
About realization of the principle of the Hungarian algorithm, there are articles very interesting interpretation [fun to write algorithm Series - Hungary algorithm - CSDN blog](https://blog.csdn.net/dark_scope/article/details/8880547)

### 6.2 Track Motion Estimation

Kalman filtering is used to estimate the state of the track, using robust statistical techniques to eliminate the effects of anomalous data.

For those who do not understand the Kalman filter principle, please refer to: [the principle of the Kalman filter and its implementation in matlab](https://v.qq.com/x/page/o03766f94ru.html) . 

This part of the filtering appears to be a standard Kalman filter as a whole. On this basis, the Apollo team has added some changes. According to the official documentation, Apollo's tracking dynamics estimate has three highlights:

**Observing redundancy**

> The velocity measurement is selected in a series of repeated observations, that is, the input of the filtering algorithm, including anchor shift, boundary frame center offset, and bounding box corner shift. Redundant observations will provide additional robustness to filtering measurements because the probability of all observation failures is much less than the probability of a single observation failure.

The observed value of the Kalman update is **speed** . Observe **three speed values** each time :

Anchor displacement speed, bounding box center offset velocity, and boundary frame corner point displacement velocity.

From the three velocities, according to **the consistency of the motion** , the **speed at which the deviation from the previous observation speed is the smallest is selected** as the final observation.

Based on the **last three** speed observations, the observed value of the **acceleration** is calculated .

**break down**

> Gaussian filter algorithms always assume that their Gaussian distribution produces noise. However, this assumption may fail in the motion estimation problem because the noise it measures may come from a histogram. To overcome the overestimation of the update gain, a fault threshold is used during the filtering process.

The fault threshold here should correspond to the breakdown_threshold_ in the program.

This parameter is used in the following two functions. When the updated gain is too large, it is used to overcome the overestimation of the gain:

-   KalmanFilter::UpdateVelocity
-   KalmanFilter::UpdateAcceleration

The difference between the two is:

**Speed** fault threshold is **dynamically calculated** , and the **velocity error covariance matrix**related

```
velocity_gain *= breakdown_threshold_; 

```

The fault threshold for acceleration is fixed. The default is 2

```
acceleration_gain *= breakdown_threshold;  

```

**Update association quality (UpdateQuality)**

> The original Kalman filter updates its state without distinguishing its measured quality. However, quality is a useful hint for filtering noise and can be estimated. For example, the distance calculated in the association step can be a reasonable measurement quality estimate. The robustness and smoothness of the motion estimation problem is enhanced by updating the state of the filtering algorithm according to the associated quality.

The update association quality update_quality defaults to 1.0. When the adaptation function is turned on (`s_use_adaptive ==true`) Apollo uses the following two strategies to calculate the update association quality:

-   Calculated according to the object's own property - the association score (association_score)
-   According to the change of the number of new and old object point clouds

Firstly, according to these two strategies, the update association quality is calculated separately, and then the small result is obtained to control the filter noise.

---

## 7. Obstacle type fusion (call type fuser)

This function seems to be removed from the version 3.0.

This part is responsible for the type fusion of the object sequence.

The type of object is shown in the following code:

```
enum ObjectType {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,
  BICYCLE = 4,
  VEHICLE = 5,
  MAX_OBJECT_TYPE = 6,
};

```

Apollo treats the objects being tracked as sequences.

When object is background, its type is "UNKNOW_UNMOVABLE".

When the object is foreground, the object type of the object sequence is fused using the Linear Chain Conditional Random Fields and the Viterbi algorithm.

---

# Radar part

Entry function location: [obstacle/onboard/radar_process_subnode.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/radar_process_subnode.cc)

```
void RadarProcessSubnode::OnRadar(const ContiRadar &radar_obs) ;

```

Compared to the amazing Lidar part, Radar in Apollo 2.0 is a bit less sincere. The Radar detector module in Apollo 2.0 is called "ModestRadarDetector" ("Moderate Radar Detector", and from the name it is felt that Apollo 2.0 is somewhat unconfident for Radar).

The Radar part is generally implemented in a similar way to Lidar, and can be seen as a simplified version. The general flow is shown in the following figure.  
![Write a picture description here](https://img-blog.csdn.net/20180801222120140?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
From the point of view of code implementation, Radar's work can be divided into the following four steps:

## I. Calculate the coordinate transformation matrix of Radar

There is a very interesting detail here. The conversion matrix of the world coordinates of the radar is implemented by the following code.

```
*radar2world_pose = *velodyne2world_pose *
short_camera_extrinsic_ *  radar_extrinsic_;

```

This code describes such an expression:

Radar world coordinates = Lidar world coordinates * short camera parameters * radar parameters.

This formula faintly reveals that Apollo's coordinate system is based on Lidar. Apollo may think that Velodyne's position is the most accurate, so Camera's position calibration is referenced to Velodyne, Radar's calibration reference Camera.

## Obtain the ROI area

Read the ROI area from a high-precision map and convert it to a map polygon (map_polygons) for backup. Radar and Lidar use the same function to get the ROI area and the map polygon, and the obtained ROI map polygon is in world coordinates.

Function location: [obstacle/onboard/hdmap_input.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/hdmap_input.cc)

```
void HdmapROIFilter::MergeHdmapStructToPolygons(
    const HdmapStructConstPtr& hdmap_struct_ptr,
    std::vector<PolygonDType>* polygons)

```

Function location: [obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/lidar/roi_filter/hdmap_roi_filter/hdmap_roi_filter.cc)

```
void HdmapROIFilter::MergeHdmapStructToPolygons(
    const HdmapStructConstPtr& hdmap_struct_ptr,
    std::vector<PolygonDType>* polygons) 

```

## Calculate the speed of the car line

## Detection of obstacles

Core function location: [obstacle/radar/modest/modest_radar_detector.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/radar/modest/modest_radar_detector.cc)

```
bool ModestRadarDetector::Detect(const ContiRadar &raw_obstacles,
                                 const std::vector<PolygonDType> &map_polygons,
                                 const RadarDetectorOptions &options,
                                 std::vector<ObjectPtr> *objects) ;

```

This part is the core part of Radar, which can be divided into the following four steps.

### 1. Construct an object from raw data

Implementation function location: [obstacle/radar/modest/object_builder.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/radar/modest/object_builder.cc)

```
void ObjectBuilder::Build(const ContiRadar &raw_obstacles,
                          const Eigen::Matrix4d &radar_pose,
                          const Eigen::Vector2d &main_velocity,
                          SensorObjects *radar_objects) ;

```

This section covers the following aspects:

-   Determine if the obstacle is a backgroundobject
-   Coordinate system transformation of anchor (pointer): Radar coordinates -> world coordinates
-   Speed ​​conversion: relative speed -> absolute speed;

We focus on the first item. Apollo provides three ways to determine if the obstacle is background:

#### a. According to the number of obstacles appearing

When the number of obstacles is less than delay_frames_ (the default is 4), it is considered to be background.

#### b. Probability of existence and root mean square (rms) based on obstacles

Implementation function location: [obstacle/radar/modest/conti_radar_util.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/radar/modest/conti_radar_util.cc)

```
bool ContiRadarUtil::IsFp(const ContiRadarObs &contiobs,
                          const ContiParams &params, const int delay_frames,
                          const int tracking_times)

```

When the one of the following two conditions is met, the obstacle is considered to be the background

-   The probability of an obstacle is less than the default value of the Radar parameter.
-   The rms of the distance or speed of the obstacle in both the horizontal and vertical directions of the vehicle is greater than the Radar parameter preset value.

This function is implemented in the code (for convenience of explanation, some code is hidden here).

```
bool ContiRadarUtil::IsFp(const ContiRadarObs &contiobs,
                          const ContiParams &params, const int delay_frames,
                          const int tracking_times) {
    int cls = contiobs.obstacle_class();
      ...
    if (cls == CONTI_CAR || cls == CONTI_TRUCK) {
      ...
    } else if (cls == CONTI_PEDESTRIAN) {
      ...
    } else if (cls == CONTI_MOTOCYCLE || cls == CONTI_BICYCLE) {
      ...
    } else if (cls == CONTI_POINT || cls == CONTI_WIDE ||
               cls == CONTI_UNKNOWN) {
      ...
    }
      ...
}

```

It can be seen that Apollo handles different types of obstacles separately, that is, Radar used by Apollo can distinguish between different obstacles such as cars, pedestrians and bicycles.

The millimeter wave recommended by Apollo is Continental's ARS 408-21. The introduction to ARS 408-21 is also briefly mentioned, it can classify obstacles.  
![Write a picture description here](https://img-blog.csdn.net/20180801223309134?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
But in fact, I still have doubts about the reliability and usability of the resolution of the obstacle types of the millimeter wave radar. The author also consulted colleagues and friends. It is believed that millimeter-wave radars may be able to distinguish obstacles based on echo characteristics and the speed of the object being tested. However, the resolution, reliability and usability of this approach need to be improved. In addition, the condition of "> 120 single cluster" in brackets in the document, I have been unable to understand, I hope to discuss with you.

#### c. According to the angle between the obstacle speed and the body speed (both at absolute speed)

When both the vehicle speed and the obstacle speed are greater than the threshold (velocity_threshold, default value 1e-1), the angle between the two speeds is calculated. When the angle is outside the range of (1/4 Pi, 3/4 Pi) or (-3/4 Pi, -1/4 Pi ), ​​the obstacle is considered to be background.

There is also a slot here. The code for Apollo 2.0 when constructing the object shape is as follows:

```
object_ptr->length = 1.0;
object_ptr->width = 1.0;
object_ptr->height = 1.0;
object_ptr->type = UNKNOWN; 

```

As you can see from the code, Apollo sets the obstacle length and height to 1 meter. Millimeter wave radar is really difficult to get the contour information of the object, but this treatment is still a bit simple and rough.

In addition, the obstacle type here is directly set to UNKNOW, which makes people wonder. Since Apollo had previously been able to obtain obstacle types directly from Radar when constructing objects, why not use them here? Does Apollo actually think that the type of object that Radar acquired is not reliable? Isn't this a bit inconsistent?

### 2. Call the ROI filter

Implementation function location: [obstacle/radar/modest/modest_radar_detector.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/radar/modest/modest_radar_detector.cc)

```
void RoiFilter(const std::vector<PolygonDType> &map_polygons,
                 std::vector<ObjectPtr>* filter_objects)

```

Radar's ROI filter is different from Lidar and has been simplified a lot.

-   Instead of using bitmaps and scan lines, Radar directly determines whether each object is inside a map polygon.
-   The coordinates of the points and map polygons used in the Radar ROI filter are world coordinates, while Lidar's ROI uses local coordinates.

### 3. Obstacle tracking

Entry function location: [obstacle/radar/modest/radar_track_manager.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/radar/modest/radar_track_manager.cc)

```
// @brief: process radar obstacles
// @param [in]: built radar obstacles
// @return nothing
void Process(const SensorObjects &radar_obs);

```

Radar data association is relatively simple, and only consider a target object of both the tracked object **track_ID** and **the geometric distance** of two factors.

When the track_id of the two is the same and the distance between the two is less than RADAR_TRACK_THRES (default 2.5), the two are considered to be the same target, that is, the target object belongs to the track.

Geometric distance = distance from the center point of two objects + speed of object * time difference.

In addition, when Lidar compares the data, the correlation distance of obstacles is calculated and the Hungarian algorithm is used for optimal allocation. The data association of Radar is too simple and too thin.

In addition, during obstacle tracking, Radar's tracker does not store object history data, only saves the latest object, and does not use Kalman filter to estimate the state of objects.

### 4. Collect Objects

Collect objects and prepare for the final integration.

In general, the Radar part is "innocent" to the name of modest, and Radar's implementation is too thin compared to the Lidar part. Although Radar's accuracy is relatively low compared to Lidar, I also believe that Lidar is the future trend. However, at this stage, Radar's performance in bad weather is better than Lidar in speed detection and detection distance. Radar has also played an extremely important role in many production programs (such as the Audi A8, Tesla). Radar can also be used as a redundancy for Lidar to ensure system security in the event of a Lidar failure.

I expect Baidu to make Radar play a bigger role and further improve the Apollo system.

---


# Fusion part

Entry function location: [obstacle/onboard/fusion_subnode.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/fusion_subnode.cc)

```
apollo::common::Status Process(const EventMeta &event_meta,
                                 const std::vector<Event> &events);

```

Core function location: [obstacle/fusion/probabilistic_fusion/probabilistic_fusion.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/fusion/probabilistic_fusion/probabilistic_fusion.cc)

```
// @brief: fuse objects from multi sensors(64-lidar, 16-lidar, radar...)
// @param [in]: multi sensor objects.
// @param [out]: fused objects.
// @return true if fuse successfully, otherwise return false
virtual bool Fuse(const std::vector<SensorObjects> &multi_sensor_objects,
                    std::vector<ObjectPtr> *fused_objects);

```

In general, there is no particularly big surprise in the fusion part, and the traditional Kalman filter is still used.

In the introduction of Apollo, they used object-level data fusion, and the input of this part is the object obtained by each sensor.  
![Write a picture description here](https://img-blog.csdn.net/20180801224237206?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
By the way, in the data fusion of multi-source information, according to the data abstraction level, the fusion can be divided into three levels [2]:

-   Data-level fusion sensor bare data fusion, high precision, poor real-time performance, requiring sensors to be homogeneous
-   Feature-level fusion fusion sensor abstract feature vector (speed, direction, etc.), small amount of data, loss of partial information
-   The decision-level fusion sensor itself makes decisions and integrates decision-making results with low precision, small traffic, and strong anti-interference.

Apollo should be fused at the feature level to the objects. Whenever a node receives a new frame of data, the fusion part is called. The input of the fusion part is SensorObjects, and the output is the merged object. The general flow is shown in the figure below.  
![Write a picture description here](https://img-blog.csdn.net/20180801224421267?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
The most important step is naturally the fusion. According to the measurement results of the sensor, Apollo divides the objects into ForegroundObjects and BackgroundObjects according to the is_background flag. Only ForegroundObjects is processed during the fusion.

Fusion function location: [obstacle/fusion/probabilistic_fusion/probabilistic_fusion.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/fusion/probabilistic_fusion/probabilistic_fusion.cc)

```
void FuseForegroundObjects(
      std::vector<PbfSensorObjectPtr> *foreground_objects,
      Eigen::Vector3d ref_point, const SensorType &sensor_type,
      const std::string &sensor_id, double timestamp);

```

We can see from previous experience, sensor data fusion has two parts is more important, that is, **data association** and **dynamic estimates** .

## Data association

The interface for data association is defined as follows:

```
  // @brief match sensor objects to global tracks build previously
  // @params[IN] fusion_tracks: global tracks
  // @params[IN] sensor_objects: sensor objects
  // @params[IN] options: matcher options for future use
  // @params[OUT] assignments: matched pair of tracks and measurements
  // @params[OUT] unassigned_tracks: unmatched tracks
  // @params[OUT] unassigned_objects: unmatched objects
  // @params[OUT] track2measurements_dist:minimum match distance to measurements
  // for each track
  // @prams[OUT] measurement2track_dist:minimum match distacne to tracks for
  // each measurement
  // @return nothing
  virtual bool Match(const std::vector<PbfTrackPtr> &fusion_tracks,
                     const std::vector<PbfSensorObjectPtr> &sensor_objects,
                     const TrackObjectMatcherOptions &options,
                     std::vector<TrackObjectPair> *assignments,
                     std::vector<int> *unassigned_fusion_tracks,
                     std::vector<int> *unassigned_sensor_tracks,
                     std::vector<double> *track2measurements_dist,
                     std::vector<double> *measurement2track_dist) = 0;

```

The implementation of data association is at:

[Obstacle/fusion/probabilistic_fusion/pbf_hm_track_object_matcher.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/fusion/probabilistic_fusion/pbf_hm_track_object_matcher.cc)

```
bool PbfHmTrackObjectMatcher::Match(
    const std::vector<PbfTrackPtr> &fusion_tracks,
    const std::vector<PbfSensorObjectPtr> &sensor_objects,
    const TrackObjectMatcherOptions &options,
    std::vector<TrackObjectPair> *assignments,
    std::vector<int> *unassigned_fusion_tracks,
    std::vector<int> *unassigned_sensor_objects,
    std::vector<double> *track2measurements_dist,
    std::vector<double> *measurement2track_dist) ;

```

In fact, from the "hm" in the file name, we can see some clues. Similar to Lidar, the data association of the Fusion part is also allocated using the Hungarian Matcher.

However, when calculating the associated distance, there is a big difference between the two. The Fusion section calculates only the geometric distance between the two object centers. Let's take a look at the code here, and the details are interesting.

Fusion uses the following code when calculating the associated distance.

Function location: [obstacle/fusion/probabilistic_fusion/pbf_track_object_distance.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track_object_distance.cc)

```
float PbfTrackObjectDistance::Compute(
    const PbfTrackPtr &fused_track, const PbfSensorObjectPtr &sensor_object,
    const TrackObjectDistanceOptions &options) {

  //fused_track 已经融合过obj的航迹
  //sensor_object 来自传感器的，待融合的obj

  //获取此帧数据来源于的传感器类型
  const SensorType &sensor_type = sensor_object->sensor_type;
  ADEBUG << "sensor type: " << sensor_type;
  // 获取上次融合的obj
  PbfSensorObjectPtr fused_object = fused_track->GetFusedObject();
  if (fused_object == nullptr) {
    ADEBUG << "fused object is nullptr";
    return (std::numeric_limits<float>::max)();
  }

  Eigen::Vector3d *ref_point = options.ref_point;
  if (ref_point == nullptr) {
    AERROR << "reference point is nullptr";
    return (std::numeric_limits<float>::max)();
  }
  
  float distance = (std::numeric_limits<float>::max)();
  //获取航迹中最近的来自Lidar的obj
  const PbfSensorObjectPtr &lidar_object = fused_track->GetLatestLidarObject();
  //获取航迹中最近的来自Radar的obj
  const PbfSensorObjectPtr &radar_object = fused_track->GetLatestRadarObject();

  //下面是重点
  if (is_lidar(sensor_type)) {  //如果这次要融合obj是来自源于Lidar
    if (lidar_object != nullptr) {    
    // 如果航迹中已经有来自 Lidar 的obj, 则计算两者的几何距离 
      distance =
          ComputeVelodyne64Velodyne64(fused_object, sensor_object, *ref_point);
    } else if (radar_object != nullptr) {
     // 如果航迹中没有来自 Lidar 的obj, 则计算与 radar obj 的距离,注意这里 
     // sensor_object 和 fused_object 与上面的位置是相反的。原因是这个函数
     // 在计算距几何距离时的实现，是以第一个参数为速度基准，计算v*time_diff
     // 也就是说，当 fused_object 为 radar 时，以 sensor_object 为准。  
      distance =
          ComputeVelodyne64Radar(sensor_object, fused_object, *ref_point);
    } else {
      AWARN << "All of the objects are nullptr";
    }
  } else if (is_radar(sensor_type)) { // 如果这次要融合的obj是来源于Radar
    if (lidar_object != nullptr) { 
      // 如果航迹中已经有来自 Lidar 的obj, 则计算两者的几何距离 
      distance =
          ComputeVelodyne64Radar(fused_object, sensor_object, *ref_point);
    } else if (radar_object != nullptr) {
     // 如果航迹中没有来自 Lidar 的obj, 返回 float 的极值
      distance = std::numeric_limits<float>::max();
      //    distance = compute_radar_radar(fused_object, sensor_object,
      //    *ref_point);
    } else {
      AWARN << "All of the objects are nullptr";
    }
  } else {
    AERROR << "fused sensor type is not support";
  }
  return distance;
}

```

The above code shows that when calculating the geometric distance, Fusion requires that at least one of the two objs calculated is from Lidar, and the distance is measured based on Lidar.

However, the geometric distance is only used for data association, which is prone to Miss Match. That is, when the two tracks intersect, the geometric distance cannot be associated with the appropriate path for the object. I am still not sure how Apollo 2.0 intends to circumvent this problem.  
![Write a picture description here](https://img-blog.csdn.net/20180801224852770?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

##动态 estimation

The dynamic estimation is still the Kalman filter used. There are two interesting points in this part:

Apollo's fusion part uses the non-simplified estimation error covariance matrix Pfor the Kalman filter of the standard Kalman filter Apollo fusion part for scalability.k ∣ k\mathbf {P} _{k|k}Pk | k Update formula  
![Write a picture description here](https://img-blog.csdn.net/20180801225202940?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

We compare the formula of the standard Kalman filter above to illustrate why I feel that the above two points are interesting.

###1. Using standard Kalman filter

From the code details, Apollo uses standard Kalman filtering, and when updating objects from Lidar and Radar, Apollo uses the same observation matrix \mathbf {H} for both.

The interesting thing here is that, in general, the observation matrices of Lidar and Radar are different because the data obtained by the two are different. To better illustrate this problem, I suggest that you first read the following article [extended Kalman filter EKF and multi-sensor fusion](https://blog.csdn.net/young_gy/article/details/78468153) .  
This question has been answered. The mainland Radar provides data from the Cartesian coordinate system, so it is possible to use an observation matrix.

###2. Using a non-simplified estimation error covariance matrix Pk ∣ k\mathbf {P} _{k|k}Pk | k Update formula

Let's take a look at the difference

Standard Kalman Filter: ${\displaystyle \mathbf {P} _{k|k}=(\mathbf{I-\mathbf {K} _{k}\mathbf {H} _{k}})\mathbf { P} _{k|k-1}} $  
Apollo:${\displaystyle \mathbf {P} _{k|k}=(\mathbf{I-\mathbf {K} _{k}\mathbf {H} _{k}})\mathbf {P} _{k|k-1}}(\mathbf{I-\mathbf {K} _{k}\mathbf {H} _{k}})^{\mathbf{T }}+ \mathbf{K}_ {k}\mathbf{R}_ {k}\mathbf {K} _{k}^{\mathrm {T}} $

In conjunction with the introduction of Kalman filtering on Wikipedia, I will summarize the background of the problem:

-   The updated formula for the **estimated error covariance matrix** $\mathbf {P} _{k|k} $ used by Apollo is the so-called Joseph form, while the standard Kalman filter usually uses a simplified version of the update formula.
-   The simplified version of the update formula is computationally intensive and widely used in practice, but only effective when the Kalman gain is optimal.
-   Two cases of Joseph form must be used:  
    – Non-optimal Kalman gain is used  
    – the algorithm is too low precision, causing numerical stability related problems

The Apollo community answered two reasons, one for the accuracy of the algorithm; the other is that because of the power (and expensive) of the computational unit, the non-simplified version of the Kalman filter does not take too long to calculate. Taken together, Apollo chose the update equation for **Joseph form** .

Finally, we use a table to make a simple summary of the obstacle-aware part.  
![Write a picture description here](https://img-blog.csdn.net/20180801230750927?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

#模块 inlet selection

When I analyzed Apollo, I focused on the implementation and solution of the module, not the writing skills of the code. So for the code, just at the end of this article, simply talk about the function entry mentioned in the previous article.

##Preception Module entry

The main entry to the Preception module is actually very clear:

[Modules/perception/main.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/main.cc)

```
APOLLO_MAIN(apollo::perception::Perception);

```

This is a macro that is expanded after

```
int main(int argc, char **argv) {                            
    google::InitGoogleLogging(argv[0]);                        
    google::ParseCommandLineFlags(&argc, &argv, true);         
    signal(SIGINT, apollo::common::apollo_app_sigint_handler); 
    APP apollo_app_;                                           
    ros::init(argc, argv, apollo_app_.Name());                 
    apollo_app_.Spin();                                        
    return 0;                                                  
  }

```

Regarding the specific meaning of this code, it is strongly recommended to refer to the [source code analysis of the Apollo Planning module](https://blog.csdn.net/davidhopper/article/details/79176505) in the article of Zhixing Heyi 2018 . It is worth mentioning that most of Apollo's modules start in a similar form, and the analysis is similar.

The initialization code for the awareness module is as follows:

```
Status Perception::Init() {
  AdapterManager::Init(FLAGS_perception_adapter_config_filename);

  RegistAllOnboardClass();
  /// init config manager
  ConfigManager* config_manager = ConfigManager::instance();
  if (!config_manager->Init()) {
    AERROR << "failed to Init ConfigManager";
    return Status(ErrorCode::PERCEPTION_ERROR, "failed to Init ConfigManager.");
  }
  AINFO << "Init config manager successfully, work_root: "
        << config_manager->work_root();
  //---------------------------注意这段-------------------------------
  const std::string dag_config_path = apollo::common::util::GetAbsolutePath(
      FLAGS_work_root, FLAGS_dag_config_path);   

  if (!dag_streaming_.Init(dag_config_path)) {
    AERROR << "failed to Init DAGStreaming. dag_config_path:"
           << dag_config_path;
 //-----------------------------分割线---------------------------------------
    return Status(ErrorCode::PERCEPTION_ERROR, "failed to Init DAGStreaming.");
  }
  callback_thread_num_ = 5;

  return Status::OK();
}

```

The program startup code is:

```
Status Perception::Start() {
  dag_streaming_.Start();
  return Status::OK();
}

```

From these two paragraphs, we can see the figure of directed acyclic graph (DAG). In fact, it is mentioned in the official documentation that the framework of the perceptual module is based on DAG diagrams, and each function appears as a sub-node in the DAG [7].

> The perception framework is a directed acyclic graph (DAG). There are three components in DAG configuration, including sub-nodes, edges and shared data. Each function is implemented as a sub-node in DAG. The sub-nodes that share data have An edge from producer to customer.

The DAG diagram consists of the following Subnodes

Obstacle perception:

-   LidarProcessSubnode
-   RadarProcessSubnode
-   FusionSubnode

Traffic light detection

-   TLPreprocessorSubnode
-   TLProcSubnode

The DAG diagram is shown below:

![Write a picture description here](https://img-blog.csdn.net/20180801233837321?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)  
So far, in fact, the entry of the program implementation of each part should be relatively clear, as long as you find the execution code of each Subnode.

#Doxygen

Baidu provides Doxygen's documentation system, which helps us understand the relationship between specific parts of the code [8].

> Doxygen is an open source cross-platform, documented system similar to JavaDoc style, fully supports C, C++, Java, Objective-C and IDL languages, and partially supports PHP and C#. The syntax of the annotations is compatible with Qt-Doc, KDoc and JavaDoc. Doxygen can start from a set of archive source files, generate an online class browser in HTML format, or offline LATEX, RTF reference manual Doxygen

Apollo's [Doxygen website](https://apolloauto.github.io/doxygen/apollo/index.html)  
is shown as the obstacle dependency graph of the obstacle perception (obstacle).  
![Write a picture description here](https://img-blog.csdn.net/20180801234015688?watermark/2/text/aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L3l1eHVhbjIwMDYyMDA3/font/5a6L5L2T/fontsize/400/fill/I0JBQkFCMA==/dissolve/70)

As can be seen from the image, the obstacle perception calls the related functions of Radar, Lidar and Fusion, and its entry is in the onboard folder. Let's see what files are in the onboard folder.

```
.
├── BUILD
├── fusion_subnode.cc
├── fusion_subnode.h
├── hdmap_input.cc
├── hdmap_input.h
├── hdmap_input_test.cc
├── lidar_process.cc
├── lidar_process.h
├── lidar_process_subnode.cc
├── lidar_process_subnode.h
├── lidar_process_test.cc
├── object_shared_data.h
├── obstacle_perception.cc
├── obstacle_perception.h
├── radar_process_subnode.cc
├── radar_process_subnode.h
└── sensor_raw_frame.h

```

It can be inferred that these files are the execution code of the Subnode involved in the obstacle perception. I selected the following 3 files as the entry point for the analysis.

-   Radar -> [radar_process_subnode.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/radar_process_subnode.cc)
-   Lidar -> [lidar_process_subnode.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/lidar_process_subnode.cc)
-   Fusion -> [fusion_subnode.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/fusion_subnode.cc)

There are actually some problems left here. Since I am not going to analyze the traffic light perception, I jumped directly here to start analyzing the obstacle perception part. The problem is that I am not quite sure about the specific role of certain documents.

For example, [obstacle_perception.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/obstacle_perception.cc) , this file looks a lot like the portal, the description is also very similar to the portal, the content is also very similar to the portal, but it seems to have nothing to do with the Subnode framework.

```
/**
   * @brief The main process to detect, recognize and track objects
   * based on different kinds of sensor data.
   * @param frame Sensor data of one single frame
   * @param out_objects The obstacle perception results
   * @return True if process successfully, false otherwise
   */
  bool Process(SensorRawFrame* frame, std::vector<ObjectPtr>* out_objects);

```

This function directly calls the three core functions of obstacle perception.

```
1. lidar_perception_->Process(velodyne_frame->timestamp_,
                                    velodyne_frame->cloud_, velodyne_pose);
2. radar_detector_->Detect(radar_frame->raw_obstacles_, map_polygons,
                                options, &objects);
3. fusion_->Fuse(multi_sensor_objs, &fused_objects);

```

Another example is [lidar_process.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/lidar_process.cc) . The contents of this file are similar to those of [lidar_process_subnode.cc](https://github.com/ApolloAuto/apollo/blob/master/modules/perception/obstacle/onboard/lidar_process_subnode.cc) , but they are not identical and are also confusing. I hope that everyone can give me some advice.
