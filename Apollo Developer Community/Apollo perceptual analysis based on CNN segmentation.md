# Apollo perceptual analysis based on convolutional neural network segmentation


- 이전 챕터에서 살펴본 **high-precision map ROI filtering**를 통해서 배경 물체들이 제거 된다. 이후 ROI의 포인트 클라우드 만이 segmentation module에 입려 된다. 세그멘테이션 모듈은 차량들을 분류해 낸다. `Last week, we introduced the high-precision map ROI filtering , Apollo got filtered and only included point clouds belonging to the ROI. Most of the background obstacles, such as buildings on the road side, trees, etc., were removed. The point cloud within the ROI is passed to the segmentation module . The segmentation module detects and divides foreground obstacles such as cars, trucks, bicycles and pedestrians.`


세그멘테이션은 4단계로 구분 된다. `This phase consists of 4 sub-processes:`
- Channel feature extraction
- Obstacle prediction based on convolutional neural network
- Obstacle cluster
- Post processing

## 1. Channel feature extraction


입력된 포인트 클라우드는 2D Top-view로 변환 된다. `Given a point cloud framework (cloud_roi), Apollo builds a top view (ie projected onto the XY plane) 2D grid in the local coordinate system. `

변환된 2D x,y좌표계를 기반으로 각 포인트들은 사전에 정의된 그리드로 정량화(quantized) 된다. `Based on the X, Y coordinates of the point, each point is quantized into a unit of the 2D grid within a predetermined range relative to the origin of the LiDAR sensor. `

정량화후 각 셀의 포인트들에 대해 **8개의 통계치(Feature)**를 계산 한다. 이 값은 다은 단계에서 CNN에 입력 된다. `After quantification, Apollo calculates eight statistical measurements of the points in each cell in the grid , which will be the input channel characteristics passed to the CNN in the next step.`

statistical measurements calculated:
- The maximum height of the dot in the cell --max_height_data
- The strength of the highest point in the cell -- top_intensity_data
- The average height of the points in the cell --mean_height_data
- The average strength of the points in the cell --mean_intensity_data
- The number of points in the cell --count_data
- The angle of the cell center relative to the origin -- direction_data
- The distance between the center of the cell and the origin --distance_data
- Binary value indicates whether the cell is empty or occupied -- nonempty_data

```cpp
/// file in apollo/modules/perception/obstacle/onboard/lidar_process_subnode.cc
void LidarProcessSubnode::OnPointCloud(const sensor_msgs::PointCloud2& message) {
  /// call hdmap to get ROI
  ...
  /// call roi_filter
  ...
  /// call segmentor
  std::vector<ObjectPtr> objects;
  if (segmentor_ != nullptr) {
    SegmentationOptions segmentation_options;
    segmentation_options.origin_cloud = point_cloud;
    PointIndices non_ground_indices;
    non_ground_indices.indices.resize(roi_cloud->points.size());
    std::iota(non_ground_indices.indices.begin(), non_ground_indices.indices.end(), 0);
    if (!segmentor_->Segment(roi_cloud, non_ground_indices, segmentation_options, &objects)) {
      ...
    }
  }
}

/// file in apollo/master/modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.cc
bool CNNSegmentation::Segment(const pcl_util::PointCloudPtr& pc_ptr,
                              const pcl_util::PointIndices& valid_indices,
                              const SegmentationOptions& options,
                              vector<ObjectPtr>* objects) {
  // generate raw features
  if (use_full_cloud_) {
    feature_generator_->Generate(options.origin_cloud);
  } else {
    feature_generator_->Generate(pc_ptr);
  }
  ...
}
```

모든 작업들은 `CNNSegmentation`클래스에서 수행된다. `As can be seen from the above code, like the high-precision map ROI filter, all the split operations are done in the CNNSegmentation class. `

입력된 포인트 클라우드 `(x, y, z, i)`에서 8개의 feature를 추출 후 CNN의 입력 매트릭스(`[1, 8 The matrix of , w, h]`)를 생성한다. `Next, we start with the code to see how to get the above 8 types of data from a set of point clouds {(x, y, z, i)}, and finally the point cloud set mapped by cloud_local will generate a [1, 8 The matrix of , w, h] is used as the input to the CNN, where w and h are defined in the external file, both of which are 512. `


옵션 `use_full_cloud_ flag`는 입력데이터가 ROI필터링이 된것인지 아닌지(Original Data)를 나타낸다. `The use_full_cloud_ flag here is actually processing the original point cloud or processing the roi point cloud (without the background), using the original point cloud use_full_cloud_=true by default.`


먼거리의 라이다 입력 데이터 전부를 분석하는것는 비 효율적이므로 `point_cloud_range`를 이용하여서 60m이내로 제한 할수 있다. `There is a caveat here. The x and y of the original point cloud have their range, which is the range of Lidar's perception. There is such a premise: in fact, the lidar detects a point cloud within a 360-degree range. If the point cloud is too far away from the lidar lidar, then these points do not need to be processed. Handling point clouds near the vehicle (within 60 meters of Eg) saves computing resources while reducing complexity. The scope of this filter is controlled by the parameter point_cloud_range parameter, which is 60 meters by default.`


### 1.1 Map the actual xy coordinates of the point cloud to the input matrix HxW plane coordinates, and filter the point cloud height

라이다의 측정 형식은 x,y 이지만, CNN의 입력은 H,W방식이다. `It is known from the above that the point cloud processed in the segmentation phase is actually a point cloud in the Lidar physical distance x: [-60, 60], y: [-60, 60], but the input size accepted by CNN is 1x8xHxW. `


따라서 포이트 클라우드의 좌표방식을 평면의 HxW를 변경할 필요가 있다. `Therefore, it is necessary to remap the point cloud coordinates in this range to the plane of size HxW.`

 
The conversion is actually very simple:
- Eg 1. If the point X-axis coordinate px is mapped from the range [a, b], stretch/compression to the range [c, d], then the mapped new coordinate `px2 = c + (dc) / (ba) * (px -a)`

- Eg 2. If the point X-axis coordinate px is mapped from the range [-a, a], stretch/compression to the range [0, c], then the mapped new coordinate `px2 = c/2a * (px-(-a) )`


Then go back to the code and look at the **mapping process**:
```cpp
/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.cc
void FeatureGenerator<Dtype>::Generate(const apollo::perception::pcl_util::PointCloudConstPtr& pc_ptr) {
  const auto& points = pc_ptr->points;

  map_idx_.resize(points.size());
  float inv_res_x = 0.5 * static_cast<float>(width_) / static_cast<float>(range_);   // E.g.2 inv_res_x == c/2a(a=range_, c=widht_)
  float inv_res_y = 0.5 * static_cast<float>(height_) / static_cast<float>(range_);  // E.g.2 inv_res_x == c/2a(a=range_, c=widht_)

  for (size_t i = 0; i < points.size(); ++i) {
    // 1. remove the cloud points which height is out of the interval [-5.0,5.0]
    if (points[i].z <= min_height_ || points[i].z >= max_height_) {          
      map_idx_[i] = -1;
      continue;
    }
    // * the coordinates of x and y are exchanged here
    int pos_x = F2I(points[i].y, range_, inv_res_x);  // compute mapping coordinate: col
    int pos_y = F2I(points[i].x, range_, inv_res_y);  // compute mapping coordinate: row
    // 2. remove the cloud points which out of the interval x:[-60,60], y:[-60,60]
    if (pos_x >= width_ || pos_x < 0 || pos_y >= height_ || pos_y < 0) {
      map_idx_[i] = -1;
      continue;
    }
    map_idx_[i] = pos_y * width_ + pos_x;
}

/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/util.h
inline int F2I(float val, float ori, float scale) {        // compute mapping coordinate in E.g.2, (px-(-a)) * c/2a
  return static_cast<int>(std::floor((ori - val) * scale));
}
```

맵핑 작업 + 2가지 필터링 작업 `This mapping process is easy to see from the above code, as well as two filtering processes:`

- `-5m ~ 5m` 제거 : 신호등이 5m 이므로 이보다 높은것은 빌딩 등으로 불필요 하다. `Remove point clouds with heights above 5 meters or below -5 meters. The signal light is about 5 meters high, so 5 meters or more may be an invalid point cloud such as a building, which can be removed.`

- `60m~`제거 : 멀리 있는 물체 제거 `Remove the point cloud where xy is 60 meters away. If the range is too large, the point cloud that is too far away from the car, even if it contains objects, there is no need to detect it.`


### 1.2 Calculate the 8 types of data in the cell

```cpp
/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.cc
bool FeatureGenerator<Dtype>::Init(const FeatureParam& feature_param, caffe::Blob<Dtype>* out_blob) {
  for (int row = 0; row < height_; ++row) {
    for (int col = 0; col < width_; ++col) {
      int idx = row * width_ + col;
      // * row <-> x, column <-> y
      float center_x = Pixel2Pc(row, height_, range_);     // compute mapping coordinate: center_x
      float center_y = Pixel2Pc(col, width_, range_);      // compute mapping coordinate: center_y
      constexpr double K_CV_PI = 3.1415926535897932384626433832795;
      direction_data[idx] = static_cast<Dtype>(std::atan2(center_y, center_x) / (2.0 * K_CV_PI)); // compute direction_data(channel 6)
      distance_data[idx] = static_cast<Dtype>(std::hypot(center_x, center_y) / 60.0 - 0.5);       // compute distance_data(channel 7)
    }
  }
  return true;
}

void FeatureGenerator<Dtype>::Generate(const apollo::perception::pcl_util::PointCloudConstPtr& pc_ptr) {
  for (size_t i = 0; i < points.size(); ++i) {
    // 1. remove the cloud points which height is out of the interval [-5.0,5.0]
    ...
    // 2. remove the cloud points which out of the interval x:[-60,60], y:[-60,60]
    ...
    float pz = points[i].z;    
    float pi = points[i].intensity / 255.0;
    if (max_height_data_[idx] < pz) {        // update max_height_data(channel 1)
      max_height_data_[idx] = pz;
      top_intensity_data_[idx] = pi;		 // update top_intensity_data(channel 2)
    }
    mean_height_data_[idx] += static_cast<Dtype>(pz);    // accumulated  mean_height_data
    mean_intensity_data_[idx] += static_cast<Dtype>(pi); // accumulated mean_intensity_data
    count_data_[idx] += Dtype(1);                        // compute count_data(channel 5)
  }

  for (int i = 0; i < siz; ++i) {
    constexpr double EPS = 1e-6;
    if (count_data_[i] < EPS) {
      max_height_data_[i] = Dtype(0);
    } else {
      mean_height_data_[i] /= count_data_[i];       // compute  mean_height_data(channel 3)
      mean_intensity_data_[i] /= count_data_[i];    // compute  mean_intensity_data(channel 5)
      nonempty_data_[i] = Dtype(1);                 // compute nonempty_data(channel 8)
    }
  }
}

/// file in apollo/modules/perception/obstacle/lidar/segmentation/cnnseg/util.h
inline float Pixel2Pc(int in_pixel, float in_size, float out_range) {
  float res = 2.0 * out_range / in_size;
  return out_range - (static_cast<float>(in_pixel) + 0.5f) * res;
}
```

The above code is marked to clearly understand the data generation process. 

그리드내 점들의 거리와 방향 정보는 실제 데이터에 독립적이므로 `Init function`에서 계산을 수행한다.  `The distance and direction of the point in the grid from the origin are independent of the actual data, so the calculation is completed early in the Init function;`

다른 6개 type의 데이터들은 입력단계에서 계산되므로 `Generate function` 에서 수행된다.` the other six types of data need to be calculated according to the input, so it is calculated in the Generate function. `

The Pixel2Pc function is actually the above coordinate mapping, and the soup is not changed. 

But there is a problem to be aware of, here is an additional 0.5f, this function is actually the coordinate conversion of the center point of the calculation grid. 

For example, if the first grid x coordinate is 0, then the grid center point is 0.5 (0-1 center).


## 2. Obstacle prediction based on convolutional neural network

Based on the channel characteristics described above, Apollo uses a deep full convolutional neural network (FCNN) to predict cell obstacle properties, including offset displacement (called center offset), objectivity, positivity, and object height at the center of the potential object. 

![](https://i.imgur.com/dnAwTGV.png)


입력은 `W x H x C` 이미지 이다. `As shown in Figure 2, the input to the network is a W x H x C channel image, where:`
- W represents the number of columns in the grid
- H represents the number of rows in the grid
- C represents the number of channel features



CNN은 세개의 레이어로 이루어져 있다. `The complete convolutional neural network consists of three layers:`
- Downstream coding layer (feature encoder)
- Upstream decoding layer (feature decoder)
- Obstacle property prediction layer (predictor)

형상 인코더는 입력으로 채널 형상 영상을 취하고 형상 추출이 증가함에 따라 공간 분해능을 지속적으로 하향 샘플링한다.`The feature encoder takes the channel feature image as input and continuously downsamples its spatial resolution as the feature extraction increases. `

이후, 형상 디코더는 형상 이미지를 점차 정렬한다. The feature decoder then gradually aligns with the feature image. 

입력 2D 메쉬의 공간 분해능에 대한 샘플링으로 형상 영상의 공간 세부사항을 복원하여 셀 방향의 장애물 위치와 속도 특성을 쉽게 예측할 수 있다`Upsampling to the spatial resolution of the input 2D mesh, the spatial details of the feature image can be restored to facilitate the prediction of obstacle position and velocity properties in the cell direction. `

Downsampling and upsampling operations are implemented according to a stacked convolution/distribution layer with a non-linearly active (ie, ReLu) layer.

The execution split entry function in the code is as follows, calculated by the forward calculation of caffe's Forword function:

```cpp
/// file in apollo/master/modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.cc
bool CNNSegmentation::Segment(const pcl_util::PointCloudPtr& pc_ptr,
                              const pcl_util::PointIndices& valid_indices,
                              const SegmentationOptions& options,
                              vector<ObjectPtr>* objects) {
  // generate raw features
  ...

  // network forward process
#ifdef USE_CAFFE_GPU
  caffe::Caffe::set_mode(caffe::Caffe::GPU);
#endif
  caffe_net_->Forward();
  PERF_BLOCK_END("[CNNSeg] CNN forward");
}
```

The obstacle segmentation based on convolutional neural network uses UNet's FCNN. 

The specific structure is as follows:

|![](https://i.imgur.com/Hml2Z8X.png)|![](https://i.imgur.com/a7VB7Vw.png)|
|-|-|

위 뉴럴네트워크의 결과물은 6개 파트로 나누어 진다. 자세한 파트별 기능은 테이블을 참조 하기 바란다. `The output of the above neural network is divided into six parts, and the functions of each part are shown in the above table. `

This process is the traditional CNN segmentation.

## 3. Obstacle clustering

