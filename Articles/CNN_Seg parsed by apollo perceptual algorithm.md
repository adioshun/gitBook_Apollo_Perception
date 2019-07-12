
# Cn_seg parsed by apollo perceptual algorithm

> Posted on 2018-03-28

아폴로도 `cnn_seg module`모듈에서 인식에 대한 작업을 수행한다. 하지만 코드를 공개 하지 않았기에 파악한 수준만 정리 하고자 한다. `When apollo is doing context awareness, the cnn_seg module is used to perform target detection on the point cloud. After deep understanding of the principles, I could not help but sigh, it can still be done this way. Unfortunately, I have not found a paper related to this method (if a classmate knows, please tell me), apollo does not open source its training source (= =! Say good open source). Here, I will make a record of what I have learned during this time, including some doubts, and it can be regarded as a joke, and all the gods are welcome to enlighten me.`

## 1. Cnn_seg

`cnn _seg `은 **BBox**기반의 탐지 방법대신 **semantic segmentation** 방식을 사용하고 있다. `First, cnn _seg does not use the bbox-based method for target detection (although Baidu has published several papers on this target detection method before), but uses semantic segmentation.  `

시멘틱 세그멘티이션으 가장 큰 문제점은 픽셀단위로 분석을 한다는 것이다. 그럼 어떻게 픽셀을 하나의 클러스터로 합치게 되는가? 이 것이 `cnn _seg `의 주요 기능중에 하나이다. `Semantic segmentation for target detection, there is a problem that can not be avoided, that is, semantic segmentation is done at the pixel level. After the semantic segmentation, how to cluster the detected pixels into a single target?  This is_ the wonderful thing _about cnn_ seg. `

At the same time of semantic segmentation, cnn_seg also did one thing, that is, it returned a layer called **center offset**, and then implemented the cluster according to the result of center offset and semantic segmentation. Detection of a single target.

아폴로으 `cnn _seg`는 두단계로 나누어져 있다. `Students who have read the official documentation of apollo should know that cnn _seg is actually divided into two steps. `
- 첫 단계는 딥러닝을 이용하여 5개 레이어의 정보를 예측 하는 것이다. `The first step is to use a deep learning network to predict five layers of information.`
	- 코드상으로는 5개레이어가 넘지만 다른 레이어들은 현재 사용되고 있지 않다. ` (From the source code, there are more than five layers, but other layers are not used yet. To, the estimated version will gradually open),`
	-  center offset, objectness, positiveness, object height, class probability. 
- 두번째 단계는 이 5개 레이어 정보를 이용하여서 클러스터링 하는 것이다. `The second step is to use the five layers of information for the cluster.  `

5개 레이어란 무었일까? 어떻게 이들을 이용하여서 예측 작업을 수행 하는가? `What do these five layers mean?  How does cnn_ seg perform target detection based on these five layers of information?  `
The approximate logic is:  

1. According to the **objectness** layer information, the obstacle grid point object is detected  
2. According to the **center offset** layer information, the detected obstacle grid point object is clustered to obtain clusters  
3. According to **positiveness and object height** Layer information, filter the background and the points in each cluster.  
4. According to the **class probability**, classify each cluster to get the final target.  

I will display the layer information and results separately, combined with the image to illustrate.

### 1. 1 Objectness

![](https://pic3.zhimg.com/80/v2-66cc8c860650107034c21fd5e3460a96_hd.jpg)

I use images and point clouds to illustrate the role of the objectness layer. The objectness output is a two-dimensional matrix of [0,1], corresponding to the grid diagram of the point cloud (in cnn_seg, a 2.5D grid is constructed for the point cloud, and each grid corresponds to an 8-dimensional feature. See the official website for details. Documentation). By taking a threshold on objectness (default 0.5), the pixel of the object can be detected. I use the reflection intensity information of the point cloud to display the detected area.

### 1.2 Center offset

In the previous step, by taking a threshold on objectness, only a single obstacle grid (pixels in the corresponding image) can be detected. How do you cluster these individual rasters? It is through the center offset.  
First look at the results of clustering by center offset, I display them in different colors.  

![](https://pic1.zhimg.com/80/v2-9e368604592a9b0389f8a74dac2ca944_hd.jpg)

The offset center is a two-layer two-dimensional matrix, which is the offset of the x-axis and the offset of the y-axis. Combining the two layers corresponds to the offset of each grid. I use the direction and length of the arrow to indicate the offset. .  

![](https://pic3.zhimg.com/80/v2-4f3a42f8c2129907d3e6bc376956848e_hd.jpg)

For the sake of understanding, I added a grid map to the map, the size of the grid is roughly the _same as the grid size in_ cnn _seg.  
_

![](https://pic3.zhimg.com/80/v2-6b0385b788432c9cc997ba80c1e0e49e_hd.jpg)

_It is through the center offset information of each grid that cnn_ seg achieves clustering.  
The specific logic is as follows:  
1. Determine whether the current grid point is the detected object grid  
. 2. If no, skip it. Otherwise, according to the direction and length of the offset corresponding to the current grid, gradually point to the offset. Move near the grid  
3. Until it can't move or encounter the path that has passed, stop, and record the stop position as the center _node  
  
_of the current grid. _4. Repeat 1~3 until all the grid points are traversed_ _._ Clustering _grid points with the same center_ node or center_node very close together as the same object

You can look at the distribution of center offset when the vehicle is dense.  

![](https://pic2.zhimg.com/80/v2-271a10503da886feee8f6e4752a58901_hd.jpg)

![](https://pic1.zhimg.com/80/v2-c99bb7bcc3c754150e8bd7a1dc5e4890_hd.jpg)

This method is very interesting and is more suitable for point cloud data, because after the 2.5D rasterization of the point cloud, the grid contains the distribution information of the point cloud (such as the fluctuation information of the surface of the object). As for how to train to get such an offset? amount. . . . I also want to know! !

### 1.3 Positiveness

Positiveness is a filter message. As you can see from the previous step, the clusters that are obtained have a lot of background areas. By counting the mean of each cluster's positiveness, after passing the threshold (default 0.1), some background clusters can be filtered out.

![](https://pic2.zhimg.com/80/v2-17692238effac90bce8dbb24bac83e39_hd.jpg)

### 1.4 Object height

Object height is also a filter information that filters out the higher points in the raster.  
No object height information is used  

![](https://pic4.zhimg.com/80/v2-2aeb3394cd1acf93381971bb81664ce3_hd.jpg)

Use object height information  

![](https://pic1.zhimg.com/80/v2-c131f9dd7d580db3d94095cbcd061bd0_hd.jpg)

### 1.5 Class probability

Through the above steps, cnn _seg has split the point cloud into clusters one by one. After the release of apollo 2.0, cnn_ seg added a new feature, which is to classify each cluster. It is the information of the class probability. The class probability is a five-layer two-dimensional matrix, which corresponds to the probability value of the five categories of class probability (the five categories are big car, car, pedestrian, bicycle, unknown). By counting the probability values ​​of each class of each cluster, the highest score is its class, thus realizing the classification of each cluster and obtaining the final target.  
In the picture, I use red to indicate the car, blue for the big car, blue for the bicycle, and yellow for the pedestrian.  

![](https://pic2.zhimg.com/80/v2-4c22edd7a5f53ba003ce54c9233d8ca5_hd.jpg)

## 2 postscript

The point cloud data used in this test is the Sagitar RS-16 line data. However, apollo's open source model is trained with 64 lines. It can be seen that this model is good for the generalization and effect of the data.

도로변의 물체를 오판 하는 문제가 발생 할수도 있다. 아폴로에서는 고밀도 맵을 이용하여서 도로변 지역은 ROI에 포함 시키지 않고 진행 하고 있다. 이러한 방법은 약간은 트릭이지만 현실적으로 잘 동작 하는 방법이다. ` Some people may think that there are many misidentifications on the roadside! In response to this problem, apollo is actually also solved (only in this article is not reflected, you can see the video of apollo official website). Apollo uses a high-precision map to construct a ROI filter that can directly filter out non-travelable areas. The method is somewhat tricky, but it's really practical (so it's more dependent on positioning, and it's still real-time).  `

Some people may also have the same doubts as I did at the beginning, that is, the function of objectness and positiveness seems to be somewhat repetitive. It is not good to detect the object directly with positiveness. I have also doubted this, but after doing experiments, I found that directly using positiveness for semantic segmentation will actually cause more missed detections (and it may be that my training methods are not clever enough). So my understanding is that objectness and positiveness are actually a process from coarse to fine. The target points are identified by objectness, and the lower scores are filtered out by fine recognition. One of the benefits of doing this is estimated to be able to reduce missed detection.  
Then the question comes, how to define the ground truth of objectness? The ground truth of positiveness is better understood, but what is the ground truth of objectness in order to train the effect of objectness? If some students know, please don't hesitate, sincerely ask for advice! !  
Another question is, how should the ground truth of the center offset be defined?