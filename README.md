# To DO
- [x] Custom message for PC DET
- [x] Custom message for RGB DET
- [x] modify perception to publish clusters in Custom msg
- [x] node to visualize clusters
- [x] remove plane
- [x] publish pose for each cluster Eigen Vectors + normal estimation(plane)
- [x] RGB DET NODE
- [x] FUSING NODE
- [ ] RGB DET NODE (migrate -> client)
- [ ] Integrating RGB server to perception
- [ ] Publish tf2
- [ ] 6DOF
- [ ] Testing on real feed
---
> [!WARNING]
> While visualizing pose use you'r camera frame to avoid segmentation fault on rviz.
---
```mermaid
flowchart TD
    Input[Sensor Input] --> PreProc[Preprocessing]
    PreProc --> RGB_Branch[RGB Processing Branch]
    PreProc --> PCL_Branch[Point Cloud Branch]
    
    RGB_Branch --> 2D_Det[2D Object Detection]
    RGB_Branch --> Class[Object Classification]
    
    PCL_Branch --> Segment[Segmentation]
    PCL_Branch --> Normal[Normal Estimation]
    
    2D_Det --> Fusion[Information Fusion]
    Class --> Fusion
    Segment --> Fusion
    Normal --> Fusion
    
    Fusion --> BB[Bounding Box Computation]
    BB --> Pose[Pose Estimation]
    Pose --> Output[Final Output]
```
