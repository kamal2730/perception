# To DO
- [x] Custom message for PC DET
- [x] Custom message for RGB DET
- [x] modify perception to publish clusters in Custom msg
- [x] py script to visualize clusters
- [ ] remove plane
- [ ] publish tf2 for each cluster Eigen Vectors + normal estimation(plane)
- [x] RGB DET NODE
- [x] FUSING NODE
- [ ] RGB DET NODE (migrate -> client)
- [ ] 6DOF
- [ ] Testing on real feed
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
