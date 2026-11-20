# Message storing raw 2D and 3D information about a single AprilTag detection.
#   see class TagDetection in TagDetection.h
uint64 id  # id of AprilTag (not necessarily unique)
float32 tag_size  # tag size in m

geometry_msgs/Pose pose  # pose of the marker in the camera frame

Point2D32[4] corners2d  # AprilTag corners in image [px]
Point2D32 cxy  # AprilTag center in image [px]
float32[9] homography # 3x3 homography
Point2D32 hxy # The homography is relative to image center, whose coordinates are below.

bool good
uint64 obsCode  
uint64 code  
uint32 hammingDistance  
uint32 rotation
float32 observedPerimeter # [px]
