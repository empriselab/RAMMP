# Segmentation mask (single-channel, same resolution as input image)
sensor_msgs/Image segmentation_mask

# Bounding box in pixel coordinates [x_min, y_min, x_max, y_max]
int32[4] bounding_box

# cup pose
float64[7] pose  #xyz_QxQyQzQw

# Whether successfully detected a cup
bool success
