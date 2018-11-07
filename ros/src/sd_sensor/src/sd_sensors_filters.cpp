#include "sd_sensor/angular_bounds_list_filter.h"

#include "pluginlib/class_list_macros.h"


PLUGINLIB_EXPORT_CLASS(sd_sensor::LaserScanAngularBoundsListFilter, filters::FilterBase<sensor_msgs::LaserScan>)