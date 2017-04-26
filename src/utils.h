#ifndef UTILS_H
#define UTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Utils
{
	PointCloudT::Ptr removeNaNPoints (const PointCloudT::ConstPtr &cloud);
    PointCloudT::Ptr getCloudCluster(const PointCloudT::ConstPtr &cloud);

}

#endif // UTILS_H