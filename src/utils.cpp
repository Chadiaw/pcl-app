#include "utils.h"
#include <pcl/filters/filter.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

/**
 * @brief Removes all NaN points from given point cloud
 * @param cloud : input cloud.
 * @return The filtered point cloud.
 */
PointCloudT::Ptr Utils::removeNaNPoints (const PointCloudT::ConstPtr &cloud) {
    std::vector<int> indices;
	PointCloudT::Ptr filteredCloud(new PointCloudT);

    pcl::removeNaNFromPointCloud(*cloud, *filteredCloud, indices);

    return filteredCloud;
}

/**
 * @brief extract the biggest cloud cluster from the given point cloud
 * @param cloud
 * @return A point cloud representing the biggest cluster.
 * @details RANSAC algorithm is first used to extract planar components from the cloud.
 * EuclidianClusterExtraction algorithm is then used to extract cloud clusters from remaining cloud.
 * The cluster with the most points is returned.
 */
PointCloudT::Ptr Utils::getBiggestCluster(const PointCloudT::ConstPtr &cloud) {

    PointCloudT::Ptr cloud_f (new PointCloudT);
    //std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    /*
      pcl::VoxelGrid<PointT> vg;
      PointCloudT::Ptr cloud_filtered (new PointCloudT);
      vg.setInputCloud (cloud);
      vg.setLeafSize (0.01f, 0.01f, 0.01f);
      vg.filter (*cloud_filtered);
      std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
      */

	PointCloudT::Ptr cloud_filtered(new PointCloudT);
	pcl::copyPointCloud(*cloud, *cloud_filtered);

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    PointCloudT::Ptr cloud_plane (new PointCloudT ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;


    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    // Get the cloud cluster (only the first/biggest one)
    PointCloudT::Ptr cloud_cluster (new PointCloudT);
    std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    return cloud_cluster;

    /*
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        PointCloudT::Ptr cloud_cluster (new PointCloudT);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;


        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
        j++;
    }
    */

}

/**
 * @brief Gets the closest point to the origin in the given point cloud.
 * @param cloudRGB : Point cloud with color information.
 * @return
 */
pcl::PointXYZ Utils::getClosestPoint(const PointCloudT::ConstPtr &cloudRGB) {
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloudRGB, *cloud);


	pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	tree_->setInputCloud(cloud);

	std::vector<int> nn_indices(1);
	std::vector<float> nn_dists(1);

	tree_->nearestKSearch(pcl::PointXYZ(0, 0, 0), 1, nn_indices, nn_dists);

	
	//printf("The closest point of (0, 0, 0) is: (%f, %f, %f)", cloud->points[nn_indices[0]].x, cloud->points[nn_indices[0]].y, cloud->points[nn_indices[0]].z);

	return pcl::PointXYZ(cloud->points[nn_indices[0]].x, cloud->points[nn_indices[0]].y, cloud->points[nn_indices[0]].z);
}

/**
 * @brief Extract the cluster containing the closest point in the point cloud
 * @param cloud
 * @return A point cloud representing the cluster of interest
 * @note Same algorithm than getBiggestCluster() until the cluster extraction step.
 * Instead of returning biggest cluster, we return the closest one (to the origin).
 */
PointCloudT::Ptr Utils::getClosestCluster(const PointCloudT::ConstPtr &cloud) {

	PointCloudT::Ptr cloud_f(new PointCloudT);
	
	PointCloudT::Ptr cloud_filtered(new PointCloudT);
	pcl::copyPointCloud(*cloud, *cloud_filtered);

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	PointCloudT::Ptr cloud_plane(new PointCloudT());
	pcl::PCDWriter writer;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	// Segment the largest planar component from the remaining cloud
	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0)
	{
		std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	}

	// Extract the planar inliers from the input cloud
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	extract.setNegative(false);

	// Get the points associated with the planar surface
	extract.filter(*cloud_plane);
	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

	// Remove the planar inliers, extract the rest
	extract.setNegative(true);
	extract.filter(*cloud_f);
	*cloud_filtered = *cloud_f;

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);

	// Get the initial closest point
	pcl::PointXYZ initClosest = Utils::getClosestPoint(cloud);

	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		PointCloudT::Ptr cloud_cluster (new PointCloudT);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		if (Utils::getClosestPoint(cloud_cluster).getVector3fMap() == initClosest.getVector3fMap()) {
            // If the closest point is in this cluster, return it.
			return cloud_cluster;
		}

		j++;
	}

	return cloud_filtered;	
}

/**
 * @brief Filter the given cloud by removing points outside the center region
 * @param cloud: input cloud
 * @return filtered cloud
 * @note Uses ConditionalRemover and potentially RadiusOutlierRemoval filters.
 */
PointCloudT::Ptr Utils::filterCloud(const PointCloudT::ConstPtr &cloud) {

	PointCloudT::Ptr cloud_filtered(new PointCloudT);

    // 1st filtering : remove points that aren't within specified range (~center of cloud)
    // build the condition ( -0.15 < x < 0.15 && -0.05 < y < 0.25 )
    pcl::ConditionAnd<PointT>::Ptr range_cond (new pcl::ConditionAnd<PointT> ());
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new 
        pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::GT, -0.15)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("x", pcl::ComparisonOps::LT, 0.15)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::GT, -0.05)));
    range_cond->addComparison (pcl::FieldComparison<PointT>::ConstPtr (new
        pcl::FieldComparison<PointT> ("y", pcl::ComparisonOps::LT, 0.25)));

    // then build the filter using the condition
    pcl::ConditionalRemoval<PointT> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);

    // apply filter
    condrem.filter (*cloud_filtered);

    // 2nd potential filtering : remove points without a certain number of neighbors
    /*
    pcl::RadiusOutlierRemoval<PointT> outrem;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.05);
    outrem.setMinNeighborsInRadius(300);
    // apply filter
    outrem.filter(*cloud_filtered);
    */

	return cloud_filtered;
}


