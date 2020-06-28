// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped(new pcl::PointCloud<PointT>());

    // Create filter object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    // Apply CropBox filter
    pcl::CropBox<PointT> boxFilter;
    boxFilter.setMin(minPoint);
    boxFilter.setMax(maxPoint);
    boxFilter.setInputCloud(cloud_filtered);
    boxFilter.filter(*cloud_cropped);

    // Remove points detected on the ego car
    //typename pcl::PointCloud<PointT>::Ptr car_region(new pcl::PointCloud<PointT>());
    std::vector<int> indices;
    //pcl::CropBox<PointT> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1.0, 1.0));
    boxFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1.0));
    boxFilter.setInputCloud(cloud_cropped);
    boxFilter.filter(indices);

    pcl::PointIndices::Ptr car_points {new pcl::PointIndices};
    for (int idx : indices) {
        car_points->indices.push_back(idx);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_cropped);
    extract.setIndices(car_points);
    extract.setNegative(true);
    extract.filter(*cloud_cropped);



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_cropped;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacles {new pcl::PointCloud<PointT>()};
    typename pcl::PointCloud<PointT>::Ptr road {new pcl::PointCloud<PointT>()};

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*road);
    extract.setNegative(true);
    extract.filter(*obstacles);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coeff {new pcl::ModelCoefficients}; 

    // Segmentation Object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setMaxIterations(maxIterations);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coeff);

    if (inliers->indices.size() == 0) {
        std::cout << "Could not estimate a plane model for the given data." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree {new pcl::search::KdTree<PointT>()};
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    //std::cout << "cluster_indices.size() = " << cluster_indices.size() << std::endl;
    // cluster_indices is a vector of PointIndices which contain the vector indices
    int cluster_cnt = 0;
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        //std::cout << "cluster #" << cluster_cnt << std::endl;
        typename pcl::PointCloud<PointT>::Ptr cluster {new pcl::PointCloud<PointT>()};
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
            //std::cout << *pit << " ";
            cluster->points.push_back(cloud->points[*pit]);
        }
        cluster->width = cluster->points.size();
        cluster->height =1;
        cluster->is_dense = true;
        clusters.push_back(cluster);
        cluster_cnt++;
        //std::cout << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}


// Own implementations for Lidar Obstacle Detction Project

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::mySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // Get inliers from RANSAC plane fitting
    std::unordered_set<int> inlier_idices = myRansacPlane(cloud, maxIterations, distanceThreshold);

     if (inlier_idices.size() == 0) {
        std::cout << "Could not estimate a plane model for the given data." << std::endl;
    }

    // Copy inliers to pcl object
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int idx : inlier_idices) {
        inliers->indices.push_back(idx);

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    return segResult;
}


template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::myRansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// Fill in this function
	std::vector<std::vector<float> > params; //structure: [[A1, B1, C1, inlier_cnt], ...]

	// For max iterations 
	while (maxIterations--) {
			
		std::unordered_set<int> inliers;

		while (inliers.size() < 3) {
			inliers.insert(rand() % (cloud->points.size()));
		}

		// Fit line model
		auto it = inliers.begin();
		float x1 = cloud->points[*it].x;
		float y1 = cloud->points[*it].y;
		float z1 = cloud->points[*it].z;
		it++;
		float x2 = cloud->points[*it].x;
		float y2 = cloud->points[*it].y;
		float z2 = cloud->points[*it].z;
		it++;
		float x3 = cloud->points[*it].x;
		float y3 = cloud->points[*it].y;
		float z3 = cloud->points[*it].z;

		// parameters of the cross product 
		float cp_x = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1); 
		float cp_y = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);  
		float cp_z = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		
		float A = cp_x;
		float B = cp_y;
		float C = cp_z;
		float D = -(cp_x*x1 + cp_y*y1 + cp_z*z1);
		
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		int inlier_cnt = 0;
		for (int idx=0; idx<cloud->points.size(); idx++) {
			float x = cloud->points[idx].x;
			float y = cloud->points[idx].y;
			float z = cloud->points[idx].z;

			float d = fabs(A*x + B*y + C*z + D) / sqrt(pow(A,2) + pow(B, 2) + pow(C,2));

			if (d <= distanceTol) {
				inliers.insert(idx);
				inlier_cnt++;
			}
		}

		if (inliers.size() > inliersResult.size()) {
			inliersResult = inliers;
		}
	}
	
	return inliersResult;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::myClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize) {

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;   

    // Build KD Tree
	KdTree* mytree = new KdTree;
  
    // Iterate over point cloud and enter points to the tree
    std::vector<std::vector<float>> points;
    int id = 0;
    for (auto it = cloud->begin(); it != cloud->end(); ++it) {
        std::vector<float> point;
        point.push_back(it->x);
        point.push_back(it->y);
        point.push_back(it->z);
        points.push_back(point);
        //std::cout << "points: " << it->x << ", " << it->y << ", " << it->z << std::endl;
        mytree->insert(point, id++);
    }

    // Call Euclidean clustering
  	std::vector<std::vector<int>> myclusters = myEuclideanCluster(points, mytree, clusterTolerance, minSize, maxSize);

    // Create vector of point clouds; each point cloud is a cluster 
  	int clusterId = 0;
  	for(std::vector<int> cluster : myclusters) {

        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for(int idx : cluster) {
            // PointT p;
            // p.x = points[idx][0];
            // p.y = points[idx][1];
            // p.z = points[idx][2];
            //clusterCloud->points.push_back(p);
            PointT& p = cloud->points[idx];
            clusterCloud->points.push_back(cloud->points[idx]);
        }
        clusterCloud->width = cluster.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
        clusters.push_back(clusterCloud);

        clusterId++;
  	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    
    return clusters;

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::myEuclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{

	// Find a list of indices for each cluster
	std::vector<std::vector<int>> clusters;
	std::vector<bool> is_processed(points.size(), false);

	std::cout << "number of points: " << points.size() << std::endl;
	for (int id=0; id<points.size(); id++) {
		if (!is_processed[id]) {
			std::vector<int> cluster;
			proximity(id, points, cluster, is_processed, tree, distanceTol);
            if (cluster.size() >= minSize && cluster.size() <= maxSize)
			    clusters.push_back(cluster);
		}
	}

	return clusters;

}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int id, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& is_processed, KdTree* tree, float distanceTol) {

	is_processed[id] = true;

	cluster.push_back(id);

	std::vector<int> neighbours = tree->search(points[id], distanceTol);

	for (auto id : neighbours) {
		if (!is_processed[id]) {
			proximity(id, points, cluster, is_processed, tree, distanceTol);
		}
	}
}


