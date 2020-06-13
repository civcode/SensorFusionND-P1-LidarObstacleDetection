/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <pcl/filters/random_sample.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

// bool sortcol(const std::vector<float> &v1, const std::vector<float> &v2) {
// 	return v1[0] > v2[0];
// }

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	std::vector<std::vector<float> > params; //structure: [[A1, B1, C1, inlier_cnt], ...]
	//std::vector<std::unordered_set
	

	// For max iterations 
	for (int i=0; i<maxIterations; i++) {

		// Randomly sample subset and fit line
		//pcl::RandomSample<pcl::PointXYZ> rand; // {new pcl::RandomSample<pcl::PointXYZ>};
		//rand.setInputCloud(cloud);
		//rand.setSample(2);
			
		pcl::PointCloud<pcl::PointXYZ> samples;
		//rand.setSeed(i);
		//rand.filter(samples);

		std::unordered_set<int> random_samples;

		while (random_samples.size() < 2) {
			random_samples.insert(rand() % (cloud->points.size()));
		}

		auto it = random_samples.begin();
		samples.points.push_back(cloud->points[*it]);
		samples.points.push_back(cloud->points[*(++it)]);


		for (auto point : samples) {
			std::cout << "sample: [" << point.x << ", " << point.y << "]" << std::endl;
		}
		

		if (samples.size() < 2) {
			std::cout << "Not enough samples." << std::endl;
		}

		// std::unordered_set<int> inliers;

		// while (inliers.size() , 2) {
		// 	inliers.insert(rand() % cloud->points.size());
		// }

		// Fit line model
		float A = samples[0].y - samples[1].y;
		float B = samples[1].x - samples[0].x;
		float C = samples[0].x * samples[1].y - samples[1].x * samples[0].y;
		
		// float A = samples[0].y - samples[1].y;
		// float B = samples[1].x - samples[0].x;
		// float C = samples[0].x * samples[1].y - samples[1].x * samples[0].y;

		std::vector<float> model;
		model.push_back(A);
		model.push_back(B);
		model.push_back(C);
		model.push_back(0);


		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		int inlier_cnt = 0;
		int idx = 0;
		for (auto point : *cloud) {
			float d = fabs(A*point.x + B*point.y + C) / sqrt(pow(A,2) + pow(B, 2));
			//std::cout << "d = " << d << std::endl;
			if (d <= distanceTol) {
				//std::cout << "smaller" << std::endl;
				model.push_back(idx);
				inlier_cnt++;
			}
			idx++;
		}
		
		//model.push_back(inlier_cnt);
		model[3] = inlier_cnt;

		params.push_back(model);

	}

	// for (int row=0; row<params.size(); row++) {
	// 	for (int col=0; col<params[0].size(); col++) {
	// 		std::cout << params[row][col] << " ";
	// 	}
	// 	std::cout << std::endl;
	// }

	auto printData = [](std::vector<std::vector<float> > &params) {
		for (auto row : params) {
			for (auto item : row) {
				std::cout << item << " ";
			}
			std::cout << std::endl;
		}
	};
	printData(params);

	std::cout << std::endl;
	std::sort(params.begin(), params.end(), [](const std::vector<float> &v1, const std::vector<float> &v2) -> bool {
		return v1[3] > v2[3]; 
		} 
	);
	//std::sort(params.begin(), params.end(), sortcol);
	printData(params);


	// Return indicies of inliers from fitted line with most inliers
	for (auto it = params[0].begin()+4; it != params[0].end(); ++it) {
		inliersResult.insert(static_cast<int>(*it));
	}
	
	return inliersResult;

}


std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// Fill in this function
	std::vector<std::vector<float> > params; //structure: [[A1, B1, C1, inlier_cnt], ...]

	// For max iterations 
	while (maxIterations--) {
			
		std::unordered_set<int> inliers;

		while (inliers.size() < 2) {
			inliers.insert(rand() % (cloud->points.size()));
		}

		// Fit line model
		auto it = inliers.begin();
		float x1 = cloud->points[*it].x;
		float y1 = cloud->points[*it].y;
		it++;
		float x2 = cloud->points[*it].x;
		float y2 = cloud->points[*it].y;
		
		float A = y1 - y2;
		float B = x2 - x1;
		float C = (x1 * y2) - (x2 * y1);
		
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		int inlier_cnt = 0;
		for (int idx=0; idx<cloud->points.size(); idx++) {
			float x = cloud->points[idx].x;
			float y = cloud->points[idx].y;
			float d = fabs(A*x + B*y + C) / sqrt(pow(A,2) + pow(B, 2));

			if (d <= distanceTol) {
				//std::cout << "smaller" << std::endl;
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

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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
				//std::cout << "smaller" << std::endl;
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

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
	//std::unordered_set<int> inliers = RansacLine(cloud, 50, 0.5);
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
