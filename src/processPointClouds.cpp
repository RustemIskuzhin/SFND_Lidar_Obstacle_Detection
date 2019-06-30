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

    std::cout << "Input cloud has " << cloud->points.size()<< " points" << std::endl;
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT> ());
  
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloudFiltered);
    
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT> ());
  
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);
  
    std::vector<int> indices;
  
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f ( -1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f ( 2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);
  
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point : indices)
      inliers->indices.push_back(point);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudRegion);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds and filtered cloud has " << cloudRegion->points.size()<< " points" << std::endl;


    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr groundCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT> ());

    for (int index : inliers->indices){
        groundCloud->points.push_back(cloud->points[index]);
    }
  
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacleCloud);
  
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud,groundCloud);
    return segResult;
}

template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::vector<int> inliersResult;
    srand(time(NULL));
	
	// TODO: Fill in this function
	while(maxIterations--){
		//select 3 points randomly
        std::vector<int> inliers;

        while (inliers.size() < 3)
            inliers.push_back(rand()%cloud->points.size());

        auto itr = inliers.begin();
		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
		x1=cloud->points[*itr].x;
		y1=cloud->points[*itr].y;
		z1=cloud->points[*itr].z;
		itr++;
		x2=cloud->points[*itr].x;
		y2=cloud->points[*itr].y;
		z2=cloud->points[*itr].z;
		itr++;
		x3=cloud->points[*itr].x;
		y3=cloud->points[*itr].y;
		z3=cloud->points[*itr].z;
		//build the plane
		float a = ((y2-y1)*(z3-z1)-(z2-z1)*(y3-y1));
		float b = ((z2-z1)*(x3-x1)-(x2-x1)*(z3-z1));
		float c = ((x2-x1)*(y3-y1)-(y2-y1)*(x3-x1));
		float d = (-(a*x1+b*y1+c*z1));
      
	    float vect = sqrt(a*a+b*b+c*c);

      //number of points within distanceTol to line
		for(int id=0; id < cloud->points.size();id++)
        {
			PointT pt = cloud->points[id];
			float x4 = pt.x;
			float y4 = pt.y;
			float z4 = pt.z;
			float dist = fabs(a*x4+b*y4+c*z4+d)/vect;
           
           if (dist <= distanceTol)
                inliers.push_back(id);
	   }

        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
	}
    
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point : inliersResult)
		inliers->indices.push_back(point);
    
    return inliers;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
 
    
    /*
    pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0){
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    */
  
    inliers = Ransac(cloud,maxIterations,distanceThreshold);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration<double, std::milli> (endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

//cluster obstacles
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	if(processed[indice]) 
        return;
    processed[indice]=true;
	cluster.push_back(indice);
	std::vector<int> nearest= tree->search(points[indice],distanceTol);
	for(int id : nearest)
       clusterHelper(id,points,cluster,processed,tree,distanceTol);
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol,int minSize,int maxSize)
{
	std::vector<std::vector<int>> clusters_indices;
	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	std::vector<bool> processed(points.size(),false);

	for(int i = 0; i < points.size(); i++){
		if(processed[i])
			continue;
		std::vector<int> cluster;
		clusterHelper(i,points,cluster,processed,tree,distanceTol);
      
        uint cluster_size = cluster.size();
        if(cluster_size >= minSize && cluster_size <= maxSize)
		    clusters_indices.push_back(cluster);
	}
  

  
	for(std::vector<int> cluster : clusters_indices){
		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new typename pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = cluster.begin (); pit != cluster.end (); ++pit){
            clusterCloud->points.push_back (cloud->points[*pit]);
        }
		clusterCloud->width = clusterCloud->points.size ();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
		clusters.push_back(clusterCloud);
	}
	return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<float>> points;
    for(PointT &p : cloud->points){
		points.push_back({p.x, p.y, p.z});
	}
    KdTree* tree = new KdTree;
    tree->dim = 3;
  	int it = 0;
    for (int i = 0; i < points.size(); i++){
        tree->insert(points[i], i);
    }

	 
	clusters = euclideanCluster(points, cloud, tree, clusterTolerance,minSize, maxSize);
   
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  /*
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
  
	for (pcl::PointIndices getIndices : cluster_indices){
      typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
      for (int index : getIndices.indices)
        cloudCluster->points.push_back (cloud->points[index]);
      cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;
      
      clusters.push_back (cloudCluster);
    }
    */
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters for " << std::endl;
  
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