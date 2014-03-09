#include <iostream>
#include <pcl/point_types.h>

#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

//convenient typedefs
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

void printUsage(const char* programName)
{
    cout << "Usage: " << programName << " [options]. Input ply files should be in same dir as exe with names such as plyfile0, plyfile1..etc"
         << endl
         << endl
         << "Options:\n"
         << endl
         << "\t-f				   specify input base file name (without index numbers).\n"
		 << "\t-n				   how many files there are.\n";
}

void FilterBackgroundFromPointCloud( PointCloud& iInputCloud,PointCloud& iOutputCloud, int min_dist_thresh )
{
	//determine size of new cloud (# points within filter threshold)
	//can probably get rid of this step to be more performant...but deadlines approaching so fuck it
	int numPointsInRange = 0;
	double tempDistance = 0;
	for (size_t i = 0; i < iInputCloud.points.size (); ++i)
	{
		if ( ! _isnan(iInputCloud.points[i].x ) )
		{
			tempDistance = sqrt( pow(iInputCloud.points[i].x,2) + 
								 pow(iInputCloud.points[i].y, 2) + 
								 pow(iInputCloud.points[i].z, 2) );
			if ( tempDistance < min_dist_thresh )
				numPointsInRange ++;
		}
	}

	//cout << "Number points < " << min_dist_thresh <<"m : " << numPointsInRange << endl;
		
	iOutputCloud.width    = numPointsInRange;
	iOutputCloud.height   = 1;
	iOutputCloud.is_dense = false;
	iOutputCloud.points.resize (iOutputCloud.width * iOutputCloud.height);
	
	int counter = 0;
	for (size_t i = 0; i < iInputCloud.points.size (); ++i)
	{
		if ( ! _isnan(iInputCloud.points[i].x ) )
		{
			tempDistance = sqrt( pow(iInputCloud.points[i].x,2) + 
								pow(iInputCloud.points[i].y, 2) + 
								pow(iInputCloud.points[i].z, 2) );
			if ( tempDistance < min_dist_thresh )
			{
				iOutputCloud.points[counter].x = iInputCloud.points[i].x;
				iOutputCloud.points[counter].y = iInputCloud.points[i].y;
				iOutputCloud.points[counter].z = iInputCloud.points[i].z;
				counter++;
			}
		}
	}
}

void CopyPointCloud(PointCloud& PointCloudTarget, PointCloud& PointCloudData)
{
	PointCloudTarget.width    = PointCloudData.width;
	PointCloudTarget.height   = 1;
	PointCloudTarget.is_dense = false;
	PointCloudTarget.points.resize (PointCloudTarget.width * PointCloudTarget.height);
	
	for (size_t i = 0; i < PointCloudData.points.size (); ++i)
	{
		PointCloudTarget.points[i].x = PointCloudData.points[i].x;
		PointCloudTarget.points[i].y = PointCloudData.points[i].y;
		PointCloudTarget.points[i].z = PointCloudData.points[i].z;
	}
}

void TranslatePointCloud( PointCloud& iPointCloud, float x, float y, float z )
{
	Eigen::Matrix4f translateMatrix;
	translateMatrix <<	1,	0,	0,	x,
						0,	1,	0,	y,
						0,	0,	1,	z,
						0,	0,	0,	1;
	pcl::transformPointCloud( iPointCloud, iPointCloud, translateMatrix);
}

void RotatePointCloud( PointCloud& iPointCloud, float iAngle )
{
	//rotates around centre, should rotate after applying translation
	Eigen::Matrix4f rotateMatrix;
	rotateMatrix <<	cos(iAngle),		0,	sin(iAngle),	0,
					0,					1,	0,				0,
					(-1)*sin(iAngle),	0,	cos(iAngle),	0,
					0,					0,	0,				1;
	pcl::transformPointCloud( iPointCloud, iPointCloud, rotateMatrix);
}

int main (int argc, char** argv)
{
	int numFiles = 0;
	string baseFileName = "";

	//Print Help
    if(pcl::console::find_argument(argc, argv, "-h") >= 0)
    {
        printUsage(argv[0]);
        return 0;
    }
	if(pcl::console::find_argument(argc, argv, "-f") >= 0)
	{
		baseFileName = argv[2];
	}
	if(pcl::console::find_argument(argc, argv, "-n") >= 0)
	{
		numFiles = atoi(argv[4]);
	}

	//Create and load point cloud objects
	cout << "********Loading Point Clouds:\n";
	PointCloudPtr * inPointClouds = new PointCloudPtr[numFiles];
	PointCloudPtr tempPC(new PointCloud); //intermediate temp pc for filters
	for(int i = 0; i < numFiles; i++)
	{
		inPointClouds[i] = PointCloudPtr(new PointCloud);
		stringstream ss;
		ss << baseFileName << i << ".ply";
		string fileName = ss.str();
		pcl::io::loadPLYFile( ss.str(), *inPointClouds[i] );
	}
	cout<<"Done Loading Point Cloud Files.\n";

	//Apply basic thresholding filter to remove background
	cout << "\n********APPLYING THRESHOLDING FILTER:\n";
	int min_dist = 800;
	for(int i = 0; i < numFiles; i++)
	{
		FilterBackgroundFromPointCloud(*inPointClouds[i], *tempPC, min_dist);
		CopyPointCloud(*inPointClouds[i], *tempPC);
	}
	cout<<"Done Thresholding Filter.\n";

	//Filter outliers from point clouds
	cout << "\n********APPLYING OUTLIER FILTER:\n";
	//create filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	for (int i = 0; i < numFiles; i++)
	{
		cout << "Filtering " << baseFileName << i << "..." << endl;
		sor.setInputCloud (inPointClouds[i]);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		sor.filter (*tempPC);
		CopyPointCloud(*inPointClouds[i], *tempPC);
		//***debug
		//stringstream ss;
		//ss << "filtered" << i << ".ply";
		//string fileName = ss.str();
		//pcl::io::savePLYFile(fileName, *inPointClouds[i]);
		//***
	}
	cout<<"Done Outlier Filtering.\n";
	//***debug


	//***debug
	//cout << "\n********Loading Point Clouds:\n";
	//PointCloudPtr * inPointClouds = new PointCloudPtr[numFiles];
	//PointCloudPtr tempPC(new PointCloud); //intermediate temp pc for filters
	//
	//for(int i = 0; i < numFiles; i++)
	//{
	//	inPointClouds[i] = PointCloudPtr(new PointCloud);
	//	stringstream ss;
	//	ss << "./Debug/" << "filtered" << i << ".ply";
	//	string fileName = ss.str();
	//	pcl::io::loadPLYFile( ss.str(), *inPointClouds[i] );
	//}
	//cout<<"Done Loading Point Cloud Files.\n";
	//***
	//Apply ICP to align the point clouds
	cout << "\n********APPLYING ICP TO ALIGN THE POINT CLOUDS:\n";
	//Create ICP Object
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(5);
	icp.setTransformationEpsilon (0.000001);
	icp.setRANSACIterations(0);
	icp.setMaximumIterations(100);
	//Create Grid object for downsampling point clouds
	pcl::VoxelGrid<Point> grid;
	grid.setLeafSize (2, 2, 2);
	//Create temp intermediate downsapmled point clouds
	PointCloudPtr srcDS (new PointCloud);
	PointCloudPtr tgtDS (new PointCloud);

	//initialize target to be the first point cloud
	PointCloudPtr targetCloud(new PointCloud);
	CopyPointCloud(*targetCloud, *inPointClouds[0]);
	//Apply ICP on all frames
	for(int i = 1; i < numFiles; i++)
	{
		cout << "Aligning " << baseFileName << i << "..." << endl;
		//pcl::io::savePLYFile( "./cloud_input.ply", *inPointClouds[i] );
		//Obtain initial estimate on transformation of input frame (dependent on turntable angle)
		float tx = -52.4;
		float ty = 0;
		float tz = 781.841;
		float rAngle = (360.0f/numFiles)*i;

		TranslatePointCloud(*inPointClouds[i],tx,ty,tz);
		RotatePointCloud(*inPointClouds[i],rAngle);
		TranslatePointCloud(*inPointClouds[i],-tx,ty,-tz);

		//debug : save the transformed
		stringstream sst;
		sst << "transformed" << i << ".ply";
		string fileName = sst.str();
		pcl::io::savePLYFile(fileName, *inPointClouds[i]);

		//Downsample the point clouds
		grid.setInputCloud (targetCloud);
		grid.filter (*tgtDS);
		grid.setInputCloud (inPointClouds[i]);
		grid.filter (*srcDS);

		icp.setInputCloud(srcDS);
		icp.setInputTarget(tgtDS);
		//perform alignment
		icp.align(*tempPC); //new point cloud of input cloud that's been transformed
		//***debug
		//std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
		//std::cout << icp.getFinalTransformation() << std::endl;
		//pcl::io::savePLYFile("./Debug/cloud_out.ply", *tempPC);
		//***

		//Transform point cloud of non-sampled input cloud and concatenate it to the non-sampled target cloud
		Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, sourceToTarget;
		Ti = icp.getFinalTransformation();
		sourceToTarget = Ti;//Ti.inverse();
		pcl::transformPointCloud(*inPointClouds[i], *tempPC, sourceToTarget);
		//pcl::io::savePLYFile( "./Debug/cloud_target.ply", *targetCloud );
		*targetCloud += *tempPC;
			
		
		//pcl::io::savePLYFile( "./Debug/cloud_transformed_aftericp.ply", *tempPC );
		//pcl::io::savePLYFile( "./Debug/cloud_concatenated.ply", *targetCloud );
		//	int x = 0;
	}
	pcl::io::savePLYFile( "./cloud_final.ply", *targetCloud );
	return 0;
	//cout << "Loading point cloud in and target\n";
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::io::loadPLYFile( "./Debug/shoe2.ply", *cloud_target );
	//pcl::io::loadPLYFile( "./Debug/cloud_target_2.ply", *cloud_in );

	////Downsample point clouds first

	//PointCloud::Ptr src (new PointCloud);
	//PointCloud::Ptr tgt (new PointCloud);
	//pcl::VoxelGrid<PointT> grid;

	//grid.setLeafSize (4, 4, 4);
	//grid.setInputCloud (cloud_in);
	//grid.filter (*src);

	//grid.setInputCloud (cloud_target);
	//grid.filter (*tgt);

	//cloud_in = src;
	//cloud_target = tgt;

	//pcl::io::savePLYFile("./Debug/cloud_in_downsampled.ply", *cloud_in);
	//pcl::io::savePLYFile("./Debug/shoe_undersampled.ply", *cloud_target);

	////Create ICP Object
	//cout<<"Starting ICP on point clouds\n";
	//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//icp.setInputCloud(cloud_in);
	//icp.setInputTarget(cloud_target);
	//icp.setMaxCorrespondenceDistance(1);
	//icp.setTransformationEpsilon (0.00001);
	//icp.setRANSACIterations(0);
	//icp.setMaximumIterations(100);
	//icp.align(*cloud_out);
	//std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	//std::cout << icp.getFinalTransformation() << std::endl;
	//pcl::io::savePLYFile("./Debug/cloud_out.ply", *cloud_out);

	////Transform point cloud
	////pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
	////Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, sourceToTarget;
	////sourceToTarget = icp.getFinalTransformation();
	////pcl::transformPointCloud(*cloud_target, *cloud_transformed, sourceToTarget);
	//
	////cout<< sourceToTarget << std::endl;

	////pcl::io::savePLYFile("./Debug/cloud_icp.ply", *cloud_transformed);

	//cout<<"Finished and saved file";
	//string in;
	//cin>>in;
	//return (0);




}