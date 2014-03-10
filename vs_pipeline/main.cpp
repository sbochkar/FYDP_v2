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
#define DEBUG

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
	//brute force implementation
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
	//rotates around centre, xz plane. should rotate after applying translation
	Eigen::Matrix4f rotateMatrix;
	rotateMatrix <<	cos(iAngle),		0,	sin(iAngle),	0,
					0,					1,	0,				0,
					(-1)*sin(iAngle),	0,	cos(iAngle),	0,
					0,					0,	0,				1;
	pcl::transformPointCloud( iPointCloud, iPointCloud, rotateMatrix);
}

int main (int argc, char** argv)
{
	// input ply files should have format such as scan0.ply, scan1,ply, scan2.ply...etc
	int numFiles = 0;
	string baseFileName = "";
	string fileName = ""; //mainly for debug

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

//=========================================================================================
//Create and load point cloud objects
//=========================================================================================
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

//=========================================================================================
//Apply basic thresholding filter to remove background
//=========================================================================================
	cout << "\n********APPLYING THRESHOLDING FILTER:\n";
	int min_dist = 800;
	for(int i = 0; i < numFiles; i++)
	{
		FilterBackgroundFromPointCloud(*inPointClouds[i], *tempPC, min_dist);
		CopyPointCloud(*inPointClouds[i], *tempPC);
#ifdef DEBUG
		stringstream ss;
		ss << "backgroud_filtered" << i << ".ply";
		fileName = ss.str();
		pcl::io::savePLYFile(fileName, *inPointClouds[i]);
#endif
	}
	cout<<"Done Thresholding Filter.\n";

//=========================================================================================
//Filter outliers from point clouds
//=========================================================================================
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
#ifdef DEBUG
		stringstream ss;
		ss << "outlier_filtered" << i << ".ply";
		string fileName = ss.str();
		pcl::io::savePLYFile(fileName, *inPointClouds[i]);
#endif
	}
	cout<<"Done Outlier Filtering.\n";

//=========================================================================================
//Transform points clouds with motor angle estimate, move to origin
//=========================================================================================
	for(int i = 0; i < numFiles; i++)
	{
		float tx = -52.4;
		float ty = 0;
		float tz = 781.841;
		float rAngle = (360.0f/numFiles)*i*0.0174532925; //radians
		TranslatePointCloud(*inPointClouds[i],tx,ty,tz);
		RotatePointCloud(*inPointClouds[i],rAngle);

#ifdef DEBUG
		stringstream ss;
		ss << "estimated" << i << ".ply";
		fileName = ss.str();
		pcl::io::savePLYFile(fileName, *inPointClouds[i]);
#endif
	}

//=========================================================================================
//Apply ICP to align the point clouds
//=========================================================================================
	//note: the point clouds should have been transformed to their estimated pose before applying this ICP
	cout << "\n********APPLYING ICP TO ALIGN THE POINT CLOUDS:\n";
	//Create ICP Object
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(10);
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
		//Downsample the point clouds
		grid.setInputCloud (targetCloud);
		grid.filter (*tgtDS);
		grid.setInputCloud (inPointClouds[i]);
		grid.filter (*srcDS);
#ifdef DEBUG
		stringstream ss_ds;
		ss_ds << "downsampled" << i << ".ply";
		fileName = ss_ds.str();
		pcl::io::savePLYFile(fileName, *srcDS);
#endif
		icp.setInputCloud(srcDS);
		icp.setInputTarget(tgtDS);
		//perform alignment
		icp.align(*tempPC); //new point cloud of input cloud that's been transformed
#ifdef DEBUG
		stringstream ss_aligned;
		ss_aligned << "aligned" << i << ".ply";
		fileName = ss_aligned.str();
		pcl::io::savePLYFile(fileName, *tempPC);
#endif

		//Transform point cloud of non-sampled input cloud and concatenate it to the non-sampled target cloud
		Eigen::Matrix4f sourceToTarget = Eigen::Matrix4f::Identity ();
		sourceToTarget = icp.getFinalTransformation();
		pcl::transformPointCloud(*inPointClouds[i], *tempPC, sourceToTarget);
		*targetCloud += *tempPC;
#ifdef DEBUG
		stringstream ss_con;
		ss_con << "concatenated" << i << ".ply";
		fileName = ss_con.str();
		pcl::io::savePLYFile(fileName, *targetCloud);
#endif
	}

	//filter outlier of final cloud
	sor.setInputCloud (targetCloud);
	sor.filter (*tempPC);
	CopyPointCloud(*targetCloud, *tempPC);
	pcl::io::savePLYFile( "./cloud_final.ply", *targetCloud );

//=========================================================================================
//Mesh reconstruction
//=========================================================================================


//=========================================================================================
//Cleanup
//=========================================================================================
	delete inPointClouds, tempPC, tgtDS, srcDS, targetCloud;
	return 0;

}