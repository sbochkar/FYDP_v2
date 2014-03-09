/* This function is the pipeline implementation of the filter
*/

#include <iostream>
#include <fstream>
#include <pcl/point_types.h>

#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;
int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  ofstream unfiltered_pc;
  
  pcl::io::loadPLYFile( "./Debug/turd.ply", *cloud );
  
  cout << "Starting the filter\n";
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);
  
  pcl::io::savePLYFile("./Debug/test_pcd.ply", *cloud_filtered);
  cout << "Saved the file";


  string in;
  cin>>in;
  return (0);
}