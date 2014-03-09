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

  FILE * unfiltered_pc;
  fileCounter = 0;
  
  while(1) {
    // Iterate while file doesn't exists
    while ( unfiltered_pc_file != NULL ) {
      pFile = fopen("unfiltered_ply_" + to_string(fileCounter) +".ply","r");
    }
    fclose(unfiltered_pc_file);
    unfiltered_pc_file = NULL;
    
    pcl::io::loadPLYFile("unfiltered_ply_" + to_string(fileCounter) +".ply", *cloud );
    fileCounter++;

    cout << "Starting the filter\n";
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    pcl::io::savePLYFile("filtered_ply_" + to_string(fileCounter) + ".ply", *cloud_filtered);
    cout << "Saved the file";

  }
  return (0);
}