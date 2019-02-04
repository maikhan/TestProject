
// c++ standard library
#include <fstream>
#include <vector>
#include <ctime> // for 'strftime'
#include <exception>
#include<cmath>
#include <iostream>


// boost library
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/program_options/parsers.hpp>
namespace po = boost::program_options;

// point cloud library
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/projection_matrix.h>
#include <pcl/filters/radius_outlier_removal.h>

//STANN
#include<dpoint.hpp>
#include<test.hpp>
#include<sfcnn.hpp>


using namespace std;

//typedef reviver::dpoint<double, 3> Point;


int main()
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	
	int count;

  cout<<"current_path:"<<boost::filesystem::current_path().c_str()<<endl;

  if (pcl::io::loadPLYFile<pcl::PointXYZRGB> ("/Users/maik/Documents/PhD/MY_GRAND_PROJECT/longdress_vox10_1300.ply", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file ply file \n");
    return (-1);
  }
 /*uint32_t rgb = *reinterpret_cast<int*>(&cloud->points.rgb);
uint8_t r = (rgb >> 16) & 0x0000ff;
uint8_t g = (rgb >> 8)  & 0x0000ff;
uint8_t b = (rgb)       & 0x0000ff;*/

  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from ply file  with the following fields: "
            << std::endl;
  for (size_t i = 0; i < 200 ; ++i)
    std::cout << "    " << cloud->points[i].x
              << " "    << cloud->points[i].y
              << " "    << cloud->points[i].z 
              << " "    <<  cloud->points[i].r 
              << " "    <<  cloud->points[i].g 
              << " "    <<  cloud->points[i].b << std::endl;

              // apply stann on cloud now
              
             /* count=sizeof(cloud);
              cout<<"Count:"<<count<<endl;
              vector<long unsigned int> answer;

              sfcnn<cloud->points, 3, double> NN(&cloud->points[0], count);
              NN.ksearch(cloud->points[0], count, answer);*/


 return 0; 

}


