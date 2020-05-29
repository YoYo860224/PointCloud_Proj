#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <string>

int main(int argc, char* argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./ShowPCD file.pcd" << std::endl << std::endl;
        std::cout << "Positional arguments: " << std::endl;
        std::cout << "file.pcd  \t string " << std::endl;
        return 0;
    }
  
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

    reader.read(argv[1], *cloud);

    pcl::visualization::CloudViewer viewer(argv[1]);
    viewer.showCloud(cloud);
    while (!viewer.wasStopped())
    {
        /* code */
    }

    return 0;
}
