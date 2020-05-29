#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <string>

pcl::PointCloud<pcl::PointXYZI>::Ptr VoxelFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud, float voxelSize)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr getPoint(new pcl::PointCloud<pcl::PointXYZI>);
    *getPoint = *pointcloud;

    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setLeafSize(voxelSize, voxelSize, voxelSize);    // 設置 voxel grid 小方框參數
    vg.setInputCloud(getPoint);                         // 輸入 cloud
    vg.filter(*getPoint);                               // 輸出到 cloud_filtered

    return getPoint;
}

int main(int argc, char* argv[])
{
    if (argc != 4)
    {
        std::cout << "Usage: ./VoxelPCD inFile.pcd outFile.pcd voxelSize " << std::endl << std::endl;
        std::cout << "Positional arguments: " << std::endl;
        std::cout << "infile.pcd  \t string " << std::endl;
        std::cout << "outFile.pcd \t string " << std::endl;
        std::cout << "voxelSize   \t float " << std::endl;
        return 0;
    }
    
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZI>);

    reader.read(argv[1], *cloud);
    cloud_ds = VoxelFilter(cloud, atof(argv[3]));

    std::cout << cloud->size() << " To " << cloud_ds->size()<< std::endl;
    pcl::io::savePCDFileBinary(argv[2], *cloud_ds);

    pcl::visualization::CloudViewer viewer(argv[2]);
    viewer.showCloud(cloud_ds);
    while (!viewer.wasStopped())
    {
        /* code */
    }

    return 0;
}
