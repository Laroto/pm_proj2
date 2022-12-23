# pm_proj2
time to colide


Voxel Filter:
void voxFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,         pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered){
  pcl::PCLPointCloud2::Ptr input (new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr filter (new pcl::PCLPointCloud2());
  pcl::toPCLPointCloud2(*cloud,*input);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(input);
  sor.setLeafSize(0.095f,0.095f,0.095f);
  sor.filter(*filter);

  pcl::fromPCLPointCloud2(*filter,*cloud_filtered);
}
