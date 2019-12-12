/////////////////////////////////////////////////////////////////////////////
// Headers
/////////////////////////////////////////////////////////////////////////////
#include <librealsense/rs.hpp>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/geometry.h>
#include <pcl/common/transforms.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h> // TicToc
#include <pcl/conversions.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <fstream>
#include <iostream>
#include <string>

// Window size and frame rate
int const INPUT_WIDTH = 320;
int const INPUT_HEIGHT = 240;
int const FRAMERATE = 60;

bool next_iteration = false;

typedef pcl::PointXYZRGB PointT;

/////////////////////////////////////////////////////////////////////////////
// Called every frame gets the data from streams and displays them using PCL.
/////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(rs::device &_rs_camera) {

  rs::intrinsics depth_intrin = _rs_camera.get_stream_intrinsics(rs::stream::depth);
  rs::intrinsics color_intrin = _rs_camera.get_stream_intrinsics(rs::stream::color);

  rs::extrinsics depth_to_color = _rs_camera.get_extrinsics(rs::stream::depth, rs::stream::color);

  float scale = _rs_camera.get_depth_scale();

  // First we create a new point cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Wait until the device is ready
  _rs_camera.wait_for_frames();

  // Retrieve our images
  const uint16_t *depth_image = (const uint16_t *)_rs_camera.get_frame_data(rs::stream::depth);
  const uint8_t *color_image = (const uint8_t *)_rs_camera.get_frame_data(rs::stream::color);

  for (int dy = 0; dy < depth_intrin.height; ++dy) {
    for (int dx = 0; dx < depth_intrin.width; ++dx) {
      // Retrieve the 16-bit depth value and map it into a depth in meters
      uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
      float depth_in_meters = depth_value * scale;

      // Skip over pixels with a depth value of zero, which is used to indicate
      // no data
      if (depth_value == 0)
        continue;

      // Map from pixel coordinates in the depth image to pixel coordinates in
      // the color image
      rs::float2 depth_pixel = {(float)dx, (float)dy};
      rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
      rs::float3 color_point = depth_to_color.transform(depth_point);
      rs::float2 color_pixel = color_intrin.project(color_point);

      // Create a new point
      pcl::PointXYZRGB p;
      p.x = depth_point.x;
      p.y = depth_point.y;
      p.z = depth_point.z;

      // Use the color from the nearest color pixel, or pure white if this point
      // falls outside the color image
      const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
      if (cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height) {
        p.r = 255;
        p.g = 255;
        p.b = 255;
      } else {
        // indexing into the colour stream can be somewhat convoluted
        p.r = *(color_image + (cy * color_intrin.width + cx) * 3);
        p.g = *(color_image + (cy * color_intrin.width + cx) * 3 + 1);
        p.b = *(color_image + (cy * color_intrin.width + cx) * 3 + 2);
      }
      // Push this new point into the Point Cloud
      cloud->push_back(p);
    }
  }

  return cloud;
}

rs::device &init_realsense_camera(rs::context &rs_ctx, unsigned int index) {

  rs::device &_rs_camera = *rs_ctx.get_device(index);

  printf("-> Using device %d, an %s\n", index, _rs_camera.get_name());
  printf("    Serial number: %s\n", _rs_camera.get_serial());
  printf("    Firmware version: %s\n", _rs_camera.get_firmware_version());
  printf("\n");

  _rs_camera.enable_stream(rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8,
                           FRAMERATE);
  _rs_camera.enable_stream(rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16,
                           FRAMERATE);
  _rs_camera.start();

  return _rs_camera;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> init_pcl_visualizer() {

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("ZR300"));

  viewer->setPosition(0, 0);
  viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0); // Setting background to a dark grey

  viewer->setShowFPS(false);
  viewer->resetCamera();

  double clip0 = 0.106177;
  double clip1 = 106.177;

  double pos0 = -0.562581;
  double pos1 = -0.0597481;
  double pos2 = 3.33506;

  double view0 = 0;
  double view1 = -1;
  double view2 = 0;

  double focal0 = 0;
  double focal1 = 0;
  double focal2 = -2.02641;

  double posx = 65;
  double posy = 24;

  double sizex = 1215;
  double sizey = 1000;

  viewer->setCameraPosition(focal0, focal1, focal2, pos0, pos1, pos2, view0, view1, view2);
  viewer->setCameraClipDistances(clip0, clip1);
  viewer->setPosition(posx, posy);
  viewer->setSize(sizex, sizey);
  // pcl::console::print_info ("\npress [q] to exit!\n");

  return viewer;
}

// convenient structure to handle our pointclouds
struct PCD {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  std::string f_name;

  PCD() : cloud(new pcl::PointCloud<pcl::PointXYZRGB>){};
};

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation<pcl::PointNormal> {
  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

public:
  MyPointRepresentation() {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray(const pcl::PointNormal &p, float *out) const {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
pairAlign(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src,
          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt,
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, Eigen::Matrix4f &final_transform,
          bool downsample = false) {
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr src(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;

  if (downsample) {
    grid.setLeafSize(0.05, 0.05, 0.05);
    grid.setInputCloud(cloud_src);
    grid.filter(*src);

    grid.setInputCloud(cloud_tgt);
    grid.filter(*tgt);
  } else {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  // Compute surface normals and curvature

  pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_src(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::PointCloud<pcl::PointNormal>::Ptr points_with_normals_tgt(
      new pcl::PointCloud<pcl::PointNormal>);

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointNormal> norm_est;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(30);

  norm_est.setInputCloud(src);
  norm_est.compute(*points_with_normals_src);
  pcl::copyPointCloud(*src, *points_with_normals_src);

  norm_est.setInputCloud(tgt);
  norm_est.compute(*points_with_normals_tgt);
  pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x,
  // y, and z
  float alpha[4] = {1.0, 1.0, 1.0, 1.0};
  point_representation.setRescaleValues(alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
  reg.setTransformationEpsilon(1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(0.1);
  // Set the point representation
  reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

  reg.setInputSource(points_with_normals_src);
  reg.setInputTarget(points_with_normals_tgt);

  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
  pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations(2);
  for (int i = 0; i < 30; ++i) {
    PCL_INFO("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource(points_with_normals_src);
    reg.align(*reg_result);

    // accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation() * Ti;

    // if the difference between this transformation and the previous one
    // is smaller than the threshold, refine the process by reducing
    // the maximal correspondence distance
    if (std::abs((reg.getLastIncrementalTransformation() - prev).sum()) <
        reg.getTransformationEpsilon())
      reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

    prev = reg.getLastIncrementalTransformation();

    // visualize current state
    // showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }

  //
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

  // p->removePointCloud ("source");
  // p->removePointCloud ("target");

  // PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
  // PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
  // p->addPointCloud (output, cloud_tgt_h, "target", vp_2);
  // p->addPointCloud (cloud_src, cloud_src_h, "source", vp_2);

  //	PCL_INFO ("Press q to continue the registration.\n");
  // p->spin ();

  // p->removePointCloud ("source");
  //  p->removePointCloud ("target");

  // add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;

  return output;
}

void downsampleCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud) {
  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  // float leaf_size = 0.0075f;
  float leaf_size = 0.0075f;
  vox.setInputCloud(cloud);
  vox.setLeafSize(leaf_size, leaf_size, leaf_size);
  vox.filter(*filtered_cloud);
}

void estimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                     pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, const float radius) {
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(cloud_in);

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(radius);
  ne.compute(*cloud_normals);
}

float getScalingFactor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr source) {
  Eigen::Vector4f pt_min_t;
  Eigen::Vector4f pt_max_t;
  Eigen::Vector4f pt_min_s;
  Eigen::Vector4f pt_max_s;

  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;

  Eigen::Vector4f pt_dt_t;
  Eigen::Vector4f pt_dt_s;

  /*CONVERT XYZRGB TO XYZ*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_xyz(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::copyPointCloud(*target, *target_xyz);
  pcl::copyPointCloud(*source, *source_xyz);

  pcl::getMinMax3D(*target, pt_min_t, pt_max_t);
  // pcl::getMinMax3D(*source,pt_min_s,pt_max_s);

  float d_x(0.0f), d_y(0.0f), d_z(0.0f);
  float d(0.0f);

  pt_dt_t = pt_max_t - pt_min_t;
  pt_dt_t = pt_max_t - pt_min_t;

  d_x = std::max(pt_dt_t[0], pt_dt_s[0]);
  d_y = std::max(pt_dt_t[1], pt_dt_s[1]);
  d_z = std::max(pt_dt_t[2], pt_dt_s[2]);

  d = std::max(d_x, std::max(d_y, d_z));

  return (1 / d);
}

void calculateFeatures(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in,
                       pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals,
                       pcl::PointCloud<pcl::PointXYZI>::ConstPtr keypoints,
                       pcl::PointCloud<pcl::FPFHSignature33>::Ptr features, const float radius) {
  pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  fpfh.setSearchMethod(tree);
  fpfh.setSearchSurface(cloud_in);
  fpfh.setInputNormals(cloud_normals);
  fpfh.setInputCloud(keypoints);
  fpfh.setRadiusSearch(radius);
  fpfh.compute(*features);
}

Eigen::Matrix4f initialAlignment(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target,
                                 pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source,
                                 pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr target_feature,
                                 pcl::PointCloud<pcl::FPFHSignature33>::ConstPtr source_feature,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr aligned_source,
                                 const float d, const int N, const float cd) {

  pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>
      sac_ia;

  sac_ia.setMinSampleDistance(d);
  sac_ia.setMaxCorrespondenceDistance(cd);
  sac_ia.setMaximumIterations(N);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_kp(new pcl::PointCloud<pcl::PointXYZRGB>());
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_temp (new
  // pcl::PointCloud<pcl::PointXYZRGB>());
  /*
    for (pcl::PointCloud<pcl::FPFHSignature33>::const_iterator it =
   source->begin(); it< source->end(); it+=1){
    source_kp->points.push_back(*it);
   }*/

  std::cout << "target size:" << target->points.size() << std::endl;
  std::cout << "source size:" << source->points.size() << std::endl;

  sac_ia.setInputCloud(source);
  sac_ia.setInputTarget(target);

  sac_ia.setSourceFeatures(source_feature);
  sac_ia.setTargetFeatures(target_feature);

  std::cout << "target features size:" << target_feature->points.size() << std::endl;
  std::cout << "source features size:" << source_feature->points.size() << std::endl;

  sac_ia.align(*aligned_source);

  std::cout << "SAC-Fitness-Score : " << sac_ia.getFitnessScore() << std::endl;
  std::cout << "Initial Alignment was successfull!" << std::endl;

  return sac_ia.getFinalTransformation();
}

Eigen::Matrix4f refineAlignment(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr target,
                                pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud, double d,
                                const int N, const float trans_epsilon,
                                const float fitness_epsilon) {
  pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

  icp.setInputCloud(target);
  icp.setInputTarget(source);

  icp.setMaxCorrespondenceDistance(d);
  icp.setMaximumIterations(N);
  icp.setTransformationEpsilon(trans_epsilon);
  icp.setEuclideanFitnessEpsilon(fitness_epsilon);

  icp.align(*final_cloud);

  std::cout << "ICP-Fitness-Score : " << icp.getFitnessScore() << std::endl;
  std::cout << "ICP was successfull!" << std::endl;

  return icp.getFinalTransformation();
}

void XYZtoXYZI(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in,
               pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out) {
  for (int i = 0; i < cloud_in->points.size(); ++i) {
    pcl::PointXYZI intensity_point(1.0f);
    intensity_point.x = cloud_in->points[i].x;
    intensity_point.y = cloud_in->points[i].y;
    intensity_point.z = cloud_in->points[i].z;
    cloud_out->points.push_back(intensity_point);
  }
}

void calculateKeypoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_in,
                        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, const float radius,
                        const float threshold, const bool refine) {
  keypoints->clear();

  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

  pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI> *harris =
      new pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>(
          pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::HARRIS, radius, threshold);

  harris->setInputCloud(cloud_in);
  harris->setNormals(cloud_normals);
  harris->setSearchMethod(tree);
  harris->setRadius(radius);
  harris->setThreshold(threshold);
  harris->setRefine(refine);
  harris->compute(*keypoints);
}

int left_right(const int f, const int lb, const int ub) {
  if (f < lb && f < ub)
    return (1);
  else if (f > lb && f > ub)
    return (-1);
  else
    return (0);
}

pcl::PointCloud<pcl::Normal>::Ptr get_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points) {
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setInputCloud(points);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.03);
  ne.compute(*normals);
  return normals;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr get_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                                                   pcl::PointCloud<pcl::Normal>::Ptr &normals) {
  pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI> *harris_corner_detector =
      new pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>(
          pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::HARRIS, 1e-5, 0.0);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
  harris_corner_detector->setInputCloud(points);
  harris_corner_detector->setNormals(normals);
  harris_corner_detector->setSearchMethod(tree);
  harris_corner_detector->setRadius(1e-5);
  harris_corner_detector->setThreshold(0.0);
  harris_corner_detector->setRefine(false);
  harris_corner_detector->compute(*keypoints);
  return keypoints;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr
get_descriptors(pcl::PointCloud<pcl::PointXYZI>::Ptr &points_xyzi,
                pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints,
                pcl::PointCloud<pcl::Normal>::Ptr &normals) {
  pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh_detector;
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptors(
      new pcl::PointCloud<pcl::FPFHSignature33>());
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
  fpfh_detector.setSearchMethod(tree);
  fpfh_detector.setSearchSurface(points_xyzi);
  fpfh_detector.setInputCloud(keypoints);
  fpfh_detector.setInputNormals(normals);
  fpfh_detector.setRadiusSearch(0.05);
  fpfh_detector.compute(*descriptors);

  return descriptors;
}

int iterations_count(0);
int icp_iterations(100);
// http://www.pcl-users.org/Different-results-by-using-icp-align-and-getfinaltransformation-td4021967.html

/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
  // ----------------------------------------------
  // Setting up ZR300 Camera
  // ----------------------------------------------
  std::cout << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
  std::cout << "***      MAIN PROGRAM     ***" << std::endl;
  std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;

  pcl::console::print_highlight("Loading\n");

  // std::cout << "-> Reading camera ...";
  // rs::log_to_console(rs::log_severity::warn);

  rs::context rs_ctx;

  int num_devices = rs_ctx.get_device_count();
  int xpos = 1.0;
  int ypos = 1.0;
  int fontSize = 13;

  double r = 1.0;
  double g = 1.0;
  double b = 1.0;

  int iterations = 10; // Default number of ICP iterations
  unsigned int cont = 0;

  std::cout << "There are " << num_devices << " connected RealSense devices." << std::endl;

  rs::device &_rs_camera1 = init_realsense_camera(rs_ctx, 0); // ZR300 camera 1
  rs::device &_rs_camera2 = init_realsense_camera(rs_ctx, 1); // ZR300 camera 2

  std::cout << "-> -------------------------------" << std::endl;
  std::cout << "-> Setting visualizer" << std::endl;

  // ----------------------------------------------
  // Setting up PCL Visualizer
  // ----------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = init_pcl_visualizer();

  std::cout << "-> Setting viewport" << std::endl;
  // viewport to visualize camera 1 and 2
  int PORT1 = 0;
  viewer->createViewPort(0.0, 0.75, 0.5, 1.0, PORT1);

  int PORT2 = 0;
  viewer->createViewPort(0.5, 0.75, 1.0, 1.0, PORT2);

  int PORT3 = 0;
  viewer->createViewPort(0.0, 0.5, 0.5, 0.75, PORT3);

  int PORT4 = 0;
  viewer->createViewPort(0.5, 0.5, 1.0, 0.75, PORT4);

  int PORT5 = 0;
  viewer->createViewPort(0.0, 0.0, 1.0, 0.5, PORT5);

  std::cout << "-> Removing all pointclouds" << std::endl;
  viewer->removeAllPointClouds(PORT1);
  viewer->removeAllPointClouds(PORT2);

  std::cout << "-> Setting loop for streaming" << std::endl;
  // std::cout << std::endl;
  // ----------------------------------------------
  // Loop for streaming
  // ----------------------------------------------

  while (!viewer->wasStopped()) {

    if (_rs_camera1.is_streaming() and _rs_camera2.is_streaming()) {

      std::cout << "********************************" << std::endl;
      std::cout << "Iteration No.[" << cont << "] " << std::endl;
      std::cout << "********************************" << std::endl;
      std::cout << std::endl;

      _rs_camera1.wait_for_frames();
      _rs_camera2.wait_for_frames();

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_camera1 = points_to_pcl(_rs_camera1);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_camera2 = points_to_pcl(_rs_camera2);
      std::cout << "-> Points camera 1 before Downsampling: " << points_camera1->points.size()
                << std::endl;
      std::cout << "-> Points camera 2 before Downsampling:" << points_camera2->points.size()
                << std::endl;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_camera1_backup(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_camera2_backup(
          new pcl::PointCloud<pcl::PointXYZRGB>);
      *points_camera1_backup = *points_camera1;
      *points_camera2_backup = *points_camera2;
      // std::cout << "-> Points camera 1 backup [OK]" << std::endl;
      // std::cout << "-> Points camera 2 backup [OK]" << std::endl;
      // std::cout << std::endl;

      // Calculate the scaling factor
      float scaling_factor = getScalingFactor(points_camera1, points_camera2);

      Eigen::Matrix4f scale;
      Eigen::Matrix4f scale_i;
      scale << scaling_factor, 0, 0, 0, 0, scaling_factor, 0, 0, 0, 0, scaling_factor, 0, 0, 0, 0,
          1;

      scale_i = scale.inverse();

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr target(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr source(new pcl::PointCloud<pcl::PointXYZRGB>());

      // Perform scaling at origin clouds
      pcl::transformPointCloud(*points_camera1, *target, scale);
      pcl::transformPointCloud(*points_camera2, *source, scale);
      // Downsample the scaled cloud if it is needed
      if (target->size() > 10000) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_f(new pcl::PointCloud<pcl::PointXYZRGB>());
        downsampleCloud(target, target_f);
        points_camera1->swap(*target_f);
      }

      if (source->size() > 10000) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_f(new pcl::PointCloud<pcl::PointXYZRGB>());
        downsampleCloud(source, source_f);
        points_camera2->swap(*source_f);
      }

      // ----------------------------------------------
      // Normal estimation points camera 1
      // ----------------------------------------------
      pcl::PointCloud<pcl::Normal>::Ptr camera1_normals = get_normals(points_camera1);

      // ----------------------------------------------
      // Normal estimation points camera 2
      // ----------------------------------------------
      pcl::PointCloud<pcl::Normal>::Ptr camera2_normals = get_normals(points_camera2);

      // ----------------------------------------------
      // Keypoints extraction camera 1
      // ----------------------------------------------
      pcl::PointCloud<pcl::PointXYZI>::Ptr camera1_keypoints =
          get_keypoints(points_camera1, camera1_normals);

      // ----------------------------------------------
      // Keypoints extraction camera 2
      // ----------------------------------------------
      pcl::PointCloud<pcl::PointXYZI>::Ptr camera2_keypoints =
          get_keypoints(points_camera2, camera2_normals);

      pcl::PointCloud<pcl::PointXYZI>::Ptr points_camera1_xyzi(
          new pcl::PointCloud<pcl::PointXYZI>());
      pcl::copyPointCloud(*points_camera1, *points_camera1_xyzi);

      pcl::PointCloud<pcl::PointXYZI>::Ptr points_camera2_xyzi(
          new pcl::PointCloud<pcl::PointXYZI>());
      pcl::copyPointCloud(*points_camera2, *points_camera2_xyzi);

      // ----------------------------------------------
      // Descriptors extraction camera 1
      // ----------------------------------------------
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr camera1_descriptors =
          get_descriptors(points_camera1_xyzi, camera1_keypoints, camera1_normals);

      // ----------------------------------------------
      // Descriptors extraction camera 2
      // ----------------------------------------------
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr camera2_descriptors =
          get_descriptors(points_camera2_xyzi, camera2_keypoints, camera2_normals);

      std::cout << " ---------------------------------" << std::endl;
      std::cout << " *******     Camera 1     ******* " << std::endl;
      std::cout << " ---------------------------------" << std::endl;
      std::cout << "-> Points rgb: 	 		" << points_camera1->points.size() << std::endl;
      std::cout << "-> Points xyzi:  		" << points_camera1_xyzi->points.size() << std::endl;
      std::cout << "-> Normal points: 		" << camera1_normals->points.size() << std::endl;
      std::cout << "-> Keypoints:			" << camera1_keypoints->points.size() << std::endl;
      std::cout << "-> Descriptors:			" << camera1_descriptors->points.size() << std::endl;
      std::cout << std::endl;

      std::cout << " ---------------------------------" << std::endl;
      std::cout << " *******     Camera 2     ******* " << std::endl;
      std::cout << " ---------------------------------" << std::endl;
      std::cout << "-> Points rgb: 	 		" << points_camera2->points.size() << std::endl;
      std::cout << "-> Points xyzi:  		" << points_camera2_xyzi->points.size() << std::endl;
      std::cout << "-> Normal points: 		" << camera2_normals->points.size() << std::endl;
      std::cout << "-> Keypoints:			" << camera2_keypoints->points.size() << std::endl;
      std::cout << "-> Descriptors:			" << camera2_descriptors->points.size() << std::endl;
      std::cout << std::endl;

      // ----------------------------------------------
      // Transformation matrix camera 1 and 2
      // ----------------------------------------------
      pcl::SampleConsensusInitialAlignment<pcl::PointXYZI, pcl::PointXYZI, pcl::FPFHSignature33>
          sac_ia;
      sac_ia.setMinSampleDistance(0.05f);
      sac_ia.setMaxCorrespondenceDistance(0.01f * 0.01f);
      sac_ia.setMaximumIterations(500);

      pcl::PointCloud<pcl::PointXYZI>::Ptr sac_registration(new pcl::PointCloud<pcl::PointXYZI>());
      sac_ia.setInputCloud(camera2_keypoints);
      sac_ia.setInputTarget(camera1_keypoints);
      sac_ia.setSourceFeatures(camera2_descriptors);
      sac_ia.setTargetFeatures(camera1_descriptors);
      // sac_ia.align(*sac_registration);

      // pcl::PointCloud<pcl::PointXYZRGB>::Ptr camera2alignetocamera1(
      //   new pcl::PointCloud<pcl::PointXYZRGB>());
      // pcl::copyPointCloud(*registration_output,      *camera2alignetocamera1);

      // ----------------------------------------------
      // Transformation matrix camera 1 and 2 (ICP RefineAlignment)
      // ----------------------------------------------
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_registration(
          new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;

      // icp.setInputCloud(points_camera2_backup);
      // icp.setInputTarget(points_camera1_backup);

      icp.setInputCloud(points_camera2);
      icp.setInputTarget(points_camera1);

      icp.setMaxCorrespondenceDistance(300);
      icp.setMaximumIterations(10000);
      icp.setTransformationEpsilon(0.0000001);
      icp.setEuclideanFitnessEpsilon(0.000000000001);

      // icp.align(*icp_registration);

      // Eigen::Matrix4f transformation_matrix;
      Eigen::Matrix4f transformation_matrix_sac = sac_ia.getFinalTransformation();
      // Eigen::Matrix4f transformation_matrix_icp = icp.getFinalTransformation();

      // transformation_matrix = transformation_matrix_icp;
      // transformation_matrix = transformation_matrix_sac * scale;
      // transformation_matrix = transformation_matrix_icp * transformation_matrix;
      // transformation_matrix = scale_i * transformation_matrix;

      // transformation_matrix = transformation_matrix_sac;

      // std::cout << sac_ia.getFinalTransformation() << std::endl;

      // std::cout << " ---------------------------------" <<     std::endl;
      // std::cout << " **** Transformation matrix ***** " <<   std::endl;
      // std::cout << " ---------------------------------" <<      std::endl;
      // std::cout << "-> matrix:\n" << transformation_matrix <<    std::endl;
      /*
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_input_cloud(
                new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::transformPointCloud(*points_camera2_backup, *transformed_input_cloud,
                                     transformation_matrix, true);

            *transformed_input_cloud = *icp_registration;

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_target_cloud(
                new pcl::PointCloud<pcl::PointXYZRGB>());

             Eigen::Isometry3f pose(
                 Eigen::Isometry3f(Eigen::Translation3f(points_camera2_backup->sensor_origin_[0],
                                                        points_camera2_backup->sensor_origin_[1],
                                                        points_camera2_backup->sensor_origin_[2])) *
                 Eigen::Isometry3f(points_camera2_backup->sensor_orientation_));

            Eigen::Affine3f transform_ = Eigen::Affine3f::Identity();
            transform_.translation() << points_camera2_backup->sensor_origin_[0],
                points_camera2_backup->sensor_origin_[1], points_camera2_backup->sensor_origin_[2];
            transform_.rotate(points_camera2_backup->sensor_orientation_);

            // Eigen::Matrix4f transformation_matrix2 = pose.matrix();
            // std::cout << transformation_matrix2 << std::endl;
            // Eigen::Matrix4f transformation_matrix2 = Eigen::Matrix4f::Identity();
            pcl::transformPointCloud(*points_camera1_backup, *transformed_target_cloud, transform_,
         true);

            // *transformed_target_cloud = *points_camera1_backup;

            // now both point clouds are transformed to the origin and can be added up
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new
         pcl::PointCloud<pcl::PointXYZRGB>());
            *output_cloud = *transformed_input_cloud;
            *output_cloud += *transformed_target_cloud;*/

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_target_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>());
      Eigen::Matrix4f transformation_matrix = icp.getFinalTransformation();
      pcl::transformPointCloud(*points_camera1_backup, *transformed_target_cloud,
                               transformation_matrix);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_input_cloud(
          new pcl::PointCloud<pcl::PointXYZRGB>());
      Eigen::Isometry3f pose(
          Eigen::Isometry3f(Eigen::Translation3f(points_camera2_backup->sensor_origin_[0],
                                                 points_camera2_backup->sensor_origin_[1],
                                                 points_camera2_backup->sensor_origin_[2])) *
          Eigen::Isometry3f(points_camera2_backup->sensor_orientation_));
      transformation_matrix = pose.matrix();
      pcl::transformPointCloud(*points_camera2_backup, *transformed_input_cloud,
                               transformation_matrix);

      // now both point clouds are transformed to the origin and can be added up
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      *output_cloud += *transformed_input_cloud;
      *output_cloud += *transformed_target_cloud;

      if (cont <= 0) {

        viewer->addPointCloud(points_camera1_backup, "points_camera1", PORT1);
        viewer->addPointCloud(points_camera2_backup, "points_camera2", PORT2);
        viewer->addPointCloud(transformed_target_cloud, "points_camera1_transformed", PORT3);
        viewer->addPointCloud(transformed_input_cloud, "points_camera2_transformed", PORT4);
        viewer->addPointCloud(output_cloud, "cloud_fusion", PORT5);

        viewer->addText("camera 1", xpos, ypos, fontSize, r, g, b, "text1", PORT1);
        viewer->addText("camera 2", xpos, ypos, fontSize, r, g, b, "text2", PORT2);
        viewer->addText("camera 1 transformed", xpos, ypos, fontSize, r, g, b, "text3", PORT3);
        viewer->addText("camera 2 transformed", xpos, ypos, fontSize, r, g, b, "text4", PORT4);
        viewer->addText("camera fusion", xpos, ypos, fontSize, r, g, b, "text5", PORT5);

        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                                 "points_camera1");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                                 "points_camera2");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                                 "points_camera1_transformed");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                                 "points_camera2_transformed");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                                 "cloud_fusion");

        cont += 1;

      } else {

        viewer->updatePointCloud(points_camera1_backup, "points_camera1");
        viewer->updatePointCloud(points_camera2_backup, "points_camera2");
        viewer->updatePointCloud(transformed_target_cloud, "points_camera1_transformed");
        viewer->updatePointCloud(transformed_input_cloud, "points_camera2_transformed");
        viewer->updatePointCloud(output_cloud, "cloud_fusion");

        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        // 5,
        //                                          "points_camera1");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        // 5,
        //                                          "points_camera2");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        // 5,
        //                                          "points_camera1_transformed");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        // 5,
        //                                          "points_camera2_transformed");
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        // 5,
        //                                          "cloud_fusion");

        transformed_target_cloud->width = (int)transformed_target_cloud->points.size();
        transformed_target_cloud->height = 1;
        transformed_target_cloud->is_dense = true;

        std::string output_dir = "transformed_target_cloud.pcd";

        std::string sav = "saved point cloud in:";
        sav += output_dir;

        pcl::console::print_info(sav.c_str());
        std::cout << std::endl;
        pcl::io::savePCDFileBinary(output_dir.c_str(), *transformed_target_cloud);

        transformed_input_cloud->width = (int)transformed_input_cloud->points.size();
        transformed_input_cloud->height = 1;
        transformed_input_cloud->is_dense = true;

        output_dir = "transformed_input_cloud.pcd";

        sav.clear();
        sav = "saved point cloud in:";
        sav += output_dir;

        pcl::console::print_info(sav.c_str());
        std::cout << std::endl;
        pcl::io::savePCDFileBinary(output_dir.c_str(), *transformed_input_cloud);

        break;

        cont += 1;
      }

      viewer->spinOnce(500);
    } else {
      pcl::console::print_info("\ncamera is not ready!\n");
      continue;
    }
  }

  _rs_camera1.stop();
  _rs_camera2.stop();

  pcl::console::print_highlight("Visualizer closed.\n");
  // std::cout << "Visualizer closed." << std::endl;
  return EXIT_SUCCESS;
} // while end
