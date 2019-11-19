/////////////////////////////////////////////////////////////////////////////
// Headers
/////////////////////////////////////////////////////////////////////////////
#include <librealsense/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/visualization/pcl_visualizer.h>

//---
#include <pcl/surface/bilateral_upsampling.h>
#include <pcl/surface/mls.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/search/search.h>


#include <iostream>
#include <fstream>
#include <string>
//---

// Window size and frame rate
int const INPUT_WIDTH      = 320;
int const INPUT_HEIGHT     = 240;
int const FRAMERATE        = 60;

/////////////////////////////////////////////////////////////////////////////
// Called every frame gets the data from streams and displays them using PCL.
/////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(rs::device& _rs_camera){

	 rs::intrinsics depth_intrin = _rs_camera.get_stream_intrinsics(rs::stream::depth);
     rs::intrinsics color_intrin = _rs_camera.get_stream_intrinsics(rs::stream::color);

	 rs::extrinsics depth_to_color = _rs_camera.get_extrinsics(rs::stream::depth, rs::stream::color);
	 
	 float scale = _rs_camera.get_depth_scale();

	 // First we create a new point cloud
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	 // Wait until the device is ready
	 _rs_camera.wait_for_frames();

	 // Retrieve our images
	 const uint16_t * depth_image = (const uint16_t *)_rs_camera.get_frame_data(rs::stream::depth);
	 const uint8_t * color_image = (const uint8_t *)_rs_camera.get_frame_data(rs::stream::color);

	 for(int dy=0; dy<depth_intrin.height; ++dy){
	    for(int dx=0; dx<depth_intrin.width; ++dx) {
	        // Retrieve the 16-bit depth value and map it into a depth in meters
	        uint16_t depth_value = depth_image[dy * depth_intrin.width + dx];
	        float depth_in_meters = depth_value * scale;

	        // Skip over pixels with a depth value of zero, which is used to indicate no data
	        if(depth_value == 0) continue;

	        // Map from pixel coordinates in the depth image to pixel coordinates in the color image
	        rs::float2 depth_pixel = {(float)dx, (float)dy};
	        rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);
	        rs::float3 color_point = depth_to_color.transform(depth_point);
	        rs::float2 color_pixel = color_intrin.project(color_point);

	        // Create a new point
	        pcl::PointXYZRGB p;
	        p.x = depth_point.x;
	        p.y = depth_point.y;
	        p.z = depth_point.z;

	        // Use the color from the nearest color pixel, or pure white if this point falls outside the color image
	        const int cx = (int)std::round(color_pixel.x), cy = (int)std::round(color_pixel.y);
	        if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height){
	            p.r = 255;
	            p.g = 255;
	            p.b = 255;
	        }
	        else{
	            // indexing into the colour stream can be somewhat convoluted
	            p.r = * (color_image + (cy * color_intrin.width + cx) * 3);
	            p.g = * (color_image + (cy * color_intrin.width + cx) * 3+1);            
	            p.b = * (color_image + (cy * color_intrin.width + cx) * 3+2);
	        }
	        // Push this new point into the Point Cloud
	        cloud->push_back(p);
	    }
	 }

	 return cloud;
}

/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main(int argc, char * argv[]) {

	 // ----------------------------------------------
     // Setting up ZR300 Camera
     // ----------------------------------------------  
     rs::log_to_console(rs::log_severity::warn);

     rs::context rs_ctx;
	 printf("There are %d connected RealSense devices.\n", rs_ctx.get_device_count());
     if(rs_ctx.get_device_count() == 0){
     	 throw std::runtime_error("No device detected. Is it plugged in?");
     	 //rs::log_to_console( rs::log_severity::fatal );
     	 return EXIT_FAILURE;
     } 

	 rs::device&      _rs_camera = *rs_ctx.get_device( 0 );
	 rs::intrinsics   _depth_intrin;
	 rs::intrinsics  _color_intrin;

	 printf("\nUsing device 0, an %s\n", _rs_camera.get_name());
	 printf("    Serial number: %s\n", _rs_camera.get_serial());
	 printf("    Firmware version: %s\n", _rs_camera.get_firmware_version());

     _rs_camera.enable_stream( rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE );
     _rs_camera.enable_stream( rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE );
     _rs_camera.start( );

     // ----------------------------------------------
     // Setting up PCL Visualizer
     // ----------------------------------------------   
	 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("PCL VISUALIZER"));

	 viewer->setPosition(0,0);
	 viewer->setBackgroundColor(0.0, 0.0, 0.0, 0.0); // Setting background to a dark grey

	 // viewer->addCoordinateSystem(0.5);
	 viewer->setShowFPS(false);
	 // pcl::PointXYZ p1, p2, p3;

	 // p1.getArray3fMap() << 0.5, 0, 0;
	 // p2.getArray3fMap() << 0, 0.5, 0;
	 // p3.getArray3fMap() << 0,0.1,0.5;

	 // viewer->addText3D("x", p1, 0.1, 1, 0, 0, "x_");
	 // viewer->addText3D("y", p2, 0.1, 0, 1, 0, "y_");
	 // viewer->addText3D ("z", p3, 0.1, 0, 0, 1, "z_");
	 viewer->resetCamera();

	 int xpos = 1.0;
	 int ypos = 1.0;
	 int fontSize = 13;

	 double r = 1.0;
	 double g = 1.0;
	 double b = 1.0;

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

	 viewer->setCameraPosition(focal0,focal1,focal2,pos0,pos1,pos2,view0,view1,view2);
     //viewer->setCameraFieldOfView(30);
     viewer->setCameraClipDistances(clip0, clip1);
     viewer->setPosition(posx, posy);
     viewer->setSize(sizex, sizey);

     //viewer->initCameraParameters();
  
	 pcl::console::print_info ("\npress [q] to exit!\n");
	 int cont = 0;

	 // ----------------------------------------------
     // Loop for streaming
     // ----------------------------------------------        
     while( !viewer->wasStopped() ) {
         if(_rs_camera.is_streaming( )){
             _rs_camera.wait_for_frames( );

             pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud = points_to_pcl(_rs_camera);
             if(cont<=0){

             	 std::string str = "Points: ";
				 std::stringstream ss;
				 ss << input_cloud->points.size();
				 str += ss.str();

             	 viewer->addPointCloud(input_cloud,"POINTCLOUD");
             	 //viewer->initCameraParameters();
	             //viewer->resetCamera();
	             viewer->addText(str, xpos, ypos, fontSize,r,g,b,"text1");
             	 cont += 1;

             }else{             	

             	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
             	/*
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr dense_points (new pcl::PointCloud<pcl::PointXYZRGB> ());

			    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
			    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points (new pcl::PointCloud<pcl::PointNormal>());
			    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;

			    double search_radius = 0.03; //0.03
			    double sampling_radius = 0.005; //0.005
			    double step_size = 0.005; //0.005
			    double gauss_param = (double)std::pow(search_radius,2);
			    int pol_order = 2;
			    unsigned int num_threats = 1;

			    mls.setComputeNormals(true);
			    mls.setInputCloud(input_cloud);
			    mls.setSearchMethod(kd_tree);
			    mls.setSearchRadius(search_radius);
			    mls.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB>::UpsamplingMethod::SAMPLE_LOCAL_PLANE);
			    mls.setUpsamplingRadius(sampling_radius);
			    mls.setUpsamplingStepSize(step_size);
			    mls.setPolynomialOrder(pol_order);
			    mls.setSqrGaussParam(gauss_param);// (the square of the search radius works best in general)
			    mls.setCacheMLSResults(true);//Set whether the mls results should be stored for each point in the input cloud.
			    mls.setNumberOfThreads(num_threats);
			    //mls.setDilationVoxelSize();//Used only in the VOXEL_GRID_DILATION upsampling method 
			    //mls.setPointDensity(15); //15
			    mls.process(*dense_points);
			    */

			    *output_cloud = *input_cloud;

			    /*
			    *output_cloud += *dense_points;

			    pcl::console::print_info("\nNew points: ");
			    pcl::console::print_value("%d", dense_points->points.size());

			    pcl::console::print_info("\nOutput cloud points: ");
			    pcl::console::print_value("%d", output_cloud->points.size());
			    pcl::console::print_info("\n");
			    */

             	 std::string str = "Points: ";
				 std::stringstream ss;
				 ss << output_cloud->points.size();
				 str += ss.str();

				 viewer->updatePointCloud(output_cloud,"POINTCLOUD");
	             viewer->updateText(str, xpos, ypos, fontSize,r,g,b,"text1");

             }

             viewer->spinOnce(500);

         }else{
         	pcl::console::print_info ("\ncamera is not ready!\n");
         	continue;
         }
     }

     _rs_camera.stop( ); 
     return EXIT_SUCCESS;  
}

