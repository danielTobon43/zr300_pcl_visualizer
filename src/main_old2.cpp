/////////////////////////////////////////////////////////////////////////////
// Headers
/////////////////////////////////////////////////////////////////////////////
#include <librealsense/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

#include <iostream>
#include <fstream>
#include <string>

// Window size and frame rate
int const INPUT_WIDTH      = 320;
int const INPUT_HEIGHT     = 240;
int const FRAMERATE        = 60;

bool next_iteration = false;

typedef pcl::PointXYZRGB PointT;

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

rs::device& init_realsense_camera(rs::context& rs_ctx,unsigned int index){

	 rs::device& _rs_camera = *rs_ctx.get_device(index);

	 printf("\nUsing device %d, an %s\n",index ,_rs_camera.get_name());
	 printf("    Serial number: %s\n", _rs_camera.get_serial());
	 printf("    Firmware version: %s\n", _rs_camera.get_firmware_version());

     _rs_camera.enable_stream( rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE );
     _rs_camera.enable_stream( rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE );
     _rs_camera.start();

     return _rs_camera;
} 

boost::shared_ptr<pcl::visualization::PCLVisualizer>  init_pcl_visualizer(){

	 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("ZR300"));

	 viewer->setPosition(0,0);
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

	 viewer->setCameraPosition(focal0,focal1,focal2,pos0,pos1,pos2,view0,view1,view2);
     viewer->setCameraClipDistances(clip0, clip1);
     viewer->setPosition(posx, posy);
     viewer->setSize(sizex, sizey);
     pcl::console::print_info ("\npress [q] to exit!\n");

     return viewer;

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
	
	 int input_COM1 = 0; // ZR300 camera 1
	 int input_COM2 = 1; // ZR300 camera 2
	 rs::device& _rs_camera1 = init_realsense_camera(rs_ctx,input_COM1);
	 rs::device& _rs_camera2 = init_realsense_camera(rs_ctx,input_COM2);

     // ----------------------------------------------
     // Setting up PCL Visualizer
     // ----------------------------------------------   
     boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = init_pcl_visualizer();   
     int xpos = 1.0;
	 int ypos = 1.0;
	 int fontSize = 13;

	 double r = 1.0;
	 double g = 1.0;
	 double b = 1.0; 
     int cont = 0;

     // viewport to visualize camera 1 and 2 
	 int PORT1 = 0;
	 viewer->createViewPort(0.0, 0.0, 0.5, 1.0, PORT1);

	 int PORT2 = 0;
	 viewer->createViewPort(0.5, 0.0, 1.0, 1.0, PORT2);



	 int iterations = 5;  // Default number of ICP iterations

	 // ----------------------------------------------
     // Loop for streaming
     // ----------------------------------------------        
     while(!viewer->wasStopped() ) {
         if(_rs_camera1.is_streaming( ) and _rs_camera2.is_streaming( )){
             _rs_camera1.wait_for_frames( );
             _rs_camera2.wait_for_frames( );

             pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_camera1 = points_to_pcl(_rs_camera1);
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_camera2 = points_to_pcl(_rs_camera2);

             pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_camera2_backup(new pcl::PointCloud<pcl::PointXYZRGB>);
             *points_camera2_backup = *points_camera2;  // We backup cloud_icp into cloud_tr for later use

		      // // The Iterative Closest Point algorithm
		      // pcl::console::TicToc time;
			  // time.tic ();
			  // pcl::IterativeClosestPoint<PointT, PointT> icp;
			  // icp.setMaximumIterations (iterations);
			  // icp.setInputSource (points_camera2);
			  // icp.setInputTarget (points_camera1);
			  // icp.align (*points_camera2);
			  // icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
			  // std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

			  // if (icp.hasConverged ()){
			  //   std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
			  //   //std::cout << "\nICP transformation " << iterations << " : points_camera2 -> points_camera1" << std::endl;
			  //   //transformation_matrix = icp.getFinalTransformation ().cast<double>();
			  //   //print4x4Matrix (transformation_matrix);
			  // }else{
			  //   PCL_ERROR ("\nICP has not converged.\n");
			  //   return (-1);
			  // }


             if(cont<=0){

             	 std::string str = "Points: ";
				 std::stringstream ss;
				 ss << points_camera1->points.size();
				 str += ss.str();

				 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(points_camera1,255,255,255);

             	 viewer->addPointCloud(points_camera1,color_handler,"POINTCLOUD1",PORT1);           	 
	             viewer->addText(str, xpos, ypos, fontSize,r,g,b,"text1",PORT1);

	             str.clear();
	             ss.str(std::string());
	             str = "Points: ";
	             ss << points_camera2->points.size();
				 str += ss.str();

				 viewer->addPointCloud(points_camera2,"POINTCLOUD2",PORT2);
	             viewer->addText(str, xpos, ypos, fontSize,r,g,b,"text2",PORT2);

             	 cont += 1;

             }else{            	
            
			     std::string str = "Points: ";
				 std::stringstream ss;
				 ss << points_camera1->points.size();
				 str += ss.str();

				 pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(points_camera1,255,255,255);

				 viewer->updatePointCloud(points_camera1,color_handler,"POINTCLOUD1");
				 viewer->updateText(str, xpos, ypos, fontSize,r,g,b,"text1");

	             str.clear();
	             ss.str(std::string());
	             str = "Points: ";
	             ss << points_camera2->points.size();
				 str += ss.str();       

				 viewer->updatePointCloud(points_camera2,"POINTCLOUD2");				 
	             viewer->updateText(str, xpos, ypos, fontSize,r,g,b,"text2");

             }

             viewer->spinOnce(200);

         }else{
         	pcl::console::print_info ("\ncamera is not ready!\n");
         	continue;
         }

     }
     _rs_camera1.stop( ); 
     _rs_camera2.stop( ); 

     return EXIT_SUCCESS;  
}

