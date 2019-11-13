#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace rs;


// Window size and frame rate
int const INPUT_WIDTH      = 320;
int const INPUT_HEIGHT     = 240;
int const FRAMERATE        = 60;

// Named windows
char* const WINDOW_DEPTH = "Depth Image";
char* const WINDOW_RGB     = "RGB Image";


context      _rs_ctx;
device&      _rs_camera = *_rs_ctx.get_device( 0 );
intrinsics   _depth_intrin;
intrinsics  _color_intrin;
bool         _loop = true;


// Initialize the application state. Upon success will return the static app_state vars address

bool initialize_streaming( )
{
       bool success = false;
       if( _rs_ctx.get_device_count( ) > 0 )
       {
             _rs_camera.enable_stream( rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE );
             _rs_camera.enable_stream( rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE );
             _rs_camera.start( );

             success = true;
       }
       return success;
}




/////////////////////////////////////////////////////////////////////////////
// If the left mouse button was clicked on either image, stop streaming and close windows.
/////////////////////////////////////////////////////////////////////////////
static void onMouse( int event, int x, int y, int, void* window_name )
{
       if( event == cv::EVENT_LBUTTONDOWN )
       {
             _loop = false;
       }
}


/////////////////////////////////////////////////////////////////////////////
// Create the depth and RGB windows, set their mouse callbacks.
// Required if we want to create a window and have the ability to use it in
// different functions
/////////////////////////////////////////////////////////////////////////////
void setup_windows( )
{
       cv::namedWindow( WINDOW_DEPTH, 0 );
       cv::namedWindow( WINDOW_RGB, 0 );

       cv::setMouseCallback( WINDOW_DEPTH, onMouse, WINDOW_DEPTH );
       cv::setMouseCallback( WINDOW_RGB, onMouse, WINDOW_RGB );
}


/////////////////////////////////////////////////////////////////////////////
// Called every frame gets the data from streams and displays them using OpenCV.
/////////////////////////////////////////////////////////////////////////////
bool display_next_frame( )
{

       _depth_intrin       = _rs_camera.get_stream_intrinsics( rs::stream::depth );
       _color_intrin       = _rs_camera.get_stream_intrinsics( rs::stream::color );
	   

       // Create depth image
       cv::Mat depth16( _depth_intrin.height,
                                  _depth_intrin.width,
                                  CV_16U,
                                  (uchar *)_rs_camera.get_frame_data( rs::stream::depth ) );

       // Create color image
       cv::Mat rgb( _color_intrin.height,
                            _color_intrin.width,
                            CV_8UC3,
                            (uchar *)_rs_camera.get_frame_data( rs::stream::color ) );

       // < 800
       cv::Mat depth8u = depth16;
       depth8u.convertTo( depth8u, CV_8UC1, 255.0/1000 );

       imshow( WINDOW_DEPTH, depth8u );
       cv::waitKey(1);

       cv::cvtColor( rgb, rgb, cv::COLOR_BGR2RGB );
       imshow( WINDOW_RGB, rgb );
       cv::waitKey(1);

       return true;
}

/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main( ) try
{
       rs::log_to_console( rs::log_severity::warn );

       if( !initialize_streaming( ) )
       {
             std::cout << "Unable to locate a camera" << std::endl;
             rs::log_to_console( rs::log_severity::fatal );
             return EXIT_FAILURE;
       }

       setup_windows( );

       // Loop until someone left clicks on either of the images in either window.
       while( _loop )
       {
             if( _rs_camera.is_streaming( ) )
                    _rs_camera.wait_for_frames( );

             display_next_frame( );
       }


       _rs_camera.stop( );
       cv::destroyAllWindows( );
	   

       return EXIT_SUCCESS;

}
catch( const rs::error & e )
{
       std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
       return EXIT_FAILURE;
}
catch( const std::exception & e )
{
       std::cerr << e.what() << std::endl;
       return EXIT_FAILURE;
}
/*
#include <librealsense/rs.hpp>
#include "example.hpp"

#include <sstream>
#include <iostream>
#include <iomanip>
#include <thread>
#include <algorithm>
#include <map>
#include <memory>

#include <opencv2/opencv.hpp>   // Include OpenCV API

texture_buffer buffers[RS_STREAM_COUNT];
bool align_depth_to_color = false;
bool align_color_to_depth = false;
bool color_rectification_enabled = false;

int main(int argc, char * argv[]) try
{

     rs::log_to_console(rs::log_severity::warn);

    rs::context ctx;
    if (ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    std::vector<rs::stream> supported_streams;

    for (int i = (int)rs::capabilities::depth; i <= (int)rs::capabilities::fish_eye; i++)
        if (dev.supports((rs::capabilities)i))
            supported_streams.push_back((rs::stream)i);

    // Configure all supported streams to run at 30 frames per second
    for (auto & stream : supported_streams)
        dev.enable_stream(stream, rs::preset::best_quality);

    // Start our device
    dev.start();                                

    using namespace cv;
    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        

        // Query frame size (width and height)
        const int w = 640;
        const int h = 480;

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat image(Size(w, h), CV_8UC3, (void*)dev.get_data(), Mat::AUTO_STEP);

        // Update the window with new data
        imshow(window_name, image);
}
*/

/*





    rs::log_to_console(rs::log_severity::warn);

    rs::context ctx;
    if (ctx.get_device_count() == 0) throw std::runtime_error("No device detected. Is it plugged in?");
    rs::device & dev = *ctx.get_device(0);

    std::vector<rs::stream> supported_streams;

    for (int i = (int)rs::capabilities::depth; i <= (int)rs::capabilities::fish_eye; i++)
        if (dev.supports((rs::capabilities)i))
            supported_streams.push_back((rs::stream)i);

    // Configure all supported streams to run at 30 frames per second
    for (auto & stream : supported_streams)
        dev.enable_stream(stream, rs::preset::best_quality);

    // Start our device
    dev.start();

    // Open a GLFW window
    glfwInit();
    std::ostringstream ss; ss << "Viewer (" << dev.get_name() << ")";

    int tile_w = 640; // pixels
    int tile_h = 480; // pixels
    GLFWwindow * win = glfwCreateWindow(tile_w, tile_h, ss.str().c_str(), 0, 0);
      
    glfwMakeContextCurrent(win);

    while (!glfwWindowShouldClose(win))
    {
        // Wait for new images
        glfwPollEvents();
        dev.wait_for_frames();

        // Clear the framebuffer
        int w, h;
        glfwGetFramebufferSize(win, &w, &h);
        //glViewport(0, 0, w, h);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw the images
        glPushMatrix();
  
        glOrtho(0, w, h, 0, -1, +1);
        buffers[0].show(dev, align_color_to_depth ? rs::stream::color_aligned_to_depth : (color_rectification_enabled ? rs::stream::rectified_color : rs::stream::color), 0, 0, tile_w, tile_h);

        
        glPopMatrix();
        glfwSwapBuffers(win);
    }

    glfwDestroyWindow(win);
    glfwTerminate();
    return EXIT_SUCCESS;
*/
/*
}
catch (const rs::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception & e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
*/
