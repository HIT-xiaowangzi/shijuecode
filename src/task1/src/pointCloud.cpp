#include <iostream>
#include <algorithm> 
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>
#include <stdio.h>
#include <tchar.h>
#include <map>
#include <mutex>                    // std::mutex, std::lock_guard
#include <cmath>                    // std::ceil
//#include <conio.h>

// Intel Realsense Headers  相机头文件
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include "example.hpp"   

// PCL Headers  PCL头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>

// Include OpenCV API  OpenCV头文件
#include <opencv2/opencv.hpp>   
//#include "cv-helpers.hpp"

using namespace std;  //定义命名空间
using namespace cv;


//获取深度像素对应长度单位转换
float get_depth_scale(rs2::device dev);

//检查摄像头数据管道设置是否改变
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);

typedef pcl::PointXYZRGB RGB_Cloud;  //定义点云类型
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
typedef point_cloud::Ptr prevCloud;


// Prototypes 原型
void Load_PCDFile(void);
bool userInput(void);
void cloudViewer(void);

// Global Variables  全局变量
string cloudFile; // .pcd file name
string prevCloudFile; // .pcd file name (Old cloud)
int i = 1; // Index for incremental file name  增量文件名的索引


//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("3D Viewer"));全局共享指针
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer2"));
//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3(new pcl::visualization::PCLVisualizer("Captured Frame"));
声明
//std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY);
//cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color);
//boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);


const std::string no_camera_message = "No camera connected, please connect 1 or more";
const std::string platform_camera_name = "Platform Camera";

//设备容器
class device_container  
{
	// Helper struct per pipeline  
	struct view_port
	{
		std::map<int, rs2::frame> frames_per_stream;
		rs2::colorizer colorize_frame;
		texture tex;
		rs2::pipeline pipe;
		rs2::pipeline_profile profile;
	};

public:

	void enable_device(rs2::device dev)
	{
		std::string serial_number(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		std::lock_guard<std::mutex> lock(_mutex);

		if (_devices.find(serial_number) != _devices.end())
		{
			return; //already in
		}

		// Ignoring platform cameras (webcams, etc..)
		if (platform_camera_name == dev.get_info(RS2_CAMERA_INFO_NAME))
		{
			return;
		}
		// Create a pipeline from the given device
		rs2::pipeline p;
		rs2::config cfg;
		cfg.enable_device(serial_number);
		
		cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
		cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
		cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
		
		//cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
		//cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
		//cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);

		// Start the pipeline with the configuration
		rs2::pipeline_profile profile = p.start(cfg);
		// Hold it internally
		_devices.emplace(serial_number, view_port{ {},{},{}, p, profile });

	}

	void remove_devices(const rs2::event_information& info)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		// Go over the list of devices and check if it was disconnected
		auto itr = _devices.begin();
		while (itr != _devices.end())
		{
			if (info.was_removed(itr->second.profile.get_device()))
			{
				itr = _devices.erase(itr);
			}
			else
			{
				++itr;
			}
		}
	}

	size_t device_count()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _devices.size();
	}

	int stream_count()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		int count = 0;
		for (auto&& sn_to_dev : _devices)
		{
			for (auto&& stream : sn_to_dev.second.frames_per_stream)
			{
				if (stream.second)
				{
					count++;
				}
			}
		}
		return count;
	}

	void poll_frames()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		// Go over all device
		for (auto&& view : _devices)
		{
			// Ask each pipeline if there are new frames available
			rs2::frameset frameset;
			if (view.second.pipe.poll_for_frames(&frameset))
			{
				for (int i = 0; i < frameset.size(); i++)
				{
					rs2::frame new_frame = frameset[i];
					int stream_id = new_frame.get_profile().unique_id();
					//view.second.frames_per_stream[stream_id] = view.second.colorize_frame.process(new_frame); //update view port with the new stream
					view.second.frames_per_stream[stream_id] = new_frame; 不将深度图彩色化
				}
			}
		}
	}

	void render_textures()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		int stream_num = 0;
		rs2::colorizer color_map;
		for (auto&& view : _devices)
		{
			// For each device get its frames
			for (auto&& id_to_frame : view.second.frames_per_stream)
			{

				if (rs2::video_frame vid_frame = id_to_frame.second.as<rs2::video_frame>())
				{

					auto format = vid_frame.get_profile().format();
					auto w = vid_frame.get_width();
					auto h = vid_frame.get_height();

					if (format == RS2_FORMAT_BGR8)  彩色图
					{
						auto colorMat = Mat(Size(w, h), CV_8UC3, (void*)vid_frame.get_data(), Mat::AUTO_STEP);
						imshow("color1_" + to_string(stream_num), colorMat);
					}

					else if (format == RS2_FORMAT_RGB8)
					{
						auto colorMat = Mat(Size(w, h), CV_8UC3, (void*)vid_frame.get_data(), Mat::AUTO_STEP);
						cvtColor(colorMat, colorMat, COLOR_RGB2BGR);
						imshow("color2_" + to_string(stream_num), colorMat);
					}

					else if (format == RS2_FORMAT_Z16)
					{
						auto depth = vid_frame.apply_filter(color_map);
						auto colorMat = Mat(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
						imshow("color_depth_" + to_string(stream_num), colorMat);
						//auto depthMat = Mat(Size(w, h), CV_16UC1, (void*)vid_frame.get_data(), Mat::AUTO_STEP);
					}

					waitKey(1);

				}
				stream_num++;

			}
		}
	}

	void pointcloud_process() {
		std::lock_guard<std::mutex> lock(_mutex);
		int stream_num = 0;
		rs2::frame color_frame_1, depth_frame_1;
		rs2::frame color_frame_1, color_frame_2, depth_frame_1, depth_frame_2;
		cloud_pointer cloud1;
		cloud_pointer cloud1, cloud2;
		for (auto&& view : _devices) 遍历每个设备
		{
			// For each device get its frames
			for (auto&& id_to_frame : view.second.frames_per_stream) 每个设备一个stream里有3 帧 数据：深度帧，红外帧，彩色帧
			{
				if (rs2::video_frame vid_frame = id_to_frame.second.as<rs2::video_frame>())
				{

					auto format = vid_frame.get_profile().format();
					auto w = vid_frame.get_width();
					auto h = vid_frame.get_height();
					int cur_num = stream_num / 3; 

					只获取深度帧和彩色帧
					if (format == RS2_FORMAT_BGR8)  彩色帧
					{
						if (cur_num == 0)
							color_frame_1 = vid_frame;
						/*
						else
							color_frame_2 = vid_frame;
						*/
					}

					else if (format == RS2_FORMAT_Z16) 深度帧
					{
						if (cur_num == 0)
							depth_frame_1 = vid_frame;
						/*
						else
							depth_frame_2 = vid_frame;
						*/
					}
				}
				stream_num++;
			}
		}
		if (color_frame_1 && depth_frame_1)
			if (color_frame_1 && depth_frame_1 && color_frame_2 && depth_frame_2) 若两个设备的深度帧和彩色帧均获取到，则生成点云
		{
			pc1.map_to(color_frame_1);
			auto points1 = pc1.calculate(depth_frame_1);
			//cloud1 = PCL_Conversion(points1, color_frame_1);
		}
		else
		{
			cout << "depth frame and color frame not aligned" << endl;
		}
	}
private:
	std::mutex _mutex;
	std::map<std::string, view_port> _devices;
public:
	rs2::pointcloud pc1, pc2;

};
//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values. 
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
//RGB图像与点云映射
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
	// Get Width and Height coordinates of texture
	int width = texture.get_width();  // Frame width in pixels  帧宽度
	int height = texture.get_height(); // Frame height in pixels  帧高度

									   // Normals to Texture Coordinates conversion  纹理坐标的法线转换
	int x_value = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
	int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);

	int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
	int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
	int Text_Index = (bytes + strides);

	const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());

	// RGB components to save in tuple   RGB组件保存在元组中
	int NT1 = New_Texture[Text_Index];
	int NT2 = New_Texture[Text_Index + 1];
	int NT3 = New_Texture[Text_Index + 2];

	return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.

//=================================================== 
cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color) {

	// Object Declaration (Point Cloud) 点云对象声明
	cloud_pointer cloud(new point_cloud);
	cloud_pointer newCloud(new point_cloud);
	// Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)  声明RGB值存储的元组
	std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;

	//================================
	// PCL Cloud Object Configuration
	//================================
	// Convert data captured from Realsense camera to Point Cloud  将从Realsense相机捕获的数据转换为点云
	auto sp = points.get_profile().as<rs2::video_stream_profile>();

	cloud->width = static_cast<uint32_t>(sp.width());
	cloud->height = static_cast<uint32_t>(sp.height());
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto Texture_Coord = points.get_texture_coordinates();
	auto Vertex = points.get_vertices();

	// Iterating through all points and setting XYZ coordinates
	// and RGB values
	for (int i = 0; i < points.size(); i++)
	{
		//===================================
		// Mapping Depth Coordinates（深度坐标,深度数据以XYZ的值存储） 
		// - Depth data stored as XYZ values
		//===================================
		cloud->points[i].x = Vertex[i].x;
		cloud->points[i].y = Vertex[i].y;
		cloud->points[i].z = Vertex[i].z;

		// Obtain color texture for specific point  获取特定点的颜色纹理
		RGB_Color = RGB_Texture(color, Texture_Coord[i]);

		// Mapping Color (BGR due to Camera Model)
		cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
		cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
		cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>

	}

	return cloud; // PCL RGB Point Cloud generated
}

//boost::mutex updateModelMutex;

//int main(int argc, char * argv[]) try




int main() {
	const char* depth_win = "depth_Image";
	namedWindow(depth_win, WINDOW_AUTOSIZE);
	const char* color_win = "color_Image";
	namedWindow(color_win, WINDOW_AUTOSIZE);
	rs2::colorizer c;
	int ch;
	bool captureLoop = true; // Loop control for generating point clouds 用于生成点云的循环控制
	device_container connected_devices;

	rs2::context ctx;    // Create librealsense context for managing devices  为管理设备创建librealsense上下文

						 // Register callback for tracking which devices are currently connected
						//寄存器回调，用于跟踪当前连接的设备
	ctx.set_devices_changed_callback([&](rs2::event_information& info)
	{
		connected_devices.remove_devices(info);
		for (auto&& dev : info.get_new_devices())
		{
			connected_devices.enable_device(dev);
		}
	});

	// Initial population of the device list  设备列表的初始填充
	for (auto&& dev : ctx.query_devices()) // Query the list of connected RealSense devices 查询已连接的RealSense设备的列表
	{
		connected_devices.enable_device(dev);
	}

	//rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
	if ((int)connected_devices.device_count() == 0)
	{
		cerr << no_camera_message << endl;
		return  EXIT_FAILURE;
	}
	点云显示
	else if ((int)connected_devices.device_count() == 1)
	{
		//
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		boost::shared_ptr<pcl::visualization::PCLVisualizer> openCloud;
		/
		//创建数据管道流
		rs2::pointcloud pc;  //声明pointcloud对象，用于计算点云和纹理映射
		rs2::pipeline pipe;  //声明RealSense管道，封装实际的设备和传感器  //
		rs2::config cfg;  //为使用非默认配置文件配置管道创建配置 //
		rs2::colorizer color_map;
		
		cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);  /// 
		cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);  //
		cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30); // 
		
		//cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);  /// 
		//cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);  //
		//cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30); // 

		rs2::pipeline_profile selection = pipe.start(cfg);  //  selection    
		rs2::device selected_device = selection.get_device();
		auto depth_sensor = selected_device.first<rs2::depth_sensor>();

		if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED))
		{
			depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1.f); // Enable emitter
			/*depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);*/ // Disable emitter
		}
		if (depth_sensor.supports(RS2_OPTION_LASER_POWER))
		{
			// Query min and max values:  查询最小和最大值
			auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
			depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
			depth_sensor.set_option(RS2_OPTION_LASER_POWER, 1.f); // 
		}
		/
		float depth_scale = get_depth_scale(selection.get_device());
		rs2_stream align_to = RS2_STREAM_COLOR;
		rs2::align align(align_to);
		float depth_clipping_distance = 1.f;
		//
		// Loop and take frame captures upon user input
		while (captureLoop == true) {

			// Set loop flag based on user input

			if (captureLoop == false) { break; }
			///
			 // Wait for frames from the camera to settle  等待来自相机的帧稳定下来
			for (int i = 0; i < 50; i++) {
				auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure  放下几帧自动曝光
			}
			//滤波后处理
							
			/*rs2::colorizer color_map;
			color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);*/
			//*********范围滤波*********//
			rs2::decimation_filter dec;
			dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
			//rs2::hole_filling_filter hole;
			//hole.set_option(RS2_OPTION_HOLES_FILL,1);
			rs2::threshold_filter threshoid;
			threshoid.set_option(RS2_OPTION_MIN_DISTANCE,0.3);
			threshoid.set_option(RS2_OPTION_MAX_DISTANCE, 0.8);
			//rs2::disparity_transform depth2disparity;
			//rs2::disparity_transform disparity2depth(false);
			//*********空间滤波********//
			rs2::spatial_filter spat;
			spat.set_option(RS2_OPTION_HOLES_FILL, 0);
			spat.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
			spat.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
			/*spat.set_option(RS2_OPTION_HOLES_FILL,0);*/
			//**********时间滤波********//
			rs2::temporal_filter temp;
			temp.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4);
			temp.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
			/*temp.set_option(RS2_OPTION_FILTER_PERSISTENCY_INDEX,3);*/
			rs2::hole_filling_filter hole;
			hole.set_option(RS2_OPTION_HOLES_FILL,1);


			rs2::align align_to(RS2_STREAM_DEPTH);

			LOOP:
			while (cvGetWindowHandle(depth_win) && cvGetWindowHandle(color_win)) // Application still alive?
			{
				depth_sensor.supports(RS2_OPTION_LASER_POWER);
				// Query min and max values:  查询最小和最大值
				auto range = depth_sensor.get_option_range(RS2_OPTION_LASER_POWER);
				depth_sensor.set_option(RS2_OPTION_LASER_POWER, range.max); // Set max power
				depth_sensor.set_option(RS2_OPTION_LASER_POWER, 1.f); //  laser


				// Using the align object, we block the application until a frameset is available
				//堵塞程序直到新的一帧捕获
				rs2::frameset frameset = pipe.wait_for_frames();
				//frameset = frameset.apply_filter(align_to);
				// 抽取会降低深度图像的分辨率，关闭小孔并加快算法
				frameset = frameset.apply_filter(dec);
				frameset = frameset.apply_filter(spat);  
				frameset = frameset.apply_filter(threshoid);
				frameset = frameset.apply_filter(temp);
				frameset = frameset.apply_filter(color_map);
				// rs2::pipeline::wait_for_frames() can replace the device it uses in case of device error or disconnection.
				// Since rs2::align is aligning depth to some other stream, we need to make sure that the stream was not changed
				//因为rs2::align 正在对齐深度图像到其他图像流，我们要确保对齐的图像流不发生改变
				//  after the call to wait_for_frames();
				if (profile_changed(pipe.get_active_profile().get_streams(), selection.get_streams()))
				{
					//If the profile was changed, update the align object, and also get the new device's depth scale
					//如果profile发生改变，则更新align对象，重新获取深度图像像素到长度单位的转换比例
					selection = pipe.get_active_profile();
					align = rs2::align(align_to);
					depth_scale = get_depth_scale(selection.get_device());
				}
				//Get processed aligned frame
				//获取对齐后的帧
				auto processed = align.process(frameset);
				// Trying to get both other and aligned depth frames
				//尝试获取对齐后的深度图像帧和其他帧
				rs2::frame aligned_color_frame = processed.get_color_frame();//processed.first(align_to);
				rs2::frame aligned_depth_frame = processed.get_depth_frame().apply_filter(c);;
				//获取对齐之前的color图像
				rs2::frame before_depth_frame = frameset.get_depth_frame().apply_filter(c);
				//获取宽高
				const int depth_w = aligned_depth_frame.as<rs2::video_frame>().get_width();
				const int depth_h = aligned_depth_frame.as<rs2::video_frame>().get_height();
				const int color_w = aligned_color_frame.as<rs2::video_frame>().get_width();
				const int color_h = aligned_color_frame.as<rs2::video_frame>().get_height();
				const int b_color_w = before_depth_frame.as<rs2::video_frame>().get_width();
				const int b_color_h = before_depth_frame.as<rs2::video_frame>().get_height();
				//If one of them is unavailable, continue iteration
				if (!aligned_depth_frame || !aligned_color_frame)
				{
					continue;
				}
				//创建OPENCV类型 并传入数据
				Mat aligned_depth_image(Size(depth_w, depth_h), CV_8UC3, (void*)aligned_depth_frame.get_data(), Mat::AUTO_STEP);
				Mat aligned_color_image(Size(color_w, color_h), CV_8UC3, (void*)aligned_color_frame.get_data(), Mat::AUTO_STEP);
				Mat before_color_image(Size(b_color_w, b_color_h), CV_8UC3, (void*)before_depth_frame.get_data(), Mat::AUTO_STEP);
				//显示
				imshow(depth_win, aligned_depth_image);
				imshow(color_win, aligned_color_image);
				imshow("before aligned", before_color_image);
				waitKey(1);   //注意循环时间为5s一次，相应比较慢
			//while (captureLoop) {
			//	 单台相机
				auto depth = processed.get_depth_frame();
				auto RGB = processed.get_color_frame();
				 单台相机
				 Map Color texture to each point 将颜色纹理映射到每个点
				//pc.map_to(RGB);
				 Generate Point Cloud 生成点云
				//auto points = pc.calculate(depth);
				Convert generated Point Cloud to PCL Formatting将生成的点云转换为PCL格式
				//cloud_pointer cloud = PCL_Conversion(points, RGB);
				//
				转换点云数据的坐标与实际场景相对应
				 改变默认情况下，X轴和Y轴的坐标与实际相反的问题
				//for (int i = 0; i < cloud->points.size(); i++)
				//{
				//	cloud->points[i].x = -cloud->points[i].x;
				//	cloud->points[i].y = -cloud->points[i].y;
				//}
				if (_kbhit()) {//如果有按键按下，则_kbhit()函数返回真
					pc.map_to(RGB);
					// Generate Point Cloud 生成点云
					auto points = pc.calculate(depth);
					//Convert generated Point Cloud to PCL Formatting将生成的点云转换为PCL格式
					cloud_pointer cloud = PCL_Conversion(points, RGB);
					//转换点云数据的坐标与实际场景相对应
					// 改变默认情况下，X轴和Y轴的坐标与实际相反的问题
					for (int i = 0; i < cloud->points.size(); i++)
					{
						cloud->points[i].x = -cloud->points[i].x;
						cloud->points[i].y = -cloud->points[i].y;
					}

					ch = _getch();//使用_getch()函数获取按下的键值
					//cout << ch;
					if (ch == 13) {
						//当按下Enter时循环，Enter键的键值时13.
						captureLoop = userInput();
						if (captureLoop == false)
						{
							goto LOOP;
						}
						cloudFile = "Captured_Frame" + to_string(i) + ".pcd";
						//==============================
						// Write PC to .pcd File Format
						//==============================
						// Take Cloud Data and write to .PCD File Format
						cout << "Generating PCD Point Cloud File... " << endl;
						pcl::io::savePCDFileASCII("F://test2.pcd", *cloud); // Input cloud to be saved to .pcd
						cout << cloudFile << " successfully generated. " << endl;

						//Load generated PCD file for viewing
						Load_PCDFile();
						i++; // Increment File Name
					}
				}
				/*waitKey(1000);*/
            }
			cout << "Exiting Program... " << endl;
	        return EXIT_SUCCESS;
        }
	}
}

void Load_PCDFile(void)
{
	string openFileName;

	// Generate object to store cloud in .pcd file
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudView(new pcl::PointCloud<pcl::PointXYZRGB>);

	openFileName = "Captured_Frame" + to_string(i) + ".pcd";
	pcl::io::loadPCDFile("F://test2.pcd", *cloudView); // Load .pcd File

	//==========================
	// Pointcloud Visualization
	//==========================
	// Create viewer object titled "Captured Frame"
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Captured Frame"));

	//set CoordinateSystem
	viewer->addCoordinateSystem(0.1);
	// Set background of viewer to black
	viewer->setBackgroundColor(0, 0, 0);
	// Add generated point cloud and identify with string "Cloud"
	viewer->addPointCloud<pcl::PointXYZRGB>(cloudView, "Cloud");
	// Default size for rendered points
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
	// Viewer Properties
	viewer->initCameraParameters();  // Camera Parameters for ease of viewing
	cout << endl;
	cout << "Press [Q] in viewer to continue. " << endl;
	viewer->spin(); // Allow user to rotate point cloud and view it
	// Note: No method to close PC visualizer, pressing Q to continue software flow only solution.
}
//========================================
// userInput
// - Prompts user for a char to 
// test for decision making.
// [y|Y] - Capture frame and save as .pcd
// [n|N] - Exit program
//========================================
bool userInput(void) {
	bool setLoopFlag;
	bool inputCheck = false;
	char takeFrame; // Utilize to trigger frame capture from key press ('t')
	do {
		// Prompt User to execute frame capture algorithm
		cout << endl;
		cout << "Generate a Point Cloud? [y/n] ";
		cin >> takeFrame;
		cout << endl;
		// Condition [Y] - Capture frame, store in PCL object and display
		if (takeFrame == 'y' || takeFrame == 'Y') {
			setLoopFlag = true;
			inputCheck = true;
			takeFrame = 0;
		}
		// Condition [N] - Exit Loop and close program
		else if (takeFrame == 'n' || takeFrame == 'N') {
			setLoopFlag = false;
			inputCheck = true;
			takeFrame = 0;
		}
		// Invalid Input, prompt user again.
		else {
			inputCheck = false;
			cout << "Invalid Input." << endl;
			takeFrame = 0;
		}
	} while (inputCheck == false);

	return setLoopFlag;
}

float get_depth_scale(rs2::device dev)
{
	// Go over the device's sensors
	for (rs2::sensor& sensor : dev.query_sensors())
	{
		// Check if the sensor if a depth sensor
		if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
		{
			return dpt.get_depth_scale();
		}
	}
	throw std::runtime_error("Device does not have a depth sensor");
}
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
	for (auto&& sp : prev)
	{
		//If previous profile is in current (maybe just added another)
		auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp) { return sp.unique_id() == current_sp.unique_id(); });
		if (itr == std::end(current)) //If it previous stream wasn't found in current
		{
			return true;
		}
	}
	return false;
}

