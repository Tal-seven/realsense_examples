#include <iostream>
#include <memory>
#include <thread>
#include <atomic>
#include <algorithm>
#include <librealsense2/rs.hpp>
#include "/home/talha/librealsense/examples/example.hpp"
#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_glfw.h"
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
using pclPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using pclPtr2 = pcl::PCLPointCloud2::Ptr;

void pointsToPcl(const rs2::points& points,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto ptr = points.get_vertices();

	for(auto& pt: cloud->points)
	{
		pt.x = ptr->x;
	  pt.y = ptr->y;
	  pt.z	= ptr->z;
		ptr++;
	}
}


std::tuple<uint8_t,uint8_t,uint8_t> getTexColor(rs2::video_frame& frame,rs2::texture_coordinate tcords)
{
	const int width = frame.get_width(); const int height = frame.get_height();

	int x = std::min(std::max(int(tcords.u*width+0.5f),0),width-1);
	int y = std::min(std::max(int(tcords.v*height+0.5f),0),height-1);

	int idx = x*frame.get_bytes_per_pixel() + y*frame.get_stride_in_bytes();
	const auto texData = reinterpret_cast<const uint8_t*>(frame.get_data());
	return std::tuple<uint8_t,uint8_t,uint8_t>(texData[idx],texData[idx+1],texData[idx+2]);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointsToPclTexture(const rs2::points& points,rs2::video_frame& frame)
{
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto texCoords = points.get_texture_coordinates();
	auto vertices = points.get_vertices();

	for(int i =0;i<points.size();i++)
	{
		cloud->points[i].x = vertices[i].x;
		cloud->points[i].y = vertices[i].y;
		cloud->points[i].z = vertices[i].z;

		std::tuple<uint8_t,uint8_t,uint8_t> color;
		color = getTexColor(frame,texCoords[i]);

		cloud->points[i].r = std::get<2>(color);
		cloud->points[i].g = std::get<1>(color);
		cloud->points[i].b = std::get<0>(color);
	}

	return cloud;
}

void render_ui(float w,float h)
{
	static const int flags = ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar
													| ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove
													| ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoSavedSettings;

	ImGui_ImplGlfw_NewFrame(1);
	ImGui::SetNextWindowSize({w,h});
	ImGui::Begin("app",nullptr,flags);
	const float offsetX = w/4;
	const float offsetY = h/1.1;
	ImGui::SetCursorPos({offsetX,offsetY});
	ImGui::Text("Original Frame");
	
	ImGui::SetCursorPos({offsetX+w/2,offsetY});
	ImGui::Text("Filtered Frame");
	ImGui::End();
	ImGui::Render();
}

float3 colors[] = {{0.8,0.3,0.0},
									{0.7,0.2,0.1},
									{0.0,0.8,0.1}};

void draw_pointcloud(window& app,glfw_state& state,std::vector<pclPtr>& vec){
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	float width = app.width(); float height = app.height();
	glClearColor(153.f/255,153.f/255,153.f/255,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60,width/height,0.01f,10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0,0,0,0,0,1,0,-1,0);

	glTranslatef(0,0,0.5f+ state.offset_y*0.05f);
	glRotated(state.pitch,1,0,0);
	glRotated(state.yaw,0,1,0);
	glTranslatef(0,0,-0.5f);
	
	glPointSize(width/640);
	glEnable(GL_TEXTURE_2D);

	int color = 0;
	
	for(auto&& point: vec)	
	{
	auto c = colors[(color++)%(sizeof(colors)/sizeof(float3))];
	glBegin(GL_POINTS);
	glColor3f(c.x,c.y,c.z);
		
	for(int temp = 0;temp < point->points.size();temp++)
	{
				auto&& pc = point->points[temp];
		if(pc.z)
			glVertex3f(pc.x,pc.y,pc.z);
	}
	glEnd();
	}
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glPopAttrib();
	glPushMatrix();
}

pcl::visualization::PCLVisualizer::Ptr viewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr vis(new pcl::visualization::PCLVisualizer("rgb visualizer"));
	vis->setBackgroundColor(0,0,0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	vis->addPointCloud<pcl::PointXYZRGB>(cloud,rgb,"sample cloud");
	vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"sample cloud");
	vis->addCoordinateSystem(1.0);
	vis->initCameraParameters();
	return vis;
}

void registerGlfwCallbacks(window& app,glfw_state& state)
{
	app.on_left_mouse = [&](bool pressed){state.ml = pressed;};
	app.on_mouse_scroll = [&](double xOffset,double yOffset) 
	{
		state.offset_x += xOffset;
		state.offset_y += yOffset;
	};

	app.on_mouse_move = [&](double xpos,double ypos){
	if(state.ml)
	{
		state.yaw -= (xpos - state.last_x);
		state.yaw = std::max(state.yaw,-120.0);
		state.yaw = std::min(state.yaw,+120.0);
		state.pitch += (ypos - state.last_y);
		state.pitch = std::max(state.pitch,-80.0);
		state.pitch = std::min(state.pitch,+80.0);

	}

	state.last_x = xpos;
	state.last_y = ypos;
	};

	app.on_key_release = [&](int key)	{
		if(key == 32)
		{
			state.pitch = state.yaw=0;
			state.offset_x=state.offset_y=0;
		}
	};
}


int main(int argc,char** argv)
{
	window app(1280,720,"point cloud processing example");
	ImGui_ImplGlfw_Init(app,false);

	glfw_state original_state;
	glfw_state filtered_state;
	
	registerGlfwCallbacks(app,original_state);
	registerGlfwCallbacks(app,filtered_state);

	float w = app.width();
	float h = app.height();
	rs2::pipeline pipe;


	pipe.start();
	rs2::colorizer  color_map;

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr originalCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
	if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/talha/realsense/pointCloudLibrary/table_scene_lms400.pcd",*originalCloud)==-1)
	{
		PCL_ERROR("couldnt read the file");
		return -1;
	}
	pcl::PCLPointCloud2::Ptr originalCloud2(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr filteredCloud2(new pcl::PCLPointCloud2);
	std::vector<pclPtr> original_data;
	std::vector<pclPtr> filtered_data;
	pcl::VoxelGrid<pcl::PCLPointCloud2> filter;
	filter.setLeafSize(0.05f,0.05f,0.05f);
	rs2::pointcloud pcloud;
	rs2::points points;
			
	rs2::frameset frame = pipe.wait_for_frames();
	auto depth_frame = frame.get_depth_frame();
	auto color_frame = frame.get_color_frame();

	if(!depth_frame)
		return -1;
			
	pcloud.map_to(color_frame);
	points = pcloud.calculate(depth_frame);
	auto cloud = pointsToPclTexture(points,color_frame);

	pcl::visualization::PCLVisualizer::Ptr vis(new pcl::visualization::PCLVisualizer("rgb visualizer"));
	vis->setBackgroundColor(0,0,0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	vis->addPointCloud<pcl::PointXYZ>(originalCloud,"sample cloud");
	vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3,"sample cloud");
	vis->addCoordinateSystem(1.0);
	vis->initCameraParameters();
	bool firstIter = true;

	
	
	while(!vis->wasStopped() && app)
	{
			//render_ui(w,h);
			vis->spinOnce(100);
			//std::this_thread::sleep_for(100ms);
			frame = pipe.wait_for_frames();
			depth_frame = frame.get_depth_frame();
			color_frame = frame.get_color_frame();

			if(!depth_frame)
				return -1;
			
			pcloud.map_to(color_frame);
			points = pcloud.calculate(depth_frame);
			cloud = pointsToPclTexture(points,color_frame);
		
			//vis->updatePointCloud<pcl::PointXYZRGB>(cloud,"sample cloud");
			/*pcl::toPCLPointCloud2(*originalCloud,*originalCloud2);

			filter.setInputCloud(originalCloud2);
			filter.filter(*filteredCloud2);
			pcl::fromPCLPointCloud2(*filteredCloud2,*filteredCloud);
			
			filtered_data.push_back(filteredCloud);
			original_data.push_back(originalCloud);

			glViewport(0,int(h/1.2),int(w/2),int(h/2));
			draw_pointcloud(app,original_state,original_data);

			glViewport(int(w/2),int(h/1.2),int(w/2),int(h/2));
			draw_pointcloud(app,filtered_state,filtered_data);*/
	}
}
