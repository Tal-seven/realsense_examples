#include <librealsense2/rs.hpp>
#include "/home/talha/librealsense/examples/example.hpp"
#include <iostream>
#include <algorithm>
#include <thread>
#include <atomic>
#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_glfw.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>


void draw_pointCloud(window& app,glfw_state& state,const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& points);

void pointsToPcl(const rs2::points& points,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	
	auto sp = points.get_profile().as<rs2::video_stream_profile>();
	cloud->width = sp.width();
	cloud->height = sp.height();
	cloud->is_dense = false;
	cloud->points.resize(points.size());

	auto ptr = points.get_vertices();
	
	for(auto& pts: cloud->points)
	{
		pts.x = ptr->x;
		pts.y = ptr->y;
		pts.z = ptr->z;
		ptr++;
	}
}

int main(int argc,char** argv) try
{
	window app(1280,720,"pointcloud demo");

	ImGui_ImplGlfw_Init(app,false);
	glfw_state original_view;
	glfw_state filtered_view;

	rs2::pointcloud originalPc;
	rs2::points originalPoints;
	//rs2::points filteredPoints;
	
	rs2::pipeline pipe;
	
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> original;
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> filtered;

	rs2::colorizer color_map;
	
	pcl::PCLPointCloud2::Ptr originalpcl2Ptr;
	pcl::PCLPointCloud2::Ptr filteredpcl2Ptr;
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setLeafSize(0.01f,0.01f,0.01f);

	std::atomic_bool stopped(false);
	pipe.start();
	/*std::thread process_thread([&](){
			while(!stopped)
			{
				rs2::frameset set = pipe.wait_for_frames();
				auto color = set.get_color_frame();
				
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
				
				originalPc.map_to(color);
				rs2::depth_frame frame = set.get_depth_frame();
				rs2::points originalPoints = originalPc.calculate(frame);
				pointsToPcl(originalPoints,cloud);
				
				pcl::toPCLPointCloud2(*cloud,*originalpcl2Ptr);
				pcl::toPCLPointCloud2(*filteredCloud,*filteredpcl2Ptr);

				sor.setInputCloud(originalpcl2Ptr);
				sor.filter(*filteredpcl2Ptr);
				pcl::fromPCLPointCloud2(*filteredpcl2Ptr,*filteredCloud);
				original.push_back(cloud);
				filtered.push_back(filteredCloud);		
			}			
		});*/

	auto last_time = std::chrono::high_resolution_clock::now();
	auto maxAngle = 15.0f;
	auto maxVel = 0.3f;

	draw_text(10,50,"originalData");
	draw_text(int(app.width()/2),50,"filteredData");
	while(app)
	{
		glViewport(0,app.height(),app.width()/2,app.height());
		draw_pointCloud(app,original_view,original);

		glViewport(app.width()/2,app.height(),app.width()/2,app.height());
		draw_pointCloud(app,filtered_view,filtered);
	}

	stopped = true;
	//process_thread.join();	

	return EXIT_SUCCESS;
}
catch(const rs2::error& err)
{
	std::cerr << "realsense error calling: "<<err.get_failed_function() << "("<<err.get_failed_args()<<"):\n"<<err.what()<<std::endl;
	return EXIT_FAILURE;
}
catch(const std::exception& ex)
{
	std::cerr << ex.what() <<std::endl;
	return EXIT_FAILURE;
}

float3 colors[]={{0.8f,0.1f,0.3f},
									{0.1f,0.9f,0.5f}};

void draw_pointCloud(window& app,glfw_state& app_state,const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& points)
{
	glPopMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	float width = app.width();
	float height = app.height();

	glClearColor(153/255.0f,153/255.0f,153/255.0f,1);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	gluPerspective(60,width/height,0.01f,10.0f);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	gluLookAt(0,0,0,0,0,1,0,-1,0);

	glPointSize(width/640);
	glEnable(GL_TEXTURE_2D);

	int color=0;
	
	for(auto&& pc: points)
	{
		auto c = colors[(color++)%sizeof(colors)/sizeof(float3)];
		glBegin(GL_POINTS);
		glColor3f(c.x,c.y,c.z);

		for(int i=0;i<pc->points.size();i++)
		{
			auto&& p = pc->points[i];
			if(p.z)
			{
				glVertex3f(p.x,p.y,p.z);
			}
		}

		glEnd();
	}

}
