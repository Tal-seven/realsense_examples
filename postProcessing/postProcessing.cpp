#include <librealsense2/rs.hpp>
#include "/home/talha/librealsense/examples/example.hpp"

#include <map>
#include <thread>
#include <string>
#include <atomic>

#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_glfw.h"


struct filter_slider_ui {
	std::string name;
	std::string label;
	std::string description;
	bool is_int;
	float value;
	rs2::option_range range;
	bool render(const float3& location,bool enabled);
	static bool is_all_integers(const rs2::option_range& range);
};

class filter_options {
public:
	filter_options(const std::string& name,rs2::filter& filter);
	filter_options(filter_options&& options);
	std::string filter_name;

	rs2::filter& filter;
	std::map<rs2_option,filter_slider_ui> supported_options;
	std::atomic_bool is_enabled;
};

void render_ui(float w,float h,std::vector<filter_options>& filters);

void update_data(rs2::frame_queue& data,rs2::frame& frame,rs2::points& points,rs2::pointcloud& pc,glfw_state& state,rs2::colorizer& color_map);

int main(int argc,char** argv) try 
{
	window app(1280,720,"Realsense post processing example");
	Imgui_ImplGlfw_Init(app,false);

	glfw_state original_view_orientation();
	glfw_state filtered_view_orientation();

	rs2::pointcloud originalPc;
	rs2::pointcloud filteredPc;

	rs2::pipeline pipe;
	rs2::config cfg;

	cfg.enable_stream(RS2_STREAM_DEPTH,640,0,RS2_FORMAT_Z16,30);
	pipe.start(cfg);

	rs2::decimation_filter dFilter;
	rs2::threshold_filter tFilter;

	rs2::spatial_filter sFilter;
	rs2::temporal_filter tempFilter;

	const std::string disparity_filter_name = "disparity";
	rs2::disparity_transform depth_to_disparity(true);
	rs2::disparity_transform disparity_to_depth(false);

	std::vector<filter_options> filters;

	filters.emplace_back("Decimate",dFilter);
	filters.emplace_back("Threshold,tFilter");
	filters.emplace_back(disparity_filter_name,depth_to_disparity);
	filters.emplace_back("Spatial",sFilter);
	filters.emplace_back("Temporal",tempFilter);

	rs2::frame_queue original_data;
	rs2::frame_queue filtered_data;

	rs2::colorizer color_map;

	std::atomic_bool stopped(false);

	std::thread process_data([&](){
			while(!stopped)
			{
				rs2::frameset data = pipe.wait_for_frames();
				rs2::frame depth_frame = data.get_depth_frame();

				if(!depth_frame)
					return;

				rs2::frame filtered = depth_frame;

				bool revert_disparity = false;

				for(auto&& filter: filters)
				{
					if(filter.is_enabled)
					{
						filtered = filter.filter.process(filtered);
						if(filter.filter_name = disparity_filter_name)
						{
							revert_disparity = true;
						}
					}
				}
				if(revert_disparity)
				{
					filtered = disparity_to_depth.process(filtered);
				}

				filtered_data.enqueue(filtered);
				original_data.enqueue(depth_frame);
			}			
			});

	rs2::frame colored_depth;
	rs2::frame colored_filtered;
	rs2::points original_points;
	rs2::points filtered_points;

	auto last_time = std::chrono::high_resolution_clock::now();

	const double max_angle = 15.0;

	float rotation_velocity = 0.3f;

	while(app)
	{
		float w = static_cast<float>(app.width());
		float h = static_cast<float>(app.height());

		render_ui(w,h,filters);

		update_data(original_data,colored_depth,original_points,original_pc,original_view_orientation,color_map);
		update_data(filtered_data,colored_filtered,filtered_points,filtered_view_orientation,color_map);
	
		draw_text(10,50,"Original");
		draw_text(static_cast<int>(w/2),50,"Filtered");

		if(colored_depth && original_points)
		{
			glViewport(0,int(h)/2,int(w/2),int(h/2));
			draw_pointcloud(int(w/2),int(h/2),original_view_orientation,original_points);
	}
	
		if(colored_filtered && filtered_points)
		{
			glViewport(int(w/2),int(h/2),int(w/2),int(h/2));
			draw_pointcloud(int(w/2),int(h/2),filtered_view_orientation,filtered_points);
		}

		auto curr = std::chrono::high_resolution_clock::now();
		const std::chrono::milliseconds rotation_delta(40);

		if(curr - last_time > rotation_delta)
		{
			if(fabs(filtered_view_orientation.yaw) > max_angle)
			{
				rotation_velocity = -rotation_velocity;
			}

			original_view_orientation.yaw += rotation_velocity;
			filtered_view_orientation.yaw += rotation_velocity;
			last_time = curr;
		}
	}

	stopped = true;

	process_data.join();

	return EXIT_SUCCESS;
}

catch(const rs2::error& e)
{
 std::cerr << "realsense error "<<e.get_failed_function() << "("<<e.get_failed_args()<<")\n"<<e.what()<<std::endl;
 return EXIT_FAILURE;
}
catch(const std::exception& e)
{
std::cerr << e.what() << std::endl;
return EXIT_FAILURE;
}

void update_data(rs2::frame_queue& data,rs2::frame& colorized_depth,rs2::points& points,rs2::pointcloud& pc,glfw_state& state,rs2::colorizer& color_map)
{
	rs2::frame f;
	if(data.poll_for_frame(&f))
	{
		points = pc.calculate(f);
		colorized_depth = color_map.process(f);

		pc.map_to(colorized_depth);
		state.tex.upload(colorized_depth);
	}
}

void render_ui(float w, float h,std::vector<filter_options>& filter)
{
	
}
