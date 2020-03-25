#include <librealsense2/rs.hpp>
#include <iostream>

int main(int argc,char** argv) try
{
	rs2::pipeline p;
	p.start();

	while(true)
	{
				rs2::frameset frames = p.wait_for_frames();
				rs2::depth_frame dframe = frames.get_depth_frame();

				float width = dframe.get_width();
				float height = dframe.get_height();

				float distance = dframe.get_distance(width/2,height/2);

				std::cout << "camera is facing towards an object "<<distance<< " meters away"<<std::endl;
	}

	return EXIT_SUCCESS;
}

catch(const rs2::error& e)
{
	std::cerr << "Failed calling function: " << e.get_failed_function() << "("<<e.get_failed_args()<<")\n"<<e.what()<<std::endl;
	return EXIT_FAILURE;
}

catch(const std::exception& s)
{
	std::cerr<< s.what() << std::endl;
	return EXIT_FAILURE;
}
