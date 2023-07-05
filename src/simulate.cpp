#include <render_kinect/kinectSimulator.h>
#include <csetjmp>
#include <csignal>

static std::jmp_buf jump_buffer;

extern "C" void segfault_handler(int signal)
{
  std::longjmp(jump_buffer, signal);
}

extern "C"
{
  int simulate(float *vertices,
               int num_verts,
               int *faces,
               int num_faces,
               float *out_depth,
               int width,
               int height,
               float fx,
               float fy,
               float cx,
               float cy,
               const char *dot_pattern_path,
               bool debug)
  {
    if (setjmp(jump_buffer) == 0)
    {
      std::signal(SIGSEGV, segfault_handler);

      render_kinect::CameraInfo cam_info;

      cam_info.width = width;
      cam_info.height = height;
      cam_info.cx_ = cx;
      cam_info.cy_ = cy;

      cam_info.z_near = 0.5;
      cam_info.z_far = 6.0;
      cam_info.fx_ = fx;
      cam_info.fy_ = fy;
      cam_info.tx_ = 0.075; // baseline between IR projector and IR camera

      // cam_info.noise_ = render_kinect::GAUSSIAN;
      cam_info.noise_ = render_kinect::PERLIN;
      // cam_info.noise_ = render_kinect::NONE;

      if (debug)
      {
        std::cout << "Instantiating KinectSimulator" << std::endl;
      }
      std::string dot_path(dot_pattern_path);
      render_kinect::KinectSimulator object_model(cam_info, vertices, num_verts, faces, num_faces, dot_path);

      Eigen::Affine3d current_tf = Eigen::Affine3d::Identity();
      cv::Mat point_cloud_, labels_;
      cv::Mat depth_im_(cam_info.height, cam_info.width, CV_32FC1, out_depth);
      cv::Mat scaled_im_(cam_info.height, cam_info.width, CV_32FC1);

      if (debug)
      {
        std::cout << "Simulating measurement" << std::endl;
      }
      object_model.intersect(current_tf, point_cloud_, depth_im_, labels_, debug);
      if (debug)
      {
        std::cout << "Done." << std::endl;
      }
      return 0;
    }
    else
    {
      std::cout << "Caught segfault" << std::endl;
      return 1;
    }
  }
}
