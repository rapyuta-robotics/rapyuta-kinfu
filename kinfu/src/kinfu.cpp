/*
 *    kinfu.cpp
 *
 *    Copyright 2013 Dominique Hunziker
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 *
 *     \author/s: Dominique Hunziker
 *
 *    Based on:
 *      ROS wrapper for Kinfu Large Scale from pcl_unstable
 *      maintained by Michael Korn
 *
 *      http://fsstud.is.uni-due.de/svn/ros/is/kinfu
 */

#include <iostream>

#include <pcl/common/angles.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>

#include <pcl/gpu/containers/initialization.h>
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/screenshot_manager.h>

#include <std_msgs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <tf/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <kinfu/kinfu_Config.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class SampledScopeTime: public pcl::StopWatch
{
private:
	enum
	{
		EACH = 33
	};

public:
	SampledScopeTime(int& time_ms) :
			time_ms_(time_ms)
	{
	}

	~SampledScopeTime()
	{
		static int i_ = 0;
		time_ms_ += getTime();
		if (i_ % EACH == 0 && i_)
		{
			cout << "Average frame time = " << time_ms_ / EACH << "ms ( "
					<< 1000.f * EACH / time_ms_ << "fps)" << endl;
			time_ms_ = 0;
		}
		++i_;
	}

private:
	int& time_ms_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class KinFuLSApp
{
private:
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
			sensor_msgs::CameraInfo, sensor_msgs::Image> DRGBSyncPolicy;
	typedef message_filters::Synchronizer<DRGBSyncPolicy> DRGBSync;
	typedef message_filters::TimeSynchronizer<sensor_msgs::Image,
			sensor_msgs::CameraInfo> DepthSync;

public:
	KinFuLSApp(ros::NodeHandle &nh, const float vsz, const float shiftDistance,
			const bool integrate_colors, const bool enable_texture_extraction,
			const int snapshot_rate) :
			frame_counter_(0), time_ms_(0), /* integrate_colors_(integrate_colors), */
			enable_texture_extraction_(enable_texture_extraction), registration_(
					false), snapshot_rate_(snapshot_rate), /* pcd_source_(false), */nh_(
					nh)
	{
		init_kinfu(vsz, shiftDistance);
		init_ros();
	}

private:
	void init_kinfu(const float vsz, const float shiftDistance)
	{
		//Init Kinfu Tracker
		Eigen::Vector3f volume_size = Eigen::Vector3f::Constant(vsz/*meters*/);

		ROS_INFO("--- CURRENT SETTINGS ---\n");
		ROS_INFO("Volume size is set to %.2f meters\n", vsz);
		ROS_INFO(
				"Volume will shift when the camera target point is farther than %.2f meters from the volume center\n", shiftDistance);
		ROS_INFO(
				"The target point is located at [0, 0, %.2f] in camera coordinates\n", 0.6*vsz);
		ROS_INFO("------------------------\n");

		// warning message if shifting distance is abnormally big compared to volume size
		if (shiftDistance > 2.5 * vsz)
			ROS_WARN(
					"WARNING Shifting distance (%.2f) is very large compared to the volume size (%.2f).\nYou can modify it using --shifting_distance.\n", shiftDistance, vsz);

		kinfu_ = boost::make_shared<pcl::gpu::kinfuLS::KinfuTracker>(
				volume_size, shiftDistance);

		Eigen::Matrix3f R = Eigen::Matrix3f::Identity(); // * AngleAxisf(pcl::deg2rad(-30.f), Vector3f::UnitX());
		Eigen::Vector3f t = volume_size * 0.5f
				- Eigen::Vector3f(0, 0, volume_size(2) / 2 * 1.2f);

		Eigen::Affine3f pose = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);

		kinfu_->setInitialCameraPose(pose);
		kinfu_->volume().setTsdfTruncDist(0.030f/*meters*/);
		kinfu_->setIcpCorespFilteringParams(0.1f/*meters*/,
				sin(pcl::deg2rad(20.f)));
		//kinfu_->setDepthTruncationForICP(3.f/*meters*/);
		kinfu_->setCameraMovementThreshold(0.001f);

		raycaster_ = boost::make_shared<pcl::gpu::kinfuLS::RayCaster>(
				kinfu_->rows(), kinfu_->cols());
	}

	void init_ros()
	{
		ros::NodeHandle nh;
		image_transport::ImageTransport it(nh);

		pub_kinfu_reset_ = nh.advertise<std_msgs::Empty>("/kinfu_reset", 2);
		pub_scene_ = it.advertise("/camera/kinfuLS/depth", 10);

		if (enable_texture_extraction_)
		{
			sub_depth_ = boost::shared_ptr<image_transport::SubscriberFilter>(
					new image_transport::SubscriberFilter(it,
							"/camera/depth/image_raw", 2));
			sub_info_ = boost::shared_ptr<
					message_filters::Subscriber<sensor_msgs::CameraInfo> >(
					new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,
							"/camera/depth/camera_info", 2));
			sub_rgb_ = boost::shared_ptr<image_transport::SubscriberFilter>(
					new image_transport::SubscriberFilter(it,
							"/camera/rgb/image_color", 2));

			//the depth and the rgb cameras are not hardware synchronized
			//hence the depth and rgb images normally do not have the EXACT timestamp
			//so use approximate time policy for synchronization
			texture_sync_ = boost::shared_ptr<DRGBSync>(
					new DRGBSync(DRGBSyncPolicy(500), *sub_depth_, *sub_info_,
							*sub_rgb_));
			texture_sync_->registerCallback(
					boost::bind(&KinFuLSApp::execute, this, _1, _2, _3));
			ROS_INFO("Running KinFu with texture extraction");
		}
		else
		{
			sub_depth_ = boost::shared_ptr<image_transport::SubscriberFilter>(
					new image_transport::SubscriberFilter(it,
							"/camera/depth/image_raw", 1));
			sub_info_ = boost::shared_ptr<
					message_filters::Subscriber<sensor_msgs::CameraInfo> >(
					new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,
							"/camera/depth/camera_info", 1));

			depth_only_sync_ = boost::shared_ptr<DepthSync>(
					new DepthSync(*sub_depth_, *sub_info_, 500));
			depth_only_sync_->registerCallback(
					boost::bind(&KinFuLSApp::execute, this, _1, _2,
							sensor_msgs::ImageConstPtr()));
			ROS_INFO("Running KinFu without texture extraction");
		}

		reconf_server_ = boost::make_shared<
				dynamic_reconfigure::Server<kinfu::kinfu_Config> >(nh);
		reconf_server_->setCallback(
				boost::bind(&KinFuLSApp::reconf_callback, this, _1, _2));
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//callback function, called with every new depth topic message
	void execute(const sensor_msgs::ImageConstPtr& depth,
			const sensor_msgs::CameraInfoConstPtr& cameraInfo,
			const sensor_msgs::ImageConstPtr& rgb =
					sensor_msgs::ImageConstPtr())
	{
		frame_counter_++;

		if (kinfu_->icpIsLost())
		{
			kinfu_->reset();
			ROS_INFO("KinFu was reset!\n");
			pub_kinfu_reset_.publish(std_msgs::Empty());
		}

		depth_device_.upload(&(depth->data[0]), depth->step, depth->height,
				depth->width);

//		if (integrate_colors_ && rgb)
//			colors_device_.upload(&(rgb->data[0]), rgb->step, rgb->height,
//					rgb->width);

		/*
		 *      [fx  0 cx]
		 * K = 	[ 0 fy cy]
		 * 		[ 0  0  1]
		 */
		const float focal_length = (cameraInfo->K[0] + cameraInfo->K[4]) / 2;

		kinfu_->setDepthIntrinsics(cameraInfo->K[0], cameraInfo->K[4],
				cameraInfo->K[2], cameraInfo->K[5]);

		SampledScopeTime fps(time_ms_);
		(*kinfu_)(depth_device_);

		if (kinfu_->isFinished())
			nh_.shutdown();

		publish_scene(depth->header.frame_id, 0);
		publish_pose(depth->header.stamp);

		//save snapshots
		if (enable_texture_extraction_)
		{
			if (frame_counter_ % snapshot_rate_ == 0)
			{

				screenshot_manager_.setCameraIntrinsics(focal_length,
						cameraInfo->height, cameraInfo->width);

				//convert sensor_msgs::Image to pcl::gpu::PixelRGB
				const unsigned int pixelCount = rgb->height * rgb->width;
				pcl::gpu::kinfuLS::PixelRGB * pixelRgbs =
						new pcl::gpu::kinfuLS::PixelRGB[pixelCount];
				for (unsigned int i = 0; i < pixelCount; ++i)
				{
					//the encoding given in the image is "bgr8"
					pixelRgbs[i].b = rgb->data[i * 3];
					pixelRgbs[i].g = rgb->data[i * 3 + 1];
					pixelRgbs[i].r = rgb->data[i * 3 + 2];
				}
				pcl::gpu::PtrStepSz<const pcl::gpu::kinfuLS::PixelRGB> rgb24(
						rgb->height, rgb->width, pixelRgbs, rgb->step);
				screenshot_manager_.saveImage(kinfu_->getCameraPose(), rgb24);
				delete[] pixelRgbs;
			}
		}
	}

	void reconf_callback(kinfu::kinfu_Config &config, uint32_t level)
	{
		if (config.stop)
			kinfu_->performLastScan();
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	void publish_scene(const std::string &frame_id, const Eigen::Affine3f *pose)
	{
		sensor_msgs::Image msg;

		if (pub_scene_.getNumSubscribers() == 0)
			return;

		if (pose)
		{
			raycaster_->run(kinfu_->volume(), *pose,
					kinfu_->getCyclicalBufferStructure());
			raycaster_->generateSceneView(view_device_);
		}
		else
		{
			kinfu_->getImage(view_device_);
		}

//		if (paint_image_ && registration_ && !has_pose)
//		{
//			colors_device_.upload(rgb24.data, rgb24.step, rgb24.rows,
//					rgb24.cols);
//			paint3DView(colors_device_, view_device_);
//		}

		//convert image to sensor message
		msg.header.seq = frame_counter_;
		msg.header.stamp = ros::Time::now(); // TODO: Maybe use input timestamp?
		msg.header.frame_id = frame_id;
		msg.width = view_device_.cols();
		msg.height = view_device_.rows();
		msg.step = view_device_.cols() * 3;
		msg.encoding = "rgb8";
		msg.is_bigendian = 0;
		msg.data.resize(msg.step * msg.height);
		view_device_.download(&msg.data[0], msg.step);

		pub_scene_.publish(msg);
	}

	void publish_pose(const ros::Time& stamp)
	{
		const Eigen::Matrix<float, 3, 3, Eigen::RowMajor> erreMats =
				kinfu_->getCameraPose().linear();
		const Eigen::Vector3f teVecs = kinfu_->getCameraPose().translation();

		//TODO: start position: init_tcam_ = volume_size * 0.5f - Vector3f (0, 0, volume_size (2) / 2 * 1.2f);

		tf::Transform transform(
				tf::Matrix3x3(erreMats(0, 0), erreMats(0, 1), erreMats(0, 2),
						erreMats(1, 0), erreMats(1, 1), erreMats(1, 2),
						erreMats(2, 0), erreMats(2, 1), erreMats(2, 2)),
				tf::Vector3(teVecs[0], teVecs[1], teVecs[2]));

		pub_odom_.sendTransform(
				tf::StampedTransform(transform, stamp, "/odom",
						"/kinfu_frame"));
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	int frame_counter_;
	int time_ms_;

//	const bool integrate_colors_;
	const bool enable_texture_extraction_;
	const bool registration_;
	const int snapshot_rate_;

	boost::shared_ptr<pcl::gpu::kinfuLS::KinfuTracker> kinfu_;
	boost::shared_ptr<pcl::gpu::kinfuLS::RayCaster> raycaster_;

	pcl::gpu::kinfuLS::KinfuTracker::DepthMap depth_device_;
//	pcl::gpu::kinfuLS::KinfuTracker::View colors_device_;
	pcl::gpu::kinfuLS::KinfuTracker::View view_device_;

	pcl::kinfuLS::ScreenshotManager screenshot_manager_;

	/* ROS node handle */
	ros::NodeHandle &nh_;

	/* Publishers */
	tf::TransformBroadcaster pub_odom_;
	image_transport::Publisher pub_scene_;
	ros::Publisher pub_kinfu_reset_;

	/* Subscribers */
	boost::shared_ptr<image_transport::SubscriberFilter> sub_rgb_;
	boost::shared_ptr<image_transport::SubscriberFilter> sub_depth_;
	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > sub_info_;
	boost::shared_ptr<DRGBSync> texture_sync_;
	boost::shared_ptr<DepthSync> depth_only_sync_;

	/* Dynamic reconfigure server */
	boost::shared_ptr<dynamic_reconfigure::Server<kinfu::kinfu_Config> > reconf_server_;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int print_cli_help()
{
	std::cout << "\nKinFu parameters:" << std::endl;
	std::cout << "    --help, -h                        : print this message"
			<< std::endl;
	std::cout << "\nkinfuLS node parameters:" << std::endl;
	std::cout << "    volume_size <in_meters>, vs       : "
			<< "define integration volume size" << std::endl;
	std::cout << "    shifting_distance <in_meters>, sd : "
			<< "define shifting threshold (distance target-point / cube center)"
			<< std::endl;
	std::cout << "    snapshot_rate <X_frames>, sr      : "
			<< "Extract RGB textures every <X_frames>. Default: 45"
			<< std::endl;
	std::cout << "    extract_textures, et              : "
			<< "extract RGB PNG images to KinFuSnapshots folder. Default: true"
			<< std::endl;

	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[])
{
	// check arguments
	if (pcl::console::find_switch(argc, argv, "--help")
			|| pcl::console::find_switch(argc, argv, "-h"))
		return print_cli_help();

	ros::init(argc, argv, "kinfuLS");
	ros::NodeHandle nh("~");

	// assign values from parameter server, with default.
	int device = 0;
	double volume_size = 3.0; //pcl::device::VOLUME_SIZE
	double shift_distance = 1.5; //pcl::device::DISTANCE_THRESHOLD;
	bool integrate_colors = false;
	bool enable_texture_extraction = false;
	int snapshot_rate = 45;

	nh.getParam("device", device);

	nh.getParam("volume_size", volume_size);
	nh.getParam("vs", volume_size);

	nh.getParam("shift_distance", shift_distance);
	nh.getParam("sd", shift_distance);

	nh.getParam("integrate_colors", integrate_colors);
	nh.getParam("ic", integrate_colors);

	nh.getParam("extract_textures", enable_texture_extraction);
	nh.getParam("et", enable_texture_extraction);

	nh.getParam("snapshot_rate", snapshot_rate);
	nh.getParam("sr", snapshot_rate);

	pcl::gpu::setDevice(device);
	pcl::gpu::printShortCudaDeviceInfo(device);

	KinFuLSApp app(nh, volume_size, shift_distance, integrate_colors,
			enable_texture_extraction, snapshot_rate);

	ros::Rate loop_rate(40);
	while (nh.ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
