extern "C"
{
    #include "lsd.h"
};
#include "VPDetection.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

class VPD_ROS{
public:
// LSD line segment detection
VPD_ROS();
~VPD_ROS(){}
cv::Mat image;

private:
void LineDetect( cv::Mat image, double thLength, std::vector<std::vector<double> > &lines );
void drawClusters( cv::Mat &img, std::vector<std::vector<double> > &lines, std::vector<std::vector<int> > &clusters );
void image_cb(const sensor_msgs::Image::ConstPtr &msg);

cv::Point2d pp;
double f, thLength;
ros::NodeHandle nh;
ros::Subscriber image_sub;

VPDetection detector;

};

VPD_ROS::VPD_ROS()
{
	double inter_param_u0 = nh.param("projection_parameters/u0", 367);
	double inter_param_v0 = nh.param("projection_parameters/v0", 238);
	double inter_param_f = nh.param("projection_parameters/gamma1", 1115);

	pp = cv::Point2d( inter_param_u0, inter_param_v0 );

	image_sub = nh.subscribe<sensor_msgs::Image>("/mv_26806346/image", 1, &VPD_ROS::image_cb, this);
	// LSD line segment detection
	thLength = nh.param("thLength", 30.0);

	ROS_INFO("initialized...");
}

void VPD_ROS::LineDetect( cv::Mat image, double thLength, std::vector<std::vector<double> > &lines )
{
	cv::Mat grayImage;
	if ( image.channels() == 1 )
		grayImage = image;
	else
		cv::cvtColor(image, grayImage, CV_BGR2GRAY);

	image_double imageLSD = new_image_double( grayImage.cols, grayImage.rows );
	unsigned char* im_src = (unsigned char*) grayImage.data;

	int xsize = grayImage.cols;
	int ysize = grayImage.rows;
	for ( int y = 0; y < ysize; ++y )
	{
		for ( int x = 0; x < xsize; ++x )
		{
			imageLSD->data[y * xsize + x] = im_src[y * xsize + x];
		}
	}

	ntuple_list linesLSD = lsd( imageLSD );
	free_image_double( imageLSD );

	int nLines = linesLSD->size;
	int dim = linesLSD->dim;
	std::vector<double> lineTemp( 4 );
	for ( int i = 0; i < nLines; ++i )
	{
		double x1 = linesLSD->values[i * dim + 0];
		double y1 = linesLSD->values[i * dim + 1];
		double x2 = linesLSD->values[i * dim + 2];
		double y2 = linesLSD->values[i * dim + 3];

		double l = sqrt( ( x1 - x2 ) * ( x1 - x2 ) + ( y1 - y2 ) * ( y1 - y2 ) );
		if ( l > thLength )
		{
			lineTemp[0] = x1;
			lineTemp[1] = y1;
			lineTemp[2] = x2;
			lineTemp[3] = y2;

			lines.push_back( lineTemp );
		}
	}

	free_ntuple_list(linesLSD);
}

void VPD_ROS::drawClusters( cv::Mat &img, std::vector<std::vector<double> > &lines, std::vector<std::vector<int> > &clusters )
{
	int cols = img.cols;
	int rows = img.rows;

	//draw lines
	std::vector<cv::Scalar> lineColors( 3 );
	lineColors[0] = cv::Scalar( 0, 0, 255 );
	lineColors[1] = cv::Scalar( 0, 255, 0 );
	lineColors[2] = cv::Scalar( 255, 0, 0 );

	for ( int i=0; i<lines.size(); ++i )
	{
		int idx = i;
		cv::Point pt_s = cv::Point( lines[idx][0], lines[idx][1]);
		cv::Point pt_e = cv::Point( lines[idx][2], lines[idx][3]);
		cv::Point pt_m = ( pt_s + pt_e ) * 0.5;

		cv::line( img, pt_s, pt_e, cv::Scalar(0,0,0), 2, CV_AA );
	}

	for ( int i = 0; i < clusters.size(); ++i )
	{
		for ( int j = 0; j < clusters[i].size(); ++j )
		{
			int idx = clusters[i][j];

			cv::Point pt_s = cv::Point( lines[idx][0], lines[idx][1] );
			cv::Point pt_e = cv::Point( lines[idx][2], lines[idx][3] );
			cv::Point pt_m = ( pt_s + pt_e ) * 0.5;

			cv::line( img, pt_s, pt_e, lineColors[i], 2, CV_AA );
		}
	}
}

void VPD_ROS::image_cb(const sensor_msgs::Image::ConstPtr &msg)
{
	ROS_INFO("getting new image...");
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
	  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return;
	}
	cv_ptr->image.copyTo(image);

	std::vector<std::vector<double> > lines;
	LineDetect( image, thLength, lines );

	std::vector<cv::Point3d> vps;
	std::vector<std::vector<int> > clusters;
	
	// detector.run( lines, pp, f, vps, clusters );

	drawClusters( image, lines, clusters );	

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vpd_node");
	VPD_ROS vpd_ros;
	ros::Rate rate(10);
	while(ros::ok()){
		ros::spinOnce();
		if (!vpd_ros.image.empty()){
			ROS_INFO("Showing new image...");
			imshow("Test", vpd_ros.image);
			cv::waitKey( 20 );
		}

		rate.sleep();
	}
	
	return 0;
}