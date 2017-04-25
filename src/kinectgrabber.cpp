#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "kinectgrabber.h"

using namespace std;
using namespace pcl;


Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
PointCloud<PointXYZRGBA>::Ptr outCld(new PointCloud<PointXYZRGBA>);

KinectGrabber::KinectGrabber() {
    wasStopped = false;
    started = false;
    //qRegisterMetaType<pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr>();

}

void KinectGrabber::cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
    if (!wasStopped) {
        pcl::transformPointCloud(*cloud, *outCld, transform_2);
        emit cloudChanged();

    }

}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr KinectGrabber::getPointCloud()
{
    return outCld;
}

void KinectGrabber::run()
{
    pcl::Grabber* interface = new pcl::io::OpenNI2Grabber();

    //transform_2.translation() << 0, -0.5, 0.0;
    transform_2.rotate(Eigen::AngleAxisf(3.14, Eigen::Vector3f::UnitX()));

    boost::function<void(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
        boost::bind(&KinectGrabber::cloud_cb_, this, _1);

    interface->registerCallback(f);

    interface->start();
    cout << "Grabber started. " << endl;

    while (started)
    {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
    }

    interface->stop();
}

void
KinectGrabber::start() {
    wasStopped = false;
    if(!started) {
        started = true;
        boost::thread grabberThread(&KinectGrabber::run, this);
    }
}

void
KinectGrabber::stop() {
    wasStopped = true;
}
