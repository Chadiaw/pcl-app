#ifndef KINECTGRABBER_H
#define KINECTGRABBER_H

#include <QObject>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

class KinectGrabber : public QObject
{
	Q_OBJECT

public:
	KinectGrabber();
	void run();
	void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getPointCloud();
signals:
	void cloudChanged();
public slots:
	void start();
	void stop();
private:
    bool wasStopped;
    bool started;

};

//Q_DECLARE_METATYPE(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr);

#endif // KINECTGRABBER_H
