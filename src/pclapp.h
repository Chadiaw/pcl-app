#ifndef PCLAPP_H
#define PCLAPP_H

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/io/pcd_io.h>

#include <kinectgrabber.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLApp;
}

class PCLApp : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLApp (QWidget *parent = 0);
  ~PCLApp ();

public slots:

  void
  cloudCaptured ();

  void
  saveButtonPressed ();

  void
  loadButtonPressed ();

  void
  pSliderValueChanged (int value);

  void
  startButtonPressed ();

  void
  stopButtonPressed ();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> kinectViewer;
  pcl::visualization::Camera cameraParams;

  unsigned int red;
  unsigned int green;
  unsigned int blue;

private:
  Ui::PCLApp *ui;
  KinectGrabber *grabber;

};

#endif // PCLAPP_H
