#include "pclapp.h"
#include "../build/ui_pclapp.h"
#include <QtWidgets/QFileDialog>

PCLApp::PCLApp (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLApp)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL App");

  grabber = new KinectGrabber();

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);
  // The number of points in the cloud
  cloud->points.resize (200);

  // The default color
  red   = 128;
  green = 128;
  blue  = 128;

  // Fill the cloud with some points
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

    cloud->points[i].r = red;
    cloud->points[i].g = green;
    cloud->points[i].b = blue;
  }

  // Set up the QVTK viewer windows
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  kinectViewer.reset (new pcl::visualization::PCLVisualizer ("kinectViewer", false));
  ui->qvtkWidget_view->SetRenderWindow (viewer->getRenderWindow ());
  ui->qvtkWidget_kinect->SetRenderWindow (kinectViewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget_view->GetInteractor (), ui->qvtkWidget_view->GetRenderWindow ());
  kinectViewer->setupInteractor (ui->qvtkWidget_kinect->GetInteractor (), ui->qvtkWidget_kinect->GetRenderWindow ());
  ui->qvtkWidget_view->update ();
  ui->qvtkWidget_kinect->update ();

  // Connect KinectGrabber, start and stop buttons
  connect (grabber, SIGNAL (cloudChanged()), this, SLOT(cloudCaptured()));
  ui->pushButton_stop->setEnabled(false);
  connect (ui->pushButton_start, SIGNAL (clicked ()), this, SLOT (startButtonPressed()));=
  connect (ui->pushButton_stop, SIGNAL (clicked ()), this, SLOT (stopButtonPressed()));

  // Connect "random" button and the function
  connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));
  connect (ui->pushButton_load,  SIGNAL (clicked ()), this, SLOT (loadButtonPressed ()));

  // Connect point size slider
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

  viewer->addPointCloud (cloud, "cloud");
  pSliderValueChanged (4);
  viewer->resetCamera ();
  //kinectViewer->resetCamera ();
  ui->qvtkWidget_view->update ();
  ui->qvtkWidget_kinect->update ();
  
  kinectViewer->getCameraParameters(cameraParams);
}

void
PCLApp::cloudCaptured() {
    kinectViewer->removeAllPointClouds();
    kinectViewer->addPointCloud(grabber->getPointCloud(), "kinectCloud");
    ui->qvtkWidget_kinect->update();
}

void
PCLApp::startButtonPressed() {
	kinectViewer->setCameraParameters(cameraParams);
	grabber->start();
    ui->pushButton_start->setEnabled(false);
    ui->pushButton_stop->setEnabled(true);
    ui->qvtkWidget_kinect->setDisabled(true);
}

void
PCLApp::stopButtonPressed() {
    grabber->stop();
    ui->pushButton_start->setEnabled(true);
    ui->pushButton_stop->setEnabled(false);
    ui->qvtkWidget_kinect->setDisabled(false);
}

void
PCLApp::randomButtonPressed ()
{
  printf ("Random button was pressed\n");

  // Set the new color
  for (size_t i = 0; i < cloud->size(); i++)
  {
    cloud->points[i].r = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    cloud->points[i].g = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    cloud->points[i].b = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
  }

  viewer->updatePointCloud (cloud, "cloud");
  ui->qvtkWidget_view->update ();
}

void
PCLApp::loadButtonPressed ()
{
  QString fileName = QFileDialog::getOpenFileName(this, "Load a file...", "",
                               tr("Point Cloud Data (*.pcd)"));

  if (pcl::io::loadPCDFile(fileName.toStdString(), *cloud) == -1) //* load the file
  {
      PCL_ERROR ("Couldn't read specified file \n");

  }
  printf ("File was succesfully loaded\n");

  viewer->updatePointCloud (cloud, "cloud");
  viewer->resetCamera();
  ui->qvtkWidget_view->update ();
}

void
PCLApp::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->qvtkWidget_view->update ();
}

PCLApp::~PCLApp ()
{
  delete ui;
  delete grabber;
}
