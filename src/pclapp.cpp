#include "pclapp.h"
#include "../build/ui_pclapp.h"
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QStatusBar>
#include <pcl/filters/filter.h>
#include "utils.h"

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
  connect (ui->pushButton_start, SIGNAL (clicked ()), this, SLOT (startButtonPressed()));
  connect (ui->pushButton_stop, SIGNAL (clicked ()), this, SLOT (stopButtonPressed()));

  // Connect buttons and functions
  connect (ui->pushButton_load,  SIGNAL (clicked ()), this, SLOT (loadButtonPressed ()));
  connect (ui->pushButton_saveKinect,  SIGNAL (clicked ()), this, SLOT (saveKinectCloudPressed ()));
  connect (ui->pushButton_saveViewer,  SIGNAL (clicked ()), this, SLOT (saveViewerCloudPressed ()));
  connect (ui->pushButton_nan,  SIGNAL (clicked ()), this, SLOT (nanButtonPressed ()));
  connect (ui->pushButton_biggest,  SIGNAL (clicked ()), this, SLOT (biggestButtonPressed ()));
  connect (ui->pushButton_closest,  SIGNAL (clicked ()), this, SLOT (closestButtonPressed ()));
  connect (ui->pushButton_filter,  SIGNAL (clicked ()), this, SLOT (filterButtonPressed ()));

  // Connect point size slider
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

  viewer->addPointCloud (cloud, "cloud");
  pSliderValueChanged (4);
  viewer->resetCamera ();
  //kinectViewer->resetCamera ();
  ui->qvtkWidget_view->update ();
  ui->qvtkWidget_kinect->update ();
  
  kinectViewer->getCameraParameters(cameraParams);

  //this->statusBar()->
}

void
PCLApp::cloudCaptured() {
	PointCloudT::Ptr tempCloud = grabber->getPointCloud();
	if (!tempCloud->empty()) {
		kinectViewer->removeAllPointClouds();
		kinectViewer->addPointCloud(tempCloud, "kinectCloud");
		ui->qvtkWidget_kinect->update();
	}
}

void
PCLApp::startButtonPressed() {
	kinectViewer->setCameraParameters(cameraParams);
	grabber->start();
    ui->pushButton_start->setEnabled(false);
    ui->pushButton_stop->setEnabled(true);
    ui->qvtkWidget_kinect->setDisabled(true);
    ui->pushButton_saveKinect->setEnabled(false);
    this->statusBar()->showMessage(tr("Kinect Grabber started."));
}

void
PCLApp::stopButtonPressed() {
    grabber->stop();
    ui->pushButton_start->setEnabled(true);
    ui->pushButton_stop->setEnabled(false);
    ui->qvtkWidget_kinect->setDisabled(false);
    ui->pushButton_saveKinect->setEnabled(true);
    this->statusBar()->showMessage(tr("Kinect Grabber stopped."));
}

void
PCLApp::saveKinectCloudPressed ()
{
    QString fileName = QFileDialog::getSaveFileName(this,
           tr("Save Frame to PCD File"), "../pcd files",
           tr("Point Cloud Data (*.pcd);;All Files (*)"));
    if (fileName.isEmpty())
        return;
    else {
        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly)) {
            QMessageBox::information(this, tr("Unable to open file"),
                file.errorString());
            this->statusBar()->showMessage(tr("Unable to save file."));
            return;
        }
                pcl::io::savePCDFile (fileName.toStdString(), *grabber->getPointCloud());
                this->statusBar()->showMessage(tr("Kinect data saved to file ") + fileName + ".");

    }
}

void
PCLApp::saveViewerCloudPressed ()
{
    QString fileName = QFileDialog::getSaveFileName(this,
           tr("Save Frame to PCD File"), "../pcd files",
           tr("Point Cloud Data (*.pcd);;All Files (*)"));
    if (fileName.isEmpty())
        return;
    else {
        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly)) {
            QMessageBox::information(this, tr("Unable to open file"),
                file.errorString());
            return;
        }
                pcl::io::savePCDFile (fileName.toStdString(), *cloud);
                this->statusBar()->showMessage(tr("Current point cloud saved to file ") + fileName + ".");

    }
}
void
PCLApp::nanButtonPressed()
{
    ui->pushButton_saveViewer->setEnabled(false);
    //PointCloudT::Ptr filteredCloud = Utils::removeNaNPoints(cloud);
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	viewer->removeAllPointClouds();
	viewer->addPointCloud(cloud, "cloud");
	ui->qvtkWidget_view->update();
    ui->pushButton_saveViewer->setEnabled(true);
    this->statusBar()->showMessage(tr("Removed all NaN points from the point cloud."));
}

void
PCLApp::biggestButtonPressed()
{
    cloud = Utils::getBiggestCluster(cloud);
    viewer->removeAllPointClouds();
    viewer->addPointCloud(cloud, "cloud");
    ui->qvtkWidget_view->update();
    this->statusBar()->showMessage(tr("Extracted biggest cloud cluster from point cloud."));
}

void
PCLApp::closestButtonPressed()
{
    cloud = Utils::getClosestCluster(cloud);
    viewer->removeAllPointClouds();
    viewer->addPointCloud(cloud, "cloud");
    ui->qvtkWidget_view->update();
    this->statusBar()->showMessage(tr("Extracted closest cloud cluster from point cloud."));
}

void
PCLApp::filterButtonPressed()
{
	cloud = Utils::filterCloud(cloud);
	viewer->removeAllPointClouds();
	viewer->addPointCloud(cloud, "cloud");
	ui->qvtkWidget_view->update();
	this->statusBar()->showMessage(tr("Filtered current point cloud"));
}

void
PCLApp::loadButtonPressed ()
{
  QString fileName = QFileDialog::getOpenFileName(this, "Load a file...", "../pcd files",
                               tr("Point Cloud Data (*.pcd)"));

  if (pcl::io::loadPCDFile(fileName.toStdString(), *cloud) == -1) //* load the file
  {
      printf("Error while loading the file");
      PCL_ERROR ("Couldn't read specified file \n");
      this->statusBar()->showMessage(tr("Unable to open load."));

  }
 this->statusBar()->showMessage(tr("PCD file was succesfully loaded."));

  viewer->removeAllPointClouds();
  viewer->addPointCloud (cloud, "cloud");
  //viewer->resetCamera();
  viewer->setCameraParameters(cameraParams);
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
