#include "pclapp.h"
#include "../build/ui_pclapp.h"
#include <QtWidgets/QFileDialog>
#include <QMessageBox>

PCLApp::PCLApp (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLApp)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL App");

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

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  // Connect "random" button and the function
  //connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));
  connect (ui->pushButton_load,  SIGNAL (clicked ()), this, SLOT (loadButtonPressed ()));
  connect (ui->pushButton_save,  SIGNAL (clicked ()), this, SLOT (saveButtonPressed ()));
  
  // Connect point size slider
  connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

  viewer->addPointCloud (cloud, "cloud");
  pSliderValueChanged (2);
  viewer->resetCamera ();
  ui->qvtkWidget->update ();
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
  ui->qvtkWidget->update ();
}

void
PCLApp::saveButtonPressed ()
{

    QString fileName = QFileDialog::getSaveFileName(this,
           tr("Save Frame to PCD File"), "",
           tr("objectData (*.pcd);;All Files (*)"));
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
                //std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

        }

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
  ui->qvtkWidget->update ();
}

void
PCLApp::pSliderValueChanged (int value)
{
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
  ui->qvtkWidget->update ();
}

PCLApp::~PCLApp ()
{
  delete ui;
}
