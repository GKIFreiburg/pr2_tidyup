#include <QDialog>
#include <QString>
#include <QStatusBar>
#include "ui_tidyupTeachingGui.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include "tidyup_utils/geometryPoses.h"
#include "tidyup_msgs/DetectTable.h"
#include <vector>

class TidyupTeachingGui : public QDialog, protected Ui::mainDialog
{
   Q_OBJECT

   public:
      TidyupTeachingGui(const std::string & ff, const std::string & tf);
      ~TidyupTeachingGui();

      void showMessage(QString msg);

   protected Q_SLOTS:
      void on_recordManipulationLocationBtn_clicked();
      void on_recordTidyManipulationLocationBtn_clicked();
      void on_recordDoorLocationBtn_clicked();

      //void on_writeFilesBtn_clicked();
      void on_loadPackageBtn_clicked();
      void on_writePackageBtn_clicked();

      void recordedLocationsList_contextMenu(const QPoint &);
      void recordedTablesList_contextMenu(const QPoint &);

   private:

      bool estimateCurrentPose(geometry_msgs::PoseStamped & pose);

      unsigned int nextFreeLocationNumber() const;

      std::string getTableWriteString(
              const std::pair<std::string, tidyup_msgs::DetectTable::Response> & table);

      bool writeLocations(QString filename);
      bool writeTables(QString filename);

      bool writeActions(QString actionsTemplate, QString actionsFile);

   protected:
      ros::NodeHandle nh;
      ros::ServiceClient detect_table;
      ros::ServiceClient collision_check;

      std::string fixed_frame;
      std::string target_frame;

      QString last_door;

      QStatusBar* _statusBar;
};

