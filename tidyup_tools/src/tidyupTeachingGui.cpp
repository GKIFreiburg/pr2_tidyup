#include "tidyupTeachingGui.h"
#include <QString>
#include <QMenu>
#include <QRegExp>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QInputDialog>
#include <QTextDocument>
#include <sstream>
#include <iomanip>
#include <QMessageBox>
#include <fstream>
#include <ros/package.h>
#include <tidyup_utils/geometryPoses.h>
#include <sbpl_3dnav_planner/FullBodyCollisionCheck.h>
#include <tf/tf.h>
#include <tidyup_utils/planning_scene_interface.h>

TidyupTeachingGui::TidyupTeachingGui(const std::string & ff, const std::string & tf) : fixed_frame(ff), target_frame(tf)
{
    setupUi(this);

    detect_table = nh.serviceClient<tidyup_msgs::DetectTable>("tidyup/detect_table");
    collision_check = nh.serviceClient<sbpl_3dnav_planner::FullBodyCollisionCheck>("sbpl_full_body_planning/collision_check");

    recordTidyManipulationLocationBtn->setVisible(false);

    _statusBar = new QStatusBar(this);
    this->layout()->addWidget(_statusBar);

    recordedLocationsList->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(recordedLocationsList, SIGNAL(customContextMenuRequested(const QPoint &)),
            this, SLOT(recordedLocationsList_contextMenu(const QPoint &)));
    recordedTablesList->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(recordedTablesList, SIGNAL(customContextMenuRequested(const QPoint &)),
            this, SLOT(recordedTablesList_contextMenu(const QPoint &)));
}

TidyupTeachingGui::~TidyupTeachingGui()
{
}


void TidyupTeachingGui::showMessage(QString msg)
{
    _statusBar->showMessage(msg, 2000);
}

bool TidyupTeachingGui::estimateCurrentPose(geometry_msgs::PoseStamped & pose)
{
    // get robot state from planning scene
    PlanningSceneInterface::instance()->resetPlanningScene();
    const arm_navigation_msgs::RobotState& robotState = PlanningSceneInterface::instance()->getRobotState();

    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = fixed_frame; //robotState.multi_dof_joint_state.frame_ids[0];
    pose.pose = robotState.multi_dof_joint_state.poses[0];
    if(writeYawOnlyCk->isChecked())
    {
        tf::Quaternion quat;
        tf::quaternionMsgToTF(pose.pose.orientation, quat);
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(quat));
    }

    // check whether pose is in collision
    sbpl_3dnav_planner::FullBodyCollisionCheck service;
    service.request.robot_states.push_back(robotState);
    if (! collision_check.call(service))
    {
        ROS_ERROR("service call failed: %s", collision_check.getService().c_str());
        QMessageBox::critical(this, "Error: collision checking failed.", (std::string("service call failed: ") + collision_check.getService()).c_str());
        return false;
    }
    if ( service.response.error_codes[0].val != service.response.error_codes[0].SUCCESS)
    {
        QMessageBox::critical(this, "robot in collision", QString("Current robot state is in collision. Error-code: %1").arg(service.response.error_codes[0].val));
        return false;
    }

    return true;
}

unsigned int TidyupTeachingGui::nextFreeLocationNumber() const
{
    unsigned int freeLoc = 1;
    for(int i = 0; i < recordedLocationsList->count(); i++) {
        QListWidgetItem* it = recordedLocationsList->item(i);
        if(it == NULL)
            continue;
        QString entry = it->text();
        QStringList values = entry.split(" ");
        QString name = values[0];
    //for(std::vector<GeometryPoses::NamedPose>::const_iterator it = poses.begin(); it != poses.end(); it++) {
    //    const GeometryPoses::NamedPose & np = *it;
    //    QString name = np.first.c_str();
        QStringList parts = name.split("_");
        if(parts.size() != 3)
            continue;
        QString locName = parts.at(1);
        QRegExp reNr("[0-9]");
        int index = locName.indexOf(reNr);
        if(index < 0)
            continue;
        QString locNrStr = locName.mid(index);
        bool ok = false;
        unsigned int locNr = locNrStr.toUInt(&ok);
        if(!ok)
            continue;
        if(locNr >= freeLoc)
            freeLoc = locNr + 1;
    }
    return freeLoc;
}

void TidyupTeachingGui::on_recordManipulationLocationBtn_clicked()
{
    unsigned int locNr = nextFreeLocationNumber();
    GeometryPoses::NamedPose np;
    std::stringstream ss;
    ss << "table" << std::fixed << std::setfill('0') << locNr <<
        "_loc" << locNr << "_" << qPrintable(currentRoomEdit->text());
    np.first = ss.str();
    if(estimateCurrentPose(np.second)) {
        //poses.push_back(np);
        recordedLocationsList->addItem(GeometryPoses::getPoseWriteString(np).c_str());
        QListWidgetItem* item = recordedLocationsList->item(recordedLocationsList->count() - 1);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Checked);
        showMessage(QString("Saved pose: %1").arg(np.first.c_str()));
        //QMessageBox::information(this, "Pose save", "Saved pose");
    } else {
        return;
    }

    if(detectTablesCk->isChecked()) {
        tidyup_msgs::DetectTable srv;
        if(!detect_table.call(srv)) {
            QMessageBox::critical(this, "Adding table failed",
                    QString("Service call to %1 failed.").arg(detect_table.getService().c_str()));
        } else {
            std::stringstream ss2;
            ss2 << "table" << std::fixed << std::setfill('0') << locNr;
            std::pair<std::string, tidyup_msgs::DetectTable::Response> tableEntry = std::make_pair(ss2.str(), srv.response);
            recordedTablesList->addItem(getTableWriteString(tableEntry).c_str());
            showMessage(QString("Saved table: %1").arg(tableEntry.first.c_str()));
        }
    }
}

void TidyupTeachingGui::on_recordTidyManipulationLocationBtn_clicked()
{
    int nPoses = recordedLocationsList->count();

    // add pose + table, new one is last
    on_recordManipulationLocationBtn_clicked();

    // no new entry
    if(recordedLocationsList->count() == nPoses)
        return;

    if(tidyLocationEdit->text() != "") {
        QMessageBox::critical(this, "Error adding tidy location",
                QString("A tidy location is already set at %1").arg(tidyLocationEdit->text()));
    } else {
        QListWidgetItem* item = recordedLocationsList->item(recordedLocationsList->count() - 1);
        if(item)
            tidyLocationEdit->setText(item->text().split(" ")[0]);
    }
}

void TidyupTeachingGui::on_recordDoorLocationBtn_clicked()
{
    bool ok = false;
    QString doorName = QInputDialog::getText(this, "What door does the robot stand at?",
            "Door Name", QLineEdit::Normal, last_door, &ok);
    if(!ok)
        return;
    if(doorName.length() == 0) {
        QMessageBox::critical(this, "Error adding door location",
                "Door name was empty");
        return;
    }
    if(doorName.contains("_")) {
        QMessageBox::critical(this, "Error adding door location",
                "Door names cannot contain '_'");
        return;
    }

    unsigned int locNr = nextFreeLocationNumber();
    GeometryPoses::NamedPose np;
    std::stringstream ss;
    ss << qPrintable(doorName) << std::fixed << std::setfill('0') <<
        "_loc" << locNr << "_" << qPrintable(currentRoomEdit->text());
    np.first = ss.str();
    if(estimateCurrentPose(np.second)) {
        //poses.push_back(np);
        recordedLocationsList->addItem(GeometryPoses::getPoseWriteString(np).c_str());
        QListWidgetItem* item = recordedLocationsList->item(recordedLocationsList->count() - 1);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Checked);
        showMessage(QString("Saved pose: %1").arg(np.first.c_str()));
        //QMessageBox::information(this, "Pose save", "Saved pose");
    }

    last_door = doorName;
}

void TidyupTeachingGui::recordedLocationsList_contextMenu(const QPoint & p)
{
    QMenu myMenu;
    QAction* removeAction = myMenu.addAction(QString("Remove Entry"));

    QAction* selectedAction = myMenu.exec(recordedLocationsList->mapToGlobal(p));
    if(selectedAction == removeAction) {
        QListWidgetItem* it = recordedLocationsList->itemAt(p);
        delete it;
    }
}

void TidyupTeachingGui::recordedTablesList_contextMenu(const QPoint & p)
{
    QMenu myMenu;
    QAction* tidyLocationAction = myMenu.addAction(QString("Set as tidy table"));
    QAction* removeAction = myMenu.addAction(QString("Remove Entry"));

    QAction* selectedAction = myMenu.exec(recordedTablesList->mapToGlobal(p));
    if(selectedAction == tidyLocationAction) {
        QListWidgetItem* it = recordedTablesList->itemAt(p);
        if(it) {
            QString locName = it->text().split(" ")[0];
            tidyLocationEdit->setText(locName);
        }
    } else if(selectedAction == removeAction) {
        QListWidgetItem* it = recordedTablesList->itemAt(p);
        delete it;
    }
}

std::string TidyupTeachingGui::getTableWriteString(
        const std::pair<std::string, tidyup_msgs::DetectTable::Response> & table)
{
    std::stringstream ss;
    ss << table.first << " ";
    ss << table.second.pose.header.stamp.sec << " " << table.second.pose.header.stamp.nsec << " ";
    ss << table.second.pose.header.frame_id << " ";
    ss << table.second.pose.pose.position.x << " ";
    ss << table.second.pose.pose.position.y << " ";
    ss << table.second.pose.pose.position.z << " ";
    ss << table.second.pose.pose.orientation.x << " ";
    ss << table.second.pose.pose.orientation.y << " ";
    ss << table.second.pose.pose.orientation.z << " ";
    ss << table.second.pose.pose.orientation.w << " ";
    ss << table.second.bounding_box_size.x << " ";
    ss << table.second.bounding_box_size.y << " ";
    ss << table.second.bounding_box_size.z; 
    return ss.str();
}

bool TidyupTeachingGui::writeLocations(QString filename)
{
    if(filename.size() <= 0) {
        QString fn = QFileDialog::getSaveFileName(this, "Select locations file");
        if(fn.length() < 1)
            return false;
        filename = fn;
    }

    std::ofstream of(qPrintable(filename));
    if(!of.good())
        return false;

    for(int i = 0; i < recordedLocationsList->count(); i++) {
    //for(std::vector<GeometryPoses::NamedPose>::iterator it = poses.begin(); it != poses.end(); it++) {
        QListWidgetItem* item = recordedLocationsList->item(i);
        if(item == NULL)
            continue;
        if(item->checkState() != Qt::Checked)
            of << "## ";
        GeometryPoses::NamedPose np = GeometryPoses::getPoseFromString(qPrintable(item->text()));
        of << GeometryPoses::getPoseWriteString(np) << std::endl;
    }

    if(of.good()) {
        QMessageBox::information(this, "Write file", "Locations file written.");
    } else {
        return false;
    }
    of.close();
    return true;
}

bool TidyupTeachingGui::writeTables(QString filename)
{
    if(filename.size() <= 0) {
        QString fn = QFileDialog::getSaveFileName(this, "Select tables file");
        if(fn.length() < 1)
            return false;
        filename = fn;
    }

    std::ofstream of(qPrintable(filename));
    if(!of.good())
        return false;

    of << "# tables id timestamp frame x y z qx qy qz qw sizex sizey sizez" << std::endl;
    for(int i = 0; i < recordedTablesList->count(); i++) {
        QListWidgetItem* item = recordedTablesList->item(i);
        if(item == NULL)
            continue;
        of << qPrintable(item->text()) << std::endl;
    }

    if(of.good()) {
        QMessageBox::information(this, "Write file", "Tables file written.");
    } else {
        return false;
    }

    of.close();
    return true;
}

bool TidyupTeachingGui::writeActions(QString actionsTemplate, QString actionsFile)
{
    if(tidyLocationEdit->text() == "") {
        QMessageBox::critical(this, "writeActions", "Tidy location not defined.");
        return false;
    }

    std::string templ_script = ros::package::getPath("tidyup_tools");
    if(templ_script.empty()) {
        QMessageBox::critical(this, "Error", "Could not find package tidyup_tools.");
        return false;
    }
    templ_script += "/scripts/create_actions_file.py";

    std::string templ_cmd = "python " + templ_script + " " + qPrintable(actionsTemplate)
        + " " + qPrintable(actionsFile) + " " + qPrintable(tidyLocationEdit->text());
    int ret = system(templ_cmd.c_str());
    if(ret != 0) {
        QMessageBox::critical(this, "Write Actions", QString("Could not create config dir using \"%1\". Error: %2").arg(templ_cmd.c_str()).arg(ret));
        return false;
    }
    return true;
}

//void TidyupTeachingGui::on_writeFilesBtn_clicked()
//{
//    if(!writeLocations(QString())) {
//        QMessageBox::critical(this, "Write Files", "Write Locations failed.");
//    }
//    if(!writeTables(QString())) {
//        QMessageBox::critical(this, "Write Files", "Write Tables failed.");
//    }
//
//    // write tidy location
//    ROS_DEBUG("Tidy location is: %s", qPrintable(tidyLocationEdit->text()));
//    // TODO write actions
//}

void TidyupTeachingGui::on_loadPackageBtn_clicked()
{
    bool clear = true;
    if(recordedLocationsList->count() > 0 || recordedTablesList->count() > 0) {
        clear = false;
        QMessageBox::StandardButton result = QMessageBox::question(this, "Load Package", "Locations or tables list is not empty. Should the lists be cleared before loading?",
                QMessageBox::Yes | QMessageBox::No);
        if(result == QMessageBox::Yes)
            clear = true;
    }
    if(clear) {
        recordedLocationsList->clear();
        recordedTablesList->clear();
    }
    QString packageDir = QFileDialog::getExistingDirectory(this,
            "Select configuration package directory", QString(), QFileDialog::ShowDirsOnly);

    if(packageDir.length() < 1)
        return;

    ROS_DEBUG("Chosen package dir: %s", qPrintable(packageDir));

    QString locationsFile = packageDir + "/config/planning/locations_tidyup.dat";

    QFile f(locationsFile);
    if(!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "Load Package", QString("Failed to open locations from %1.").arg(locationsFile));
        return;
    }
    QTextStream ins(&f);
    QString line = "";
    while(!line.isNull()) {
        line = ins.readLine();
        if(line.length() == 0)
            continue;
        bool active = true;
        if(line.startsWith("#")) {
            if(line.startsWith("## ")) {   // commented location, read but don't enable
                active = false;
                line = line.mid(3); // skip '## '
            } else
                continue;
        }
        recordedLocationsList->addItem(line);
        QListWidgetItem* item = recordedLocationsList->item(recordedLocationsList->count() - 1);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        if(active)
            item->setCheckState(Qt::Checked);
        else
            item->setCheckState(Qt::Unchecked);
    }

    QString tablesFile = packageDir + "/config/planning/tables_tidyup.dat";

    QFile ft(tablesFile);
    if(!ft.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "Load Package", QString("Failed to open tables from %1.").arg(tablesFile));
        return;
    }
    QTextStream inst(&ft);
    line = "";
    while(!line.isNull()) {
        line = inst.readLine();
        if(line.length() == 0)
            continue;
        if(line.startsWith("#")) {
            continue;
        }
        recordedTablesList->addItem(line);
    }
}

void TidyupTeachingGui::on_writePackageBtn_clicked()
{
    QString packageDir = QFileDialog::getExistingDirectory(this,
            "Select configuration package directory", QString(), QFileDialog::ShowDirsOnly);

    if(packageDir.length() < 1)
        return;

    ROS_DEBUG("Chosen package dir: %s", qPrintable(packageDir));

    std::string template_dir = ros::package::getPath("tidyup_tools");
    if(template_dir.empty()) {
        QMessageBox::critical(this, "Error", "Could not find package tidyup_tools.");
        return;
    }

    std::string mkdir_cmd = std::string("mkdir -p ") + qPrintable(packageDir) + "/config/planning";
    int ret = 0;
    ret = system(mkdir_cmd.c_str());
    if(ret != 0) {
        QMessageBox::critical(this, "Write Package", QString("Could not create config dir using \"%1\". Error: %2").arg(mkdir_cmd.c_str()).arg(ret));
    }
    mkdir_cmd = std::string("mkdir -p ") + qPrintable(packageDir) + "/config/mapping";
    ret = system(mkdir_cmd.c_str());
    if(ret != 0) {
        QMessageBox::critical(this, "Write Package", QString("Could not create config dir using \"%1\". Error: %2").arg(mkdir_cmd.c_str()).arg(ret));
    }

    QString locationsFile = packageDir + "/config/planning/locations_tidyup.dat";
    if(!writeLocations(locationsFile)) {
        QMessageBox::critical(this, "Write Package", QString("Write Locations to %1 failed.").arg(locationsFile));
    }
    QString tablesFile = packageDir + "/config/planning/tables_tidyup.dat";
    if(!writeTables(tablesFile)) {
        QMessageBox::critical(this, "Write Package", QString("Write Tables to %1 failed.").arg(tablesFile));
    }

    QString actionsTemplate = template_dir.c_str();
    actionsTemplate += "/tidyup_config_template/planning/tidyup_actions.yaml";
    QString actionsFile = packageDir + "/config/planning/tidyup_actions.yaml";

    if(!writeActions(actionsTemplate, actionsFile)) {
        QMessageBox::critical(this, "Write Package", QString("Write Actions from %1 to %2 failed.").arg(actionsTemplate).arg(actionsFile));
    }

    // Read map name from target config dir
    // create collision space config
    QString map_file = packageDir + "/maps/octomap-tidyup.bt";
    QString sbplTemplate1 = QString(template_dir.c_str()) + "/tidyup_config_template/mapping/pr2_both_arms_tidyup.yaml.1";
    QString sbplTemplate2 = QString(template_dir.c_str()) + "/tidyup_config_template/mapping/pr2_both_arms_tidyup.yaml.2";
    QString sbplFile = packageDir + "/config/mapping/pr2_both_arms_tidyup.yaml";

    // first create the config snippet
    QString get_collision_map_cmd = QString("rosrun tidyup_tools octree_to_collision_space ") + map_file + " > /tmp/col_space_template";
    ret = system(qPrintable(get_collision_map_cmd));
    if(ret != 0) {
        QMessageBox::critical(this, "Write Package", QString("Determine Collision Space for %1 to /tmp/col_space_template failed.").arg(map_file));
    }

    // next put it between the two parts of the config
    QString set_collision_map_cmd = "cat " + sbplTemplate1 + " /tmp/col_space_template " + sbplTemplate2 + " > " + sbplFile;
    ret = system(qPrintable(set_collision_map_cmd));
    if(ret != 0) {
        QMessageBox::critical(this, "Write Package", QString("Write Collision Space from %1 using %2(/.2) to %3 failed.").arg("/tmp/col_space_template").arg(sbplTemplate1).arg(sbplFile));
    }
}

