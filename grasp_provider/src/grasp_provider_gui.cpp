#include "grasp_provider_gui.h"
#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetPlanningScene.h>
#include "grasp_provider_msgs/grasp_provider_msgs.h"
#include <QItemSelectionModel>
#include <QMessageBox>
#include <QComboBox>
#include <QLineEdit>

namespace grasp_provider
{

GraspProviderGui::GraspProviderGui() : collision_objects_model_(NULL), grasp_providers_model_(NULL)
{
    setupUi(this);

    ros::NodeHandle nh;

    srvGetPlanningScene_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);

    ROS_INFO("Waiting for %s service.", move_group::GET_PLANNING_SCENE_SERVICE_NAME.c_str());
    srvGetPlanningScene_.waitForExistence();

    on_refreshObjectsBtn_clicked();
    on_refreshGraspProvidersBtn_clicked();

    grasp_providers_delegate_ = new GraspProviderDelegate(graspProvidersList);
    graspProvidersList->setItemDelegate(grasp_providers_delegate_);
}

void GraspProviderGui::on_refreshObjectsBtn_clicked()
{
    ROS_INFO("Refreshing objects");
    delete collision_objects_model_;
    collision_objects_model_ = NULL;
    moveit_msgs::GetPlanningScene::Request request;
    moveit_msgs::GetPlanningScene::Response response;
    request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
    if (!srvGetPlanningScene_.call(request, response)) {
        ROS_ERROR("%s: planning scene request failed.", __func__);
        return;
    }
    collision_objects_ = response.scene.world.collision_objects;
    collision_objects_model_ = new CollisionObjectsModel(collision_objects_);
    objectsList->setModel(collision_objects_model_);
    connect(objectsList->selectionModel(), SIGNAL(selectionChanged(const QItemSelection &, const QItemSelection &)),
            this, SLOT(objectsList_selectionChanged(const QItemSelection &, const QItemSelection &)));

    // trigger selection changed handler to current selection
    objectsList_selectionChanged(objectsList->selectionModel()->selection(),
            objectsList->selectionModel()->selection());
}

void GraspProviderGui::on_refreshGraspProvidersBtn_clicked()
{
    ROS_INFO("Refreshing Grasp Providers");
    delete grasp_providers_model_;
    grasp_providers_ = grasp_provider_storage_.getAllGraspProviders();
    grasp_providers_model_ = new GraspProvidersModel(grasp_providers_, &grasp_provider_storage_);
    graspProvidersList->setModel(grasp_providers_model_);
}

void GraspProviderGui::on_createGraspProviderBtn_clicked()
{
    ROS_INFO("Creating GraspProvider");
    // get selected from objectsList
    QModelIndexList selectedItems = objectsList->selectionModel()->selectedIndexes();
    if(selectedItems.size() == 0) {
        QMessageBox::warning(this, "Create GraspProvider", "Select an object to create a GraspProvider for.");
        return;
    } else if(selectedItems.size() > 3) {   // row selection with 3 cols = single row
        QMessageBox::warning(this, "Create GraspProvider", "More than one object selected.");
        return;
    }
    int index = selectedItems.front().row();
    if(index < 0 || index >= collision_objects_.size()) // shouldnt happen
        return;

    // add to db from key with dummy entries
    const moveit_msgs::CollisionObject & co = collision_objects_.at(selectedItems.front().row());
    grasp_provider_msgs::GraspProvider gp;
    gp.provider_name = "moveit_simple_grasps";
    gp.shape_type = 0;
    gp.grasp_types = 0;
    grasp_provider_storage_.addGraspProvider(gp, co.type);

    // refresh GPs to get the new one in the list
    on_refreshGraspProvidersBtn_clicked();

    // retrigger objectsList_selectionChanged, although the selection didnt change
    // the grasp_providers_ did, so the new one can be automatically selected
    objectsList_selectionChanged(objectsList->selectionModel()->selection(),
            objectsList->selectionModel()->selection());
}

void GraspProviderGui::on_deleteGraspProviderBtn_clicked()
{
    ROS_INFO("Deleting GraspProvider");

    // get selected from graspProvidersList
    QModelIndexList selectedItems = graspProvidersList->selectionModel()->selectedIndexes();
    if(selectedItems.size() == 0) {
        QMessageBox::warning(this, "Delete GraspProvider", "Select a GraspProvider to delete.");
        return;
    } else if(selectedItems.size() > 5) {   // row selection with 5 cols = single row
        QMessageBox::warning(this, "Delete GraspProvider", "More than one GraspProvider selected.");
        return;
    }
    int index = selectedItems.front().row();
    if(index < 0 || index >= grasp_providers_.size()) // shouldnt happen
        return;

    // remove from db
    grasp_provider_storage_.removeGraspProvider(grasp_providers_.at(index).first);

    // refresh from db
    on_refreshGraspProvidersBtn_clicked();

    // retrigger this as we might have removed the gp for something
    objectsList_selectionChanged(objectsList->selectionModel()->selection(),
            objectsList->selectionModel()->selection());
}

void GraspProviderGui::objectsList_selectionChanged(const QItemSelection & selected, const QItemSelection & old)
{
    QAbstractItemModel* gpm = graspProvidersList->model();
    QAbstractItemModel* om = objectsList->model();
    if(gpm == NULL || om == NULL)
        return;

    // by default we assume none match:
    // deselect all GPs and disable the create button
    // This will be changed if we find a match below
    QItemSelection all(gpm->index(0, 0, QModelIndex()), gpm->index(gpm->rowCount() - 1, 4, QModelIndex()));
    graspProvidersList->selectionModel()->select(all, QItemSelectionModel::Clear);
    createGraspProviderBtn->setEnabled(false);

    QModelIndexList selectedItems = selected.indexes();
    if(selectedItems.size() == 0) {
        return;
    } else if(selectedItems.size() > 3) {   // row selection with 3 cols = single row
        return;
    }
    int index = selectedItems.front().row();
    if(index < 0 || index >= collision_objects_.size()) {   // shouldnt happen
        return;
    }

    // single valid index - check match
    // enable create for now unless we find a match
    createGraspProviderBtn->setEnabled(true);

    QVariant key = om->data(om->index(index, 1), Qt::DisplayRole);
    QVariant db = om->data(om->index(index, 2), Qt::DisplayRole);
    for(int row = 0; row < gpm->rowCount(); row++) {
        QVariant gp_key = gpm->data(gpm->index(row, 0), Qt::DisplayRole);
        QVariant gp_db = gpm->data(gpm->index(row, 1), Qt::DisplayRole);
        if(key == gp_key && db == gp_db) {
            // select match
            graspProvidersList->selectionModel()->select(gpm->index(row, 0, QModelIndex()),
                    QItemSelectionModel::Select | QItemSelectionModel::Rows);
            // disable the create button - this one exists and should only be edited
            createGraspProviderBtn->setEnabled(false);
            break;
        }
    }
}


CollisionObjectsModel::CollisionObjectsModel(const std::vector<moveit_msgs::CollisionObject> & objects,
        QObject* parent) : QAbstractTableModel(parent), collision_objects_(objects)
{
}

int CollisionObjectsModel::rowCount(const QModelIndex & parent) const
{
    return collision_objects_.size();
}

int CollisionObjectsModel::columnCount(const QModelIndex & parent) const
{
    return 3;
}

QVariant CollisionObjectsModel::data(const QModelIndex & index, int role) const
{
    if(!index.isValid())
        return QVariant();

    if(index.row() >= collision_objects_.size())
        return QVariant();
    if(index.column() >= 3)
        return QVariant();

    if(role != Qt::DisplayRole)
        return QVariant();

    const moveit_msgs::CollisionObject & co = collision_objects_.at(index.row());
    switch(index.column()) {
        case 0:
            return QString(co.id.c_str());
            break;
        case 1:
            return QString(co.type.key.c_str());
            break;
        case 2:
            return QString(co.type.db.c_str());
            break;
        default:
            return QVariant();
            break;
    }
    return QVariant();
}

QVariant CollisionObjectsModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole)
        return QVariant();

    if (orientation == Qt::Horizontal) {
        switch(section) {
            case 0:
                return QString("Id");
                break;
            case 1:
                return QString("Key");
                break;
            case 2:
                return QString("Db");
                break;
        }
    }
    return QVariant();
}

GraspProvidersModel::GraspProvidersModel(std::vector<GraspProviderStorageEntry> & grasp_providers,
        GraspProviderStorage* grasp_provider_storage,
        QObject* parent) : QAbstractTableModel(parent),
    grasp_providers_(grasp_providers), grasp_provider_storage_(grasp_provider_storage)
{
}

int GraspProvidersModel::rowCount(const QModelIndex & parent) const
{
    return grasp_providers_.size();
}

int GraspProvidersModel::columnCount(const QModelIndex & parent) const
{
    // key, db -> name, shape, grasps
    return 5;
}

QVariant GraspProvidersModel::data(const QModelIndex & index, int role) const
{
    if(!index.isValid())
        return QVariant();

    if(index.row() >= grasp_providers_.size())
        return QVariant();
    if(index.column() >= 5)
        return QVariant();

    if(role != Qt::DisplayRole && role != Qt::EditRole)
        return QVariant();

    const GraspProviderStorageEntry & se = grasp_providers_.at(index.row());
    switch(index.column()) {
        case 0:
            return QString(se.first.key.c_str());
            break;
        case 1:
            return QString(se.first.db.c_str());
            break;
        case 2:
            return QString(se.second.provider_name.c_str());
            break;
        case 3:
            if(role == Qt::DisplayRole)
                return QString(grasp_provider_msgs::shapeTypeToString(se.second.shape_type).c_str());
            else
                return QVariant(se.second.shape_type);
            break;
        case 4:
            if(role == Qt::DisplayRole)
                return QString(grasp_provider_msgs::graspTypesToString(se.second.grasp_types).c_str());
            else
                return QVariant(static_cast<int>(se.second.grasp_types));
            break;
        default:
            return QVariant();
            break;
    }
    return QVariant();
}

Qt::ItemFlags GraspProvidersModel::flags(const QModelIndex & index) const
{
    if(!index.isValid())
        return Qt::ItemIsEnabled;

    // cant edit key/db entries
    if(index.column() >= 2 && index.column() <= 4)
        return QAbstractTableModel::flags(index) | Qt::ItemIsEditable;

    return QAbstractTableModel::flags(index);
}

bool GraspProvidersModel::setData(const QModelIndex & index, const QVariant & value, int role)
{
    if(index.isValid() && role == Qt::EditRole) {
        // get the entry at index
        ROS_ASSERT(index.row() >= 0 && index.row() < grasp_providers_.size());
        GraspProviderStorageEntry & gpe = grasp_providers_.at(index.row());
        switch(index.column()) {
            case 2:
                gpe.second.provider_name = qPrintable(value.toString());
                break;
            case 3:
                gpe.second.shape_type = value.toInt();
                break;
            case 4:
                gpe.second.grasp_types = value.toUInt();
                break;
        }
        grasp_provider_storage_->addGraspProvider(gpe.second, gpe.first);   // will replace with updated vals

        Q_EMIT dataChanged(index, index);
        return true;
    }
    return false;
}

QVariant GraspProvidersModel::headerData(int section, Qt::Orientation orientation, int role) const
{
    if (role != Qt::DisplayRole)
        return QVariant();

    if (orientation == Qt::Horizontal) {
        switch(section) {
            case 0:
                return QString("Key");
                break;
            case 1:
                return QString("Db");
                break;
            case 2:
                return QString("Provider");
                break;
            case 3:
                return QString("Shape");
                break;
            case 4:
                return QString("Grasp Types");
                break;
        }
    }
    return QVariant();
}


GraspProviderDelegate::GraspProviderDelegate(QObject* parent) : QItemDelegate(parent)
{
}

QWidget* GraspProviderDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem & option,
        const QModelIndex & index) const
{
    if(index.column() == 2) {
        QLineEdit* edit = new QLineEdit(parent);
        return edit;
    }

    QComboBox* edit = new QComboBox(parent);
    switch(index.column()) {
        case 3:
            {
                std::vector<int> shape_types = grasp_provider_msgs::getAllShapeTypes();
                for(std::vector<int>::iterator it = shape_types.begin(); it != shape_types.end(); ++it) {
                    edit->addItem(grasp_provider_msgs::shapeTypeToString(*it).c_str(), *it);
                }
                break;
            }
        case 4:
            {
                std::vector<unsigned char> grasp_types = grasp_provider_msgs::getAllGraspTypes();
                for(std::vector<unsigned char>::iterator it = grasp_types.begin(); it != grasp_types.end(); ++it) {
                    edit->addItem(grasp_provider_msgs::graspTypesToString(*it).c_str(), static_cast<int>(*it));
                }
                break;
            }
    }

    return edit;
}

void GraspProviderDelegate::setEditorData(QWidget* editor, const QModelIndex & index) const
{
    if(index.column() == 2) {
        QLineEdit* line = static_cast<QLineEdit*>(editor);
        line->setText(index.model()->data(index, Qt::EditRole).toString());
        return;
    }
    // select item in combo with matching user data to type
    QComboBox* combo = static_cast<QComboBox*>(editor);
    QVariant type = index.model()->data(index, Qt::EditRole);
    int dataIdx = combo->findData(type);
    combo->setCurrentIndex(dataIdx);
}

void GraspProviderDelegate::setModelData(QWidget* editor, QAbstractItemModel* model,
        const QModelIndex & index) const
{
    if(index.column() == 2) {
        QLineEdit* line = static_cast<QLineEdit*>(editor);
        model->setData(index, line->text(), Qt::EditRole);
        return;
    }
    // get from combo and set
    QComboBox* combo = static_cast<QComboBox*>(editor);
    if(combo->currentIndex() >= 0) {
        QVariant selected = combo->itemData(combo->currentIndex());
        model->setData(index, selected, Qt::EditRole);
    }
}

void GraspProviderDelegate::updateEditorGeometry(QWidget* editor,
        const QStyleOptionViewItem & option, const QModelIndex & index) const
{
    editor->setGeometry(option.rect);
}

}

