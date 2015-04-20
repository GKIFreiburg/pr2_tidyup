#ifndef GRASP_PROVIDER_GUI_H
#define GRASP_PROVIDER_GUI_H

#include <QMainWindow>
#include <QAbstractTableModel>
#include <QItemDelegate>
#include "ui_grasp_provider_gui.h"
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include "grasp_provider/grasp_provider_storage.h"

namespace grasp_provider
{

class CollisionObjectsModel;
class GraspProvidersModel;
class GraspProviderDelegate;

class GraspProviderGui : public QMainWindow, protected Ui::MainWindow
{
    Q_OBJECT

    public:
        GraspProviderGui();

    protected Q_SLOTS:
        void on_refreshObjectsBtn_clicked();
        void on_refreshGraspProvidersBtn_clicked();

        void on_createGraspProviderBtn_clicked();
        void on_deleteGraspProviderBtn_clicked();

        void objectsList_selectionChanged(const QItemSelection & selected, const QItemSelection & old);

    protected:
        ros::ServiceClient srvGetPlanningScene_;
        CollisionObjectsModel* collision_objects_model_;
        std::vector<moveit_msgs::CollisionObject> collision_objects_;

        GraspProviderStorage grasp_provider_storage_;
        GraspProvidersModel* grasp_providers_model_;
        std::vector<GraspProviderStorageEntry> grasp_providers_;
        GraspProviderDelegate* grasp_providers_delegate_;
};

class CollisionObjectsModel : public QAbstractTableModel
{
    Q_OBJECT

    public:
        CollisionObjectsModel(const std::vector<moveit_msgs::CollisionObject> & objects, QObject* parent = NULL);

        int rowCount(const QModelIndex & parent = QModelIndex()) const;
        int columnCount(const QModelIndex & parent = QModelIndex()) const;
        QVariant data(const QModelIndex & index, int role) const;
        QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;

    protected:
        const std::vector<moveit_msgs::CollisionObject> & collision_objects_;


};

class GraspProvidersModel : public QAbstractTableModel
{
    Q_OBJECT

    public:
        GraspProvidersModel(std::vector<GraspProviderStorageEntry> & grasp_providers,
                GraspProviderStorage* grasp_provider_storage,
                QObject* parent = NULL);

        int rowCount(const QModelIndex & parent = QModelIndex()) const;
        int columnCount(const QModelIndex & parent = QModelIndex()) const;
        QVariant data(const QModelIndex & index, int role) const;
        QVariant headerData(int section, Qt::Orientation orientation, int role = Qt::DisplayRole) const;

        Qt::ItemFlags flags(const QModelIndex & index) const;
        bool setData(const QModelIndex & index, const QVariant & value, int role = Qt::EditRole);

    protected:
        std::vector<GraspProviderStorageEntry> & grasp_providers_;
        GraspProviderStorage* grasp_provider_storage_;
};

class GraspProviderDelegate : public QItemDelegate
{
    Q_OBJECT

    public:
        GraspProviderDelegate(QObject* parent = 0);

        QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem & option,
                const QModelIndex & index) const;

        void setEditorData(QWidget* editor, const QModelIndex & index) const;
        void setModelData(QWidget* editor, QAbstractItemModel* model,
                const QModelIndex & index) const;

        void updateEditorGeometry(QWidget* editor,
                const QStyleOptionViewItem & option, const QModelIndex & index) const;
};

}

#endif

