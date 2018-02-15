#ifndef SERVOTABLE_HH
#define SERVOTABLE_HH

#include <QAbstractTableModel>
#include "dogbot/DogBotAPI.hh"
#include "dogbot/Servo.hh"
#include <functional>
#include <assert.h>
#include <mutex>

class ServoTable
  : public QAbstractTableModel
{
  Q_OBJECT
public:
  //! Construct from an API
  ServoTable(const std::shared_ptr<DogBotN::DogBotAPIC> &api);

  //! Destructor, disconnect from API.
  ~ServoTable();

  enum Column {
    ColumnDeviceId = 0,
    ColumnName = 1,
    ColumnStatus = 2,
    ColumnMode = 3,
    ColumnDynamic = 4,
    ColumnHomed = 5,
    ColumnAngle = 6,
    ColumnSpeed = 7,
    ColumnTorque = 8,
    ColumnDriveTemperature = 9,
    ColumnMotorTemperature = 10,
    ColumnSupplyVoltage = 11,
    ColumnIndex = 12,
    ColumnNotes = 13,
    ColumnCount = 14
  };

  Qt::ItemFlags flags(const QModelIndex &index) const override;

  int columnCount(const QModelIndex &parent) const override;
  int rowCount(const QModelIndex &parent) const override;

  QVariant data(const QModelIndex &index, int role) const override;
  bool setData(const QModelIndex &index, const QVariant &value, int role) override;
  QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

signals:
  void dataUpdated();

private slots:
  //! Queue an update for a row
  void queueDataUpdate();

protected:
  DogBotN::JointC *Row2Joint(int row);

  //! Do any pending updates.
  void updateRows();

  std::shared_ptr<DogBotN::DogBotAPIC> m_api;
  DogBotN::CallbackSetC m_callbacks;
  std::map<DogBotN::JointC *,int> m_joint2row;
  std::map<int,DogBotN::JointC *> m_row2joint;

  int m_rowCount = 0;
  std::mutex m_mutexUpdateRows;
  bool m_updateQueued = false;
  std::vector<bool> m_updateRows;
};

#endif // DEVICETABLE_HH
