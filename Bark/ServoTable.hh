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
    ColumnTemperature = 9,
    ColumnSupplyVoltage = 10,
    ColumnIndex = 11,
    ColumnNotes = 12,
    ColumnCount = 13
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
  //! Do any pending updates.
  void updateRows();

  std::shared_ptr<DogBotN::DogBotAPIC> m_api;
  int m_statusCallbackId = -1;

  int m_rowCount = 0;
  std::mutex m_mutexUpdateRows;
  bool m_updateQueued = false;
  std::vector<bool> m_updateRows;
};

#endif // DEVICETABLE_HH
