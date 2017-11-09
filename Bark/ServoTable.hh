#ifndef SERVOTABLE_HH
#define SERVOTABLE_HH

#include <QAbstractTableModel>

class ServoTable
  : public QAbstractTableModel
{
public:
  ServoTable();

  enum Column {
    ColumnDeviceId = 0,
    ColumnName = 1,
    ColumnStatus = 2,
    ColumnCalibrated = 3,
    ColumnAngle = 4,
    ColumnSpeed = 5,
    ColumnTorque = 6,
    ColumnTemperature = 7,
    ColumnCount = 8
  };

  Qt::ItemFlags flags(const QModelIndex &index) const override;

  int columnCount(const QModelIndex &parent) const override;
  int rowCount(const QModelIndex &parent) const override;

  //QVariant data(const QModelIndex &index, int role) const override;
  //bool setData(const QModelIndex &index, const QVariant &value, int role) override;
  //QVariant headerData(int section, Qt::Orientation orientation, int role) const override;

protected:
  //std::shared_ptr<DogBotAPIC> m_api;
};

#endif // DEVICETABLE_HH
