#include "ServoTable.hh"

ServoTable::ServoTable(const std::shared_ptr<DogBotN::DogBotAPIC> &api)
 : m_api(api)
{

}

Qt::ItemFlags ServoTable::flags(const QModelIndex &index) const
{
  if (!index.isValid())
      return 0;
  //if(index.row() >= m_layers.size())
 //   return QAbstractItemModel::flags(index);
  return QAbstractItemModel::flags(index);
}

int ServoTable::columnCount(const QModelIndex &parent) const
{
  return ColumnCount;
}

int ServoTable::rowCount(const QModelIndex &parent) const
{
  return m_api->ListServos().size();
}

QVariant ServoTable::data(const QModelIndex &index, int role) const
{
  const std::vector<std::shared_ptr<DogBotN::ServoC> > &servoList = m_api->ListServos();
  if(index.row() < 0 || index.row() >= servoList.size() )
    return QVariant();
  if(!servoList[index.row()])
    return QVariant(); // Doesn't seem to exist.
  const DogBotN::ServoC &servo = *servoList[index.row()];
  if (role == Qt::DisplayRole) {
    switch(index.column())
    {
    case  ColumnDeviceId:
      return servo.Id();
    case ColumnName:
      return servo.Name().c_str();
    case ColumnStatus:
      return DogBotN::FaultCodeToString(servo.FaultCode());
#if 0
    case ColumnCalibrated:
      break;
    ColumnAngle = 4,
    ColumnSpeed = 5,
    ColumnTorque = 6,
    ColumnTemperature = 7,
    ColumnCount = 8
#endif

    }
  }

  return QVariant();
}
