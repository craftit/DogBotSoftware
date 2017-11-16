#include "ServoTable.hh"
#include <QColor>
#include <QTimer>

ServoTable::ServoTable(const std::shared_ptr<DogBotN::DogBotAPIC> &api)
 : m_api(api)
{
  m_statusCallbackId = m_api->AddServoStatusHandler(
          [this](int deviceId,DogBotN::DogBotAPIC::ServoUpdateTypeT opType) mutable {
            // Can't handle updates from devices without an ID.
            if(deviceId == 0)
              return ;
            int rowId = deviceId -1;
            std::lock_guard<std::mutex> lock(m_mutexUpdateRows);

            while(m_updateRows.size() <= rowId)
              m_updateRows.push_back(false);
            m_updateRows[rowId] = true;
            if(!m_updateQueued) {
              m_updateQueued = true;
              emit dataUpdated();
            }
          }
        );

  // Use a signal to call on a Qt thread.
  connect(this,SIGNAL(dataUpdated()),this,SLOT(queueDataUpdate()));

}

//! Destructor, disconnect from API.
ServoTable::~ServoTable()
{
  if(m_statusCallbackId >= 0) {
    m_api->RemoveServoStatusHandler(m_statusCallbackId);
    m_statusCallbackId = -1;
  }
}

//! Queue an update for a row
void ServoTable::queueDataUpdate()
{
  // Delay update by 100ms and accumulate any changes. This
  // limits the rate at which the display is updated.
  QTimer::singleShot(100, this,[this](){
    updateRows();
  });
}

void ServoTable::updateRows()
{
  std::vector<int> rowChanged;
  rowChanged.reserve(32);
  int newRowCount = 0;
  {
    std::lock_guard<std::mutex> lock(m_mutexUpdateRows);
    m_updateQueued = false;
    for(int i = 0;i < m_updateRows.size();i++) {
      if(m_updateRows[i]) {
        m_updateRows[i] = false;
        rowChanged.push_back(i);
      }
    }
    newRowCount = m_updateRows.size();
  }
  // Inserted a new device ?
  if(newRowCount > m_rowCount) {
    QModelIndex parent;
    beginInsertRows(parent,m_rowCount,newRowCount-1);
    // The data is there already...
    m_rowCount = newRowCount;
    endInsertRows();
  }
  for(auto a : rowChanged) {
    // Update existing device.
    QModelIndex from = createIndex(a,ColumnName);
    QModelIndex to = createIndex(a,ColumnTemperature);
    emit dataChanged(from,to);
  }
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
  return m_rowCount;
}

QVariant ServoTable::headerData(int section, Qt::Orientation orientation, int role) const
{
  if (orientation != Qt::Horizontal)
      return QVariant();

  if (role == Qt::DisplayRole) {
    switch (section) {
    case ColumnDeviceId:
      return tr("Id");
    case ColumnName:
      return tr("Name");
    case ColumnStatus:
      return tr("Status");
    case ColumnMode:
      return tr("Mode");
    case ColumnCalibrated:
      return tr("Cal");
    case ColumnAngle:
      return tr("Angle");
    case ColumnSpeed:
      return tr("Speed");
    case ColumnTorque:
      return tr("Torque");
    case ColumnTemperature:
      return tr("Temperature");
    case ColumnSupplyVoltage:
      return tr("Supply Voltage");
    default:
        break;
    }
  }

  return QVariant();
}

QVariant ServoTable::data(const QModelIndex &index, int role) const
{
  const std::vector<std::shared_ptr<DogBotN::ServoC> > &servoList = m_api->ListServos();
  int deviceId = index.row()+1;
  if(deviceId < 0 || deviceId >= (int) servoList.size() )
    return QVariant();
  if(!servoList[deviceId])
    return QVariant(); // Doesn't seem to exist.
  const DogBotN::ServoC &servo = *servoList[deviceId];
  if (role == Qt::DisplayRole) {
    switch(index.column())
    {
    case ColumnDeviceId:
      return servo.Id();
    case ColumnName:
      return servo.Name().c_str();
    case ColumnStatus:
      return DogBotN::FaultCodeToString(servo.FaultCode());
    case ColumnMode:
      return DogBotN::ControlStateToString(servo.ControlState());
    case ColumnCalibrated:
      return DogBotN::CalibrationStateToString(servo.CalibrationState());
    case ColumnAngle:
      return servo.Position();
    case ColumnSpeed:
      return servo.Speed();
    case ColumnTorque:
      return servo.Torque();
    case ColumnTemperature:
      return servo.Temperature();
    case ColumnSupplyVoltage:
      return servo.SupplyVoltage();
    default:
      break;
    }
  }  
  if (role == Qt::BackgroundRole) {
    switch(index.column())
    {
    case ColumnStatus:
      if(servo.FaultCode() == FC_Ok)
        return QColor(Qt::green);
      else
        return QColor(Qt::red);
    case ColumnTemperature:
      if(servo.Temperature() < 60.0)
        return QColor(Qt::white);
      if(servo.Temperature() < 70.0)
        return QColor(Qt::yellow);
      return QColor(Qt::red);
    case ColumnSupplyVoltage:
      if(servo.SupplyVoltage() < 8.0)
        return QColor(Qt::red);
      if(servo.SupplyVoltage() < 11.0)
        return QColor(Qt::yellow);
      if(servo.SupplyVoltage() > 40.0)
        return QColor(Qt::red);
      return QColor(Qt::white);
    default:
      break;
    }
  }

  return QVariant();
}
