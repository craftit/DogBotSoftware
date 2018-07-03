#include "ServoTable.hh"
#include <QColor>
#include <QTimer>

ServoTable::ServoTable(const std::shared_ptr<DogBotN::DogBotAPIC> &api)
 : m_api(api)
{
  m_callbacks += m_api->AddServoStatusHandler(
        [this](DogBotN::JointC *device,DogBotN::DogBotAPIC::ServoUpdateTypeT opType) mutable {
          if(device == 0)
            return ;
          std::lock_guard<std::mutex> lock(m_mutexUpdateRows);
          auto it = m_joint2row.find(device);
          int rowId = -1;
          if(it == m_joint2row.end()) {
            //  std::cerr << "New joint " << device->Name() << " Type:"  << device->JointType() << " " << std::endl;
            rowId = m_updateRows.size();
            m_updateRows.push_back(false);
            m_joint2row[device] = rowId;
            m_row2joint[rowId] = device;
          } else {
            rowId = it->second;
          }
          assert(rowId < (int) m_updateRows.size());
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
  m_callbacks.RemoveAll();
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
    for(int i = 0;i < (int) m_updateRows.size();i++) {
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
    QModelIndex to = createIndex(a,ColumnMotorTemperature);
    emit dataChanged(from,to);
  }
}


Qt::ItemFlags ServoTable::flags(const QModelIndex &index) const
{
  if(!index.isValid())
      return QAbstractItemModel::flags(index);
  if(index.row() >= m_rowCount)
    return QAbstractItemModel::flags(index);
  switch (index.column()) {
  case ColumnName:
    return Qt::ItemIsEditable |  QAbstractItemModel::flags(index);
  case ColumnNotes:
    return Qt::ItemIsEditable |  QAbstractItemModel::flags(index);
  default:
    break;
  }
  return QAbstractItemModel::flags(index);

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
    case ColumnDynamic:
      return tr("Dynamic");
    case ColumnHomed:
      return tr("Homed");
    case ColumnAngle:
      return tr("Angle");
    case ColumnSpeed:
      return tr("Speed");
    case ColumnTorque:
      return tr("Torque");
    case ColumnDriveTemperature:
      return tr("Drive Temp.");
    case ColumnMotorTemperature:
      return tr("Motor Temp.");
    case ColumnSupplyVoltage:
      return tr("Supply");
    case ColumnFlags:
      return tr("Flags");
    case ColumnNotes:
      return tr("Notes");
    default:
        break;
    }
  }

  return QVariant();
}

QVariant ServoTable::data(const QModelIndex &index, int role) const
{
  auto it = m_row2joint.find(index.row());
  if(it == m_row2joint.end()) {
    return QVariant(); // Doesn't seem to exist.
  }
  const DogBotN::JointC *joint = it->second;
  const DogBotN::ServoC *servo = dynamic_cast<const DogBotN::ServoC *>(joint);

  if (role == Qt::DisplayRole) {
    switch(index.column())
    {
    case ColumnDeviceId:
      if(servo != 0)
        return servo->Id();
      return "?";
    case ColumnName:
      return joint->Name().c_str();
    case ColumnStatus:
      if(servo == 0)
        return "";
      if(servo->ControlState() == CS_BootLoader) {
        return "Waiting";
      }
      if(!servo->IsPresent()) {
        return "Disconnected";
      }
      if(!servo->IsFirmwareVersionOk()) {
        return "Firmware Mismatch";
      }
      if(!servo->IsEnabled()) {
        return "Disabled";
      }
      return DogBotN::FaultCodeToString(servo->FaultCode());
    case ColumnMode:
      if(servo != 0)
        return DogBotN::ControlStateToString(servo->ControlState());
      return "";
    case ColumnDynamic:
      if(servo != 0)
        return DogBotN::ControlDynamicToString(servo->ControlDynamic());
      return "";
    case ColumnHomed:
      if(servo != 0)
        return DogBotN::HomedStateToString(servo->HomedState());
      return "";
    case ColumnAngle: {
      double posDeg =  joint->Position() * 180.0 / M_PI;
      QString value;
      value.setNum(posDeg,'f',2);
      if(servo != 0) {
        if(servo->PositionReference() != PR_Absolute) {
          return QString("(") + value + ")";
        } else {
          // If we're not homed and the last reported position
          // was absolute we don't know where we are.
          if(servo->HomedState() != MHS_Homed)
            return "";
        }
      }
      return value;
    }
    case ColumnSpeed: {
      QString numStr;
      numStr.setNum(joint->Velocity(),'f',2);
      return numStr;
    }
    case ColumnTorque: {
      QString numStr;
      numStr.setNum(joint->Torque(),'f',2);
      return numStr;
    }
    case ColumnDriveTemperature:
      if(servo != 0) {
        QString numStr;
        numStr.setNum(servo->DriveTemperature(),'f',1);
        return numStr;
      }
      return "";
    case ColumnMotorTemperature:
      if(servo != 0) {
        float motorTemp = servo->MotorTemperature();
        if(motorTemp < 10)
          return 0;
        QString numStr;
        numStr.setNum(servo->MotorTemperature(),'f',1);
        return numStr;
      }
      return "";
    case ColumnSupplyVoltage:
      if(servo != 0) {
        QString numStr;
        numStr.setNum(servo->SupplyVoltage(),'f',1);
        return numStr;
      }
      return "";
    case ColumnFlags:
      if(servo != 0) {
        QString ret;
        uint8_t val = servo->IndexState();

        if(val & DOGBOT_SERVOREPORTMODE_INDEXSENSOR) {
          ret += "I1";
        } else {
          ret += "I0";
        }
        if(val & DOGBOT_SERVOREPORTMODE_EMERGENCYSTOP) {
          ret += " ES";
        }
        if(val & DOGBOT_SERVOREPORTMODE_LIMITVELOCITY) {
          ret += " VL";
        }
        if(val & DOGBOT_SERVOREPORTMODE_LIMITTORQUE) {
          ret += " TL";
        }
        if(val & DOGBOT_SERVOREPORTMODE_LIMITPOSITION) {
          ret += " PL";
        }
        return ret;
      }
      return "";
    case ColumnNotes:
      if(servo)
        return servo->Notes().c_str();
      return "";
    default:
      break;
    }
  }  
  if (role == Qt::BackgroundRole) {
    switch(index.column())
    {
    case ColumnStatus:
      if(servo == 0)
        return QColor(Qt::white);
      if(servo->ControlState() == CS_BootLoader) {
        return QColor(Qt::yellow);
      }
      if(!servo->IsPresent() || !servo->IsFirmwareVersionOk()) {
        return QColor(Qt::red);
      }
      if(!servo->IsEnabled()) {
        return QColor(Qt::yellow);
      }
      if(servo->FaultCode() == FC_Ok)
        return QColor(Qt::green);
      else
        return QColor(Qt::red);
    case ColumnHomed:
      if(servo == 0)
        return QColor(Qt::white);
      if(servo->FaultCode() == FC_Unknown)
        return QColor(Qt::yellow);
      switch(servo->HomedState())
      {
      case MHS_Lost:
        return QColor(Qt::red);
      case MHS_Measuring:
        return QColor(Qt::yellow);
      case MHS_Homed:
        return QColor(Qt::green);
      default:
        return QColor(Qt::red);
      }
      break;
    case ColumnDriveTemperature:
      if(servo == 0)
        return QColor(Qt::white);
      if(servo->FaultCode() == FC_Unknown)
        return QColor(Qt::yellow);
      if(servo->DriveTemperature() < 10.0)
        return QColor(Qt::yellow);
      if(servo->DriveTemperature() < 60.0)
        return QColor(Qt::white);
      if(servo->DriveTemperature() < 70.0)
        return QColor(Qt::yellow);
      return QColor(Qt::red);
    case ColumnMotorTemperature:
      if(servo == 0)
        return QColor(Qt::white);
      if(servo->FaultCode() == FC_Unknown)
        return QColor(Qt::yellow);
      if(servo->MotorTemperature() < 10.0)
        return QColor(Qt::yellow);
      if(servo->MotorTemperature() < 40.0)
        return QColor(Qt::white);
      if(servo->MotorTemperature() < 50.0)
        return QColor(Qt::yellow);
      return QColor(Qt::red);
    case ColumnSupplyVoltage:
      if(servo == 0)
        return QColor(Qt::white);
      if(servo->FaultCode() == FC_Unknown)
        return QColor(Qt::yellow);
      if(servo->SupplyVoltage() < 25.0)
        return QColor(Qt::red);
      if(servo->SupplyVoltage() < 28.0)
        return QColor(Qt::yellow);
      if(servo->SupplyVoltage() > 40.0)
        return QColor(Qt::red);
      return QColor(Qt::white);
    case ColumnTorque:
      // Have we hit a limit ?
      if(servo->IndexState() &
         (DOGBOT_SERVOREPORTMODE_LIMITVELOCITY |
          DOGBOT_SERVOREPORTMODE_LIMITTORQUE |
          DOGBOT_SERVOREPORTMODE_LIMITPOSITION))
         return QColor(Qt::red);
      if(servo->FaultCode() == FC_Unknown)
        return QColor(Qt::yellow);
      return QColor(Qt::white);
    case ColumnFlags:
    case ColumnAngle:
    case ColumnSpeed:
    case ColumnMode:
    case ColumnDynamic:
      if(servo->FaultCode() == FC_Unknown)
        return QColor(Qt::yellow);
      return QColor(Qt::white);
    default:
      break;
    }
  }

  return QVariant();
}

bool ServoTable::setData(const QModelIndex &index, const QVariant &value, int role)
{
  auto it = m_row2joint.find(index.row());
  if(it == m_row2joint.end()) {
    return false; // Doesn't seem to exist.
  }
  DogBotN::JointC *joint = it->second;
  DogBotN::ServoC *servo = dynamic_cast<DogBotN::ServoC *>(joint);

  if (role == Qt::EditRole) {
    switch (index.column()) {
    case ColumnName: {
      std::string newName = value.toString().toStdString();
      if(joint->Name() != newName) {
        joint->SetName(newName);
        emit dataChanged(index,index);
      }
      return true;
    } break;
    case ColumnNotes: {
      if(servo != 0) {
        std::string newNotes = value.toString().toStdString();
        if(servo->Notes() != newNotes) {
          servo->SetNotes(newNotes);
          emit dataChanged(index,index);
        }
        return true;
      }
    } break;
    default:
      break;
    }
  }

  if (role == Qt::CheckStateRole)
  {
    switch (index.column()) {
#if 0
    case ColumnEnable: {
      bool newValue = value.toBool();
      return true;
    } break;
#endif
    default:
      return false;
    }
  }

  return false;
}
