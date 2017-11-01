#include "ServoTable.hh"

ServoTable::ServoTable()
{

}

Qt::ItemFlags ServoTable::flags(const QModelIndex &index) const
{
  if (!index.isValid())
      return 0;
  if(index.row() >= m_layers.size())
    return QAbstractItemModel::flags(index);
  return QAbstractItemModel::flags(index);
}

int ServoTable::columnCount(const QModelIndex &parent) const
{
  return ColumnCount;
}

int ServoTable::rowCount(const QModelIndex &parent) const
{

}
