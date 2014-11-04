#include "PCH.h"
#include "IntsModel.h"
#include "Plugin.h"

int IntsModel::rowCount( const QModelIndex &parent /*= QModelIndex( ) */ ) const
{
    return ints->size();
}

QVariant IntsModel::data( const QModelIndex &index, int role /*= Qt::DisplayRole */ ) const
{
    if (role == Qt::DisplayRole && index.row() <= ints->size()) {
        return QVariant(ints->at(index.row()));
    }

    return QVariant();
}

void IntsModel::clear()
{
    ints->clear();
	beginResetModel();
	endResetModel();
}

void IntsModel::addValue( int val )
{
    int count = ints->size();
    beginInsertRows(QModelIndex(), count, count );
    ints->push_back(val);
    endInsertRows();
}
