/********************************************************************
	created:	2013/04/02
	created:	2:4:2013   12:26
	filename: 	exampleIntRemoteSource.h
	author:		Wojciech Kniec
	
	purpose:	
*********************************************************************/

#ifndef HEADER_GUARD_EXAMPLE__INTSMODEL_H__
#define HEADER_GUARD_EXAMPLE__INTSMODEL_H__

#include <QtCore/QAbstractListModel>
#include "Plugin.h"

class IntsModel : public QAbstractListModel
{
    Q_OBJECT;
public:
    explicit IntsModel(IntsPtr ints, QObject *parent = 0) : QAbstractListModel(parent), ints(ints) {}

    virtual int rowCount( const QModelIndex &parent = QModelIndex( ) ) const;
    virtual QVariant data( const QModelIndex &index, int role = Qt::DisplayRole ) const;

public slots:
    void clear();
    void addValue(int val);

private:
    IntsPtr ints;
};




#endif
