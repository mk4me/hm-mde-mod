/********************************************************************
    created:  2013/07/22
    created:  22:7:2013   9:26
    filename: FittingWizzard.h
    author:   Mateusz Janiak
    
    purpose:  Wizzard prowadz¹cy nas przez proces dopasowania jednego sygna³u
				orientacji w drugi. Uwzglêdnia offsety, szuka rotacji
				uk³adów odniesienia
*********************************************************************/
#ifndef HEADER_GUARD___FITTINGWIZZARD_H__
#define HEADER_GUARD___FITTINGWIZZARD_H__

#include "ui_FittingWizzard.h"
#include <QtGui/QWizard>
#include <IMU/Data/Types.h>

class FittingWizard : public QWizard, private Ui::FittingWizzard
{
	Q_OBJECT

public:
	//! Wyliczenie danych wejœciowych opisuj¹cych u³o¿enie sensora lub jego uk³adu
	//! odniesienia w ramach VICON
	enum InputOrientation {
		LOCAL_SENSOR,			//! Lokalna rotacja sensora w ramach VICON
		GLOBAL_SENSOR_FRAME		//! Rotacja uk³adu odniesienia sensora w ramach VICON
	};

public:

	FittingWizard(unsigned int sizeSrc, unsigned int sizeDest,
		QWidget * parent = nullptr, Qt::WindowFlags f = 0);

	virtual ~FittingWizard();

	//! \return Czy wykonaæ autodetekcjê offsetu
	const bool offsetAutodetection() const;

	//! \return Offset o jaki trzeba przsu¹æ sygna³ src by otrzymaæ sygna³ pokrywaj¹cy siê z dest
	const int offset() const;
	//! \return Krok przy automatycznym wyznaczaniu offsetu
	const unsigned int offsetDetectionStep() const;
	//! Minimalny obszar pokrycia sygna³ów
	const unsigned int minimalMatchDistance() const;
	//! \return Wejœciowa orientacja
	const IMU::Quat & orientation() const;	

	//! \return Typ rotacji wejsciowej
	const InputOrientation inputOrientationType() const;

private slots:

	void orientationGroupChanged(int idx);
	void offsetGroupChanged(int idx);

	void quaternionChanged();
	void cardanAngleChanged();

	void offsetChanged(int offset);
	void offsetStepChanged(int offsetStep);
	void minFitSizeChanged(int minFitSize);

private:

	void updateQuaternionLimits();
	void updateCardanAngles();
	void updateQuaternion();

private:

	//! Przesuniêcie sygna³u Ÿród³owego
	int offset_;
	//! Krok przy automatycznym wyznaczaniu offsetu
	unsigned int offsetDetectionStep_;
	//! Minimalny obszar pokrycia sygna³ów
	unsigned int minimalMatchDistance_;
	//! Czy wykonaæ autodetekcjê offsetu
	bool offsetAutodetection_;

	//! Lokalny obrót sensora w uk³adzie VICON
	IMU::Quat orientation_;
	//! Typ rotacji wejsciowej
	InputOrientation inputOrientationType_;

	//! Rozmiar sygna³u dopasowywanego
	const unsigned int sizeSrc_;
	//! Rozmiar sygna³u do którego dopasowujemy
	const unsigned int sizeDest_;
};

#endif	//	HEADER_GUARD___FITTINGWIZZARD_H__
