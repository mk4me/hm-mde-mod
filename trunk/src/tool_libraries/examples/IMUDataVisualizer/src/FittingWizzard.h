/********************************************************************
    created:  2013/07/22
    created:  22:7:2013   9:26
    filename: FittingWizzard.h
    author:   Mateusz Janiak
    
    purpose:  Wizzard prowadz�cy nas przez proces dopasowania jednego sygna�u
				orientacji w drugi. Uwzgl�dnia offsety, szuka rotacji
				uk�ad�w odniesienia
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
	//! Wyliczenie danych wej�ciowych opisuj�cych u�o�enie sensora lub jego uk�adu
	//! odniesienia w ramach VICON
	enum InputOrientation {
		LOCAL_SENSOR,			//! Lokalna rotacja sensora w ramach VICON
		GLOBAL_SENSOR_FRAME		//! Rotacja uk�adu odniesienia sensora w ramach VICON
	};

public:

	FittingWizard(unsigned int sizeSrc, unsigned int sizeDest,
		QWidget * parent = nullptr, Qt::WindowFlags f = 0);

	virtual ~FittingWizard();

	//! \return Czy wykona� autodetekcj� offsetu
	const bool offsetAutodetection() const;

	//! \return Offset o jaki trzeba przsu�� sygna� src by otrzyma� sygna� pokrywaj�cy si� z dest
	const int offset() const;
	//! \return Krok przy automatycznym wyznaczaniu offsetu
	const unsigned int offsetDetectionStep() const;
	//! Minimalny obszar pokrycia sygna��w
	const unsigned int minimalMatchDistance() const;
	//! \return Wej�ciowa orientacja
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

	//! Przesuni�cie sygna�u �r�d�owego
	int offset_;
	//! Krok przy automatycznym wyznaczaniu offsetu
	unsigned int offsetDetectionStep_;
	//! Minimalny obszar pokrycia sygna��w
	unsigned int minimalMatchDistance_;
	//! Czy wykona� autodetekcj� offsetu
	bool offsetAutodetection_;

	//! Lokalny obr�t sensora w uk�adzie VICON
	IMU::Quat orientation_;
	//! Typ rotacji wejsciowej
	InputOrientation inputOrientationType_;

	//! Rozmiar sygna�u dopasowywanego
	const unsigned int sizeSrc_;
	//! Rozmiar sygna�u do kt�rego dopasowujemy
	const unsigned int sizeDest_;
};

#endif	//	HEADER_GUARD___FITTINGWIZZARD_H__
