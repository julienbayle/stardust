/*
	SharpIR

	Arduino library for retrieving distance (in cm) from the analog GP2Y0A21Y and GP2Y0A02YK

	Pierre VALLEAU
    from an original version of Dr. Marcal Casas-Cartagena (marcal.casas@gmail.com)     
*/

#ifndef SharpIR_h
#define SharpIR_h

#define NB_SAMPLE 10

#include "Arduino.h"
#include <assert.h>

#ifndef ARDUINO_ARCH_AVR
#include "WMath.h"
#endif

class ZSharpIR
{
	
  public:

    ZSharpIR (int irPin, const uint32_t _sensorType);
    int distance();
	int getAccuracy() ;
	static const uint32_t GP2Y0A41SK0F = 430 ;
	static const uint32_t GP2Y0A21YK0F = 1080 ;
	static const uint32_t GP2D12_24 = 1081 ;
	static const uint32_t GP2Y0A02YK0F = 20150 ;
	static const uint32_t GP2Y0A710K0F = 100500 ;
	static const uint32_t CALIBRATED = -1 ;
	
    int getMax();
    int getMin() ;
    int getRaw();
    void setARefVoltage(int refV);
    void SetAnalogReadResolution(int res);
	
    void CalibrateStart();
    void CalibrateNextStep();
    void ApplyCalibration(int atable[20]);

    void DisplayCalibration(Stream & Serial);

  private:
    
    int compute(int ir_val) ;
    int    _Adcres;
    int _refVoltage;
     
    void sort(int a[], int size);
    
    int _irPin;
    uint32_t _model;
};

#endif
