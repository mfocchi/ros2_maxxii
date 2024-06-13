#ifndef ENCODER_H
#define ENCODER_H

#include <vector>
#include <iostream>
#include <memory>
#include <cmath>
#include <stdexcept> // exception handling

class Encoder
{
private:
    double ppr_;
    long reference_pulse_;
    long pulses_;
    long speed_;
public:
    Encoder(double ppr);
    
    void setReferencePulseCount(long pulse_count);
    void setPulseCount(long pulses);

    long getPulseCount() const;
    double getRevolutions() const;
    double getRadiants() const;
    double getDegrees() const;

    void setSpeed(long speed);
    double getSpeedRads() const;
    int getPPR() const;
};

#endif