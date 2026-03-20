#pragma once

#include "TinyGPSPlus.h"

class GpsTask
{
public:
    void start();
private:
    TinyGPSPlus m_gps;
};