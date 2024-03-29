// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "subsystems/LED_Groups.h"

void LED_Group::SetColor(int _r, int _g, int _b)
{
    for(int i: _group)
    {
        _candle->SetLEDs(_r, _g, _b, 0, i, 1);
    }
}

void LED_Group::SetLarson(int _r, int _g, int _b, int Length)
{
    auto temp = ctre::phoenix::led::LarsonAnimation(_r, _g, _b, 0, 0.5, _group.size(), ctre::phoenix::led::LarsonAnimation::BounceMode::Front, 2, _group[0]);
    _candle->Animate(temp, _slotID);
}
// This method will be called once per scheduler run

void LED_Group::SetDisco(int _r, int _g, int _b)
{
    for(int i: _group)
    {
     _candle->SetLEDs(_r, _g, _b, 0, i, 1);
     _candle->SetLEDs(0, 0, 0, 0, i - 1, 1);
    }
}

void LED_Group::SetFlash(int _r, int _g, int _b, int r, int g, int b, units::time::second_t Period)
{    
    _timer.Start();

    if(_timer.HasElapsed(Period))
    {
        if(_status)
        {
            SetColor(_r, _g, _b);
            _status = false;
        }
        else
        {
            SetColor(r, g, b);
            _status = true;
        }

        _timer.Reset();
    }
}

void LED_Group::SetGradient(int _r, int _g, int _b, int _R, int _G, int _B)
{
    int r = _r;
    int g = _g;
    int b = _b;
    int R = _R;
    int G = _G;
    int B = _B;

    int _rStep;
    int _gStep;
    int _bStep;

    int RStep;
    int GStep;
    int BStep;

    int i = 0;

    _rStep = r - R;
    _gStep = g - G;
    _bStep = b - B;

    RStep = (_rStep)/(int(_group.size()) - 1);
    GStep = (_gStep)/(int(_group.size()) - 1);
    BStep = (_bStep)/(int(_group.size()) - 1);

    for(i = 0; i <= _group.size(); i++)
    {
        _candle->SetLEDs(r, g, b, 0, _group[i], 1);
        
        r -= RStep;
        g -= GStep;
        b -= BStep;
    }
}

void LED_Group::SetScrollingGradient(int _r, int _g, int _b, int _R, int _G, int _B)
{
    std::vector<std::vector<int>> gradValues = ComputeGradent(_r, _g, _b, _R, _G, _B);

    for(int i = 0; i <= _group.size(); i++)
    {
        int idx = (i+_start)%gradValues.size();
        _candle->SetLEDs(gradValues[idx][0], gradValues[idx][1], gradValues[idx][2], 0, _group[i], 1);
    }
    _start++;
}

std::vector<std::vector<int>> LED_Group::ComputeGradent(int _r, int _g, int _b, int _R, int _G, int _B)
{
    int r = _r;
    int g = _g;
    int b = _b;
    int R = _R;
    int G = _G;
    int B = _B;

    int _rStep;
    int _gStep;
    int _bStep;

    int RStep;
    int GStep;
    int BStep;

    std::vector<std::vector<int>> gradValues;

    _rStep = r - R;
    _gStep = g - G;
    _bStep = b - B;

    RStep = (_rStep)/(int(_group.size()));
    GStep = (_gStep)/(int(_group.size()));
    BStep = (_bStep)/(int(_group.size()));

    for(int i = 0; i <= _group.size(); i++)
    {
        gradValues.push_back({r, g, b});
        r -= RStep;
        g -= GStep;
        b -= BStep;
    }

    for(int i = _group.size(); i <= _group.size()*2; i++)
    {
        gradValues.push_back({r, g, b});
        
        r += RStep;
        g += GStep;
        b += BStep;
    }

    return gradValues;
}

void LED_Group::SetRainbow(int slot)
{
    auto rainbow = ctre::phoenix::led::RainbowAnimation(1.0, 0.7, _group.size(), false, _group[0]);
    _candle->Animate(rainbow, slot);
}

void LED_Group::SetInvertedRainbow(int slot)
{
    auto rainbow = ctre::phoenix::led::RainbowAnimation(1.0, 0.7, _group.size(), true, _group[0]);
    _candle->Animate(rainbow, slot);
}

void LED_Group::SetRGBFade(int slot)
{
    auto rgbFade = ctre::phoenix::led::RgbFadeAnimation(1.0, 0.7, _group.size(), _group[0]);
    _candle->Animate(rgbFade, slot);
}

void LED_Group::SetInvertedRGBFade(int slot)
{
    auto rgbFade = ctre::phoenix::led::RgbFadeAnimation(1.0, 0.7, _group.size(), _group[0]);
    _candle->Animate(rgbFade, slot);
}