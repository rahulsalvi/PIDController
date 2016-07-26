/*
The MIT License (MIT)

Copyright (c) 2016 Rahul Salvi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "PIDController.h"

inline double absoluteValue(double x) {
    if (x < 0) {
        return -x;
    }
    return x;
}

PIDController::PIDController() : PIDController(nullptr, 0, 0, 0, 0) {}

PIDController::PIDController(Timer* timer, double p, double i, double d, double f) :
    _timer(timer),
    _p(p),
    _i(i),
    _d(d),
    _f(f) {
        reset();
        _minOutput = 0;
        _maxOutput = 0;
        _minInput = 0;
        _minOutput = 0;
        _isContinuous = false;
        _absoluteTolerance = 0;
        _percentTolerance = 0;
        _isWithinTolerance = false;
        _integratorLimit = 0;
        _isRunning = false;
}

void PIDController::start() {
    if (_timer) {
        _timer->start();
    }
    _isRunning = true;
}

void PIDController::reset() {
    _error = 0;
    _previousError = 0;
    _integral = 0;
    _derivative = 0;
    _setpoint = 0;
    _output = 0;
    if (_timer) {
        _timer->reset();
    }
}

void PIDController::setTimer(Timer* timer) {
    _timer = timer;
}

void PIDController::setPIDF(double p, double i, double d, double f) {
    _p = p;
    _i = i;
    _d = d;
    _f = f;
}

void PIDController::setP(double p) {
    _p = p;
}

void PIDController::setI(double i) {
    _i = i;
}

void PIDController::setD(double d) {
    _d = d;
}

void PIDController::setF(double f) {
    _f = f;
}

void PIDController::setOutputLimits(double min, double max) {
    _minOutput = min;
    _maxOutput = max;
}

void PIDController::setInputLimits(double min, double max) {
    _minInput = min;
    _maxInput = max;
}

void PIDController::setContinuous(bool isContinuous) {
    _isContinuous = isContinuous;
}

void PIDController::setAbsoluteTolerance(double tolerance) {
    _absoluteTolerance = tolerance;
}

void PIDController::setPercentTolerance(double tolerance) {
    _percentTolerance = tolerance;
}

void PIDController::setIntegratorLimit(double limit) {
    _integratorLimit = limit;
}

Timer* PIDController::timer() const {
    return _timer;
}

double PIDController::p() const {
    return _p;
}

double PIDController::i() const {
    return _i;
}

double PIDController::d() const {
    return _d;
}

double PIDController::f() const {
    return _f;
}

double PIDController::minOutput() const {
    return _minOutput;
}

double PIDController::maxOutput() const {
    return _maxOutput;
}

double PIDController::minInput() const {
    return _minInput;
}

double PIDController::maxInput() const {
    return _maxInput;
}

bool PIDController::isContinuous() const {
    return _isContinuous;
}

bool PIDController::isWithinTolerance() const {
    return _isWithinTolerance;
}

bool PIDController::isRunning() const {
    return _isRunning;
}

double PIDController::error() const {
    return _error;
}

double PIDController::integral() const {
    return _integral;
}

double PIDController::derivative() const {
    return _derivative;
}

double PIDController::setpoint() const {
    return _setpoint;
}

double PIDController::output() const {
    return _output;
}

double PIDController::getOutput(double input, double setpoint, double dt) {
    if (_minInput && _maxInput) {
        if (input > _maxInput) {
            input = _maxInput;
        } else if (input < _minInput) {
            input = _minInput;
        }
    }

    _setpoint = setpoint;
    _error = _setpoint - input;

    if (_absoluteTolerance) {
        if (input > _setpoint - _absoluteTolerance && input < _setpoint + _absoluteTolerance) {
            _isWithinTolerance = true;
        } else {
            _isWithinTolerance = false;
        }
    } else if (_percentTolerance) {
        double tolerance = _percentTolerance * _setpoint;
        if (input > _setpoint - tolerance && input < _setpoint + tolerance) {
            _isWithinTolerance = true;
        } else {
            _isWithinTolerance = false;
        }
    } else {
        _isWithinTolerance = false;
    }

    if (dt) {
        _dt = dt;
        return calculateOutput(false);
    }

    return calculateOutput(true);
}

double PIDController::getOutput(double error, double dt) {
    _error = error;
    _setpoint = 0;
    _isWithinTolerance = false;

    if (dt) {
        _dt = dt;
        return calculateOutput(false);
    }

    return calculateOutput(true);
}

double PIDController::calculateOutput(bool useTimer) {
    if (_isContinuous) {
        if (absoluteValue(_error) > ((absoluteValue(_minInput) + absoluteValue(_maxInput)) / 2)) {
            if (_error > 0) {
                _error = _error - _maxInput + _minInput;
            } else {
                _error = _error + _maxInput - _minInput;
            }
        }
    }

    if (useTimer && _timer) {
        _dt = _timer->dt();
    }
    _integral += _error * _dt;
    _derivative = ((_error - _previousError) / _dt);
    _previousError = _error;

    if (_integratorLimit && absoluteValue(_integral) > _integratorLimit) {
        if (_integral < 0) {
            _output = (_p * _error) + (_i * -_integratorLimit) + (_d * _derivative) + (_f * _setpoint);
        } else {
            _output = (_p * _error) + (_i *  _integratorLimit) + (_d * _derivative) + (_f * _setpoint);
        }
    } else {
        _output = (_p * _error) + (_i * _integral) + (_d * _derivative) + (_f * _setpoint);
    }

    if (_minOutput && _maxOutput) {
        if (_output > _maxOutput) {
            _output = _maxOutput;
        } else if (_output < _minOutput) {
            _output = _minOutput;
        }
    }

    return _output;
}
