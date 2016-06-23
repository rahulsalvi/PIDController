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

#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

//An abstract class to give the PIDController a way to get the time difference at every cycle
class Timer {
    public:
        Timer() {};
        virtual ~Timer() {};
        virtual void start() {};
        virtual void reset() {};
        virtual double dt() = 0; //in seconds
};

class PIDController {
    public:
        PIDController();
        PIDController(Timer* timer, double p = 0, double i = 0, double d = 0, double f = 0);
        virtual ~PIDController();

        void start();
        void reset();

        void setTimer(Timer* timer);

        void setPIDF(double p, double i, double d, double f);
        void setP(double p);
        void setI(double i);
        void setD(double d);
        void setF(double f);

        void setOutputLimits      (double min, double max);
        void setInputLimits       (double min, double max);
        void setContinuous        (bool isContinuous);
        void setAbsoluteTolerance (double tolerance);    //will prefer absolute tolerance over percent
        void setPercentTolerance  (double tolerance);
        void setIntegratorLimit   (double limit);

        Timer* timer() const;

        double p() const;
        double i() const;
        double d() const;
        double f() const;

        double minOutput() const;
        double maxOutput() const;
        double minInput()  const;
        double maxInput()  const;

        bool isContinuous()      const;
        bool isWithinTolerance() const;
        bool isRunning()         const;

        double error()      const;
        double integral()   const;
        double derivative() const;
        double setpoint()   const;
        double output()     const;

        double getOutput(double input, double setpoint, double dt = 0);
        double getOutput(double error, double dt = 0);

    private:
        double calculateOutput(bool useTimer);

        Timer *_timer;

        double _p;
        double _i;
        double _d;
        double _f;

        double _minOutput;
        double _maxOutput;
        double _minInput;
        double _maxInput;
        bool   _isContinuous;

        double _absoluteTolerance;
        double _percentTolerance;
        bool   _isWithinTolerance;

        double _integratorLimit;

        double _error;
        double _previousError;
        double _integral;
        double _derivative;
        double _setpoint;
        double _output;

        double _dt;
        bool   _isRunning;
};

#endif //PIDCONTROLLER_H
