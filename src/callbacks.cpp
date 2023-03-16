#include <Arduino.h>
class GaitGuide
{
public:
    // ...

    double getCurrentPressure() const
    {
        return m_currentPressure;
    }

    // Define a callback function type that takes a double as a parameter
    typedef std::function<void(double)> CurrentPressureChangedCallback;

    // Set up a callback function to be called when m_currentPressure is changed
    void onCurrentPressureChanged(const CurrentPressureChangedCallback &callback)
    {
        m_currentPressureChangedCallback = callback;
    }

private:
    double m_currentPressure;
    CurrentPressureChangedCallback m_currentPressureChangedCallback;
};

class BLE_handler
{
public:
    BLE_handler(GaitGuide &gaitGuide) : m_gaitGuide(gaitGuide)
    {
        // Set up a callback function to be called when m_currentPressure is changed
        m_gaitGuide.onCurrentPressureChanged([this](double /* unused */)
                                             { this->onPressureChanged(); });
    }

    void onPressureChanged()
    {
        // Get the current pressure value using the public getter function
        double pressure = m_gaitGuide.getCurrentPressure();

        // Handle the pressure change here
    }

private:
    GaitGuide &m_gaitGuide;
};
