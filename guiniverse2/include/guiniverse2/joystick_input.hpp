#pragma once

#include <GLFW/glfw3.h>
#include <string>

class JoystickInput 
{
public:
    JoystickInput(const std::string& default_controller);
    void update();
    void ImGuiPanel(char* label);

    bool getButton(unsigned int i);
    float getAxis(unsigned int i);
    const std::string& getDeviceName();

private:
    
    std::string m_DefaultDeviceName = "NoDevice";

    bool m_InputDevicesConnected[GLFW_JOYSTICK_LAST + 1];
    std::string m_InputDeviceNames[GLFW_JOYSTICK_LAST + 1];
    bool m_InvertAxes[GLFW_JOYSTICK_LAST + 1][16] = {{true, true, false, true}};
    int m_InputDeviceSelected = 0;

    bool m_RefreshButton = true;

    bool m_Buttons[32] = {false};
    float m_Axes[16] = {0};
};