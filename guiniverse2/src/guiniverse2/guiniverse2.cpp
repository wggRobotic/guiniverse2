#include <guiniverse2/guiniverse2.hpp>
#include <unistd.h>

#include <quac/quac.hpp>
#include <quac/quac_cmd.hpp>

void Guiniverse2::run()
{
    running.store(true);
    robot_index = 0;

    rclcpp::on_shutdown([this]() { running.store(false); });

    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_TRANSPARENT_FRAMEBUFFER, GLFW_TRUE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    auto window = glfwCreateWindow(800, 600, "Guiniverse2 ROS2 GUI", nullptr, nullptr);
    if (!window) return;

    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);
    glEnable(GL_MULTISAMPLE);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    auto &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags &= ~ImGuiConfigFlags_ViewportsEnable;
    io.FontGlobalScale = 2.0f; // 200%

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    float clear_color[]{0.00f, 0.00f, 0.00f, 0.35f};
    bool show_styles = false;
    
    while (!glfwWindowShouldClose(window) && running.load())
    {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_PassthruCentralNode);

        int old_robot_index = robot_index;

        if (ImGui::Begin("Home"))
        {
            ImGui::ColorEdit4("Clear Color", clear_color);
            ImGui::Checkbox("Show Styles", &show_styles);
            
            const char* names[] =
            {
                "None",
                "Quac",
                "QuacCmd"
            };

            if (ImGui::BeginCombo("Select which robot to control", names[robot_index]))
            {
                for (int i = 0; i < sizeof(names)/sizeof(names[0]); i++)
                {
                    if (ImGui::Selectable(names[i], robot_index == i)) 
                        robot_index = i;
                }
                
                ImGui::EndCombo();
            }

        }
        ImGui::End();

        if (show_styles)
        {
            if (ImGui::Begin("Styles", &show_styles))
            {
                ImGui::ShowStyleEditor();

            }
            ImGui::End();   
        }

        if (robot_controller != nullptr) robot_controller->on_gui_frame(window);

        ImGui::Render();

        glClearColor(clear_color[0], clear_color[1], clear_color[2], clear_color[3]);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);

        if (old_robot_index != robot_index)
        {

            robot_controller.reset();
            robot_controller = nullptr;

            switch (robot_index)
            {
                
            case 0: break;
            case 1: {robot_controller = std::make_shared<Quac>(); break;}
            case 2: {robot_controller = std::make_shared<QuacCmd>(); break;}
            default: break;

            }
        }
    }

    robot_controller.reset();

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);

    running.store(false);
}