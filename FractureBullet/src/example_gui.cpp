#include "example_gui.h"

#include <engine_gui/factory/engine_gui_factory.h>
#include <engine_gui/engine_gui.h>
#include <gui/imgui/imgui.h>
#include <physics/bullet_extensions/btFractureDynamicsWorld.h>
#include <physics/simulations/bullet_physics_simulation.h>

ExampleGUI::ExampleGUI(GLFWwindow* window,
                       std::shared_ptr<ifx::SceneContainer> scene,
                       std::shared_ptr<ifx::PhysicsSimulation> physics_simulation) :
        ifx::GUI(window),
        physics_simulation_(physics_simulation){
    engine_gui_ = ifx::EngineGUIFactory().CreateEngineGUI(scene,
                                                          physics_simulation);
}
ExampleGUI::~ExampleGUI(){}

void ExampleGUI::Render(){
    NewFrame();

    RenderFractureInfo();
    engine_gui_->Render();

    ImGui::Render();
}

void ExampleGUI::RenderFractureInfo(){
    ImGui::Begin("Fracture");

    RenderFractureMode();
    RenderFractureImpactTreshold();

    ImGui::End();
}

void ExampleGUI::RenderFractureMode(){
    std::shared_ptr<btFractureDynamicsWorld> fracture_world
            = std::static_pointer_cast<btFractureDynamicsWorld>
                    (std::static_pointer_cast<ifx::BulletPhysicsSimulation>(physics_simulation_)
                             ->dynamics_world_bt());
    static bool fracture_mode;
    fracture_mode = fracture_world->getFractureMode();
    ImGui::Checkbox("Fracture Mode", &fracture_mode);
    fracture_world->setFractureMode(fracture_mode);
}

void ExampleGUI::RenderFractureImpactTreshold(){
    std::shared_ptr<btFractureDynamicsWorld> fracture_world
            = std::static_pointer_cast<btFractureDynamicsWorld>
                    (std::static_pointer_cast<ifx::BulletPhysicsSimulation>(physics_simulation_)
                             ->dynamics_world_bt());
    static float v;
    v = fracture_world->impact_threshold();
    ImGui::SliderFloat("Impact threshold", &v, 1, 100);
    fracture_world->impact_threshold(v);
}