#ifndef PROJECT_EXAMPLE_GUI_H
#define PROJECT_EXAMPLE_GUI_H

#include "gui/gui.h"

#include <memory>

namespace ifx{
class EngineGUI;
class SceneContainer;
class PhysicsSimulation;
}

class ExampleGUI : public ifx::GUI{
public:

    ExampleGUI(GLFWwindow* window,
               std::shared_ptr<ifx::SceneContainer> scene,
               std::shared_ptr<ifx::PhysicsSimulation> physics_simulation);
    ~ExampleGUI();

    virtual void Render() override;
private:
    void RenderFractureInfo();
    void RenderFractureMode();
    void RenderFractureImpactTreshold();

    std::shared_ptr<ifx::EngineGUI> engine_gui_;
    std::shared_ptr<ifx::PhysicsSimulation> physics_simulation_;

};


#endif //PROJECT_EXAMPLE_GUI_H
