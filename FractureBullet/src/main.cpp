#include <game/game_loop.h>
#include <game/factory/game_loop_factory.h>
#include <graphics/factory/render_object_factory.h>
#include <graphics/rendering/renderer.h>
#include <game/factory/game_factory.h>
#include <game/game.h>
#include <game/scene_container.h>
#include <object/game_object.h>
#include <graphics/factory/scene_factory.h>
#include <graphics/lighting/light_source.h>
#include <graphics/lighting/types/light_directional.h>
#include <graphics/lighting/types/light_spotlight.h>

#include <graphics/rendering/camera/camera.h>
#include <engine_gui/factory/engine_gui_factory.h>
#include <graphics/rendering/renderer.h>
#include <engine_gui/engine_gui.h>
#include <example_gui.h>
#include <physics/rigid_body.h>
#include <physics/collision/shapes/box_collision_shape.h>
#include "physics/collision/shapes/static_plane_shape.h"
#include <physics/factory/bullet_physics_simulation_factory.h>
#include <physics/bullet_extensions/btFractureDynamicsWorld.h>

#include <LinearMath/btTransform.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <physics/simulations/bullet_physics_simulation.h>
#include <physics/rigid_bodies/fracture_rigid_body.h>


std::shared_ptr<ifx::LightDirectional> CreateDirectionalLight();
std::shared_ptr<ifx::LightSpotlight> CreateSpotLight();
std::shared_ptr<ifx::RigidBody> CreateRigidBox(glm::vec3 scale);
std::shared_ptr<ifx::RigidBody> CreateFractureRigidBox(
        glm::vec3 scale,
        std::shared_ptr<ifx::BulletPhysicsSimulation> simulation);
std::shared_ptr<ifx::RigidBody> CreateRigidFloor();

std::shared_ptr<ifx::GameObject> CreateGameObjectBox();
std::shared_ptr<ifx::GameObject> CreateGameObjectFractureBox(
        const glm::vec3& scale,
        std::shared_ptr<ifx::BulletPhysicsSimulation> simulation);
std::shared_ptr<ifx::GameObject> CreateGameObjectFloor();
std::shared_ptr<ifx::GameObject> CreateGameObjectLight();

std::vector<std::shared_ptr<ifx::GameObject>> CreateFractureBlock(
        std::shared_ptr<ifx::BulletPhysicsSimulation> simulation);


std::shared_ptr<ifx::LightDirectional> CreateDirectionalLight(){
    ifx::LightParams light;

    light.ambient = glm::vec3(0.5f, 0.5f, 0.5f);
    light.diffuse = glm::vec3(0.5f, 0.5f, 0.5f);
    light.specular = glm::vec3(1.0f, 1.0f, 1.0f);

    auto light_source = std::shared_ptr<ifx::LightDirectional>(
            new ifx::LightDirectional(light));
    light_source->rotateTo(glm::vec3(0, 270, 0));
    light_source->rotateTo(glm::vec3(322, 295, 0));

    return light_source;
}

std::shared_ptr<ifx::LightSpotlight> CreateSpotLight(){
    ifx::LightParams light;

    light.ambient = glm::vec3(0.5f, 0.5f, 0.5f);
    light.diffuse = glm::vec3(0.5f, 0.5f, 0.5f);
    light.specular = glm::vec3(1.0f, 1.0f, 1.0f);

    auto light_source = std::shared_ptr<ifx::LightSpotlight>(
            new ifx::LightSpotlight(light));
    light_source->rotateTo(glm::vec3(0, 270, 0));

    return light_source;
}

std::shared_ptr<ifx::RigidBody> CreateRigidBox(glm::vec3 scale){
    float a = 1;
    auto box_collision = std::shared_ptr<ifx::BoxCollisionShape>(
            new ifx::BoxCollisionShape(glm::vec3(a,a,a)));
    box_collision->collision_shape_bt()->setLocalScaling(btVector3(
            scale.x, scale.y, scale.z));

    auto mass = 1.0f;
    auto rigid_body = std::shared_ptr<ifx::RigidBody>(
            new ifx::RigidBody(box_collision, mass));

    return rigid_body;
}

std::shared_ptr<ifx::RigidBody> CreateFractureRigidBox(
        glm::vec3 scale,
        std::shared_ptr<ifx::BulletPhysicsSimulation> simulation){
    float a = 1;
    auto box_collision = std::shared_ptr<ifx::BoxCollisionShape>(
            new ifx::BoxCollisionShape(glm::vec3(a,a,a)));
    box_collision->collision_shape_bt()->setLocalScaling(btVector3(
            scale.x, scale.y, scale.z));

    auto mass = 0.30f;

    auto rigid_body = std::shared_ptr<ifx::FractureRigidBody>(
            new ifx::FractureRigidBody(box_collision, mass, simulation));

    return rigid_body;
}

std::shared_ptr<ifx::RigidBody> CreateRigidFloor(){
    auto box_collision = std::shared_ptr<ifx::BoxCollisionShape>(
            new ifx::BoxCollisionShape(glm::vec3(500,0.01,500)));

    auto mass = 0.0f;
    auto rigid_body = std::shared_ptr<ifx::RigidBody>(
            new ifx::RigidBody(box_collision, mass));

    return rigid_body;
}

std::shared_ptr<ifx::GameObject> CreateGameObjectBox(){
    auto game_object = std::shared_ptr<ifx::GameObject>(new ifx::GameObject());
    float scale = 0.15;

    auto render_object = ifx::RenderObjectFactory().CreateCube();
    render_object->scale(scale);

    game_object->Add(CreateRigidBox(glm::vec3(scale,scale,scale)));
    game_object->Add(render_object);

    return game_object;
}

std::shared_ptr<ifx::GameObject> CreateGameObjectFractureBox(
        const glm::vec3& scale,
        std::shared_ptr<ifx::BulletPhysicsSimulation> simulation){
    auto game_object = std::shared_ptr<ifx::GameObject>(new ifx::GameObject());

    auto render_object = ifx::RenderObjectFactory().CreateCube();
    render_object->scale(scale);

    game_object->Add(CreateFractureRigidBox(scale, simulation));
    game_object->Add(render_object);

    return game_object;
}

std::shared_ptr<ifx::GameObject> CreateGameObjectFloor(){
    auto game_object = std::shared_ptr<ifx::GameObject>(new ifx::GameObject());

    game_object->Add(ifx::RenderObjectFactory().CreateFloor());
    game_object->Add(CreateRigidFloor());

    return game_object;
}

std::shared_ptr<ifx::GameObject> CreateGameObjectLight(){
    auto game_object = std::shared_ptr<ifx::GameObject>(new ifx::GameObject());
    auto lamp = ifx::RenderObjectFactory().CreateLampObject();

    game_object->Add(std::move(lamp));
    game_object->Add(CreateSpotLight());
    game_object->Add(CreateDirectionalLight());
    game_object->moveTo(glm::vec3(0.0f, 3.0f, 0.0f));

    return game_object;
}

std::vector<std::shared_ptr<ifx::GameObject>> CreateFractureBlock(
        std::shared_ptr<ifx::BulletPhysicsSimulation> simulation){
    const float a = 2.0;
    const float scale_factor = 0.05f;
    const glm::vec3 scale = glm::vec3(scale_factor, scale_factor, scale_factor);
    float epsilon = 0.1;
    std::vector<std::shared_ptr<ifx::GameObject>> objects;
    float start_y = (a * scale_factor) / 2.0f;
    //float start_y = 5.0;
/*
    const int x = 5;
    const int y = 20;
    const int z = 5;
*/
/*
    const int x = 4;
    const int y = 12;
    const int z = 4;
*/

    const int x = 4;
    const int y = 12;
    const int z = 4;

    const float platform_height = 3;
    const float platform_width = 2;

    for(int i = 0; i < x+platform_width ; i++){
        for(int j = 0; j < platform_height; j++){
            for(int k = 0; k < z+platform_width ; k++) {
                float height = start_y + a * scale_factor * j;

                auto game_object = CreateGameObjectFractureBox(scale,
                                                               simulation);
                game_object->moveTo(glm::vec3(
                        a * scale_factor * i,
                        height,
                        a * scale_factor * k));
                objects.push_back(game_object);
            }
        }
    }

    for(int i = 0; i < x; i++){
        for(int j = 0; j < y; j++){
            for(int k = 0; k < z; k++) {
                const float start_x = a * scale_factor * platform_width / 2.0f;
                const float start_z = a * scale_factor * platform_width / 2.0f;
                float height = start_y + a * scale_factor * (j+platform_height);

                auto game_object = CreateGameObjectFractureBox(scale,
                                                               simulation);
                game_object->moveTo(glm::vec3(
                        start_x + a * scale_factor * i,
                        height,
                        start_z + a * scale_factor * k));
                objects.push_back(game_object);
            }
        }
    }
    return objects;
}

int main() {
    auto physics_factory = std::shared_ptr<ifx::BulletPhysicsSimulationFactory>(
            new ifx::BulletPhysicsSimulationFactory());
    physics_factory->SetDynamicWorldType(ifx::DynamicWorldType::FRACTURE);
    auto game_factory
            = std::shared_ptr<ifx::GameFactory>(new ifx::GameFactory());
    game_factory->SetBulletPhysicsSimulationFactory(physics_factory);
    auto game = game_factory->Create();

    auto game_object1 = CreateGameObjectFloor();
    auto game_object2 = CreateGameObjectLight();

    auto game_object3 = std::shared_ptr<ifx::GameObject>(new ifx::GameObject());
    game_object3->Add(
            ifx::SceneFactory().CreateCamera(game->game_loop()->renderer()->window()));
    game_object3->moveTo(glm::vec3(-7, 2, 0));

    auto game_object4 = CreateGameObjectBox();
    game_object4->moveTo(glm::vec3(0.0f, 0.25f*1.0f, 0.0f));
    game_object4->moveTo(glm::vec3(-5.0f, 0.25f*1.0f, 0.0f));

    game->scene()->Add(game_object1);
    game->scene()->Add(game_object2);
    game->scene()->Add(game_object3);
    game->scene()->Add(game_object4);

    auto block = CreateFractureBlock(
            std::static_pointer_cast<ifx::BulletPhysicsSimulation>(
                    game->game_loop()->physics_simulation()));
    // <Fracture>
    for(auto& object : block)
        game->scene()->Add(object);

    std::shared_ptr<btFractureDynamicsWorld> fracture_world
            = std::static_pointer_cast<btFractureDynamicsWorld>
                    (
    std::static_pointer_cast<ifx::BulletPhysicsSimulation>(game->game_loop()
                                                 ->physics_simulation())
                                    ->dynamics_world_bt());
    fracture_world->SetScene(game->scene());

    bool is_running = game->game_loop()->physics_simulation()->is_running();
    game->game_loop()->physics_simulation()->is_running(!is_running);
/*
    fracture_world->setFractureMode(false);
    for(int i = 0; i < 20; i++){
        game->scene()->Update();
        game->game_loop()->physics_simulation()->Update(1.0/60.0);

        fracture_world->glueCallback();
    }
    fracture_world->setFractureMode(true);*/

    fracture_world->setFractureMode(false);
    game->scene()->Update();
    game->game_loop()->physics_simulation()->Update(1.0/60.0);
    fracture_world->glueCallback();
    fracture_world->setFractureMode(false);
    fracture_world->impact_threshold(40);
    game->game_loop()->physics_simulation()->is_running(is_running);
    // </Fracture>

    auto gui = std::shared_ptr<ExampleGUI>(
            new ExampleGUI(
                    game->game_loop()->renderer()->window()->getHandle(),
                    game->scene(),
                    game->game_loop()->physics_simulation()));
    game->game_loop()->renderer()->SetGUI(gui);

    game->Start();
}
