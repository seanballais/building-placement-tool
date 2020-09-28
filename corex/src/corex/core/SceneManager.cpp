#include <EASTL/unordered_map.h>

#include <corex/core/AssetManager.hpp>
#include <corex/core/Camera.hpp>
#include <corex/core/SceneManager.hpp>
#include <corex/core/SceneManagerStatus.hpp>
#include <corex/core/SceneStatus.hpp>
#include <corex/core/utils.hpp>

namespace corex::core
{
  SceneManager::SceneManager(
        entt::registry& registry,
        entt::dispatcher& eventDispatcher,
        AssetManager& assetManager,
        Camera& camera)
    : registry(registry)
    , eventDispatcher(eventDispatcher)
    , assetManager(assetManager)
    , camera(camera)
    , scenes() // Can't initialize scenes with an initializer list, since it is
               // a vector of unique pointers to Scene instances, but keeping it
               // here for sake of consistency.
    , sceneEdges({})
    , currentScene(nullptr)
    , rootScene(nullptr)
    , status(SceneManagerStatus::RUNNING)
    , isCurrentSceneInitialized(false) {}

  void SceneManager::connectScenes(Scene& from, Scene& to, SceneStatus status)
  {
    auto sceneIter = this->sceneEdges.find(&from);
    if (sceneIter == this->sceneEdges.end()) {
      // Time to create an entry for the from scene.
      this->sceneEdges[&from] = {};
    }

    this->sceneEdges[&from][status] = &to;
  }

  void SceneManager::setRootScene(Scene& scene)
  {
    this->rootScene = &scene;
    this->currentScene = this->rootScene;
    this->isCurrentSceneInitialized = false;  // Just making sure it is false.
  }

  void SceneManager::update(float timeDelta)
  {
    if (!(this->isCurrentSceneInitialized)) {
      this->currentScene->init();
      this->currentScene->setSceneStatus(SceneStatus::RUNNING);
      this->isCurrentSceneInitialized = true;
    }

    SceneStatus currentSceneStatus = this->currentScene->getStatus();
    switch (currentSceneStatus) {
      case SceneStatus::RUNNING:
        this->currentScene->update(timeDelta);
        break;
      case SceneStatus::DONE:
        this->currentScene->dispose();

        if (this->currentScene == this->rootScene) {
          // Okay, time to exit the game.
          this->status = SceneManagerStatus::DONE;
        } else {
          this->currentScene = this->getNextScene(*(this->currentScene),
                                                  SceneStatus::DONE);
          this->isCurrentSceneInitialized = false;
        }

        break;
      case SceneStatus::INTERRUPTED:
        STUBBED("SceneStatus::INTERRUPTED handler not yet implemented.");
        break;
    }
  }

  SceneManagerStatus SceneManager::getStatus()
  {
    return this->status;
  }

  float SceneManager::getCurrentScenePPMRatio()
  {
    return this->currentScene->getPPMRatio();
  }

  Scene* SceneManager::getNextScene(Scene& from, SceneStatus status)
  {
    return this->sceneEdges[&from][status];
  }
}
