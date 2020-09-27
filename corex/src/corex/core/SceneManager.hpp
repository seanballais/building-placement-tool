#ifndef COREX_CORE_SCENE_MANAGER_HPP
#define COREX_CORE_SCENE_MANAGER_HPP

#include <EASTL/memory.h>
#include <EASTL/type_traits.h>
#include <EASTL/unique_ptr.h>
#include <EASTL/unordered_map.h>
#include <EASTL/vector.h>
#include <entt/entt.hpp>

#include <corex/core/AssetManager.hpp>
#include <corex/core/Camera.hpp>
#include <corex/core/Scene.hpp>
#include <corex/core/SceneManagerStatus.hpp>
#include <corex/core/SceneStatus.hpp>

namespace corex::core
{
  class SceneManager
  {
  public:
    SceneManager(entt::registry& registry,
                 entt::dispatcher& eventDispatcher,
                 AssetManager& assetManager,
                 Camera& camera);

    template <class T>
    Scene& addScene()
    {
      static_assert(eastl::is_base_of<Scene, T>::value);

      auto scene = eastl::make_unique<T>(this->registry,
                                         this->eventDispatcher,
                                         this->assetManager,
                                         this->camera);
      auto* scenePtr = scene.get();
      this->scenes.push_back(eastl::move(scene));

      return *scenePtr;
    }

    void connectScenes(Scene& from, Scene& to, SceneStatus status);
    void setRootScene(Scene& scene);
    void update(float timeDelta);
    SceneManagerStatus getStatus();
    float getCurrentScenePPMRatio();

  private:
    Scene* getNextScene(Scene& from, SceneStatus status);

    entt::registry& registry;
    entt::dispatcher& eventDispatcher;
    AssetManager& assetManager;
    Camera& camera;

    eastl::vector<eastl::unique_ptr<Scene>> scenes;
    eastl::unordered_map<Scene*, eastl::unordered_map<SceneStatus, Scene*>>
      sceneEdges;

    Scene* currentScene;
    Scene* rootScene;

    SceneManagerStatus status;

    bool isCurrentSceneInitialized;
  };
}

#endif