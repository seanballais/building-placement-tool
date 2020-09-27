#ifndef COREX_CORE_SCENE_HPP
#define COREX_CORE_SCENE_HPP

#include <cstdint>

#include <EASTL/array.h>
#include <EASTL/vector.h>

#include <entt/entt.hpp>

#include <corex/core/AssetManager.hpp>
#include <corex/core/Camera.hpp>
#include <corex/core/SceneStatus.hpp>

namespace corex::core
{
  // Forward declare SceneManager so that we can make it a friend.
  class SceneManager;

  class Scene
  {
  public:
    using Entity = entt::entity;
    using Layer = int8_t;
    using SortingLayer = int8_t;

    struct EntitySceneProperty
    {
      Layer layer;
      SortingLayer sortingLayer;
    };

    Scene(entt::registry& registry,
          entt::dispatcher& eventDispatcher,
          AssetManager& assetManager,
          Camera& camera,
          float ppmRatio = 32);

    virtual void init() = 0;
    virtual void update(float timeDelta) = 0;
    virtual void dispose() = 0;
    virtual ~Scene() = default;

    Scene::Entity createSceneEntity();
    void setEntityLayer(Scene::Entity entity, Scene::Layer layer);
    Scene::Layer getEntityLayer(Scene::Entity entity);
    void setEntitySortingLayer(Scene::Entity entity, Scene::SortingLayer layer);
    Scene::SortingLayer getEntitySortingLayer(Scene::Entity entity);

    void enableLayer(Scene::Layer layer);
    void disableLayer(Scene::Layer layer);
    bool isLayerEnabled(Scene::Layer layer);

    void enableSortingLayer(Scene::SortingLayer layer);
    void disableSortingLayer(Scene::SortingLayer layer);
    bool isSortingLayerEnabled(Scene::SortingLayer layer);

    SceneStatus getStatus();
    float getPPMRatio();

  protected:
    static const int kNumLayers = 32;
    static const int kNumSortingLayers = 32;

    entt::registry& registry;
    entt::dispatcher& eventDispatcher;
    AssetManager& assetManager;
    Camera& camera;
    eastl::vector<Entity> entities;
    eastl::array<bool, kNumLayers> layerAvailabilityStatuses;
    eastl::array<bool, kNumSortingLayers> sortingLayerAvailabilityStatuses;

    SceneStatus status;

    void setSceneStatus(SceneStatus status);

  private:
    float ppmRatio;

    // Since SceneManager is defined in another header file and includes this
    // header file, we can't make SceneManager's update function a friend of
    // Scene. Doing so would require re-defining SceneManager, which is illegal
    // and will raise a redefinition error.
    friend class SceneManager;
  };
}

#endif
