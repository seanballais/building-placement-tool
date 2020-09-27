#include <cstdint>

#include <entt/entt.hpp>

#include <corex/core/AssetManager.hpp>
#include <corex/core/Camera.hpp>
#include <corex/core/Scene.hpp>

namespace corex::core
{
  Scene::Scene(entt::registry& registry,
               entt::dispatcher& eventDispatcher,
               AssetManager& assetManager,
               Camera& camera,
               float ppmRatio)
    : registry(registry)
    , eventDispatcher(eventDispatcher)
    , assetManager(assetManager)
    , camera(camera)
    , entities({})
    , ppmRatio(ppmRatio)
  {
    for (int i = 0; i < this->kNumLayers; i++) {
      this->layerAvailabilityStatuses[i] = true;
    }

    for (int i = 0; i < this->kNumSortingLayers; i++) {
      this->sortingLayerAvailabilityStatuses[i] = true;
    }
  }

  Scene::Entity Scene::createSceneEntity()
  {
    Scene::Entity entity = this->registry.create();
    Scene::Layer defaultLayer = 1;
    Scene::SortingLayer defaultSortingLayer = 1;

    this->registry.emplace<Scene::EntitySceneProperty>(
      entity, defaultLayer, defaultSortingLayer
    );
    this->entities.push_back(entity);

    return entity;
  }

  void Scene::setEntityLayer(Scene::Entity entity, Scene::Layer layer)
  {
    this->registry.patch<Scene::EntitySceneProperty>(
      entity,
      [&layer](auto& props) {
        props.layer = layer;
      }
    );
  }

  Scene::Layer Scene::getEntityLayer(Scene::Entity entity)
  {
    const auto& props = this->registry.get<Scene::EntitySceneProperty>(entity);
    return props.layer;
  }

  void Scene::setEntitySortingLayer(Scene::Entity entity,
                                    Scene::SortingLayer layer)
  {
    this->registry.patch<Scene::EntitySceneProperty>(
      entity,
      [&layer](auto& props) {
        props.sortingLayer = layer;
      }
    );
  }

  Scene::SortingLayer Scene::getEntitySortingLayer(Scene::Entity entity)
  {
    const auto& props = this->registry.get<Scene::EntitySceneProperty>(entity);
    return props.layer;
  }

  void Scene::enableLayer(Scene::Layer layer)
  {
    this->layerAvailabilityStatuses[layer] = true;
  }

  void Scene::disableLayer(Scene::Layer layer)
  {
    this->layerAvailabilityStatuses[layer] = false;
  }

  bool Scene::isLayerEnabled(Scene::Layer layer)
  {
    return this->layerAvailabilityStatuses[layer];
  }

  void Scene::enableSortingLayer(Scene::SortingLayer layer)
  {
    this->sortingLayerAvailabilityStatuses[layer] = true;
  }

  void Scene::disableSortingLayer(Scene::SortingLayer layer)
  {
    this->sortingLayerAvailabilityStatuses[layer] = false;
  }

  bool Scene::isSortingLayerEnabled(Scene::SortingLayer layer)
  {
    return this->sortingLayerAvailabilityStatuses[layer];
  }

  SceneStatus Scene::getStatus()
  {
    return this->status;
  }

  float Scene::getPPMRatio()
  {
    return this->ppmRatio;
  }

  void Scene::setSceneStatus(SceneStatus status)
  {
    this->status = status;
  }
}
