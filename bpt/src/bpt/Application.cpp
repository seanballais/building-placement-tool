#include <iostream>

#include <EASTL/string.h>
#include <EASTL/unique_ptr.h>

#include <corex/core/Application.hpp>
#include <corex/core/Scene.hpp>

#include <bpt/Application.hpp>
#include <bpt/MainScene.hpp>

namespace bpt
{
  Application::Application(const eastl::string& windowTitle)
    : corex::core::Application(windowTitle) {}

  void Application::init()
  {
    std::cout << "Initializing BPT... Bleep, bloop, bleep." << std::endl;
    auto& mainScene = this->sceneManager->addScene<bpt::MainScene>();
    this->sceneManager->setRootScene(mainScene);
  }

  void Application::dispose()
  {
    std::cout << "Disposing BPT... Bleep, bloop, zzzzz." << std::endl;
  }
}

namespace corex
{
  eastl::unique_ptr<corex::core::Application> createApplication()
  {
    return eastl::make_unique<bpt::Application>(
      "BPT, the undergraduate thesis of Sean Ballais");
  }
}
