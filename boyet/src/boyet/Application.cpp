#include <iostream>

#include <EASTL/unique_ptr.h>

#include <corex/core/Application.hpp>
#include <corex/core/Scene.hpp>

#include <boyet/Application.hpp>
#include <boyet/scenes/TheVoid.hpp>

namespace boyet
{
  Application::Application(const eastl::string& windowTitle)
    : corex::core::Application(windowTitle) {}

  void Application::init()
  {
    std::cout << "Initializing Boyet... Bleep, bloop..." << std::endl;
    auto& theVoidScene = this->sceneManager->addScene<boyet::scenes::TheVoid>();
    this->sceneManager->setRootScene(theVoidScene);
  }

  void Application::dispose()
  {
    std::cout << "Disposing Boyet... Bleep, bloop..." << std::endl;
  }
}

namespace corex
{
  eastl::unique_ptr<corex::core::Application> createApplication()
  {
    return eastl::make_unique<boyet::Application>("Boyet: Cryptid Hunter");
  }
}
