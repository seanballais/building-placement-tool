#ifndef BPT_APPLICATION_CPP
#define BPT_APPLICATION_CPP

#include <EASTL/string.h>

#include <corex/core/Application.hpp>

namespace bpt
{
  class Application : public corex::core::Application
  {
  public:
    Application(const eastl::string& windowTitle);

    void init() override;
    void dispose() override;
  };
}

#endif

