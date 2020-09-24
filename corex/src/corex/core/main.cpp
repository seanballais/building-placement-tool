#include <EASTL/unique_ptr.h>

#include <corex/core/Application.hpp>

namespace corex {
  extern eastl::unique_ptr<corex::core::Application> createApplication();
}

int main(int argc, char** argv)
{
  auto gameApp = corex::createApplication();
  gameApp->run();

  gameApp.reset(nullptr);

  return 0;
}
