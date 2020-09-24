#ifdef __linux__
#include <climits>
#include <unistd.h>
#else
#error "No support available for non-Linux systems."
#endif
#include <filesystem>
#include <string>

#include <EASTL/string.h>

#include <corex/core/utils.hpp>

namespace corex::core
{
  std::filesystem::path getBinFolder()
  {
    // Based on the answer from:
    //   https://stackoverflow.com/a/55579815/1116098
#ifdef __linux__
    char path[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", path, PATH_MAX);
    std::string binPath = std::string(path, (count > 0) ? count : 0);

    return std::filesystem::path(binPath).parent_path();
#else
#error "No support available for non-Linux systems."
#endif
  }

  std::filesystem::path getSettingsFolder()
  {
    return getBinFolder() / "settings";
  }

  std::string eaStrToStdStr(const eastl::string& str)
  {
    return std::string(str.c_str());
  }

  eastl::string stdStrToEAStr(const std::string& str)
  {
    return eastl::string(str.c_str());
  }

  float metersToPixels(float meters, float ppmRatio)
  {
    return meters * ppmRatio;
  }
}
