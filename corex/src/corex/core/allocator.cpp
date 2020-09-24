#include <cstdlib>

// Code based from here:
//   https://web.archive.org/web/20170314154559/
//           https://wuyingren.github.io/howto/2016/02/11/
//           Using-EASTL-in-your-projects/
// EASTL requires the following functions.
void* operator new[](size_t size, const char* pName, int flags,
                     unsigned debugFlags, const char* file, int line) 
{
  return malloc(size);
}  

void* operator new[](size_t size, size_t alignment, size_t alignmentOffset,
                     const char* pName, int flags, unsigned debugFlags,
                     const char* file, int line) 
{
  return malloc(size);
}
