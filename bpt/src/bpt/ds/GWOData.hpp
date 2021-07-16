#ifndef BPT_DS_GWO_DATA_HPP
#define BPT_DS_GWO_DATA_HPP

#include <EASTL/vector.h>

#include <corex/core/ds/VecN.hpp>

namespace bpt
{
  struct GWOData
  {
    // Each iteration only has one alpha, one beta, one delta, and one alpha
    // value.
    eastl::vector<cx::VecN> alphaWolves;
    eastl::vector<cx::VecN> betaWolves;
    eastl::vector<cx::VecN> deltaWolves;
    eastl::vector<float> alphaValues;

    // Each wolf in an iteration have different values for the following.
    eastl::vector<eastl::vector<cx::VecN>> r1Alphas;
    eastl::vector<eastl::vector<cx::VecN>> r1Betas;
    eastl::vector<eastl::vector<cx::VecN>> r1Deltas;

    eastl::vector<eastl::vector<cx::VecN>> r2Alphas;
    eastl::vector<eastl::vector<cx::VecN>> r2Betas;
    eastl::vector<eastl::vector<cx::VecN>> r2Deltas;

    eastl::vector<eastl::vector<cx::VecN>> Aalphas;
    eastl::vector<eastl::vector<cx::VecN>> Abetas;
    eastl::vector<eastl::vector<cx::VecN>> Adeltas;

    eastl::vector<eastl::vector<cx::VecN>> Calphas;
    eastl::vector<eastl::vector<cx::VecN>> Cbetas;
    eastl::vector<eastl::vector<cx::VecN>> Cdeltas;

    eastl::vector<eastl::vector<cx::VecN>> Dalphas;
    eastl::vector<eastl::vector<cx::VecN>> Dbetas;
    eastl::vector<eastl::vector<cx::VecN>> Ddeltas;

    eastl::vector<eastl::vector<cx::VecN>> X1s;
    eastl::vector<eastl::vector<cx::VecN>> X2s;
    eastl::vector<eastl::vector<cx::VecN>> X3s;

    eastl::vector<eastl::vector<cx::VecN>> oldWolves;
    eastl::vector<eastl::vector<cx::VecN>> newWolves;
  };
}

#endif