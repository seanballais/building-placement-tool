#include <cstdlib>

#include <EASTL/algorithm.h>
#include <EASTL/vector.h>

#include <corex/core/Timer.hpp>
#include <corex/core/math_functions.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/VecN.hpp>

#include <bpt/evaluator.hpp>
#include <bpt/generator.hpp>
#include <bpt/HC.hpp>
#include <bpt/PSO.hpp>
#include <bpt/utils.hpp>
#include <bpt/ds/InputBuilding.hpp>
#include <bpt/ds/PSOResult.hpp>

namespace bpt
{
  PSO::PSO()
    : runTimer()
    , currRunIterationNumber(0) {}

  PSOResult PSO::generateSolutions(
    const eastl::vector<InputBuilding> &inputBuildings,
    const corex::core::NPolygon &boundingArea,
    const eastl::vector<eastl::vector<float>> &flowRates,
    const float buildingDistanceWeight,
    const int32_t numIterations,
    const int32_t numParticles,
    const float w,
    const float c1,
    const float c2)
  {
    // NOTE: Play around with currSol instead of pBest. pBest is only a
    //       holder for a particle's best solution.
    eastl::vector<eastl::vector<Solution>> solutions;
    eastl::vector<Particle> particles;
    eastl::vector<double> bestFitnesses;
    eastl::vector<double> averageFitnesses;
    eastl::vector<double> worstFitnesses;
    Solution gBest; // Can be improved by using a pointer instead.
                    // Not doing it right now, because we might break the
                    // algorithm.

    this->runTimer.start();

    const int32_t vecSize = static_cast<int32_t>(inputBuildings.size()) * 3;

    // Generate initial wolves.
    for (int32_t i = 0; i < numParticles; i++) {
      std::cout << "Generating Particle #" << i << "\n";

      Particle particle{
        Solution(),
        generateRandomSolution(inputBuildings, boundingArea),
        cx::VecN(vecSize)
      };
      particle.pBest = particle.currSol;

      particles.push_back(particle);
    }

    // Evaluate individual fitness.
    this->computeWolfValues(particles,
                            inputBuildings,
                            boundingArea,
                            flowRates,
                            buildingDistanceWeight);
    for (Particle& particle : particles) {
      particle.pBest.setFitness(particle.currSol.getFitness());
    }

    std::sort(
      particles.begin(),
      particles.end(),
      [](Particle& a, Particle& b) {
        return a.currSol.getFitness() < b.currSol.getFitness();
      }
    );

    gBest = particles[0].currSol;

    double worstFitness = particles.back().currSol.getFitness();
    double bestFitness = particles[0].currSol.getFitness();

    double averageFitness = 0.0;
    for (Particle& particle : particles) {
      averageFitness += particle.currSol.getFitness();
    }
    averageFitness /= particles.size();

    bestFitnesses.push_back(bestFitness);
    averageFitnesses.push_back(averageFitness);
    worstFitnesses.push_back(worstFitness);

    eastl::vector<Solution> currIterSolutions;
    eastl::transform(particles.begin(), particles.end(),
                     eastl::back_inserter(currIterSolutions),
                     [](Particle& p) -> Solution { return p.currSol; });
    solutions.push_back(currIterSolutions);

    for (int32_t i = 0; i < numIterations; i++) {
      this->currRunIterationNumber = i;

      auto gBestVecN = convertSolutionToVecN(gBest);

      // Update the particles.
      for (Particle& particle : particles) {
        auto pBestVecN = convertSolutionToVecN(particle.pBest);
        auto currSolVecN = convertSolutionToVecN(particle.currSol);

        auto r1 = createRandomVector(vecSize);
        auto r2 = createRandomVector(vecSize);

        particle.velocity =
          (w * particle.velocity)
          + cx::pairwiseMult(c1 * r1, pBestVecN - currSolVecN)
          + cx::pairwiseMult(c2 * r2, gBestVecN - currSolVecN);

        auto XVecN = currSolVecN + particle.velocity;

        // Correct orientations
        for (int32_t j = 0; j < vecSize / 3; j++) {
          float orientation = cx::mod(cx::abs(XVecN[(j * 3) + 2]), 360.f);
          if (cx::floatLessEqual(orientation, 180.f)) {
            XVecN[(j * 3) + 2] = 0.f;
          } else {
            XVecN[(j * 3) + 2] = 90.f;
          }
        }

        particle.currSol = convertVecNToSolution(XVecN);
      }

      // Evaluate individual fitness.
      this->computeWolfValues(particles,
                              inputBuildings,
                              boundingArea,
                              flowRates,
                              buildingDistanceWeight);
      for (Particle& particle : particles) {
        if (particle.currSol.getFitness() < particle.pBest.getFitness()) {
          particle.pBest = particle.currSol;
        }
      }

      // Sort the particles.
      std::sort(
        particles.begin(),
        particles.end(),
        [](Particle& a, Particle& b) {
          return a.currSol.getFitness() < b.currSol.getFitness();
        }
      );

      if (particles[0].currSol.getFitness() < gBest.getFitness()) {
        gBest = particles[0].currSol;
      }

      // Gather run data.
      double worstFitness = particles.back().currSol.getFitness();
      double bestFitness = particles[0].currSol.getFitness();

      double averageFitness = 0.0;
      for (Particle& particle : particles) {
        averageFitness += particle.currSol.getFitness();
      }
      averageFitness /= particles.size();

      bestFitnesses.push_back(bestFitness);
      averageFitnesses.push_back(averageFitness);
      worstFitnesses.push_back(worstFitness);

      currIterSolutions.clear();
      eastl::transform(particles.begin(), particles.end(),
                       eastl::back_inserter(currIterSolutions),
                       [](Particle& p) -> Solution { return p.currSol; });
      solutions.push_back(currIterSolutions);
    }

    double elapsedTime = this->runTimer.getElapsedTime();
    this->runTimer.stop();

    this->currRunIterationNumber = 0;

    return PSOResult{
      solutions,
      inputBuildings,
      bestFitnesses,
      averageFitnesses,
      worstFitnesses,
      elapsedTime,
      numParticles,
      numIterations,
      w,
      c1,
      c2
    };
  }

  int32_t PSO::getCurrentRunIterationNumber()
  {
    return this->currRunIterationNumber;
  }

  void PSO::computeWolfValues(
    eastl::vector<Particle>& particles,
    const eastl::vector<InputBuilding>& inputBuildings,
    const corex::core::NPolygon& boundingArea,
    const eastl::vector<eastl::vector<float>>& flowRates,
    const float buildingDistanceWeight)
  {
    for (Particle& particle : particles) {
      particle.currSol.setFitness(
        computeSolutionFitness(particle.currSol,
                               inputBuildings,
                               boundingArea,
                               flowRates,
                               buildingDistanceWeight));
    }
  }
}
