#include <cstdlib>

#include <EASTL/algorithm.h>
#include <EASTL/vector.h>

#include <corex/core/Timer.hpp>
#include <corex/core/math_functions.hpp>
#include <corex/core/ds/NPolygon.hpp>
#include <corex/core/ds/Point.hpp>
#include <corex/core/ds/VecN.hpp>

#include <bpt/evaluator.hpp>
#include <bpt/generator.hpp>
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

    cx::Point minBoundingPt = boundingArea.vertices[0];

    float boundWidth = boundingArea.vertices[1].x
                       - boundingArea.vertices[0].x;
    float boundHeight = boundingArea.vertices[3].y
                        - boundingArea.vertices[0].y;

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

        // Correct orientations, and clamp the buildings.
        for (int32_t j = 0; j < vecSize / 3; j++) {
          // Fix orientation.
          float orientation = cx::mod(cx::abs(XVecN[(j * 3) + 2]), 360.f);
          if (cx::floatLessEqual(orientation, 180.f)) {
            XVecN[(j * 3) + 2] = 0.f;
          } else {
            XVecN[(j * 3) + 2] = 90.f;
          }

          // Clamp buildings.
          cx::Rectangle buildingRect {
            XVecN[(j * 3)],
            XVecN[(j * 3) + 1],
            inputBuildings[j].width,
            inputBuildings[j].length,
            XVecN[(j * 3) + 2]
          };

          cx::Polygon<4> poly = cx::rotateRectangle(buildingRect);

          float polyWidth = poly.vertices[1].x - poly.vertices[0].x;
          float polyHeight = poly.vertices[3].y - poly.vertices[0].y;

          float minBuildingXVal = minBoundingPt.x + (polyWidth / 2);
          float maxBuildingXVal = minBoundingPt.x
                                  + (boundWidth - (polyWidth / 2));
          float minBuildingYVal = minBoundingPt.y + (polyHeight / 2);
          float maxBuildingYVal = minBoundingPt.y
                                  + (boundHeight - (polyHeight / 2));

          if (minBuildingXVal > maxBuildingXVal) {
            eastl::swap(minBuildingXVal, maxBuildingXVal);
          }

          if (minBuildingYVal > maxBuildingYVal) {
            eastl::swap(minBuildingYVal, maxBuildingYVal);
          }

          XVecN[j * 3] = cx::clamp(XVecN[j * 3],
                                   minBuildingXVal,
                                   maxBuildingXVal);
          XVecN[(j * 3) + 1] = cx::clamp(XVecN[(j * 3) + 1],
                                         minBuildingYVal,
                                         maxBuildingYVal);
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
