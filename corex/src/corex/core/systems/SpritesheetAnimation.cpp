#include <cstdlib>

#include <entt/entt.hpp>

#include <corex/core/components/Sprite.hpp>
#include <corex/core/components/SpritesheetCurrentFrameIndex.hpp>
#include <corex/core/components/SpritesheetCurrentState.hpp>
#include <corex/core/components/SpritesheetFrameData.hpp>
#include <corex/core/components/SpritesheetFrames.hpp>
#include <corex/core/events/metric_events.hpp>
#include <corex/core/systems/BaseSystem.hpp>
#include <corex/core/systems/SpritesheetAnimation.hpp>

#include <iostream>

namespace corex::core
{
  SpritesheetAnimation::SpritesheetAnimation(entt::dispatcher& eventDispatcher,
                                             entt::registry& registry)
    : BaseSystem(eventDispatcher)
    , registry(registry)
    , timeDelta(0.0f)
  {
    this->eventDispatcher.sink<FrameDataEvent>()
                         .connect<&SpritesheetAnimation::handleFrameDataEvents>(
                            this);
  }

  void SpritesheetAnimation::update()
  {
    auto view = this->registry.view<SpritesheetFrameData>();
    for (entt::entity e : view) {
      auto& frameData = view.get<SpritesheetFrameData>(e);
      frameData.currFrameElapsedTime += this->timeDelta;

      if (frameData.currFrameElapsedTime > frameData.timePerFrame) {
        // Change frame.
        auto& frames = this->registry.get<SpritesheetFrames>(e);
        auto& currState = this->registry.get<SpritesheetCurrentState>(e);
        auto& frameIndex = this->registry.get<SpritesheetCurrentFrameIndex>(e);

        int32_t currFrameIndex = frameIndex.currFrameIndex;
        if (currFrameIndex + 1 >= currState.currentState.endFrame
            && currState.currentState.isLooping) {
          currFrameIndex = currState.currentState.startFrame;
        } else if (currFrameIndex < currState.currentState.endFrame) {
          currFrameIndex++;
        }

        frameIndex.currFrameIndex = currFrameIndex;

        auto& sprite = this->registry.get<Sprite>(e);
        SDL_Rect& newFrame = frames.frames[currFrameIndex];
        sprite.x = newFrame.x;
        sprite.y = newFrame.y;
        sprite.width = newFrame.w;
        sprite.height = newFrame.h;

        frameData.currFrameElapsedTime = 0.0f;
      }
    }
  }

  void SpritesheetAnimation::handleFrameDataEvents(const FrameDataEvent& e)
  {
    this->timeDelta = e.timeDelta;
  }
}
