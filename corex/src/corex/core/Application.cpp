#include <cstdlib>
#include <filesystem>
#include <iostream>

#include <EASTL/unique_ptr.h>
#include <entt/entt.hpp>
#include <GL/glew.h>
#include <imgui.h>
#include <imgui_impls/imgui_impl_opengl3.h>
#include <imgui_impls/imgui_impl_sdl.h>
#include <implot/implot.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL_gpu.h>

#include <corex/core/AssetManager.hpp>
#include <corex/core/Application.hpp>
#include <corex/core/math_functions.hpp>
#include <corex/core/PerformanceMetrics.hpp>
#include <corex/core/SceneManager.hpp>
#include <corex/core/SceneManagerStatus.hpp>
#include <corex/core/Timer.hpp>
#include <corex/core/utils.hpp>
#include <corex/core/components/Position.hpp>
#include <corex/core/components/Renderable.hpp>
#include <corex/core/components/RenderCircle.hpp>
#include <corex/core/components/RenderableType.hpp>
#include <corex/core/components/RenderLineSegments.hpp>
#include <corex/core/components/RenderPolygon.hpp>
#include <corex/core/components/RenderRectangle.hpp>
#include <corex/core/components/Sprite.hpp>
#include <corex/core/components/Text.hpp>
#include <corex/core/ds/Polygon.hpp>
#include <corex/core/events/game_events.hpp>
#include <corex/core/events/metric_events.hpp>
#include <corex/core/events/scene_manager_events.hpp>
#include <corex/core/events/sys_events.hpp>

namespace corex::core
{
  Application::Application(const eastl::string& windowTitle)
    : sceneManager(nullptr)
    , assetManager(nullptr)
    , registry() // Nothing to do to initialize the EnTT registry. Just adding
                 // this here for sake of consistency.
    , eventDispatcher() // For sake of consistency, as well.
    , camera()
    , settings()
    , sysEventDispatcher(eventDispatcher)
    , keyboardHandler(eventDispatcher)
    , mouseHandler(eventDispatcher)
    , spritesheetAnimation(eventDispatcher, registry)
    , debugUI(eventDispatcher, camera)
    , windowManager(windowTitle, eventDispatcher, settings)
    , gameTimeWarpFactor(1.0f)
    , imGuiFilePath()
    , isGamePlaying(true)
  {
    // TODO: Get window settings from a settings module.
    if (SDL_Init(SDL_INIT_TIMER) != 0) {
      // TODO: Use a logging system.
      std::cout << "Error! Bleep, bloop. Bleh. " << SDL_GetError() << std::endl;
      STUBBED("Need to find a way to close the game on SDL init error.");
    }

    if (TTF_Init() != 0) {
      std::cout << "Error! Bleep, bloop. Bleh. " << TTF_GetError() << std::endl;
      STUBBED("Need to find a way to close the game on TTF init error.");
    }

    bool glewInitErrored = glewInit() != GLEW_OK;
    if (glewInitErrored) {
      std::cout << "Error! Bleep, bloop. Bleh. Failed to initialize OpenGL "
                << "loader."
                << std::endl;
      STUBBED("Need to find a way to close the game on init error.");
    }

    this->displayGraphicsAPIInfo();

    // Set up Dear ImGui context.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplSDL2_InitForOpenGL(this->windowManager.getWindow(),
                                 this->windowManager.getOpenGLContext());
    ImGui_ImplOpenGL3_Init("#version 150"); // OpenGL 3.2, baby! For RenderDoc!

    std::filesystem::path tempImGuiFilePath = getSettingsFolder() / "imgui.ini";
    this->imGuiFilePath = stdStrToEAStr(tempImGuiFilePath.string());

    int32_t windowWidth = 0;
    int32_t windowHeight = 0;

    SDL_GetWindowSize(this->windowManager.getWindow(),
                      &windowWidth,
                      &windowHeight);

    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize.x = windowWidth;
    io.DisplaySize.y = windowHeight;
    io.IniFilename = this->imGuiFilePath.c_str();

    // Set up the managers.
    this->assetManager = eastl::make_unique<AssetManager>();
    this->sceneManager = eastl::make_unique<SceneManager>(
      this->registry,
      this->eventDispatcher,
      *(this->assetManager),
      this->camera);

    this->eventDispatcher.sink<GameTimeWarpEvent>()
                         .connect<&Application::handleGameTimeWarpEvents>(this);
    this->eventDispatcher.sink<GameTimerStatusEvent>()
                         .connect<&Application::handleGameTimerStatusEvents>(
                            this);
    this->eventDispatcher.sink<WindowEvent>()
                         .connect<&Application::handleWindowEvents>(this);
  }

  Application::~Application()
  {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    TTF_Quit();
    SDL_Quit();
  }

  void Application::run()
  {
    this->init();

    PerformanceMetrics metrics;

    Timer gameTimer;
    Timer appTimer;

    appTimer.start();
    gameTimer.start();
    while (true) {
      this->runEventSystems();

      // We prep the rendering before the scene updates, because the scene
      // might draw something.
      this->renderPrep();
      this->sceneManager->update(metrics.timeDelta);

      if (this->sceneManager->getStatus() == SceneManagerStatus::DONE) {
        break;
      }

      this->render();

      // Do performance-related tasks.
      metrics.timeDelta = gameTimer.getElapsedTime()
                          * this->gameTimeWarpFactor
                          * this->isGamePlaying;
      metrics.appTimeDelta = appTimer.getElapsedTime();
      gameTimer.reset();
      appTimer.reset();

      this->computePerformanceMetrics(metrics);
      this->dispatchPerformanceMetrics(metrics);
    }

    this->dispose();
  }

  void Application::displayGraphicsAPIInfo()
  {
    std::cout << "OpenGL Information" << std::endl;
    std::cout << "\tOpenGL Version: " << glGetString(GL_VERSION) << std::endl;
    std::cout << "\tGLSL Version: "
              << glGetString(GL_SHADING_LANGUAGE_VERSION)
              << std::endl;
    std::cout << "\tVendor: " << glGetString(GL_VENDOR) << std::endl;
    std::cout << "\tRenderer: " << glGetString(GL_RENDERER) << std::endl;
  }

  void Application::runEventSystems()
  {
    this->sysEventDispatcher.update();
    this->keyboardHandler.update();
    this->mouseHandler.update();
    this->spritesheetAnimation.update();
    this->dispatchSceneManagerEvents();
    this->debugUI.update();
    this->eventDispatcher.update();
  }

  void Application::dispatchPerformanceMetrics(PerformanceMetrics& metrics)
  {
    this->eventDispatcher.enqueue<FrameDataEvent>(metrics.fps,
                                                  metrics.numFrames,
                                                  metrics.timeDelta,
                                                  metrics.appTimeDelta);
  }

  void Application::dispatchSceneManagerEvents()
  {
    this->eventDispatcher.enqueue<PPMRatioChange>(
      this->sceneManager->getCurrentScenePPMRatio());
  }

  void Application::computePerformanceMetrics(PerformanceMetrics& metrics)
  {
    metrics.numFrames++;

    static double totalAvgFps = 0.0;
    static float timeCounter = 0.0f;
    static int32_t numFramesPerNSeconds = 1.0f;
    constexpr float weightRatio = 0.65f; // Felt like a good ratio.
    constexpr float timeLimit = 1.0f; // In seconds.

    if (timeCounter < timeLimit) {
      timeCounter += metrics.appTimeDelta;
      numFramesPerNSeconds++;
    } else {
      totalAvgFps = (totalAvgFps * weightRatio)
                    + (numFramesPerNSeconds * (1.0f - weightRatio));

      timeCounter = 0.0f;
      numFramesPerNSeconds = 0;
    }

    metrics.fps = static_cast<int32_t>(totalAvgFps);
  }

  void Application::handleGameTimeWarpEvents(const GameTimeWarpEvent& e)
  {
    this->gameTimeWarpFactor = e.timeWarpFactor;
  }

  void Application::handleGameTimerStatusEvents(const GameTimerStatusEvent& e)
  {
    this->isGamePlaying = e.isPlaying;
  }

  void Application::handleWindowEvents(const WindowEvent& e)
  {
    switch (e.event.window.event) {
      case SDL_WINDOWEVENT_RESIZED:
        ImGuiIO& io = ImGui::GetIO();
        io.DisplaySize.x = e.event.window.data1;
        io.DisplaySize.y = e.event.window.data2;
        break;
    }
  }

  void Application::renderPrep()
  {
    // Render time!
    GPU_ClearRGBA(this->windowManager.getRenderTarget(), 115, 140, 153, 255);
    GPU_SetCamera(this->windowManager.getRenderTarget(),
                  this->camera.getGPUCamera());

    // Start the Dear ImGui frame.
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame(this->windowManager.getWindow());
    ImGui::NewFrame();
  }

  void Application::render()
  {
    // Sort the renderables first.
    auto renderGroup = this->registry.group<Position, Renderable>();
    renderGroup.sort([this](const entt::entity lhs, const entt::entity rhs) {
      const Position& lhsPos = this->registry.get<Position>(lhs);
      const Position& rhsPos = this->registry.get<Position>(rhs);

      return (lhsPos.sortingLayerID < rhsPos.sortingLayerID)
        || ((lhsPos.sortingLayerID == rhsPos.sortingLayerID)
            && (lhsPos.z < rhsPos.z));
    });

    for (entt::entity e : renderGroup) {
      const Position& pos = this->registry.get<Position>(e);
      const Renderable& renderable = this->registry.get<Renderable>(e);

      switch (renderable.type) {
        case RenderableType::TEXT: {
          const Text& text = this->registry.get<Text>(e);
          GPU_Blit(text.getRenderableText(),
                   nullptr,
                   this->windowManager.getRenderTarget(),
                   pos.x,
                   pos.y);
        } break;
        case RenderableType::SPRITE: {
          const Sprite& sprite = this->registry.get<Sprite>(e);

          GPU_Rect renderRect = {
            static_cast<float>(sprite.x),
            static_cast<float>(sprite.y),
            static_cast<float>(sprite.width),
            static_cast<float>(sprite.height)
          };
          GPU_Rect targetRect = {
            pos.x - (static_cast<float>(sprite.width) / 2.f),
            pos.y - (static_cast<float>(sprite.height) / 2.f),
            static_cast<float>(sprite.width),
            static_cast<float>(sprite.height)
          };
          GPU_BlitRect(sprite.texture.get(),
                       &renderRect,
                       this->windowManager.getRenderTarget(),
                       &targetRect);
        } break;
        case RenderableType::PRIMITIVE_RECTANGLE: {
          const RenderRectangle& rect = this->registry.get<RenderRectangle>(e);
          Polygon<4> rotatedRect = rotateRectangle(rect.x,
                                                   rect.y,
                                                   rect.width,
                                                   rect.height,
                                                   rect.angle);

          // vertices will has 8 elements because it will contain the x and y
          // coordinates of the rectangle that will be drawn.
          float vertices[8] = {
            rotatedRect.vertices[0].x, rotatedRect.vertices[0].y,
            rotatedRect.vertices[1].x, rotatedRect.vertices[1].y,
            rotatedRect.vertices[2].x, rotatedRect.vertices[2].y,
            rotatedRect.vertices[3].x, rotatedRect.vertices[3].y
          };

          if (rect.isFilled) {
            GPU_PolygonFilled(this->windowManager.getRenderTarget(),
                              4,
                              vertices,
                              rect.colour);
          } else {
            // TODO: Add a debug mode where a polygon is not filled.
            GPU_Polygon(this->windowManager.getRenderTarget(),
                        4,
                        vertices,
                        rect.colour);
          }

        } break;
        case RenderableType::PRIMITIVE_POLYGON: {
          const RenderPolygon& poly = this->registry.get<RenderPolygon>(e);

          int32_t numVertices = poly.vertices.size();
          float vertices[numVertices * 2];
          for (int32_t i = 0; i < numVertices; i++) {
            int32_t vertXIndex = i * 2;
            int32_t vertYIndex = vertXIndex + 1;
            vertices[vertXIndex] = poly.vertices[i].x;
            vertices[vertYIndex] = poly.vertices[i].y;
          }

          if (poly.isFilled) {
            GPU_PolygonFilled(this->windowManager.getRenderTarget(),
                              numVertices,
                              vertices,
                              poly.colour);
          } else {
            GPU_Polygon(this->windowManager.getRenderTarget(),
                        numVertices,
                        vertices,
                        poly.colour);
          }
        } break;
        case RenderableType::LINE_SEGMENTS: {
          const RenderLineSegments& segments = this->registry
                                                    .get<RenderLineSegments>(e);

          if (segments.vertices.size() <= 1) {
            // Not enough vertices to form a line. 
            continue;
          }

          for (int32_t i = 0; i < segments.vertices.size() - 1; i++) {
            GPU_Line(this->windowManager.getRenderTarget(),
                     segments.vertices[i].x,
                     segments.vertices[i].y,
                     segments.vertices[i + 1].x,
                     segments.vertices[i + 1].y,
                     segments.colour);
          }
        } break;
        case RenderableType::PRIMITIVE_CIRCLE: {
          const RenderCircle& circle = this->registry.get<RenderCircle>(e);

          if (circle.isFilled) {
            GPU_CircleFilled(this->windowManager.getRenderTarget(),
                             pos.x,
                             pos.y,
                             circle.radius,
                             circle.colour);
          } else {
            GPU_Circle(this->windowManager.getRenderTarget(),
                       pos.x,
                       pos.y,
                       circle.radius,
                       circle.colour);
          }
        } break;
      }
    }

    GPU_FlushBlitBuffer();

    this->debugUI.render();

    ImGui::Render();
    SDL_GL_MakeCurrent(this->windowManager.getWindow(),
                       this->windowManager.getOpenGLContext());
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    GPU_Flip(this->windowManager.getRenderTarget());
  }
}
