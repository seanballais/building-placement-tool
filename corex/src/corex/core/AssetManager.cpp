#include <cassert>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <EASTL/shared_ptr.h>
#include <EASTL/string.h>
#include <EASTL/unordered_map.h>
#include <EASTL/utility.h>
#include <EASTL/vector.h>
#include <nlohmann/json.hpp>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <SDL_gpu.h>

#include <corex/core/Asset.hpp>
#include <corex/core/AssetDBRecord.hpp>
#include <corex/core/AssetManager.hpp>
#include <corex/core/asset_types/SpritesheetData.hpp>
#include <corex/core/asset_types/SpritesheetState.hpp>
#include <corex/core/asset_types/Texture.hpp>
#include <corex/core/utils.hpp>

namespace corex::core
{
  AssetManager::AssetManager()
  {
    loadAssetDB();
  }

  Font AssetManager::getFont(eastl::string assetID,
                             int32_t size,
                             bool forceReload)
  {
    // Maybe not the best way to concatenate but this will do for now.
    const eastl::string uniqueFontID = assetID + eastl::string("#")
                                       + eastl::to_string(size);

    auto iter = this->assetDB.find(assetID);

    if (iter == this->assetDB.end()) {
      std::cout << "Attempted to get a font asset with an invalid ID, "
                << eaStrToStdStr(assetID) << ", and a font size of " << size
                << std::endl;
    } else {
      AssetDBRecord& parentFontRecord = iter->second;
      iter = this->assetDB.find(uniqueFontID);
      if (iter != this->assetDB.end()) {
        if (iter->second.assetType != "font") {
          std::cout << "Attempted to retrieve a non-font asset." << std::endl;
          STUBBED("The assert above should be an error log message.");
        }
      } else {
        this->assetDB[uniqueFontID] = AssetDBRecord();
        this->assetDB[uniqueFontID].assetType = parentFontRecord.assetType;
        this->assetDB[uniqueFontID].filePath = parentFontRecord.filePath;

        iter = this->assetDB.find(uniqueFontID);
      }

      if (eastl::get_if<Font>(&(iter->second.data)) == nullptr
          || forceReload) {
        eastl::string fontPath = iter->second.filePath;
        TTF_Font* font = TTF_OpenFont(fontPath.c_str(), size);
        if (font == nullptr) {
          std::cout << "Font file, " << eaStrToStdStr(fontPath)
                    << ", does not exist." << std::endl;
          STUBBED("The assert above should be an error log message.");
        }

        eastl::get<Font>(iter->second.data).reset(
          font, [](TTF_Font* font) { TTF_CloseFont(font); });
      }
    }

    return eastl::get<Font>(iter->second.data);
  }

  Texture AssetManager::getTexture(eastl::string assetID, bool forceReload)
  {
    auto iter = this->assetDB.find(assetID);
    if (iter != this->assetDB.end()) {
      if (iter->second.assetType != "image") {
        std::cout << "Attempted to retrieve a non-image asset." << std::endl;
        STUBBED("The assert above should be an error log message.");
      }

      if (eastl::get_if<Texture>(&(iter->second.data)) == nullptr
          || forceReload) {
        STUBBED("We should check if image exists.");
        eastl::string texturePath = iter->second.filePath;
        GPU_Image* texture = this->loadTexture(texturePath);
        eastl::get<Texture>(iter->second.data).reset(
          texture, [](GPU_Image* texture) { GPU_FreeImage(texture); });
      }
    } else {
      std::cout << "Attempted to get a texture asset with an invalid ID, "
                << eaStrToStdStr(assetID) << "." << std::endl;
    }

    return eastl::get<Texture>(iter->second.data);
  }

  SpritesheetData
  AssetManager::getSpritesheet(eastl::string assetID, bool forceReload)
  {
    auto iter = this->assetDB.find(assetID);
    if (iter != this->assetDB.end()) {
      if (iter->second.assetType != "spritesheet") {
        std::cout << "Attempted to retrieve a non-spritesheet asset."
                  << std::endl;
        STUBBED("The assert above should be an error log message.");
      }

      if (eastl::get_if<SpritesheetData>(&(iter->second.data)) == nullptr
            || forceReload) {
        eastl::string dataFilePath = iter->second.filePath;

        STUBBED("We should check if spritesheet data exists.");

        std::ifstream dataFile(eaStrToStdStr(dataFilePath));

        nlohmann::json spritesheetData;
        dataFile >> spritesheetData;

        // Get the texture.
        size_t extDotIndex = dataFilePath.find_last_of(".");
        eastl::string spriteSheetFilePath = dataFilePath.substr(0, extDotIndex);
        spriteSheetFilePath += ".png";

        STUBBED("We should check if spritesheet image exists.");

        GPU_Image* spritesheetImage = this->loadTexture(spriteSheetFilePath);

        // Get other spritesheet data.
        float timePerFrame = spritesheetData["time_per_frame"].get<float>();

        // Get frame data.
        eastl::vector<SDL_Rect> frames;
        nlohmann::json frameData = spritesheetData["frames"];
        for (auto iter = frameData.begin(); iter != frameData.end(); iter++) {
          STUBBED("Check for fail states when loading spritesheet frame data.");
          int32_t x = (*iter)["x"].get<int32_t>();
          int32_t y = (*iter)["y"].get<int32_t>();
          int32_t w = (*iter)["w"].get<int32_t>();
          int32_t h = (*iter)["h"].get<int32_t>();

          // Gah! Can't wait for C++ 20 to finally allow aggregates to be
          // initialized using parenthesis-style initialization.
          frames.push_back(eastl::move(SDL_Rect{ x, y, w, h }));
        }

        // Get custom states.
        eastl::unordered_map<eastl::string, SpritesheetState> customStates;
        nlohmann::json stateData = spritesheetData["custom_states"];
        for (auto iter = stateData.begin(); iter != stateData.end(); iter++) {
          eastl::string name = stdStrToEAStr(
            (*iter)["name"].get<std::string>());
          int32_t startFrame = (*iter)["start_frame"].get<int32_t>();
          int32_t endFrame = (*iter)["end_frame"].get<int32_t>();
          bool isLooped = (*iter)["is_looped"].get<bool>();

          customStates[name] = SpritesheetState{
            startFrame,
            endFrame,
            isLooped
          };
        }

        // Save the data.
        iter->second.data = SpritesheetData{
          eastl::shared_ptr<GPU_Image>(
            spritesheetImage,
            [](GPU_Image* texture) { GPU_FreeImage(texture); }),
          timePerFrame,
          frames,
          customStates
        };
      }
    } else {
      std::cout << "Attempted to get a spritesheet asset with an invalid ID, "
                << eaStrToStdStr(assetID) << "." << std::endl;
    }

    return eastl::get<SpritesheetData>(iter->second.data);
  }

  void AssetManager::unloadAsset(eastl::string assetID)
  {
    // This only unloads the asset from memory when there are no other
    // pre-existing references (via a shared pointer) to the asset.
    auto iter = this->assetDB.find(assetID);
    if (iter != this->assetDB.end()) {
      iter->second.data = eastl::monostate{};
    } else {
      std::cout << "Attempted to unload an asset with an invalid ID, "
                << eaStrToStdStr(assetID) << "." << std::endl;
    }
  }
  
  void AssetManager::reloadAssetDB()
  {
    // WARNING: This removes access to any assets that is already in memory, or
    //          unloads assets from memory if there are no pre-existing
    //          references (via a shared pointer) to the asset outside of the
    //          database.
    this->assetDB.clear();
    this->loadAssetDB();
  }

  void AssetManager::loadAssetDB()
  {
    std::filesystem::path assetFolderPath = getBinFolder() / "assets/";
    std::filesystem::path assetDBFileName = "assets.cxadb";
    std::ifstream assetDBFile(assetFolderPath / assetDBFileName);

    nlohmann::json assetDBData;
    assetDBFile >> assetDBData;

    for (auto iter = assetDBData.begin(); iter != assetDBData.end(); iter++) {
      STUBBED("Handle fail states for asset db loading.");
      // We ditched using an implementation of from_json(...) for eastl::string
      // here, since it causes a compile error where there is ambiguity of end()
      // in nlohmann/json.hpp (at least in Clang 10). So, we do the next best
      // thing -- converting the result of .get<std::string>() to an
      // eastl::string through a utility function.
      eastl::string assetID = stdStrToEAStr((*iter)["id"].get<std::string>());
      eastl::string assetType = stdStrToEAStr(
        (*iter)["type"].get<std::string>());
      
      std::filesystem::path assetFilePath = (*iter)["file"].get<std::string>();
      eastl::string absAssetFilePath = stdStrToEAStr(
        assetFolderPath / assetFilePath);

      this->assetDB[assetID] = AssetDBRecord();
      this->assetDB[assetID].assetType = assetType;
      this->assetDB[assetID].filePath = absAssetFilePath;
    }
  }

  GPU_Image* AssetManager::loadTexture(eastl::string texturePath)
  {
    return GPU_LoadImage(texturePath.c_str());
  }
}
