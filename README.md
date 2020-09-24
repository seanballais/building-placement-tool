# Boyet: Cryptid Hunter

A 2D story-driven pixel art-style action game with RPG elements.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

In order to successfully compile and run this game, you must have the following installed:

* CMake (`v3.17.3` or newer)
* Conan (`v1.25.2` or newer)
* Clang 10 (or newer patch versions)
* SDL 2

Note that Clang is pinned to version 10. This is so that no bugs or performance issues that may arise from a newer version will not affect the final build. A compiler is a fundamental tool, and also determines the quaiity of the final build. Any compiler issues will affect the game in a negative way. Being conservative will the compiler will version will ensure stability of the compiler and the resulting game build.

### Installing and Running the Game

At the moment, only Linux is supported by the game. However, Windows, macOS, and consoles will be supported in the future.

A compilation script is already provided for Linux. To compile, simply run `compile_linux_debug.sh` in the project root to produce a debug build. A release version of the script will be provided in the future.

To run the game, run `./build/bin/boyet-game` in the project root.

## Running the tests

TODO: Explain how to run the automated tests for this system

### Break down into end to end tests

TODO: Explain what these tests test and why

```
Give an example
```

### And coding style tests

TODO: Explain what these tests test and why

```
Give an example
```

## Deployment

TODO: Add additional notes about how to deploy this on a live system

## Built With

* [Conan](https://conan.io/) - Package manager used for this project.
* [CMake](https://cmake.org/) - Tool used to build the game.
* [SDL 2](https://www.libsdl.org/) - Cross-platform multimedia library that provides access to low-level systems, such as the input, and graphics.
- [EASTL](https://github.com/electronicarts/EASTL) - Electronic Art's implementation of the C++ standard template library (STL) with an emphasis on high performance.
- [EnTT](https://github.com/skypjack/entt) - A library that provides an entity-component-system (ECS) and more.
- [Catch 2](https://github.com/catchorg/Catch2) - A test framework for unit tests for C++.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/seanballais/boyet-game/tags). 

## Author

* **Sean Ballais** - *Creator* - [seanballais](https://github.com/seanballais)

## License

This project is proprietary. If you are viewing this code, then you should already have had permission from the author to view this.

## Acknowledgments

* Thanks to Billie Thompson ([@PurpleBooth](https://github.com/PurpleBooth) in GitHub) for providing the template for this `README.md`. Template can be accessed [here](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2).
