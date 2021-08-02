#!/bin/bash

ROOT_DIR=`pwd`/

echo "[::] Switching to build directory..."
mkdir -p build
cd build

BUILD_ACTION_TYPE=""
if [[ -n "$1" && "$1" == "full" ]]
then
    BUILD_ACTION_TYPE="full"
else
    BUILD_ACTION_TYPE="casual"
fi

BUILD_TYPE=""
if [[ -n "$2" && "$2" == "release" ]]
then
    BUILD_TYPE="release"
else
    BUILD_TYPE="debug"
fi

# Conan time.
if [ "$BUILD_ACTION_TYPE" == "full" ]
then
    echo "[::] Performing a full $BUILD_TYPE build."
    echo "[::] Running Conan..."
    conan install -pr=$ROOT_DIR/conan_profiles/linux_$BUILD_TYPE.txt \
                  -u \
                  --build missing \
                  ..

    # Hello, CMake. And we gonna use Clang 11.
    echo "[::] Running CMake for a $BUILD_TYPE build..."
    export CC=/usr/bin/clang-11
    export CXX=/usr/bin/clang++-11

    cmake -DCMAKE_BUILD_TYPE=${BUILD_TYPE^} .. \
        && echo "[::] CMake ran successfully. Time to compile..." \
        && make
    cd ..
else
    echo "[::] Performing a casual build."
    make
    if [ $? -ne 0 ]
    then
        echo "[::] Performing a casual build failed. Maybe run a full build" \
             "first?"
        cd ..
    else
        make

        cd ..

        echo "[::] Copying assets and the settings file to the build assets" \
             "folder, if any were updated or new assets are added."

        # Copy the settings file to the bin folder if changes were made.
        if [ settings/settings.cxstg -nt build/bin/settings/settings.cxstg ]
        then
            cp settings/settings.cxstg build/bin/settings/
            echo "---- Copied settings/settings.cxstg to build/bin/settings/."
        fi

        # Copy the data folder to the bin folder if changes were made.
        for file in `find data -type f`
        do
            if [[ ( -f "build/bin/${file}" && $file -nt "build/bin/${file}" ) \
               || ( ! -f "build/bin/${file}" ) ]]
            then
                cp $file "build/bin/${file}"
                echo "---- Copied ${file} to build/bin/data/."
            fi
        done

        # Copy assets to the bin folder if changes were made.
        for file in `find assets -type f -not -path "assets/raw/**"`
        do
            if [[ ( -f "build/bin/${file}" && $file -nt "build/bin/${file}" ) \
               || ( ! -f "build/bin/${file}" ) ]]
            then
                cp $file "build/bin/${file}"
                echo "---- Copied ${file} to build/bin/assets/."
            fi
        done
    fi
fi

# Go back to the main directory.
echo "[::] Done with stuff. Switching back to project root..."
