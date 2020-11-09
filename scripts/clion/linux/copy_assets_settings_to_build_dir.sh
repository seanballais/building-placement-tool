#!/bin/bash

# This script is for use with CLion. We need this so that we can copy any assets
# and settings files to the build folder before running bpt.

echo "Copying assets and the settings file to the build assets folder, if" \
	 "any were updated or new assets are added."

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
