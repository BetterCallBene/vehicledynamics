#!/usr/bin/env bash

mkdir build
cd build
omc ../scripts/checkModel.mos > output.txt
if grep -q Error: output.txt; then
    exit 1
else
    exit 0
fi