# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/esp/esp-idf/components/bootloader/subproject"
  "D:/Work/Esp32Projects/WaterPump/build/bootloader"
  "D:/Work/Esp32Projects/WaterPump/build/bootloader-prefix"
  "D:/Work/Esp32Projects/WaterPump/build/bootloader-prefix/tmp"
  "D:/Work/Esp32Projects/WaterPump/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Work/Esp32Projects/WaterPump/build/bootloader-prefix/src"
  "D:/Work/Esp32Projects/WaterPump/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Work/Esp32Projects/WaterPump/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
