# - Find fftw3f
# Find the native Fftw3f headers and libraries.
#
#  Fftw3f_INCLUDE_DIRS - where to find fftw3.h, etc.
#  Fftw3f_LIBRARIES    - List of libraries when using fftw3f.
#  Fftw3f_FOUND        - True if fftw3f found.

#=============================================================================
# Copyright 2006-2009 Kitware, Inc.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distributed this file outside of CMake, substitute the full
#  License text for the above reference.)

# Look for the header file.
FIND_PATH(Fftw3f_INCLUDE_DIR NAMES fftw3.h)

# Look for the library.
FIND_LIBRARY(Fftw3f_LIBRARY_BASE NAMES fftw3f libfftw3f)
FIND_LIBRARY(Fftw3f_LIBRARY_THREAD NAMES fftw3f_threads libfftw3f_threads)

# handle the QUIETLY and REQUIRED arguments and set Fftw3f_FOUND to TRUE if 
# all listed variables are TRUE
# INCLUDE(FindPackageHandleStandardArgs)
# FIND_PACKAGE_HANDLE_STANDARD_ARGS(Fftw3f DEFAULT_MSG Fftw3f_LIBRARY_BASE Fftw3f_INCLUDE_DIR)

SET(Fftw3f_FOUND FALSE)
IF(Fftw3f_INCLUDE_DIR)
  SET(Fftw3f_FOUND TRUE )
ENDIF(Fftw3f_INCLUDE_DIR)

# Copy the results to the output variables.
IF(Fftw3f_FOUND)
  SET(Fftw3f_LIBRARIES ${Fftw3f_LIBRARY_BASE} ${Fftw3f_LIBRARY_THREAD})
  SET(Fftw3f_INCLUDE_DIRS ${Fftw3f_INCLUDE_DIR})
ENDIF(Fftw3f_FOUND)

MARK_AS_ADVANCED(Fftw3f_INCLUDE_DIR Fftw3f_LIBRARIES)
