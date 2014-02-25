
cmake_minimum_required (VERSION 2.8)

#--------------------------#
# Documentation



FIND_PACKAGE(Doxygen)
IF(DOXYGEN)
  Set(DOCSOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc)
  #SET(DOXYGEN_IMAGE_DIRS "\"${CMAKE_CURRENT_SOURCE_DIR}/figures/\"" )
  SET(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}\" ")
  SET(DOXYGEN_SOURCE_DIRS "${DOXYGEN_SOURCE_DIRS} \"${CMAKE_CURRENT_SOURCE_DIR}/../../include/\" ")


  #SET(HTML_EXTRA_FILES "${CMAKE_CURRENT_SOURCE_DIR}/../cheatsheet/cheatsheet_latest.pdf")
  Set(HTML_DIR ${CMAKE_CURRENT_SOURCE_DIR}/doc/html)
  SET(DOXYGEN_QUIET YES)
  MAKE_DIRECTORY(${HTML_DIR})
  SET( HAVE_DOT YES )
  
  CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.config.in ${DOCSOURCE_DIR}/doxygen.config IMMEDIATE)
  
  ADD_CUSTOM_TARGET(doc ${DOXYGEN} --silent ${DOCSOURCE_DIR}/doxygen.config)
ENDIF(DOXYGEN)

#-------------------------#