# - Find Xerces-C
# Find the Xerces-C includes and library
#
#  XERCESC_INCLUDE_DIR - Where to find xercesc include sub-directory.
#  XERCESC_LIBRARIES   - List of libraries when using Xerces-C.
#  XERCESC_FOUND       - True if Xerces-C found.

IF (XERCESC_INCLUDE_DIR)
  # Already in cache, be silent.
  SET(XERCESC_FIND_QUIETLY TRUE)
ENDIF (XERCESC_INCLUDE_DIR)

FIND_PATH(XERCESC_INCLUDE_DIR xercesc)

SET(XERCESC_NAMES xerces-c xerces-c_2)
FIND_LIBRARY(XERCESC_LIBRARY NAMES ${XERCESC_NAMES} )

# Handle the QUIETLY and REQUIRED arguments and set XERCESC_FOUND to
# TRUE if all listed variables are TRUE.
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(
  XERCESC DEFAULT_MSG
  XERCESC_LIBRARY XERCESC_INCLUDE_DIR
)

IF(XERCESC_FOUND)
  SET( XERCESC_LIBRARIES ${XERCESC_LIBRARY} )
ELSE(XERCESC_FOUND)
  SET( XERCESC_LIBRARIES )
ENDIF(XERCESC_FOUND)

MARK_AS_ADVANCED( XERCESC_LIBRARY XERCESC_INCLUDE_DIR )
