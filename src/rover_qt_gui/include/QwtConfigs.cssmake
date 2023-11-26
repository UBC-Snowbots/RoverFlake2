
if (Qt5Gui_FOUND)
  get_target_property(QT_INCLUDE_DIR Qt5::Gui INTERFACE_INCLUDE_DIRECTORIES)
endif ()

find_path ( QWT_INCLUDE_DIR
  NAMES qwt_plot.h qwt_thermo.h
  HINTS ${QT_INCLUDE_DIR}
  PATH_SUFFIXES qwt qwt-qt5 qwt6
)

set ( QWT_INCLUDE_DIRS ${QWT_INCLUDE_DIR} )

# version
set ( _VERSION_FILE ${QWT_INCLUDE_DIR}/qwt_global.h )
if ( EXISTS ${_VERSION_FILE} )
  file ( STRINGS ${_VERSION_FILE} _VERSION_LINE REGEX "define[ ]+QWT_VERSION_STR" )
  if ( _VERSION_LINE )
    string ( REGEX REPLACE ".*define[ ]+QWT_VERSION_STR[ ]+\"([^\"]*)\".*" "\\1" QWT_VERSION_STRING "${_VERSION_LINE}" )
  endif ()
endif ()
unset ( _VERSION_FILE )

find_library ( QWT_LIBRARY
  NAMES qwt qwt-qt5
  HINTS ${QT_LIBRARY_DIR}
)

set ( QWT_LIBRARIES ${QWT_LIBRARY} )

include ( FindPackageHandleStandardArgs )
find_package_handle_standard_args( Qwt REQUIRED_VARS QWT_LIBRARY QWT_INCLUDE_DIR VERSION_VAR QWT_VERSION_STRING )

if (Qwt_FOUND AND NOT TARGET Qwt::Qwt)
  add_library(Qwt::Qwt UNKNOWN IMPORTED)
  set_target_properties(Qwt::Qwt PROPERTIES
                        INTERFACE_INCLUDE_DIRECTORIES "${QWT_INCLUDE_DIRS}"
                        IMPORTED_LOCATION "${QWT_LIBRARIES}")
endif ()

mark_as_advanced (
  QWT_LIBRARY
  QWT_INCLUDE_DIR
)