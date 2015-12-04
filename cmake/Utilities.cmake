include(CMakeParseArguments)

macro(CONFIGURE_BLOCK)

  set(_options )
  set(_oneValueArgs BLOCK_NAME
                    LIST_PREFIX
                    GROUP)
  set(_multiValueArgs SOURCES
                      HEADERS
                      )

  cmake_parse_arguments(_ARS "${_options}"
                             "${_oneValueArgs}"
                             "${_multiValueArgs}"
                             "${ARGN}")
    
    set_property(GLOBAL APPEND PROPERTY ${_ARS_LIST_PREFIX}_HEADERS ${_ARS_HEADERS})
    set_property(GLOBAL APPEND PROPERTY ${_ARS_LIST_PREFIX}_SOURCES ${_ARS_SOURCES})

    source_group("${_ARS_GROUP}\\${_ARS_BLOCK_NAME}\\Headers" FILES ${_ARS_HEADERS})
    source_group("${_ARS_GROUP}\\${_ARS_BLOCK_NAME}\\Sources" FILES ${_ARS_SOURCES})
endmacro()