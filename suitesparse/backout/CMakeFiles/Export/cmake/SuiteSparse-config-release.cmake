#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "metis" for configuration "Release"
set_property(TARGET metis APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(metis PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "m"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib64/libmetis.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS metis )
list(APPEND _IMPORT_CHECK_FILES_FOR_metis "${_IMPORT_PREFIX}/lib64/libmetis.a" )

# Import target "suitesparseconfig" for configuration "Release"
set_property(TARGET suitesparseconfig APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(suitesparseconfig PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libsuitesparseconfig.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS suitesparseconfig )
list(APPEND _IMPORT_CHECK_FILES_FOR_suitesparseconfig "${_IMPORT_PREFIX}/lib/libsuitesparseconfig.a" )

# Import target "amd" for configuration "Release"
set_property(TARGET amd APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(amd PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libamd.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS amd )
list(APPEND _IMPORT_CHECK_FILES_FOR_amd "${_IMPORT_PREFIX}/lib/libamd.a" )

# Import target "btf" for configuration "Release"
set_property(TARGET btf APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(btf PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libbtf.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS btf )
list(APPEND _IMPORT_CHECK_FILES_FOR_btf "${_IMPORT_PREFIX}/lib/libbtf.a" )

# Import target "camd" for configuration "Release"
set_property(TARGET camd APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(camd PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcamd.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS camd )
list(APPEND _IMPORT_CHECK_FILES_FOR_camd "${_IMPORT_PREFIX}/lib/libcamd.a" )

# Import target "ccolamd" for configuration "Release"
set_property(TARGET ccolamd APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ccolamd PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libccolamd.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ccolamd )
list(APPEND _IMPORT_CHECK_FILES_FOR_ccolamd "${_IMPORT_PREFIX}/lib/libccolamd.a" )

# Import target "colamd" for configuration "Release"
set_property(TARGET colamd APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(colamd PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcolamd.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS colamd )
list(APPEND _IMPORT_CHECK_FILES_FOR_colamd "${_IMPORT_PREFIX}/lib/libcolamd.a" )

# Import target "cholmod" for configuration "Release"
set_property(TARGET cholmod APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cholmod PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcholmod.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS cholmod )
list(APPEND _IMPORT_CHECK_FILES_FOR_cholmod "${_IMPORT_PREFIX}/lib/libcholmod.a" )

# Import target "cxsparse" for configuration "Release"
set_property(TARGET cxsparse APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cxsparse PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcxsparse.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS cxsparse )
list(APPEND _IMPORT_CHECK_FILES_FOR_cxsparse "${_IMPORT_PREFIX}/lib/libcxsparse.a" )

# Import target "klu" for configuration "Release"
set_property(TARGET klu APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(klu PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libklu.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS klu )
list(APPEND _IMPORT_CHECK_FILES_FOR_klu "${_IMPORT_PREFIX}/lib/libklu.a" )

# Import target "ldl" for configuration "Release"
set_property(TARGET ldl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ldl PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libldl.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ldl )
list(APPEND _IMPORT_CHECK_FILES_FOR_ldl "${_IMPORT_PREFIX}/lib/libldl.a" )

# Import target "umfpack" for configuration "Release"
set_property(TARGET umfpack APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(umfpack PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libumfpack.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS umfpack )
list(APPEND _IMPORT_CHECK_FILES_FOR_umfpack "${_IMPORT_PREFIX}/lib/libumfpack.a" )

# Import target "spqr" for configuration "Release"
set_property(TARGET spqr APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(spqr PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libspqr.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS spqr )
list(APPEND _IMPORT_CHECK_FILES_FOR_spqr "${_IMPORT_PREFIX}/lib/libspqr.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
