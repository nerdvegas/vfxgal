#
# In:
#  TBB_ROOT
#
# Out:
#  TBB_FOUND
#  TBB_VERSION
#  TBB_MAJOR_VERSION
#  TBB_MINOR_VERSION
#  TBB_INCLUDE_DIRS
#  TBB_LIBRARY_DIRS
#

set(TBB_FOUND FALSE)
unset(TBBROOT CACHE) # because FindTbb can be called multiple times
find_path(TBBROOT NAMES include/tbb/tbb.h HINTS ${TBB_ROOT} )

if(TBBROOT)
    set(TBB_INCLUDE_DIRS ${TBBROOT}/include)
    set(TBB_LIBRARY_DIRS ${TBBROOT}/lib)
    
    file(READ "${TBB_INCLUDE_DIRS}/tbb/tbb_stddef.h" _TBB_CONTENTS)
    string(REGEX REPLACE ".*#define TBB_VERSION_MAJOR ([0-9]+).*" "\\1" TBB_MAJOR_VERSION "${_TBB_CONTENTS}")
    string(REGEX REPLACE ".*#define TBB_VERSION_MINOR ([0-9]+).*" "\\1" TBB_MINOR_VERSION "${_TBB_CONTENTS}")
    set(TBB_VERSION "${TBB_MAJOR_VERSION}.${TBB_MINOR_VERSION}")
    
    set(TBB_FOUND TRUE)
endif(TBBROOT)

if( Tbb_FIND_REQUIRED AND NOT TBB_FOUND )
    message(FATAL_ERROR "Could not find tbb")
endif( Tbb_FIND_REQUIRED AND NOT TBB_FOUND )