# Indicate the source files
set(LIB_SRCS rpo.hpp  rpo.cpp)

# Set the library type as static
SET (LIB_TYPE STATIC)

add_library (rpo ${LIB_TYPE} ${LIB_SRCS})