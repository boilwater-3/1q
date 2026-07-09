# Third-party dependency entry point.
# Focused files below keep source resolution, package discovery, link matrix,
# and exported PackageConfig dependencies independently reviewable.

include("${CMAKE_CURRENT_LIST_DIR}/dependencies/Jsbsim.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/dependencies/ConanPackages.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/dependencies/ModuleLinkMatrix.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/PackageConfigDependencies.cmake")
