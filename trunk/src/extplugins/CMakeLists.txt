cmake_minimum_required(VERSION 2.8.11)
include("../../CMakeModules/Logic/SolutionUtils.cmake")
include("CMakeEDRModules/Logic/EDRProjectUtils.cmake")

INITIALIZE_PLUGINS_SOLUTION( Plugins )

ADD_EXTERNAL_SOLUTION_FINDERS(edrutils)
ADD_EXTERNAL_SOLUTION_FINDERS(edr)

FINALIZE_PLUGINS_SOLUTION()
