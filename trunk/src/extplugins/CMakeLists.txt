cmake_minimum_required(VERSION 2.8.11)
include("../../CMakeModules/Logic/SolutionUtils.cmake")
include("../../CMakeModules/Logic/ExternalPluginsUtils.cmake")

INITIALIZE_PLUGINS_SOLUTION( Plugins )
	
	get_filename_component(result "../../Finders" ABSOLUTE)
	ADD_EXTERNAL_SOLUTION_FINDERS_PATH(${result})

	#ADD_EXTERNAL_SOLUTION_FINDERS(edrutils)
	#ADD_EXTERNAL_SOLUTION_FINDERS(edr)

FINALIZE_PLUGINS_SOLUTION()

