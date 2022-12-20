cmake_minimum_required(VERSION 3.0.2)
project(awtube_meta)

find_package(catkin REQUIRED)

catkin_package(

)

include_directories(

)
install(PROGRAMS scripts/robovu.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# install
foreach(dir config launch )
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)