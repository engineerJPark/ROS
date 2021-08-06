cmake_minimum_required(VERSION 2.8.3)
project(cmd_vel_pub)           # 패키지 이름과 동일, 이름과 다르면 빌드가 안된다.

find_package(catkin REQUIRED COMPONENTS message_generation roscpp geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_message_files(FILES Msg_cmd_vel.msg)           # 새로 만들 메세지 이름
generate_messages(DEPENDENCIES geometry_msgs)            # 의존성

##캐킨 패키지 옵션으로 라이브러리, 캐킨 빌드 위존성, 시스템 의존 패키지를 기술한다
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS
)

###########
## Build ##
###########
 
##실행파일
add_executable(cmd_vel_pub src/cmd_vel_pub.cpp)           # topic_publisher:노드, topic_publisher.cpp: 노드를 만들 때 참고해야할 소스코드
add_dependencies(cmd_vel_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cmd_vel_pub ${catkin_LIBRARIES})
 
add_executable(sub_test src/sub_test.cpp)           # topic_subscriber:노드, topic_subscriber.cpp: 노드를 만들 때 참고해야할 소스코드
add_dependencies(sub_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sub_test ${catkin_LIBRARIES})
