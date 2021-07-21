Howto ROS2 Launch file
======================

.. contents:: Table of Contents
   :depth: 2
   :local:
   
   
Some stuff about launch file.
   
1 "Options"
^^^^^^^^^^^

.. code-block::

   package     The ROS 2 package which to use
   
   executable  The e.g. node of the previously named package to use
   
   output      "screen" means ??

   namespace   If you use e.g. "test", then when running "ros2 topic list" there are "/test/*" topics.
   
   remappings  Give e.g. a topic another name
   
   parameters
   
   arguments

   
2 Node
^^^^^^

.. code-block::

   Node(
               package='robot_state_publisher',
               executable='robot_state_publisher',
               name='robot_state_publisher',
               namespace="test",
               output='screen',
               parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
               arguments=[urdf],
               remappings=[("metacarpals_joint", "index_metacarpals_joint")] )
