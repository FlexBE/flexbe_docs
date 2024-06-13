Contributions
=============
Help Appreciated!
-----------------

The motivation behind FlexBE is to make high-level robot control as easy as possible and
accessible to everyone, without requiring learning a programming language or syntax.
There are several ways of how you can support this motivation and help extending FlexBE.
The easiest and recommended way of starting to support FlexBE is using it.
Make sure to give helpful feedback, new ideas, and optimally develop some states for your robot and share them with others.
Using your states will give more people the chance to apply FlexBE easily to their own system.

**If you would like to report any bugs or requests for FlexBE software, please report them to the appropriate GitHub repository:**

 * For the FlexBE Behavior Engine and core software: `FlexBE Issues Page <https://github.com/FlexBE/flexbe_behavior_engine/issues>`_
 * For the new FlexBE WebUI: `WebUI Issues Page <https://github.com/FlexBE/flexbe_webui/issues>`_
 * For the FlexBE App predecessor: `App Issues Page <https://github.com/FlexBE/flexbe_app/issues>`_

If you see an oportunity to support FlexBE even further, please :ref:`contact us<Contact Us!>` and we can coordinate.

Developing States
-----------------
Ideally, you would not need to develop many new states for your robot because there are already states for
moving the robot to waypoints, manipulate objects, face an object of interest with its camera, and much more.
Of course, these states need to be developed once.
If you need functionality which is not yet available as FlexBE states or developed new functionality yourself and want to make it accessible to more people, just create new states and make it available.

When developing new states, please make sure to respect a few guidelines in order to support modularity:

* **Purpose** - Each state should have one precise purpose (e.g. one state for trajectory planning, one for execution; not one for both).
* **Name** - Choose a name which describes the purpose as precisely as possible. However, a good name should not exceed 30 characters.
* **Parametrize** - Do not hard-code context-specific configuration (e.g. use state parameters with default values).
* **Interface** - Preferably use primitives data types (int, bool, float, string, lists of these) as parameters, userdata keys can also use common ROS messages.
* **Test** - Write test cases for your states and make sure they work as expected (e.g. use *flexbe_testing*)
* **Documentation** - Respect the state documentation guidelines and explain what your state expects and will do.
* **Uniform** - A last sanity check: compare your state to existing ones and make sure they fit together.

In order to make your states available to the community, create a public GIT repository and
put a ROS package containing your states there.
You can also put behaviors there which demonstrate the usage of your states or provide more complex
functionality by composing some of the states.
For maximum portability, keep the packages focused on FlexBE specific code, and not project code (e.g. robot-specific code).
Furthermore, make sure you document dependencies in the readme file, so everybody can install the required low-level functionality packages.
Finally, :ref:`contact us<Contact Us!>` so we can share the news with the larger community :ref:`website<Discover FlexBE>`.