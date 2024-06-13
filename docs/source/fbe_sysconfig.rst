System Configuration
=====================

This section will review the default FlexBE installation, pointing out the most important components within the setup.
This is helpful if you want to customize your installation.
However, if you are fine with the default setup and just want to get things running, feel free to :ref:`skip<Creating A New Behavior>` this tutorial for now and consider coming back later. 

1 FlexBE Behavior Engine
-------------------------

Depending on what you decided to clone, there are several new repositories in your ``colcon`` workspace.
The most important one is the FlexBE behavior engine itself.
Of its contained ROS packages, especially two will be interesting for you.
First, ``flexbe_states`` contains a set of generic basic states which you can use in your own behaviors.
Second, the ``flexbe_testing`` package provides a testing framework tailored to state unit testing.
Furthermore, ``flexbe_input`` provides an integrated way for data exchange from the operator to the robot and ``flexbe_widget`` holds some handy scripts such as a script for clearing temporary behaviors.

2 FlexBE WebUI
------------

The user interface of FlexBE provides a powerful behavior editor as well as runtime monitoring functionality.
You can either run it in offline mode for just developing behaviors, e.g., by using a shortcut icon, or launch it via ``ros2 launch`` in order to access its full functionality.
The following tutorials make use of the FlexBE WebUI and help you with your first steps.

The FlexBE WebUI requires consistent versions of several Python dependencies; these are specified in the install_requires field of the ``setup.py`` and
the ``requires.txt`` file.

The system has currently been tested on Ubuntu 22.04 with the following Python packages ``pip install`` -ed locally:

 * ``pip install websockets==10.3``
 * ``pip install pydantic==1.10.13``
 * ``pip install fastAPI==0.89.1``
 * ``pip install PySide6==6.7.1``

If the Python files are not installed automatically, you can directly go to the ``[your_ros_workspace]/src/flexbe_webui`` folder and ``pip install -r requires.txt`` to install all of the required files in the virtual environment.

3 FlexBE Behaviors Repository
-----------------------------

The second repository is, per default, created by running the ``create_repo`` script located in ``flexbe_widget``.
This repository will be hosted by you along with your project and basically contains all custom content you develop for using FlexBE, mainly states and behaviors.
Although being project specific, there are some common parts. 

If you did not yet initialize a behaviors repository during installation, please do so before heading over to the next tutorials:

.. code-block:: console

 cd ~/[your_ros_workspace]/src # Jump to the source directory
 ros2 run flexbe_widget create_repo [your_project_name]
 cd ~/[your_ros_workspace]
 colcon build # Make sure you build the workspace afterwards

Most importantly, the ROS package ``[your_project_name]_flexbe_behaviors`` will be present in your repository and will be the point of reference for FlexBE in order to store your behaviors.
It contains a folder called ``manifest``, holding all the behavior manifests.
A behavior manifest is an abstract interface declaration each behavior defines, stored as an XML file.
This is how FlexBE knows about which behaviors are available, where to find them, and much more.
Furthermore, if you need to store configuration files for your behaviors, ``[your_project_name]_flexbe_behaviors`` is a good place for this purpose, too. 

As already mentioned, your states and behaviors are the main content of this repository.
For states, it is recommended to create packages similar to the generic state package ``flexbe_states`` and bundle states according to their purpose as you like.
It is also very appreciated if you decide to make some of your states publicly available, for which you would store them in a separate, public repository.

Next Steps
----------

That's it! Jump here to start :ref:`creating a new behavior<Creating A New Behavior>`.