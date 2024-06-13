FlexBE - The Flexible Behavior Engine
================================================

Overview
---------

.. **FlexBE** is a powerful and user-friendly high-level behavior engine, applicable to numerous systems and scenarios.

**FlexBE** is a powerful and user-friendly high-level behavior engine for the Robotic Operation System (ROS).
Both ROS 1 (Noetic) and ROS 2 versions are available.

FlexBE is designed to ease the development and execution of complex robotic behaviors through the use of Hierarchical Finite State Machines (HFSMs).

.. image:: ../images/flexbe_example_1.png
  :width: 700
  :align: center
  :alt: An image of the FlexBE State Machine Editer should appear here.
|

FlexBE includes both an Onboard (robot) Behavior Executive and an Operator Control Station (OCS) for supervisory control and *collaborative autonomy*.
The unique separation between the Onboard behavior and the OCS allows for distinct roles and responsibilities, ensuring efficient and coordinated robotic operation.

The **Onboard Behavior Executive** is responsible for coordinating the execution of Python-based state implementations.
It typically runs on the robot's onboard computer and communicates with the OCS through a wireless network.

The **Operator Control Station (OCS)** provide a user-friendly interface through the FlexBE WebUI, which simplifies the development of HFSMs.
This interface allows users to load, save, configure, and monitor the execution of HFSMs through the Onboard Behavior Executive.

.. image:: ../images/flexbe_simplified.png
  :width: 500
  :align: center
  :alt: A flowchart showing the relationship betweem Onboard Behavior and OCS should appear here.
|

FlexBE promotes *collaborative autonomy* by facilitating seamless communication and coordination between the onboard and OCS systems.
This collaborative approach enables human operators to interact with autonomous robots on a high level when necessary through its
adjustable levels of autonomy, harnessing the strengths of both human intelligence and robotic capabilities to achieve mission objectives efficiently.
As such, FlexBE is designed as a high-level behavioral controller. It is not focused on high-rate, low-level, safety-critical use cases;
in a typical use case, FlexBE would be used to supervise such controllers and allow an operator to guide the system recovery.

FlexBE offers two user interfaces: the standalone `FlexBE App <https://github.com/FlexBE/flexbe_app>`_ and the `FlexBE WebUI <https://github.com/FlexBE/flexbe_webui>`_.
The WebUI, which supersedes the FlexBE App, provides enhanced functionality for creating and managing robot behaviors.
Both versions of the software are supported, and the basic commands are the same.

Whether managing autonomous missions in hazardous environments, performing intricate manipulation tasks in manufacturing settings, or
assisting with search and rescue operations, FlexBE empowers users to develop and deploy complex robotic behaviors with ease and confidence!

For a more detailed overview of the FlexBE system, click 'Next'. Otherwise, click :ref:`here <Getting Started>` to get started!

.. toctree::
   :hidden:

   index
   detailedoverview
   gettingstarted
   quickstart
   basictutorials
   advancedtutorials
   contribution
   extensions
   applications
   bibliography
   contactinfo
