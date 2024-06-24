Bibliography
============


Please use the following publications for reference when using FlexBE:

    * Philipp Schillinger, Stefan Kohlbrecher, and Oskar von Stryk, `"Human-Robot Collaborative High-Level Control with Application to Rescue Robotics" <http://dx.doi.org/10.1109/ICRA.2016.7487442>`_, IEEE International Conference on Robotics and Automation (ICRA), Stockholm, Sweden, May 2016.

    * Joshua Zutell, David C. Conner and Philipp Schillinger, `"ROS 2-Based Flexible Behavior Engine for Flexible Navigation" <http://dx.doi.org/10.1109/SoutheastCon48659.2022.9764047>`_, IEEE SouthEastCon, April 2022.

Please use the following references if you use FlexBE in your research project and want to cite it.

.. code-block:: console

  @conference{schillinger2016flexbe,
    Title = {{Human-Robot Collaborative High-Level Control with an Application to Rescue Robotics}},
    Author = {Philipp Schillinger and Stefan Kohlbrecher and Oskar von Stryk},
    Booktitle = {IEEE International Conference on Robotics and Automation},
    Address = {Stockholm, Sweden},
    Month = {May},
    Year = {2016}
    }

.. code-block:: console

  @conference{zutell2022ros,
    Title = {{ROS 2-Based Flexible Behavior Engine for Flexible Navigation}},
    Author = {Joshua Zutell and David C. Conner and Philipp Schillinger},
    Booktitle = {IEEE SouthEastCon},
    Month = {April},
    Year = {2022},
    }

Here are some other publications that go into the higher-level significance of FlexBE in robotic behavior development:

    * Romay, A., Kohlbrecher, S., Stumpf, A., von Stryk, O., Maniatopoulos, S., Kress-Gazit, H., Schillinger, P. and Conner, D.C. (2017),
      `"Collaborative Autonomy between High-level Behaviors and Human Operators for Remote Manipulation Tasks using Different Humanoid Robots" <https://doi.org/10.1002/rob.21671>`_.
      J. Field Robotics, 34: 333-358.

    * Kohlbrecher S, Stumpf A, Romay A, Schillinger P, von Stryk O and Conner DC (2016)
      `"A Comprehensive Software Framework for Complex Locomotion and Manipulation Tasks Applicable to Different Types of Humanoid Robots" <https://www.frontiersin.org/articles/10.3389/frobt.2016.00031/full>`_.
      Front. Robot. AI 3:31. doi: 10.3389/frobt.2016.00031

----

FlexBE, initially conceived as an extension to `ROS 1 SMACH <http://wiki.ros.org/smach>`_, was developed by Philipp Schillinger at
Technische Universität Darmstadt in support of Team ViGIR in the DARPA Robotics Challenge (DRC) from 2012-2015.
The main parts of the internal operator interaction concepts, especially the *Autonomy Level* for adapting to
uncertain situations and the usage of a remote behavior mirror for bandwidth efficiency, was developed as part
of Schillinger's 2013 Bachelor's Thesis.  The concepts were further refined into FlexBE during Schillinger's
`Master's Thesis <https://www.sim.informatik.tu-darmstadt.de/publ/da/2015_Schillinger_MA.pdf>`_ in 2015,
and released publicly for ROS 1.

The main goal of FlexBE was to provide a way for a non-developer operator to make adjustments to behaviors during runtime in
situations where the exact scenario the robot will be facing is not known in advance.
The extensive user interface was developed in order to support behavior creation and runtime modification
to reduce the cognitive load on the operator while the robot is in the field, and reduce sources for possible errors
by running verification checks and automatically generating syntax-error-free code.
See Chapter 3 of Schillinger's Master's thesis for more details.

* Philipp Schillinger, *Development of an Operator Centric Behavior Control Approach for a Humanoid Robot*. BSc thesis, Technische Universität Darmstadt, 2013.
* Philipp Schillinger, `An Approach for Runtime-Modifiable Behavior Control of Humanoid Rescue Robots <https://www.sim.informatik.tu-darmstadt.de/publ/da/2015_Schillinger_MA.pdf>`_. MSc thesis, Technische Universität Darmstadt, 2015.