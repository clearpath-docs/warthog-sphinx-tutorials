Warthog UGV Tutorials
======================

.. image:: graphics/warthog_banner.png
    :alt: Warthog Robot


This package supplies Sphinx-based tutorial content to assist you with setting up and operating your Warthog_
mobile robot. The tutorials topics are listed in the left column, and presented in the suggested reading order.

.. _Warthog: http://www.clearpathrobotics.com/warthog/

.. Warning::
  These tutorials assume that you are comfortable working with ROS.  We recommend starting with our
  `ROS tutorial <./../ros>`_ if you are not familiar with ROS already.

:doc:`Simulation <WarthogSimulation>` is a logical place for most users to start, as this is universally applicable;
understanding how to effectively operate Warthog in simulation is valuable whether you are in the testing
phase with software you intend to ultimately deploy on a real Warthog, or you do not have one and are
simply exploring the platform's capabilities.

:doc:`Driving <WarthogDriving>` covers how to teleoperate Warthog using the remote control, a well as safety procedures
for operating the real robot.  Anyone working with a physical robot should be familiar with this section.

The remainder of the subjects are more applicable to the real robot, and have to do with configuring, using,
and maintaining the platform. If you are a lab administrator rather than direct platform user, you may wish to
skip the introductory chapters and jump straight to these ones.


.. toctree::
    :maxdepth: 3
    :caption: Contents

    Overview <self>
    WarthogInstallation
    WarthogSimulation
    WarthogDriving
    WarthogNetwork
    WarthogStartup
    WarthogUpdate

.. toctree::
    :titlesonly:
    :hidden:
    :caption: Warthog Packages

    WarthogPackages
