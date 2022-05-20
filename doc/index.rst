Warthog UGV Tutorials
======================

.. image:: images/warthog_banner.png
    :alt: Warthog Robot

This package supplies Sphinx-based tutorial content to assist you with setting up and operating your Warthog_ mobile robot. The tutorials topics are listed in the left column, and presented in the suggested reading order.

.. _Warthog: https://clearpathrobotics.com/warthog-unmanned-ground-vehicle-robot/

.. Warning::
  These tutorials assume that you are comfortable working with ROS.  We recommend starting with our
  `ROS tutorial <./../ros>`_ if you are not familiar with ROS already.

.. note::

  These tutorials specifically target Warthog robots running Ubuntu 20.04 with ROS Noetic, as it is the standard OS environment for Warthog. If instead you have an older Warthog robot running Ubuntu 18.04 with ROS Melodic, please follow `this tutorial <https://www.clearpathrobotics.com/assets/guides/melodic/melodic-to-noetic/index.html>`_ to upgrade its OS environment to Ubuntu 20.04 with ROS Noetic.

:doc:`Simulation <Simulating>` is a logical place for most users to start, as this is universally applicable; understanding how to effectively operate Warthog in simulation is valuable whether you are in the testing phase with software you intend to ultimately deploy on a real Warthog, or you do not have one and are simply exploring the platform's capabilities.

:doc:`Driving <Driving>` covers how to teleoperate Warthog using the remote control, a well as safety procedures for operating the real robot. Anyone working with a physical robot should be familiar with this section.

The remainder of the subjects are more applicable to the real robot, and have to do with configuring, using, and maintaining the platform. If you are a lab administrator rather than direct platform user, you may wish to skip the introductory chapters and jump straight to these ones.

.. toctree::
    :maxdepth: 0
    :caption: Warthog Overview

    Introduction <self>
    StatusIndicators

.. toctree::
    :maxdepth: 0
    :caption: Warthog ROS Packages

    CommonPackages
    DescriptionPackage

.. toctree::
    :maxdepth: 0
    :caption: Setting Up Warthog

    Installing
    Networking

.. toctree::
    :maxdepth: 0
    :caption: Using Warthog

    Driving
    Simulating
    ExtendingStartup
    KeepingUpdated

.. toctree::
    :titlesonly:
    :hidden:
    :caption: Testing Warthog

    WarthogTests

.. toctree::
    :titlesonly:
    :hidden:
    :caption: Other

    AdditionalSimulation