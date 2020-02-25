Configuration & Environment Variables
=========================================

.. image:: graphics/warthog_urdf_banner.png
    :alt: Warthog URDF

The warthog_description package is the URDF robot description for Warthog UGV.

.. _Source: https://github.com/warthog-cpr/warthog


Overview
---------

This package provides a `URDF <http://wiki.ros.org/urdf>`_ model of Warthog.  For an example launchfile to use in visualizing this model, see `warthog_viz <http://wiki.ros.org/warthog_viz>`_.


Accessories
------------

Warthog has a suite of optional payloads called accessories. These payloads can be enabled and placed on the robot using environment variables specified at the time the `xacro <http://wiki.ros.org/xacro>`_ is rendered to URDF. Available accessory vars are:

.. raw:: html

  <table><tbody>
    <tr> <td><p><strong>Variable</strong> </p></td>
      <td><p><strong>Default</strong> </p></td>
      <td><p><strong>Description</strong> </p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_ARM_MOUNT</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>If enabled, a link called "arm_mount_link" will be created on the bulkhead</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_BULKHEAD</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Switch for enabling a large, rigid bulkhead on the front of the warthog for mount accessories like arms</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_IMU_RPY</tt> </p></td>
      <td><p><tt>0 0 0</tt> </p></td>
      <td><p>Compound rotations in radians of the IMU</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_IMU_XYX</tt> </p></td>
      <td><p><tt>0 0 0</tt> </p></td>
      <td><p>Position of the IMU in meters</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_JOY_TELEOP</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Switch to enable teleop control of the platform</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_NAVSAT_SMART6</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Is the platform equipped with a Smart7 GPS?</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_NAVSAT_SMART6_BAUD</tt> </p></td>
      <td><p><tt>57600</tt> </p></td>
      <td><p>Sets the baud rate for serial communication with the GPS module</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_NAVSAT_SMART6_MOUNT</tt> </p></td>
      <td><p><tt>navsat</tt> </p></td>
      <td><p>The mount on the robot model that the GPS antenna is mounted to.  See the Warthog URDF and <tt>WARTHOG_URDF_EXTRAS</tt> for more details on mount points.</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_NAVSAT_SMART6_OFFSET</tt> </p></td>
      <td><p><tt>0 0 0</tt> </p></td>
      <td><p>Position of the Smart 6 GPS in meters</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_NAVSAT_SMART6_PORT</tt> </p></td>
      <td><p><tt>/dev/ttyS1</tt> </p></td>
      <td><p>The serial port that the GPS module communicates over</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_NAVSAT_SMART6_RPY</tt> </p></td>
      <td><p><tt>0 0 0</tt> </p></td>
      <td><p>Compound rotations in radians of the Smart 6 GPS</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_OFFBOARD_STOP</tt> </p></td>
      <td><p><tt>false</tt> </p></td>
      <td><p>Is a remote e-stop setup on the platform?</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_TRACKS</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Used to specify that Warthog is equipped with tracks instead of wheels.</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_TWIST_MUX_EXTRAS</tt> </p></td>
      <td><p><tt>0</tt> </p></td>
      <td><p>Add additional sources to be controlled by the twise mux of the platform</p></td>
    </tr>
    <tr>  <td><span class="anchor" id="line-11"></span><p><tt>WARTHOG_URDF_EXTRAS</tt> </p></td>
      <td><p><tt>empty.urdf</tt> </p></td>
      <td><p>Path to a URDF file with additional modules connected to the robot</p></td>
    </tr>
  </tbody></table>

Configurations
----------------

As an alternative to individually specifying each accessory, some fixed configurations are provided in the package. These can be specified using the ``config arg to description.launch``, and are intended especially as a convenience for simulation launch.

====================================  ====================================================
Config:                               Description:
====================================  ====================================================
base                                  Base Warthog, includes IMU and GPS
arm_mount                             Includes mounting points for am arm payload
bulkhead                              ??? TODO
empty                                 Includes no accessories at all
====================================  ====================================================
