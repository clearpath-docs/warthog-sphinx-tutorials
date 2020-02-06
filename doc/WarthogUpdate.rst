Keeping Warthog Updated
==========================

Warthog is always being improved, both its own software and the many community ROS packages upon which it
depends! You can use the apt package management system to receive new versions all software running on the
platform.


Getting New Packages
--------------------

Each Warthog leaves the factory already configured to pull packages from http://packages.ros.org as well as
http://packages.clearpathrobotics.com. To update your package and download new package versions make sure that
Warthog is connected to the internet and run the following commands:

.. code-block:: bash

    sudo apt-get update
    sudo apt-get dist-upgrade

If you see any errors, please `get in touch`_ and we'll see if we can get you sorted out.

.. _get in touch: https://support.clearpathrobotics.com/hc/en-us/requests/new


MCU Firmware Update
-------------------

.. warning::

    This section is currently just a search-replace of the Moose docs; the instructions themselves have not been
    rewritten yet!

You need to use an external PC to update Warthog's firmware.  You cannot use Warthog's internal PC, as installing the
firmware requires power-cycling the MCU.  Warthog's MCU controls the power supply to the internal PC.  These instructions
assume the external PC is running some flavour of Linux with the firmware's .bin file downloaded to it.

When you update packages, there is periodically a new version of Warthog's firmware available. You will know this
is the case in one of two ways:

1. The firmware and PC are unable to connect to each other, which will be apparent if Warthog's heartbeat LED does not flash
   continuously while the robot is powered on; or
2. If the firmware version number in the ``/status`` message does not match the package version output by
   |dpkg_s_firmware|. In the future there will be an automated check for this which outputs
   a diagnostics warning when a firmware update is available to be applied.

If new firmware is available, follow the below procedure to flash it to Warthog's MCU:

1. Place Warthog up on blocks and/or engage the emergency stop by pressing one of the red buttons located on each corner
   of the robot. Firmware loading does not usually result in unintended motion, but it's safest to ensure the robot
   cannot move accidentally.
2. Open Warthog's computer box, located on the rear of the robot

.. image:: graphics/placeholder.png
    :alt: The inside of Warthog's computer box

3. Insert a USB drive into Warthog's PC, mount it, and run the following command to copy the firmware, replacing
   the path with the location you mounted the USB drive to:

.. substitution-code-block:: bash

    cp |fw_path| /path/to/usb/drive

Unmount and remove the USB drive when the command completes

4. While pressing ``BT0`` on the MCU, connect the external PC to Warthog's MCU using a USB cable.

.. image:: graphics/placeholder.png
    :alt: Warthog's MUC's buttons

5. After connecting the PC you should see a device with a name similar to
   ``Bus 001 Device 005: ID 0483:df11 STMicroelectronics STM Device in DFU Mode`` in the output of ``lsusb``.  Run the
   following command, replacing ``001/005`` with the value appropriate to the Bus and Device where Warthog's MCU is
   connected:

.. code-block:: bash

    sudo chmod 666 /dev/bus/usb/001/005

Now mount the same USB drive from step 3 and run the following command, replacing the path with the mount point of
the drive:

.. code-block:: bash

    dfu-util -v -d 0483:df11 -a 0 -s 0x08000000 -D /path/to/usb/drive/mcu.bin

You should see about 20 seconds worth of lines output beginning with "Download from image ...". When this is
complete you may disconnect the PC from the MCU and power-cycle the robot.


.. _scratch:

Starting From Scratch
---------------------

If Warthog's computer has become inoperable, or for any reason you want to restore it to the factory state, begin
by opening Warthog's computer box, located on the rear of the robot.  The PC is easily accessible with USB, network,
and video ports available.

.. image:: graphics/placeholder.png
    :alt: Warthog's PC.

1. Download the latest operating system image for Warthog from http://packages.clearpathrobotics.com/stable/images/latest/
2. Use unetbootin__ or rufus__ (Windows only) to write the ISO image to a USB memory stick

.. _unetbootin: https://unetbootin.github.io/linux_download.html
__ unetbootin_

.. _rufus: https://rufus.ie/
__ rufus_

.. image:: graphics/unetbootin.png
    :alt: Unetbootin

3. Connect Warthog's PC to the internet with an ethernet cable.  Also connect a monitor, mouse, and keyboard to the PC.
4. Connect Warthog to shore power to ensure it does not power down while the OS is installing.  This is optional, but
   recommended.
5. Insert the USB drive into one of the Warthog's PC's USB ports and power-cycle the PC.  You should see a purple
   Debian/Ubuntu installer.  The installer will run by itself and power off the PC when finished.  If you do
6. Remove the USB drive and power on the PC.
7. Log into Warthog.  The username is ``administrator`` and the password is ``clearpath``
8. Install Warthog's robot_upstart__ job, so that ROS will launch each time the robot starts:

.. _robot_upstart: http://wiki.ros.org/robot_upstart
__ robot_upstart_

.. code-block bash

    rosrun warthog_bringup install

You can now configure Warthog to :doc:`connect to your wi-fi <WarthogNetwork>`.
