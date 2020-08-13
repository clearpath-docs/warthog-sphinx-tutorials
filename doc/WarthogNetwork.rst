Setting Up Warthog's Network
===============================

Warthog is equipped with an 802.11b/g/n compatible Wi-Fi module. On currently-shipping units, this
is a `Microhard PX2`__.  Warthog does not come equipped with a bluetooth module by default, though
an adapter can be connected to the onboard `Advantech ARK-3520P`__ PC.

.. _Microhard: http://www.microhardcorp.com/pX2.php
__ Microhard_

.. _Advantech: https://advdownload.advantech.com/productfile/PIS/ARK-3520P/Product%20-%20Datasheet/ARK-3520P_DS(03.21.19)20190321143448.pdf
__ Advantech_


First Connection
----------------

By default, Warthog's wireless is in client mode, looking for the wireless network at the Clearpath factory. In
order to set it up to connect to your own network, you'll have to open up the chassis and connect a network cable to
the PC's ``STATIC`` port. The other end of this cable should be connected to your laptop, and you should give yourself an IP address in the ``192.168.131.x`` space, such as ``192.168.131.50``. Then, make the connection to Warthog's default
static IP:

.. code-block:: bash

    ssh administrator@192.168.131.1

The default password is ``clearpath``. You should now be logged into Warthog as the administrator user.


Changing the Default Password
-----------------------------

.. Note::

  All Clearpath robots ship from the factory with their login password set to ``clearpath``.  Upon receipt of your
  robot we recommend changing the password.

To change the password to log into your robot, run the

.. code-block:: bash

  passwd

command.  This will prompt you to enter the current password, followed by the new password twice.  While typing the
passwords in the ``passwd`` prompt there will be no visual feedback (e.g. "*" characters).

To further restrict access to your robot you can reconfigure the robot's SSH service to disallow logging in with a
password and require SSH certificates to log in.  This_ tutorial covers how to configure SSH to disable password-based
login.

.. _This: https://linuxize.com/post/how-to-setup-passwordless-ssh-login/


Connecting to Wifi Access Point
--------------------------------

Warthog's standard wireless network manager is wicd_. To connect to an access point in your lab, run:

.. code-block:: bash

    wicd-curses

You should see a browsable list of networks which the robot has detected. Use arrow keys to select the one you
would like to connect to, and then press the right arrow to configure it. You can enter your network's password
near the bottom of the page, and note that you must select the correct encryption scheme; most modern networks
use ``WPA1/2 Passphrase``, so if that's you, make sure that option is selected. You also likely want to select
the option to automatically reconnect to this network, so that Warthog will be there for you on your wireless
automatically in the future.

When you're finished, press F10 to save, and then C to connect.

Wicd will tell you in the footer what IP address it was given by your lab's access point, so you can now log out,
remove the network cable, and reconnect over wireless. When you've confirmed that all this is working as expected,
close up Warthog's chassis.

.. _wicd: https://launchpad.net/wicd


.. _remote:

Remote ROS Connection
---------------------

To use ROS desktop tools, you'll need your computer to be able to connect to Warthog's ROS master. This can be a
tricky process, but we've tried to make it as simple as possible.

In order for the ROS tools on your computer to talk to Warthog, they need to know two things:

- How to find the ROS master, which is set in the ``ROS_MASTER_URI`` environment variable, and
- How processes on the other computer can find *your computer*, which is the ``ROS_IP`` environment variable.

The suggested pattern is to create a file in your home directory called ``remote-warthog.sh`` with the following
contents:

.. code-block:: bash

    export ROS_MASTER_URI=http://cpr-warthog-0001:11311  # Warthog's hostname
    export ROS_IP=10.25.0.102                          # Your laptop's wireless IP address

If your network doesn't already resolve Warthog's hostname to its wireless IP address, you may need to add
a corresponding line to your computer's ``/etc/hosts`` file:

.. code-block:: bash

    10.25.0.101 cpr-warthog-0001

Then, when you're ready to communicate remotely with Warthog, you can source that script like so, thus defining
those two key environment variables in the present context.

.. code-block:: bash

    source remote-warthog.sh

Now, when you run commands like ``rostopic list``, ``rostopic echo``, ``rosnode list``, and others, the output
you see should reflect the activity on Warthog's ROS master, rather than on your own machine. Once you've
verified the basics (list, echo) from the prompt, try launching some of the standard visual ROS tools:

.. code-block:: bash

    roslaunch warthog_viz view_robot.launch
    rosrun rqt_robot_monitor rqt_robot_monitor
    rosrun rqt_console rqt_console

If there are particular :roswiki:`rqt` widgets you find yourself using a lot, you may find it an advantage to dock them together
and then export this configuration as the default RQT perspective. Then, to bring up your standard GUI, you can simply
run:

.. code-block:: bash

    rqt


Advanced: Hosting a Wifi Access Point
-------------------------------------

The default network manager, ``wicd``, only supports joining existing networks. It does not support creating its own wireless AP.
However, there is experimental support in Warthog for a modern network manager called connman_, which does.

.. _connman: https://01.org/connman

.. warning::

             You are unlikely to damage your hardware by switching Warthog from wicd to connman, but it's possible
             you could end up with a platform which will need to be `reflashed back to the factory state` in
             order to be usable. If you're comfortable with this and have backed up your data, proceed.

Connman is available through the Ubuntu software repositories, and can be installed by running the following command:

.. code-block:: bash

    sudo apt-get install connman

Note that there is a similarly-named ``conman`` package, which is a serial console manager, not a network manager.  Be
sure to include two N's in ``connman``.

Once connman is installed, edit the upstart job file in ``/etc/init/connman.conf``. Suggested configuration:

.. code-block:: bash

    description "Connection Manager"
     
    start on started dbus
    stop on stopping dbus
     
    console log
    respawn
     
    exec connmand --nobacktrace -n -c /etc/connman/main.conf -I eth1 -I hci0

And edit connman's general configuration in ``/etc/connman/main.conf``. Suggested:

.. code-block:: bash

    [General]
    TetheringTechnologies = wifi
    PersistentTetheringMode = true

Now, use the connmanctl command-line interface to set up an AP, which connman calls "tethering" mode:

.. code-block:: bash

    $ connmanctl
    connmanctl> enable wifi
    connmanctl> tether wifi on Warthog clearpath

If you want to use connman to connect to another AP rather than host:

.. code-block:: bash

    $ connmanctl
    connmanctl> tether wifi off
    connmanctl> agent on
    connmanctl> scan wifi
    connmanctl> services
    connmanctl> connect wifi_12345_67890_managed_psk

Use as the argument to ``connect`` one of the services listed in the ``services`` output. You will be interrogated for
the network's password, which is then cached in ``/var/lib/connman/``.
