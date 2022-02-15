Setting Up Networking
======================

Warthog is equipped with an 802.11b/g/n compatible Wi-Fi module. On currently-shipping units, this
is a `Microhard PX2`__.  Warthog does not come equipped with a bluetooth module by default, though
an adapter can be connected to the onboard `Advantech ARK-3520P`__ PC.

.. _Microhard: http://www.microhardcorp.com/pX2.php
__ Microhard_

.. _Advantech: https://advdownload.advantech.com/productfile/PIS/ARK-3520P/Product%20-%20Datasheet/ARK-3520P_DS(03.21.19)20190321143448.pdf
__ Advantech_

First Connection
-----------------

In order to set Warthog up to connect to your own wireless network, you will first need to access the Warthog's computer from you computer over ``ssh``:

1. Configure your computer to have a static IP address on the ``192.168.131.x`` subnet, e.g. ``192.168.131.100``.

2. Connect an ethernet cable between Warthog's computer and your computer.

3. ``ssh`` into Warthog's computer from your computer. In terminal, run:

.. code-block:: bash

    ssh administrator@192.168.131.1

The default password is ``clearpath``. You should now be logged into Warthog as the administrator user.

Changing the Default Password
------------------------------

.. Note::
  All Clearpath robots ship from the factory with their login password set to ``clearpath``.  Upon receipt of your robot we recommend changing the password.

To change the password to log into your robot, you can use the ``passwd`` command. In terminal, run:

.. code-block:: bash

  passwd

This will prompt you to enter the current password, followed by the new password twice.  While typing the passwords in the ``passwd`` prompt there will be no visual feedback (e.g. "*" characters).

To further restrict access to your robot you can reconfigure the robot's ``ssh`` service to disallow logging in with a password and require ``ssh`` certificates to log in.  This_ tutorial covers how to configure ``ssh`` to disable password-based login.

.. _This: https://linuxize.com/post/how-to-setup-passwordless-ssh-login/

Connecting to Wifi Access Point
--------------------------------

Warthog uses ``netplan`` for configuring its wired and wireless interfaces. After accessing Warthog's computer from your computer, you can configure ``netplan`` so that Warthog can connect to your own wireless network:

1. Create the file ``/etc/netplan/60-wireless.yaml``.

2. Populate the file ``/etc/netplan/60-wireless.yaml`` with the following:

.. code-block:: yaml

    network:
      wifis:
        # Replace WIRELESS_INTERFACE with the name of the wireless network device, e.g. wlan0 or wlp3s0
        # Fill in the SSID and PASSWORD fields as appropriate.  The password may be included as plain-text
        # or as a password hash.  To generate the hashed password, run
        #   echo -n 'WIFI_PASSWORD' | iconv -t UTF-16LE | openssl md4 -binary | xxd -p
        # If you have multiple wireless cards you may include a block for each device.
        # For more options, see https://netplan.io/reference/
        WIRELESS_INTERFACE:
          optional: true
          access-points:
            SSID_GOES_HERE:
              password: PASSWORD_GOES_HERE
          dhcp4: true
          dhcp4-overrides:
            send-hostname: true

3. Save the file ``/etc/netplan/60-wireless.yaml``. You will then need to apply your new ``netplan`` configuration and bring up your wireless connection. In terminal, run:

.. code-block:: bash

    sudo netplan apply

4. Verify that Warthog successfully connected to your wireless network. In terminal, run:

.. code-block:: bash

    ip a

This will show all active connections and their IP addresses, including the connection to your wireless network, and the IP address assigned to Warthog's computer.

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

Configuring Network Bridge
---------------------------

Warthog is configured to bridge its physical ethernet ports together.  This allows any ethernet port to be used as a
connection to the internal ``192.168.131.1/24`` network -- for connecting sensors, diagnostic equipment, or
manipulators -- or for connecting the robot to the internet for the purposes of installing updates.

Netplan is the default network configuration tool for Ubuntu 18.04 onward.  Instead of using the ``/etc/network/interfaces``
file, as was done in Ubuntu 16.04 and earlier, netplan uses YAML-formatted files located in ``/etc/netplan``.  The
default configuration file, ``/etc/netplan/50-clearpath-bridge.yaml``, is below:

.. code-block:: yaml

    # /etc/netplan/50-clearpath-bridge.yaml
    network:
    version: 2
    renderer: networkd
    ethernets:
      # bridge all wired interfaces together on 192.168.131.x
      bridge_eth:
        dhcp4: no
        dhcp6: no
        match:
          name: eth*
      bridge_en:
        dhcp4: no
        dhcp6: no
        match:
          name: en*

    bridges:
      br0:
        dhcp4: yes
        dhcp6: no
        interfaces: [bridge_eth, bridge_en]
        addresses:
          - 192.168.131.1/24

To enable network configuration using netplan you must install the ``netplan.io`` package:

.. code-block:: bash

    sudo apt-get install netplan.io
