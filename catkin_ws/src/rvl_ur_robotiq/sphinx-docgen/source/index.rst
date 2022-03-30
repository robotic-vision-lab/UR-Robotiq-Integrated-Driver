.. RVL UR-Robotiq Integrated Driver Documentation master file, created by
   sphinx-quickstart on Fri Jan 28 23:20:07 2022. You can adapt this file 
   completely to your liking, but it should at least contain the root `toctree` 
   directive.

RVL UR-Robotiq Integrated Driver
================================

The `Robotic Vision Laboratory (RVL) <https://rvl.uta.edu>`_ UR-Robotiq
Integrated Driver is a wrapper for existing ROS services and topics provided by
the `Universal Robots (UR) <https://www.universal-robots.com>`_ ROS Driver. It
has additional support for a `Robotiq <https://robotiq.com>`_ gripper (e.g.,
2F) directly attached to the UR tool port. The driver also provides functions
for actuating the gripper in various modes (e.g., grasp on open/close, specific
jaw gap, etc.) as well as integrating the `MoveIt Motion Planning Framework
<https://moveit.ros.org>`_ for tracking the entire setup, UR robot, and Robotiq
gripper.

At the time of this release, the driver is working on our setup with both
Windows 10 (Version 10.0.19044 Build 19044) and Ubuntu 20.04.

.. toctree::
   :maxdepth: 2

   pages/quickstart.md
   pages/api.md
   pages/references.md
   pages/license.md
   pages/changelog.md
   
.. Indices and tables
.. ==================

.. * :ref:`genindex`
.. * :ref:`modindex`
.. * :ref:`search`
