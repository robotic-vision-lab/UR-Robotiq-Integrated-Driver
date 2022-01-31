.. RVL UR-Robotiq Integrated Driver Documentation documentation master file, created by
   sphinx-quickstart on Fri Jan 28 23:20:07 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

RVL UR-Robotiq Integrated Driver
================================

RVL UR-Robotiq Integrated Driver is a wrapper for the existing ROS services and topics provided by 
Universal Robots (UR) ROS Driver with additional support for Robotiq gripper (currently support 2F) directly
attached to the UR tool port.

This driver also provides additional function for actuating the gripper in various modes (grasp on open/close, specific
jaw gap, etc.) as well as integrating MoveIt! motion planning that tracks the entire setup, UR and Robotiq gripper. 

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
