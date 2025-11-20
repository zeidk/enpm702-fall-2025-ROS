.. default-domain:: cpp

=====================================
ROS 2 Overview
=====================================

This document provides a concise overview of the material covered in **ENPM702 (Fall 2025)**.

Source Code
------------

Source code for all lectures is available at

.. raw:: html

   <a href="https://github.com/zeidk/enpm702-fall-2025-ROS" target="_blank" rel="noopener noreferrer">
       https://github.com/zeidk/enpm702-fall-2025-ROS
   </a>


Lecture 10 - ROS Introduction
=======================================

Lecture 10 introduces ROS 2 (Robot Operating System 2), a modern robotics middleware framework that enables developers to build distributed robotic systems through modular, reusable components. Students learn the fundamental architecture, core concepts, and practical development workflow for creating ROS 2 nodes and applications.

See the full lecture here → :doc:`lecture10/lecture10`

**Core Concepts**

- **ROS 2 Architecture:** Understanding nodes as fundamental computation units, the DDS-based communication layer, and the distributed computation graph.
- **Communication Paradigms:** Mastering the publish-subscribe pattern for continuous data streams, services for request-response interactions, and actions for long-running tasks.
- **Workspace Management:** Organizing source code, build artifacts, and installed packages in structured workspaces with proper overlay configurations.
- **Package Development:** Creating ROS 2 packages with proper manifest files (``package.xml``), build configurations (``CMakeLists.txt``), and dependency management.
- **Node Implementation:** Writing both functional and object-oriented ROS 2 nodes in C++, understanding node lifecycle, spinning mechanisms, and callback processing.
- **Publishers and Subscribers:** Implementing topic-based communication with configurable Quality of Service settings and proper message handling.
