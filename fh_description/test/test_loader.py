#!/usr/bin/env python
"""
See README.md
"""

import rospy
from rospkg.common import ResourceNotFound
from unittest import TestCase
from fh_description.urdf_loader import UrdfLoader


class TestLoader(TestCase):

    def test_urdf_loader_good_constructor(self):
        urdf_loader = None
        try:
            urdf_loader = UrdfLoader()
        except ResourceNotFound as e:
            urdf_loader = None

        self.assertIsNotNone(urdf_loader, "Default constructor broken.")

    def test_urdf_loader_bad_constructor(self):
        urdf_loader = None
        try:
            urdf_loader = UrdfLoader("bad_package_name")
        except ResourceNotFound:
            urdf_loader = None

        self.assertIsNone(urdf_loader, "The bad_package_name shouldn't be found.")

    def test_found_xacros(self):
        urdf_loader = UrdfLoader()
        self.assertGreater(len(urdf_loader.xacro_files), 0, "No xacro found.")

    def test_loaded_xacro(self):
        UrdfLoader()
        loaded_xacro = rospy.get_param("/robot_descriptions")

        self.assertGreater(len(loaded_xacro.keys()), 0, "No loaded xacro found on the parameter server.")


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("fh_description", 'test_loader', TestLoader)
