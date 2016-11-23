#!/usr/bin/env python
"""
See README.md
"""

import rospy
from rospkg import RosPack
from rospkg.common import ResourceNotFound
import os
import xacro
from fh_msgs.srv import XacroUrdfService


class UrdfLoader(object):
    """
    Loads all the urdf present in the given package, under /robot_descriptions/ parameter.
    """

    xacro_files = []

    def __init__(self, package_name="fh_description", relative_path="robots",
                 root_parameter="robot_descriptions"):
        """
        Builds the urdf loader.

        :param package_name: The package from which to load the urdfs.
        :param relative_path: The folder in which the urdfs are contained in that package.
        :param root_parameter: The root used for uploading the paramaters. They'll be uploaded to root/product_id
        """

        self.__rospack = RosPack()
        self.__package_path = self.__rospack.get_path(package_name)
        self.__package_path = os.path.join(self.__package_path, relative_path)

        if not os.path.isdir(self.__package_path):
            raise ResourceNotFound(str(os.path.realpath(self.__package_path)) + " not found.")

        # find all xacro files in the given directory and load them in the parameter server
        for root, dirs, files in os.walk(self.__package_path):
            for f in files:
                if f.endswith(".xacro"):
                    product_id = f.split(".")[0]

                    xacro_file = os.path.join(root, f)
                    self.xacro_files.append(xacro_file)

                    with open(xacro_file) as opened_xacro:
                        parsed_xacro = xacro.parse(opened_xacro)

                    xacro.process_includes(parsed_xacro, root)

                    rospy.set_param(root_parameter + "/" + product_id,
                                    parsed_xacro.toprettyxml(indent='  '))


class XacroToUrdfServer(object):
    """
    Offers a service to convert a xacro to URDF
    """

    def __init__(self):
        self._rospack = RosPack()
        self._xacro_service = rospy.Service("convert_xacro_to_urdf", XacroUrdfService, self.xacro_urdf_service)

    def xacro_urdf_service(self, req):
        self._rospack.get_path("xacro")
        urdf_doc = xacro.parse(req.xacro)
        xacro.process_doc(urdf_doc)
        rospy.logdebug(str(urdf_doc.toprettyxml(indent='  ')))
        return urdf_doc.toprettyxml(indent='  ')


if __name__ == "__main__":
    rospy.init_node("urdf_loader", anonymous=True)
    package_name_list = rospy.get_param('~package_names', None)
    relative_path_list = rospy.get_param('~relative_paths', None)

    if package_name_list is None and relative_path_list is None:
        UrdfLoader()
    elif (type(package_name_list) is not list or type(relative_path_list) is not list or
              (len(package_name_list) != len(relative_path_list))):
        rospy.logerr("urdf_loader: package_names and relative_paths should be lists of the same size")
    else:
        for package, path in zip(package_name_list, relative_path_list):
            UrdfLoader(package, path)

    XacroToUrdfServer()
    rospy.spin()
