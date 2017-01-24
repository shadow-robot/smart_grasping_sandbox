import openravepy
import multiprocessing
import numpy
import time

from scipy.optimize import minimize

from tf.transformations import *
from math import pi

import  random

DEBUG = False

class GraspEvaluator(object):
  def __init__(self, urdf_path, srdf_path, chucking_direction, target_path, viewer=True):
    self.__env = openravepy.Environment()
    plugin = openravepy.RaveCreateModule(self.__env, "urdf")

    self.set_viewer(viewer)

    self.__robot_name = plugin.SendCommand("LoadURI "+urdf_path+" "+srdf_path)
    self.__robot = self.__env.GetRobot(self.__robot_name)

    self.__end_effector = self.__robot.GetManipulators()[0]

    self.__chucking_direction = chucking_direction
    self.__end_effector.SetChuckingDirection(chucking_direction)

    self.__create_target(target_path)

  def set_viewer(self, on):
    if on:
      self.__env.SetViewer('qtcoin')

  def generate_all_grasps(self, initial_grasp, initial_translation, initial_rotation):

    transform = compose_matrix(translate=tuple(initial_translation), angles=tuple(initial_rotation))

    if True:
      print "----------------"
      print "Grasp: "
      print " -> DOFs ", initial_grasp
      print " -> translation ", initial_translation
      print " -> rotation ", initial_rotation

    grasp_joint_values = numpy.array(initial_grasp)
    self.__robot.SetDOFValues(grasp_joint_values)
    #self.__robot.SetTransform(transform)

    self.__target.SetTransform(transform)

    gmodel = openravepy.databases.grasping.GraspingModel(self.__robot, self.__target)
    #gmodel.numthreads = multiprocessing.cpu_count()

    self.__robot.SetActiveDOFs(gmodel.manip.GetGripperIndices())
    self.__env.UpdatePublishedBodies()

    if not gmodel.load():
      options = {}
      options["preshapes"] = initial_grasp

      gmodel.autogenerate(options)
      gmodel.save()

    gmodel.show()

  def __create_target(self, target):
    self.__target = self.__env.ReadKinBodyURI(target)
    self.__env.Add(self.__target)


class GraspImprover(object):

  def __init__(self, urdf_path, srdf_path, chucking_direction,
               target_path, initial_translation, initial_rotation,
               initial_grasp):
    self.__grasp_evaluator = GraspEvaluator(urdf_path, srdf_path, chucking_direction, target_path, viewer=True)

    print "INPUT: "
    print "Initial grasp:", initial_grasp
    print "Initial translation: ", initial_translation
    print "Initial rotation: ", initial_rotation

    self.__grasp_evaluator.generate_all_grasps(initial_grasp, initial_translation, initial_rotation)

if __name__=="__main__":
  urdf_path = "/code/workspace/src/smart_grasping_sandbox/fh_desc/hand_h.urdf"
  srdf_path = "/code/workspace/src/smart_grasping_sandbox/fh_desc/hand_h.srdf"
  chucking_direction = (1, 1, 1, 1, 1, 1)

  target = '/home/ugo/Downloads/hammer.stl'
  initial_translation = [-0.10, 0.01, 0.3]
  initial_rotation = [0.05, pi/2., 0.0]
  initial_grasp = [-0.05, 0.35, -0.05, 0.0, -0.05, 0.0, 0]

  # improve the grasp
  GraspImprover(urdf_path, srdf_path, chucking_direction, target,
                initial_translation, initial_rotation, initial_grasp)
