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

    gmodel = openravepy.databases.grasping.GraspingModel(self.__robot, self.__target)
    gmodel.numthreads = 3

    if not gmodel.load():
      options = {}
      standoffs = range(0, 40)
      options["standoffs"] = [x / 100. for x in standoffs]
      options["normalanglerange"] = pi/2.0
      gmodel.autogenerate(options)
      gmodel.save()

    closest_grasp = None
    closest_distance = None
    for i, grasp in enumerate(gmodel.grasps):
      print 'grasp %d/%d' % (i, len(gmodel.grasps))
      try:
        with self.__env:
          contacts, finalconfig, mindist, volume = gmodel.testGrasp(grasp=grasp, translate=True,
                                                                    forceclosure=True,
                                                                    graspingnoise=0.1)
          # contacts,finalconfig,mindist,volume = self.runGrasp(grasp=grasp,translate=True,forceclosure=True)
          if mindist == 0:
            print 'grasp is not in force closure!'
            continue


          grasp_distance = numpy.linalg.norm(finalconfig[1]-transform) + numpy.linalg.norm(finalconfig[0]-initial_grasp)
          if closest_distance is None:
            closest_distance = grasp_distance
            closest_grasp = finalconfig
          elif grasp_distance < closest_distance:
            closest_distance = grasp_distance
            closest_grasp = finalconfig
            print "new best DISTANCE = ", grasp_distance

      except openravepy.planning_error, e:
        print 'bad grasp!', e
        continue

    self.__robot.GetController().Reset(0)
    self.__robot.SetDOFValues(closest_grasp[0])
    self.__robot.SetTransform(closest_grasp[1])
    self.__env.UpdatePublishedBodies()
    raw_input("TOTO")

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
  urdf_path = "/workspace/src/smart_grasping_sandbox/fh_desc/hand_h.urdf"
  srdf_path = "/workspace/src/smart_grasping_sandbox/fh_desc/hand_h.srdf"
  chucking_direction = (1, 1, 1, 1, 1, 1)

  target = '/workspace/src/smart_grasping_sandbox/openrave/hammer.stl'
  initial_translation = [0.0, 0.01, 0.3]
  initial_rotation = [0.0, pi/2., 0.0]
  initial_grasp = [-0.05, 0.35, -0.05, 0.1, -0.05, 0.1, 0]

  # improve the grasp
  GraspImprover(urdf_path, srdf_path, chucking_direction, target,
                initial_translation, initial_rotation, initial_grasp)
