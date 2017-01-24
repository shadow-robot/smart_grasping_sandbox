import openravepy
import multiprocessing
import numpy
import time

from scipy.optimize import minimize, fminbound

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

  def evaluate(self, DOFValues, translation, rotation):
    transform = compose_matrix(translate=tuple(translation), angles=tuple(rotation))

    if True:
      print "----------------"
      print "Grasp: "
      print " -> DOFs ", DOFValues
      print " -> translation ", translation
      print " -> rotation ", rotation

    grasp_joint_values = numpy.array(DOFValues)
    self.__robot.SetDOFValues(grasp_joint_values)
    #self.__robot.SetTransform(transform)

    self.__target.SetTransform(transform)

    self.__env.UpdatePublishedBodies()

    self.gmodel = openravepy.databases.grasping.GraspingModel(self.__robot, self.__target)
    self.gmodel.init(friction=0.3, avoidlinks=None)
    self.__robot.SetActiveDOFs(self.gmodel.manip.GetGripperIndices())

    final = grasp_joint_values
    # Simply closing the fingers till they all touch the object for a quick optimisation.
    #taskmanip = openravepy.interfaces.TaskManipulation(self.__robot)
    #final, _ = taskmanip.ChuckFingers(outputfinal=True)
    #self.__robot.WaitForController(0)
    #time.sleep(0.5)
    #if DEBUG:  "Final grasp after chucking fingers : ", final

    grasp = numpy.zeros(self.gmodel.totaldof)
    grasp[self.gmodel.graspindices.get('igrasppreshape')] = final


    try:
      with self.__robot:
        contacts,finalconfig,mindist,volume = self.gmodel.grasper.Grasp(transformrobot=False, target=self.gmodel.target,
                                                                        onlycontacttarget=True, forceclosure=True,
                                                                        execute=False, outputfinal=True,
                                                                        translationstepmult=None,
                                                                        finestep=None,
                                                                        chuckingdirection=self.__chucking_direction)
      self.__env.UpdatePublishedBodies()
    except openravepy.PlanningError:
      return 0, 0
    contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
    time.sleep(2.0)

    #print "CONTACTS: ", contacts
    #raw_input("CONTACTS")

    if DEBUG:
      print finalconfig
      print "Mindist: ", mindist
      print"Volume:", volume
      print "----------------"

    # returning the two grasp qualities
    return mindist, volume


  def generate_all_grasps(self, target):
    self.__create_target(target)

    gmodel = openravepy.databases.grasping.GraspingModel(self.__robot, self.__target)
    #gmodel.numthreads = multiprocessing.cpu_count()
    if not gmodel.load():
      gmodel.autogenerate()
      gmodel.save()

  def __create_target(self, target):
    self.__target = self.__env.ReadKinBodyURI(target)
    self.__env.Add(self.__target)


class GraspImprover(object):

  def __init__(self, urdf_path, srdf_path, chucking_direction,
               target_path, initial_translation, initial_rotation,
               initial_grasp):
    self.__grasp_evaluator = GraspEvaluator(urdf_path, srdf_path, chucking_direction, target_path, viewer=True)
    self.__initial_grasp_len = len(initial_grasp)

    initial_conditions, bounds = self.to_vector(initial_grasp, initial_translation, initial_rotation)

    print "INPUT: "
    print "Initial grasp:", initial_grasp
    print "Initial translation: ", initial_translation
    print "Initial rotation: ", initial_rotation

    # bounds = bounds
    res = minimize(self.evaluate, initial_conditions, method='nelder-mead', options = {'maxiter': 1000, 'disp': True})

    print res

    self.__grasp_evaluator.set_viewer(True)
    self.evaluate(res.x)
    raw_input("Best grasp")


  def evaluate(self, input_vector):
    grasp, translation, rotation= self.from_vector(input_vector)
    mindist, volume = self.__grasp_evaluator.evaluate(grasp, translation, rotation)

    print "grasp quality: volume=", volume, " mindist=", mindist

    # we're aiming to minimise so we return minus the metric
    return -mindist

  def to_vector(self, initial_grasp, initial_translation, initial_rotation):
    initial_conditions = initial_grasp + initial_translation + initial_rotation
    bounds = []

    for dof in initial_grasp:
      bounds += [[dof - 0.5, dof+0.5]]

    for t in initial_translation:
      bounds += [[t-0.1, t+0.1]]

    for r in initial_rotation:
      bounds += [[r-0.5, r+0.5]]

    return numpy.array(initial_conditions), numpy.array(bounds)

  def from_vector(self, input_vector):
    initial_grasp = input_vector[:self.__initial_grasp_len]
    initial_translation = input_vector[self.__initial_grasp_len:self.__initial_grasp_len+3]
    initial_rotation = input_vector[self.__initial_grasp_len+3:]

    if DEBUG:
      print initial_grasp, initial_translation, initial_rotation

    return initial_grasp, initial_translation, initial_rotation

if __name__=="__main__":
  urdf_path = "/code/workspace/src/smart_grasping_sandbox/fh_desc/hand_h.urdf"
  srdf_path = "/code/workspace/src/smart_grasping_sandbox/fh_desc/hand_h.srdf"
  chucking_direction = (1, 1, 1, 1, 1, 1)

  target = '/home/ugo/Downloads/hammer.stl'
  initial_translation = [-0.10, 0.01, 0.3]
  initial_rotation = [0.05, pi/2., 0.0]
  initial_grasp = [-0.05, 0.35, -0.05, 0.2, -0.05, 0.2, 0]

  # improve the grasp
  GraspImprover(urdf_path, srdf_path, chucking_direction, target,
                initial_translation, initial_rotation, initial_grasp)
