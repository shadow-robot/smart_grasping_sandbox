import openravepy
import multiprocessing
import numpy
import time

from scipy.optimize import minimize, fminbound

from tf.transformations import *
from math import pi

DEBUG = False

class GraspEvaluator(object):
  def __init__(self, urdf_path, srdf_path, chucking_direction, target_path, viewer=True):
    self.__env = openravepy.Environment()
    plugin = openravepy.RaveCreateModule(self.__env, "urdf")

    if viewer:
      self.__env.SetViewer('qtcoin')

    self.__robot_name = plugin.SendCommand("LoadURI "+urdf_path+" "+srdf_path)
    self.__robot = self.__env.GetRobot(self.__robot_name)

    self.__end_effector = self.__robot.GetManipulators()[0]

    self.__chucking_direction = chucking_direction
    self.__end_effector.SetChuckingDirection(chucking_direction)

    self.__create_target(target_path)

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

    self.gmodel = openravepy.databases.grasping.GraspingModel(self.__robot, self.__target)


    self.gmodel.init(friction=0.3, avoidlinks=None)

    final = grasp_joint_values
    # Simply closing the fingers till they all touch the object for a quick optimisation.
    #taskmanip = openravepy.interfaces.TaskManipulation(self.__robot)
    #final, _ = taskmanip.ChuckFingers(outputfinal=True)
    #time.sleep(0.5)
    #if DEBUG:  "Final grasp after chucking fingers : ", final

    grasp = numpy.zeros(self.gmodel.totaldof)
    grasp[self.gmodel.graspindices.get('igrasppreshape')] = final


    try:
      contacts,finalconfig,mindist,volume = self.runGraspFromTrans(grasp)
    except openravepy.PlanningError:
      return 1e-10, 1e-10
    contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
    time.sleep(0.2)

    if DEBUG:
      print finalconfig
      print "Mindist: ", mindist
      print"Volume:", volume
      print "----------------"

    # returning the two grasp qualities
    if mindist == 0.0:
      mindist = 1e-10
    if volume == 0.0:
      volume = 1e-10
    return mindist, volume


  def generate_all_grasps(self, target):
    self.__create_target(target)

    gmodel = openravepy.databases.grasping.GraspingModel(self.__robot, self.__target)
    #gmodel.numthreads = multiprocessing.cpu_count()
    if not gmodel.load():
      gmodel.autogenerate()
      gmodel.save()

  def runGraspFromTrans(self, grasp, finestep=None):
    with self.__robot:
      self.__robot.SetDOFValues(grasp[self.gmodel.graspindices.get('igrasppreshape')],
                                self.gmodel.manip.GetGripperIndices())
      Tmanip = self.gmodel.manip.GetTransform()
      Tmanip[0:3, 3] += numpy.dot(Tmanip[0:3, 0:3], grasp[self.gmodel.graspindices.get('igrasptranslationoffset')])
      self.__robot.SetTransform(numpy.dot(self.gmodel.getGlobalGraspTransform(grasp),
                                          numpy.dot(numpy.linalg.inv(Tmanip), self.__robot.GetTransform())))
      self.__robot.SetActiveDOFs(self.gmodel.manip.GetGripperIndices())
      if len(self.gmodel.manip.GetGripperIndices()) == 0:
        return [], [[], self.__robot.GetTransform()], None, None
      finestep = None

      return self.gmodel.grasper.Grasp(transformrobot=False, target=self.gmodel.target, onlycontacttarget=True,
                                       forceclosure=True, execute=False, outputfinal=True, translationstepmult=None,
                                       finestep=finestep, chuckingdirection=self.__chucking_direction)

  def __create_target(self, target):
    self.__target = self.__env.ReadKinBodyURI(target)
    self.__env.Add(self.__target)


class GraspImprover(object):

  def __init__(self, urdf_path, srdf_path, chucking_direction,
               target_path, initial_translation, initial_rotation,
               initial_grasp):
    self.__grasp_evaluator = GraspEvaluator(urdf_path, srdf_path, chucking_direction, target_path)
    self.__initial_grasp_len = len(initial_grasp)

    initial_conditions, bounds = self.to_vector(initial_grasp, initial_translation, initial_rotation)

    print "INPUT: "
    print "Initial grasp:", initial_grasp
    print "Initial translation: ", initial_translation
    print "Initial rotation: ", initial_rotation

    # bounds = bounds
    res = minimize(self.evaluate, initial_conditions, method='nelder-mead', options = {'maxiter': 100, 'disp': True})

    print res


  def evaluate(self, input_vector):
    initial_grasp, initial_translation, initial_rotation = self.from_vector(input_vector)
    mindist, volume = self.__grasp_evaluator.evaluate(initial_grasp, initial_translation, initial_rotation)

    print "grasp quality: ", 1.0/volume

    return 1.0/volume

  def to_vector(self, initial_grasp, initial_translation, initial_rotation):
    initial_conditions = initial_grasp + initial_translation + initial_rotation
    bounds = []

    for dof in initial_grasp:
      bounds += [[dof - 0.3, dof+0.3]]

    for t in initial_translation:
      bounds += [[t-0.05, t+0.05]]

    for r in initial_rotation:
      bounds += [[r-0.17, r+0.17]]

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
  initial_translation = [0.0, 0.005, 0.25]
  initial_rotation = [0.0, pi/2., 0.0]
  initial_grasp = [-0.05, 0.4, -0.05, 0.4, -0.05, 0.4, 0]

  # improve the grasp
  GraspImprover(urdf_path, srdf_path, chucking_direction, target,
                initial_translation, initial_rotation, initial_grasp)
