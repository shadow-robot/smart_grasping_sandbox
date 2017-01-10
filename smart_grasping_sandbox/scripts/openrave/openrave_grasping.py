import openravepy
import multiprocessing
import numpy
import time


class GraspEvaluator(object):
  def __init__(self, urdf_path, srdf_path, chucking_direction, viewer=True):
    self.__env = openravepy.Environment()
    plugin = openravepy.RaveCreateModule(self.__env, "urdf")

    if viewer:
      self.__env.SetViewer('qtcoin')

    self.__robot_name = plugin.SendCommand("LoadURI "+urdf_path+" "+srdf_path)
    self.__robot = self.__env.GetRobot(self.__robot_name)

    self.__end_effector = self.__robot.GetManipulators()[0]

    self.__end_effector.SetChuckingDirection(chucking_direction)

  def evaluate(self, grasp, target):
    self.__create_target(target)

    grasp_joint_values = numpy.array(grasp)
    self.__robot.SetDOFValues(grasp_joint_values)
    self.__target.SetVisible(True)

    #contacts,finalconfig,mindist,volume = grasp_model.testGrasp(grasp = grasp,
    #                                                            Ngraspingtries=1000,
    #                                                            translate=True,
    #                                                            forceclosure=True) #graspingnoise


    #gmodel.showgrasp(grasp)

  def generate_all_grasps(self, target):
    self.__env.Add(target)
    gmodel = openravepy.databases.grasping.GraspingModel(self.__robot, target)
    #gmodel.numthreads = multiprocessing.cpu_count()
    if not gmodel.load():
      gmodel.autogenerate()
      gmodel.save()

  def __create_target(self, target):
    #TODO Look at target.InitFromTrimesh and use target_path instead of target
    self.__target = openravepy.RaveCreateKinBody(self.__env, '')
    self.__target.SetName(target)
    self.__target.InitFromSpheres(numpy.array([[0, 0, 0, 0.0375]]), True)
    # target = env.ReadKinBodyXMLFile('/usr/share/openrave-0.9/data/mug2.kinbody.xml')

    self.__env.Add(self.__target)

if __name__=="__main__":
  urdf_path = "/code/workspace/src/smart_grasping_sandbox/fh_desc/hand_h.urdf"
  srdf_path = "/code/workspace/src/smart_grasping_sandbox/fh_desc/hand_h.srdf"
  chucking_direction = (1,1,1,1,1,1)

  grasp_evaluator = GraspEvaluator(urdf_path, srdf_path, chucking_direction)

  target = 'cricket_ball'
  grasp = [-0.05, 0.4, -0.05, 0.4, -0.05, 0.4, 0]
  grasp_evaluator.evaluate(grasp, target)
  time.sleep(10.)