import openravepy
import multiprocessing
import numpy
import time


class GraspEvaluator(object):
  def __init__(self, urdf_path, srdf_path, chucking_direction, viewer=True):
    self.__env = openravepy.Environment()
    #plugin = openravepy.RaveCreateModule(self.__env, "urdf")
    self.__env.Load('shadowtest.env.xml')
    if viewer:
      self.__env.SetViewer('qtcoin')

    #self.__robot_name = plugin.SendCommand("LoadURI "+urdf_path+" "+srdf_path)
    #self.__robot = self.__env.GetRobot(self.__robot_name)
    self.__robot = self.__env.GetRobots()[0]
    self.__end_effector = self.__robot.GetManipulators()[0]
    print self.__end_effector
    self.chucking_direction = chucking_direction

    self.__end_effector.SetChuckingDirection(chucking_direction)

  def evaluate(self, DOFValues, transform, target):
    self.__create_target(target)
    self.gmodel = openravepy.databases.grasping.GraspingModel(self.__robot, self.__target)
    self.gmodel.init(friction=0.3, avoidlinks=None)
    
    raw_input('Before testing grasp')
    grasp_joint_values = numpy.array(DOFValues)
    self.__robot.SetDOFValues(grasp_joint_values)
    self.__robot.SetTransform(transform)
    grasp = numpy.zeros(self.gmodel.totaldof)
    grasp[self.gmodel.graspindices.get('igrasppreshape')] = grasp_joint_values

    contacts,finalconfig,mindist,volume = self.runGraspFromTrans(grasp)
    contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
    print finalconfig
    print "Mindist: ", mindist
    print "Volume:", volume
    raw_input('After testing grasp')

  def runGraspFromTrans(self,grasp, finestep=None):
    with self.__robot:
        self.__robot.SetDOFValues(grasp[self.gmodel.graspindices.get('igrasppreshape')],self.gmodel.manip.GetGripperIndices())
        Tmanip = self.gmodel.manip.GetTransform()
        Tmanip[0:3,3] += numpy.dot(Tmanip[0:3, 0:3], grasp[self.gmodel.graspindices.get('igrasptranslationoffset')])
        self.__robot.SetTransform(numpy.dot(self.gmodel.getGlobalGraspTransform(grasp),numpy.dot(numpy.linalg.inv(Tmanip),self.__robot.GetTransform())))
        self.__robot.SetActiveDOFs(self.gmodel.manip.GetGripperIndices())
        if len(self.gmodel.manip.GetGripperIndices()) == 0:
            return [],[[],self.__robot.GetTransform()],None,None
        finestep = None
        chuckingdirection = self.chucking_direction
        return self.gmodel.grasper.Grasp(transformrobot=False,target=self.gmodel.target,onlycontacttarget=True, forceclosure=True, execute=False, outputfinal=True,translationstepmult=None, finestep=finestep, chuckingdirection=chuckingdirection)

  def generate_all_grasps(self, target):
    self.__create_target(target)

    self.gmodel = openravepy.databases.grasping.GraspingModel(self.__robot, self.__target)
    #gmodel.numthreads = multiprocessing.cpu_count()
    if not self.gmodel.load():
      self.gmodel.autogenerate()
      self.gmodel.save()
    
    validgrasps,validindices = self.gmodel.computeValidGrasps(checkik=False, returnnum=1)
    validgrasp=validgrasps[0] # choose first grasp

    forceclosure = True
    graspingnoise = 0.0
    delay = None
    showcontacts = True
    try:
        with self.gmodel.env:
            contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=validgrasp,translate=True,forceclosure=forceclosure,graspingnoise=graspingnoise)
            print "DOF: finalconfig [0]: ", finalconfig[0]
            print "DOF: finalconfig [0]: ", finalconfig[1]
            #contacts,finalconfig,mindist,volume = self.gmodel.runGrasp(grasp=grasp,translate=True,forceclosure=True)
            if mindist == 0:
                print 'grasp is not in force closure!'
            print "after test _1"
            if showcontacts:
                contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
            self.gmodel.robot.GetController().Reset(0)
            self.gmodel.robot.SetDOFValues(finalconfig[0])
            self.gmodel.robot.SetTransform(finalconfig[1])
            self.gmodel.env.UpdatePublishedBodies()
        if delay is None:
            raw_input('press any key to continue: ')
        elif delay > 0:
            time.sleep(delay)
    except:
        print 'bad grasp!'

  def __create_target(self, target):
    #self.__target = self.__env.ReadKinBodyURI(target)
    self.__target = openravepy.RaveCreateKinBody(self.__env, '')        
    self.__target.SetName(target)        
    self.__target.InitFromSpheres(numpy.array([[0, 0, 0, 0.0375]]), True)
    self.__env.Add(self.__target)


if __name__=="__main__":
  urdf_path = "~/projects/grasping/src/smart_grasping_sandbox/fh_desc/hand_h.urdf"
  srdf_path = "~/projects/grasping/src/smart_grasping_sandbox/fh_desc/hand_h.srdf"
  chucking_direction = (1,1,1,1)

  grasp_evaluator = GraspEvaluator(urdf_path, srdf_path, chucking_direction)

  target = 'cricket_ball'
  #grasp_evaluator.generate_all_grasps(target)
# 
  DOFValues = [1.49100003, 1.49000003, 1.44000002, -0.01745329]
  transform = numpy.array([[-0.15986105, 0.96666667, 0.2, -0.02648], [-0.23333333,  0.15986105, -0.9591663,   0.12699362],  [-0.9591663,  -0.2,         0.2,        -0.02648],[ 0.0, 0.0, 0.0, 1.0]])

  grasp_evaluator.evaluate(DOFValues, transform, target)
