import openravepy
import multiprocessing
import numpy

env = openravepy.Environment()
env.SetViewer('qtcoin')
plugin = openravepy.RaveCreateModule(env, "urdf")

name = plugin.SendCommand("load /code/workspace/src/smart_grasping_sandbox/fh_desc/hand_h.urdf /code/workspace/src/smart_grasping_sandbox/fh_desc/hand_h.srdf")
robot = env.GetRobot(name)

#example getting the transform of the end effector
#this is a matrix containing orientation and position
eff = robot.GetManipulators()[0]
eff.SetChuckingDirection((1,1,1,1,1,1))

target = openravepy.RaveCreateKinBody(env, '')
target.SetName('cricket_ball')
target.InitFromSpheres(numpy.array([[0,0,0,0.0375]]),True)
# also look at target.InitFromTrimesh

#target = env.ReadKinBodyXMLFile('/usr/share/openrave-0.9/data/mug2.kinbody.xml')
env.Add(target)
gmodel = openravepy.databases.grasping.GraspingModel(robot,target)
#gmodel.numthreads = multiprocessing.cpu_count()
if not gmodel.load():
  gmodel.autogenerate()
  gmodel.save()


grasp = gmodel.grasps[0]
contacts,finalconfig,mindist,volume = gmodel.testGrasp(grasp = grasp, Ngraspingtries=1000,translate=True,forceclosure=True) #graspingnoise

print "grasp quality = ", mindist

# Setting grasp from joint values:
jointvalues = numpy.array([-0.05, 0.4, -0.05, 0.4, -0.05, 0.4, 0])
robot.SetDOFValues(jointvalues)


gmodel.showgrasp(grasp)

