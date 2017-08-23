This folder contains a few scripts I run on the sandbox to get data etc. Use at your own risk :)

# Grasp Quality
The grasp_quality.py runs a list of experiments and records data 
to gather a data set for a grasp quality metrics using machine learning.

It grasps an object, shakes it and computes the distance between the object and
the palm to have an objective grasp quality.

To run it, you can use (first number is the distance you go down towards the ball, second number is the number of trials): 

```
python grasp_quality.py -0.164 10
```

However to properly run it I really recommend creating a new Docker container with  
`grasp_quality.py` as its entrypoint. This way you can run it multiple times in 
parallel on a server for example. 

You can find an example entrypoint in `nodes/grasp_quality_entrypoint.sh`.