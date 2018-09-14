import logging, numpy, openravepy

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner

            
    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not gmodel.load():
            gmodel.autogenerate()

        base_pose = None
        grasp_config = None
       
        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the 
        #  grasping the bottle
        ###################################################################
        self.grasps = self.gmodel.grasps
        self.graspindices = self.gmodel.graspindices
        #using order grasp from hw1
        print("i am here")
        G = self.order_grasps()
        #self.show_grasp(G)
        grasp_transform = self.gmodel.getGlobalGraspTransform(G, collisionfree = True)
        #inbuilt inverse reachability function
        self.model = openravepy.databases.inversereachability.InverseReachabilityModel(robot=self.robot) 
        _,pose_fn,_ = self.model.computeBaseDistribution(grasp_transform)

        poses,_ = pose_fn(50) #sampling 50 poses
        curr_config = self.base_planner.planning_env.herb.GetCurrentConfiguration()
        obstacles = self.robot.GetEnv().GetBodies()
        for pose in poses:
            self.robot.SetTransform(pose)
            angle = openravepy.axisAngleFromQuat(pose)
            act_pose = copy.deepcopy([pose[4], pose[5], angle[2]])
            pose_n = self.base_planner.planning_env.discrete_env.ConfigurationToNodeId(act_pose)
            base_pose = self.base_planner.planning_env.discrete_env.NodeIdToConfiguration(pose_n)

            quaternion = openravepy.quatFromAxisAngle([0, 0, base_pose[2]])
            quaternion = np.append(quaternion, base_pose[0])
            quaternion = np.append(quaternion, base_pose[1])
            quaternion = np.append(quaternion, 0)

            #setting the robot pose and rotation to check the collision
            self.robot.SetTransform(quaternion)
            self.base_planner.planning_env.herb.SetCurrentConfiguration(base_pose)
            grasp_config = self.manip.FindIKSolution(grasp_transform,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions.IgnoreEndEffectorCollisions)

            if self.robot.GetEnv().CheckCollision(self.robot, obstacles[1]) != True and grasp_config != None:
                self.base_planner.planning_env.herb.SetCurrentConfiguration(curr_config)
                return base_pose, grasp_config
        
        #return base_pose, grasp_config

    def PlanToGrasp(self, obj):

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipultion.CloseFingers()

    def order_grasps(self):
      self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
      for grasp in self.grasps_ordered:
        grasp[self.graspindices.get('performance')] = self.eval_grasp(grasp)
      
      order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
      order = order[::-1]
      self.grasps_ordered = self.grasps_ordered[order]
      #self.show_grasp(self.grasps_ordered[3])
      return self.grasps_ordered[0]

    def show_grasp(self, grasp, delay=1.5):
      with openravepy.RobotStateSaver(self.gmodel.robot):
        with self.gmodel.GripperVisibility(self.gmodel.manip):
          time.sleep(0.1) # let viewer update?
          try:
            with self.env:
               contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
               #if mindist == 0:
               #  print 'grasp is not in force closure!'
               contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
               self.gmodel.robot.GetController().Reset(0)
               self.gmodel.robot.SetDOFValues(finalconfig[0])
               self.gmodel.robot.SetTransform(finalconfig[1])
               self.env.UpdatePublishedBodies()
               time.sleep(delay)
          except openravepy.planning_error,e:
            print 'bad grasp!',e

    def eval_grasp(self, grasp):
      contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)
      obj_position = self.gmodel.target.GetTransform()[0:3,3]
      # for each contact
      G = np.empty([6,0]) #the wrench matrix
      Q = 0
      for c in contacts:
        pos = c[0:3] - obj_position
        dir = -c[3:] #this is already a unit vector
        curr_g=np.concatenate((dir,np.cross(pos,dir)),axis=0)
        curr_g=curr_g[:,np.newaxis]
        G=np.hstack((G,curr_g))
      
      detG = np.linalg.det(np.dot(G,G.T))
      GGT = np.dot(G,np.transpose(G))
      eigG = np.linalg.eig(GGT)
      measure = np.amin(np.sqrt(np.linalg.eig(GGT)[0]), axis=0)
      if np.isnan(measure):
          measure = 1.00
      if detG >=0 :
          Q = np.sqrt(detG)
      else:
            Q = -1.00
      Q = Q + measure
      return Q

    
