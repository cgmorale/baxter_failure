# -*- coding: utf-8 -*-
"""
Created on Thu Nov  2 13:32:23 2017

@author: lab
"""

    def load_gazebo_models(self, table_pose=Pose(position=Point(x=0.84, y=0.2, z=-0.55)),
                       table_reference_frame="base"):
        
#        self.trashposes,self.baxjoints =TagsPose.makeDictofTransformedPoses(TagsPose())
#        for key, val in self.trashposes.items():
#            val = self.trashposes[klimbey]
#            self.trashloc = val.pose
#            self.trash_loc_x.append(self.trashloc.position.x)
#            self.trash_loc_y.append(self.trashloc.position.y)
#            self.trash_loc_z.append(self.trashloc.position.z)
            
        block_pose = Pose()
        block_pose.position.x = 0.3 #self.trash_loc_x[0]
        block_pose.position.y = 0.1 #self.trash_loc_y[0]
        block_pose.position.z = -0.16 #self.trash_loc_z[0]
        block_pose.orientation.x = 0.0
        block_pose.orientation.y = 0.0
        block_pose.orientation.z = 0.0
        block_pose.orientation.w = 1.0
        block_reference_frame = "base"
    # Get Models' Path
        model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
    # Load Table SDF
        table_xml = ''
        with open (model_path + "cafe_table/model.sdf", "r") as table_file:
            table_xml=table_file.read().replace('\n', '')
    # Load Block URDF
        block_xml = ''
        with open (model_path + "block/model.urdf", "r") as block_file:
            block_xml=block_file.read().replace('\n', '')
    # Spawn Table SDF
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))
    # Spawn Block URDF
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        try:
            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn URDF service call failed: {0}".format(e))  
    
    def delete_gazebo_models(self):
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model("cafe_table")
            resp_delete = delete_model("block")
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))