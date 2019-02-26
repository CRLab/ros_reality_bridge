import rospy
import ros_reality_bridge.msg
import ros_reality_bridge.srv


def plan_grasp(plan_grasp_request):
    # type: (ros_reality_bridge.srv.PlanGraspRequest) -> ros_reality_bridge.srv.PlanGraspResponse

    return ros_reality_bridge.srv.PlanGraspResponse()


rospy.init_node('test_server')

plan_grasp_service = rospy.Service("/plan_grasp", ros_reality_bridge.srv.PlanGrasp, plan_grasp)
scene_publisher = rospy.Publisher("/completed_meshes", ros_reality_bridge.msg.MeshArray, queue_size=10)
ungraspable_mesh_publisher = rospy.Publisher("/ungraspable_scene_meshes", ros_reality_bridge.msg.MeshArray, queue_size=10)

while not rospy.is_shutdown():
    rospy.spin()
