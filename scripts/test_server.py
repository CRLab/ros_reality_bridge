import rospy
import rospkg
import os
import plyfile

import std_srvs.srv
import geometry_msgs.msg
import shape_msgs.msg
import std_msgs.msg
import ros_reality_bridge.msg
import ros_reality_bridge.srv


has_planned_grasp = False


def plan_grasp(plan_grasp_request):
    # type: (ros_reality_bridge.srv.PlanGraspRequest) -> ros_reality_bridge.srv.PlanGraspResponse
    global has_planned_grasp

    joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]
    joint_positions = [0.0, 1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]

    response = ros_reality_bridge.srv.PlanGraspResponse()
    response.planned_grasp_state.position = joint_positions
    response.planned_grasp_state.name = joint_names
    response.success = True

    has_planned_grasp = True

    return response


def execute_grasp(_):
    # type: (ros_reality_bridge.srv.ExecuteGraspRequest) -> ros_reality_bridge.srv.ExecuteGraspResponse
    global has_planned_grasp
    response = ros_reality_bridge.srv.ExecuteGraspResponse()
    if has_planned_grasp:
        response.success = True
        has_planned_grasp = False
    else:
        rospy.logerr("Have not planned grasp!")
        response.success = False
        response.error_msg = "Have not planned grasp!"
    return response


def ply_path_to_mesh_msg(ply_path):
    scene_mesh_msg = shape_msgs.msg.Mesh()
    scene_mesh_ply = plyfile.PlyData().read(open(ply_path, 'r'))
    for vertex in scene_mesh_ply['vertex'].data:
        scene_mesh_msg.vertices.append(geometry_msgs.msg.Point(x=vertex['x'], y=vertex['y'], z=vertex['z']))
    for face in scene_mesh_ply['face'].data:
        scene_mesh_msg.triangles.append(shape_msgs.msg.MeshTriangle(vertex_indices=face[0].tolist()))

    return scene_mesh_msg


def publish_meshes(scene_publisher, ungraspable_mesh_publisher, mesh_path):
    scene_meshes = ros_reality_bridge.msg.MeshArray()
    for scene_mesh_path in os.listdir(os.path.join(mesh_path, "scene_meshes")):
        scene_mesh_path = os.path.join(mesh_path, "scene_meshes", scene_mesh_path)
        scene_meshes.meshes.append(ply_path_to_mesh_msg(scene_mesh_path))
    scene_publisher.publish(scene_meshes)

    background_meshes = ros_reality_bridge.msg.MeshArray()
    for background_mesh_path in os.listdir(os.path.join(mesh_path, "background_meshes")):
        background_mesh_path = os.path.join(mesh_path, "background_meshes", background_mesh_path)
        background_meshes.meshes.append(ply_path_to_mesh_msg(background_mesh_path))
    ungraspable_mesh_publisher.publish(background_meshes)


def main():

    rospy.init_node('test_server')

    plan_grasp_service = rospy.Service("/plan_grasp", ros_reality_bridge.srv.PlanGrasp, plan_grasp)
    execute_grasp_service = rospy.Service("/execute_grasp", ros_reality_bridge.srv.ExecuteGrasp, execute_grasp)
    scene_publisher = rospy.Publisher("/completed_meshes", ros_reality_bridge.msg.MeshArray, queue_size=10)
    ungraspable_mesh_publisher = rospy.Publisher("/ungraspable_scene_meshes", ros_reality_bridge.msg.MeshArray,
                                                 queue_size=10)

    ros_pack = rospkg.RosPack()
    mesh_path = os.path.join(ros_pack.get_path("ros_reality_bridge"), "test_meshes")

    rospy.loginfo("Now publishing...")

    while not rospy.is_shutdown():
        publish_meshes(scene_publisher, ungraspable_mesh_publisher, mesh_path)
        rospy.sleep(0.1)

    rospy.loginfo("Stopping...")


if __name__ == "__main__":
    main()