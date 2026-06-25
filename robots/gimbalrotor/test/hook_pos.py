import rospy
import tf
from gazebo_msgs.msg import ModelStates

br =None
def cb(msg):
    if "hook" not in msg.name:
        print("name_error")
        return
    idx = msg.name.index("hook")

    pose = msg.pose[idx]

    br.sendTransform(

        (
            pose.position.x,
            pose.position.y,
            pose.position.z
        ),

        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ),
        rospy.Time.now(),
        "hook",
        "world"
    )
    
    print("x:",pose.position.x)
    print("y:",pose.position.y)
    print("z:",pose.position.z)
    q = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    )

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(q)

    print("roll  =", roll)
    print("pitch =", pitch)
    print("yaw   =", yaw)
    print("-----")
rospy.init_node("hook_pos")

br= tf.TransformBroadcaster()
rospy.Subscriber(
    "/gazebo/model_states",
    ModelStates,
    cb)
rospy.spin()
