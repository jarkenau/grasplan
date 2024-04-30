# Copyright (c) 2024 DFKI GmbH
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, Vector3
from std_msgs.msg import ColorRGBA
from interactive_markers.interactive_marker_server import (
    InteractiveMarkerServer,
    InteractiveMarker,
)
from visualization_msgs.msg import (
    InteractiveMarkerFeedback,
    InteractiveMarkerControl,
    Marker,
)


def callback(feedback: InteractiveMarkerFeedback) -> None:
    """
    Callback function for handling interactive marker feedback.

    Args:
        feedback (InteractiveMarkerFeedback): The feedback object containing information about the interactive marker.

    Returns:
        None
    """
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        pose = feedback.pose
        print(f"Pose updated: {pose.position.x}, {pose.position.y}, {pose.position.z}")


def make_box(initial_pose: Pose, color: ColorRGBA, scale: Vector3, frame_id: str = "map") -> Marker:
    """
    Creates a Marker object representing a box.

    https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html

    Args:
        initial_pose (Pose): The initial pose of the box.
        color (ColorRGBA): The color of the box.
        scale (Vector3): The scale of the box.
        frame_id (str, optional): The frame ID of the box. Defaults to "map".

    Returns:
        Marker: The created Marker object.
    """
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = Marker.CUBE
    marker.pose = initial_pose
    marker.color = color
    marker.scale = scale
    return marker


def make_control(name: str, orientation: Quaternion, interaction_type) -> InteractiveMarkerControl:
    """
    Create a control mechanism to be displayed with the interactive marker.

    https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/InteractiveMarkerControl.html

    Args:
        name (str): Nname of the control. Must be unique within the marker!
        orientation (Quaternion): Local coordinate frame in which is being rotated or translated.
        interaction_type: The interaction mode of the control. Possible values (MOVE_AXIS or ROTATE_AXIS)

    Returns:
        InteractiveMarkerControl: The InteractiveMarkerControl object.
    """
    control = InteractiveMarkerControl()
    control.name = name
    control.orientation = orientation
    control.interaction_mode = interaction_type
    return control


def make_marker(name: str, initial_pose: Pose, frame_id: str = "map") -> InteractiveMarker:
    """
    Create an InteractiveMarker object with the given name, start position, and frame ID.

    https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/InteractiveMarker.html

    Parameters:
    - name (str): The name of the marker. Must be globally unique in the topic that it is being published in!
    - initial_pose (Pose): The initial position of the marker.
    - frame_id (str, optional): The frame ID of the marker. Defaults to "map".

    Returns:
    - InteractiveMarker: The created InteractiveMarker object.
    """
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.pose = initial_pose
    int_marker.name = name

    # add a box to controll with the marker
    # make_box_control(int_marker)

    # add the individual controls to the marker
    int_marker.controls.append(make_control("rotate_x", Quaternion(w=1, x=1), InteractiveMarkerControl.ROTATE_AXIS))
    int_marker.controls.append(make_control("move_x", Quaternion(w=1, x=1), InteractiveMarkerControl.MOVE_AXIS))
    int_marker.controls.append(make_control("rotate_y", Quaternion(w=1, y=1), InteractiveMarkerControl.ROTATE_AXIS))
    int_marker.controls.append(make_control("move_y", Quaternion(w=1, y=1), InteractiveMarkerControl.MOVE_AXIS))
    int_marker.controls.append(make_control("rotate_z", Quaternion(w=1, z=1), InteractiveMarkerControl.ROTATE_AXIS))
    int_marker.controls.append(make_control("move_z", Quaternion(w=1, z=1), InteractiveMarkerControl.MOVE_AXIS))

    # TODO: add paraemters to add a box to the marker
    # add a box for visualization purposes, the box could also be made moveable
    control = InteractiveMarkerControl()
    control.markers.append(make_box(initial_pose, ColorRGBA(r=0.1, g=0.5, b=0.5, a=1), Vector3(x=0.5, y=0.5, z=1)))
    control.name = "box_display"

    int_marker.controls.append(control)

    return int_marker


if __name__ == "__main__":
    rospy.init_node("interactive_marker_test")

    server = InteractiveMarkerServer("interactive_marker")

    pose = Pose(position=Point(x=0, y=0, z=0), orientation=Quaternion(w=1, x=0, y=0, z=0))
    int_marker = make_marker("marker1", pose)

    server.insert(int_marker, callback)

    server.applyChanges()

    rospy.spin()
