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
from pathlib import Path
import yaml
from geometry_msgs.msg import Pose, Vector3, Point, Quaternion
from marker_tools import make_box, Color, make_int_marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarkerFeedback


def callback(feedback: InteractiveMarkerFeedback) -> None:
    """
    Callback function for handling interactive marker feedback.

    Args:
        feedback (InteractiveMarkerFeedback): The feedback object containing information about the interactive marker.

    Returns:
        None
    """
    if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        print(f"Updated {feedback.marker_name}")


class PlanningSceneViz:
    def __init__(self) -> None:
        self.marker_server = InteractiveMarkerServer("interactive_marker")

    def publish_boxes(self, yaml_path: str) -> None:
        yaml_path = Path(yaml_path)
        if not yaml_path.exists() or not yaml_path.is_file() or yaml_path.suffix != ".yaml":
            raise FileNotFoundError(f"YAML file not found at: {yaml_path.absolute()}")
        with open(yaml_path, "r") as file:
            data = yaml.safe_load(file)

        # TODO: move to .sence file format from MoveIt!
        for box in data["planning_scene_boxes"]:
            pose = Pose(
                position=Point(x=box["box_position_x"], y=box["box_position_y"], z=box["box_position_z"]),
                orientation=Quaternion(
                    x=box["box_orientation_x"],
                    y=box["box_orientation_y"],
                    z=box["box_orientation_z"],
                    w=box["box_orientation_w"],
                ),
            )
            scale = Vector3(x=box["box_x_dimension"], y=box["box_y_dimension"], z=box["box_z_dimension"])
            marker = make_box(pose, Color.GREEN, scale)
            int_marker = make_int_marker(box["scene_name"], pose, marker)
            self.marker_server.insert(int_marker, callback)

        self.marker_server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("planning_scene_viz")
    viz = PlanningSceneViz()
    viz.publish_boxes("config/examples/planning_scene.yaml")
    rospy.spin()
