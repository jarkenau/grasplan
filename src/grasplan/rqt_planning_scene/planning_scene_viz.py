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


# TODO: move to .sence file format from MoveIt!
class PlanningSceneHelper:
    def __init__(self, scene_path: str) -> None:
        self.marker_server = InteractiveMarkerServer("interactive_marker")
        self.scene_path = Path(scene_path)
        self.data = None

        self._load_scene(self.scene_path)

    def _load_scene(self, path: Path) -> None:
        if not path.exists() or not path.is_file() or path.suffix != ".yaml":
            raise FileNotFoundError(f"YAML file not found at: {path.absolute()}")
        with open(path, "r") as file:
            self.data = yaml.safe_load(file)

    def _save_scene(self, path: Path) -> None:
        if path.suffix != ".yaml":
            raise ValueError(f"Path must have a '.yaml' ending: {path.absolute()}")
        with open(path, "w") as f:
            yaml.dump(self.data, f)

    def _update_scene(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            for box in self.data["planning_scene_boxes"]:
                if box["scene_name"] == feedback.marker_name:
                    box["box_orientation_w"] = feedback.pose.orientation.w
                    box["box_orientation_x"] = feedback.pose.orientation.x
                    box["box_orientation_y"] = feedback.pose.orientation.y
                    box["box_orientation_z"] = feedback.pose.orientation.z
                    box["box_position_x"] = feedback.pose.position.x
                    box["box_position_y"] = feedback.pose.position.y
                    box["box_position_z"] = feedback.pose.position.z
        self._save_scene(self.scene_path)

    def publish_boxes(self) -> None:
        for box in self.data["planning_scene_boxes"]:
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
            self.marker_server.insert(int_marker, self._update_scene)

        self.marker_server.applyChanges()


if __name__ == "__main__":
    rospy.init_node("planning_scene_viz")
    viz = PlanningSceneHelper("config/examples/planning_scene.yaml")
    viz.publish_boxes()
    rospy.spin()
