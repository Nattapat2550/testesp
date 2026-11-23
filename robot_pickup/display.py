import cv2

from robot_pickup.control.controller import ControlStatus
from robot_pickup.models import Detection, Scene
from robot_pickup.perception.camera import VisionSnapshot
from robot_pickup.perception.detector import Frame
from robot_pickup.sensors.association import BeamSettings


def draw_overlay(snapshot: VisionSnapshot, status: ControlStatus, beam: BeamSettings) -> Frame:
    frame = snapshot.frame.copy()
    scene = snapshot.scene
    center = (int((1 + beam.center_x) * scene.width / 2), int((1 + beam.center_y) * scene.height / 2))
    cv2.drawMarker(frame, center, (210, 210, 210), cv2.MARKER_CROSS, 16, 1)

    cm = status.target_distance_cm
    range_label = f" | {cm:.1f} cm" if cm is not None else " | range unknown"
    _draw_objects(frame, scene, status.target_id, range_label)

    command = status.motion
    motion = "No motion" if command is None else f"{command.kind.value}  v={command.linear:+.2f}  w={command.angular:+.2f}"
    if status.action is not None:
        motion += f"  |  {status.action.value}"
    title = f"{status.state.value}  |  Detections {len(scene.objects)}"
    if status.target_id is not None:
        title += f"  |  target #{status.target_id}" + ("" if status.target_visible else " missing")
    if status.result is not None:
        title += f"  |  {status.result.value}"
    beam_label = "Beam range unavailable" if status.beam_cm is None else f"Beam {status.beam_cm:.1f} cm"
    cv2.rectangle(frame, (0, 0), (scene.width, 73 if status.reason else 56), (20, 20, 20), -1)
    _text(frame, title, (8, 16))
    _text(frame, motion, (8, 33))
    _text(frame, beam_label, (8, 50))
    if status.reason:
        _text(frame, status.reason, (8, 67), (100, 180, 255))
    return frame


def _draw_objects(frame: Frame, scene: Scene, target_id: int | None, target_detail: str = "") -> None:
    for obj in scene.objects:
        box = obj.bbox
        selected = obj.object_id is not None and obj.object_id == target_id
        color = (0, 220, 255) if selected else (80, 230, 100)
        title = f"{obj.label} #{obj.object_id}"
        if obj.confidence is not None:
            title += f" {obj.confidence:.0%}"
        if selected:
            title += target_detail
        cv2.rectangle(frame, (box.x1, box.y1), (box.x2 - 1, box.y2 - 1), color, 2)
        cv2.circle(frame, ((box.x1 + box.x2) // 2, (box.y1 + box.y2) // 2), 4, color, -1)
        _text(frame, title, (box.x1, max(16, box.y1 - 6)), color)


def draw_position_overlay(snapshot: VisionSnapshot, target: Detection | None) -> Frame:
    frame = snapshot.frame.copy()
    scene = snapshot.scene
    if target is not None:
        _draw_objects(frame, scene, target.object_id)
        title = f"{target.label}  cx {target.cx:+.2f}  cy {target.cy:+.2f}"
    else:
        title = "Target unavailable"
    cv2.rectangle(frame, (0, 0), (scene.width, 24), (20, 20, 20), -1)
    _text(frame, title, (8, 16))
    return frame


def _text(frame: Frame, value: str, origin: tuple[int, int], color: tuple[int, int, int] = (240, 240, 240)) -> None:
    cv2.putText(frame, value, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, value, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


class DebugWindow:
    def __init__(self, title: str = "Robot pickup") -> None:
        self.title = title

    def show(self, snapshot: VisionSnapshot, status: ControlStatus, beam: BeamSettings) -> bool:
        return self._show(draw_overlay(snapshot, status, beam))

    def show_position(self, snapshot: VisionSnapshot, target: Detection | None) -> bool:
        return self._show(draw_position_overlay(snapshot, target))

    def _show(self, frame: Frame) -> bool:
        cv2.imshow(self.title, frame)
        key = cv2.waitKey(1) & 0xff
        if key in (ord("q"), 27):
            return False
        try:
            return cv2.getWindowProperty(self.title, cv2.WND_PROP_VISIBLE) >= 1
        except cv2.error:
            return False

    def close(self) -> None:
        try:
            cv2.destroyWindow(self.title)
        except cv2.error:
            pass
