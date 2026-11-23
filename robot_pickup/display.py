import cv2

from robot_pickup.control.controller import ControlStatus
from robot_pickup.perception.camera import VisionSnapshot
from robot_pickup.perception.detector import Frame
from robot_pickup.sensors.association import BeamSettings


def draw_overlay(snapshot: VisionSnapshot, status: ControlStatus, beam: BeamSettings) -> Frame:
    frame = snapshot.frame.copy()
    scene = snapshot.scene
    center = (int((1 + beam.center_x) * scene.width / 2), int((1 + beam.center_y) * scene.height / 2))
    cv2.drawMarker(frame, center, (210, 210, 210), cv2.MARKER_CROSS, 16, 1)

    for obj in scene.objects:
        box = obj.bbox
        selected = obj.object_id is not None and obj.object_id == status.target_id
        color = (0, 220, 255) if selected else (80, 230, 100)
        title = f"{obj.label} #{obj.object_id}"
        if obj.confidence is not None:
            title += f" {obj.confidence:.0%}"
        if selected:
            cm = status.target_distance_cm
            title += f" | {cm:.1f} cm" if cm is not None else " | range unknown"
        cv2.rectangle(frame, (box.x1, box.y1), (box.x2 - 1, box.y2 - 1), color, 2)
        cv2.circle(frame, ((box.x1 + box.x2) // 2, (box.y1 + box.y2) // 2), 4, color, -1)
        _text(frame, title, (box.x1, max(16, box.y1 - 6)), color)

    command = status.motion
    motion = "none" if command is None else f"{command.kind.value}  v={command.linear:+.2f}  w={command.angular:+.2f}"
    if status.action is not None:
        motion += f"  |  {status.action.value}"
    title = status.state.value
    if status.target_id is not None:
        title += f"  |  target #{status.target_id}" + ("" if status.target_visible else " missing")
    if status.result is not None:
        title += f"  |  {status.result.value}"
    if status.beam_cm is not None:
        title += f"  |  beam {status.beam_cm:.1f} cm"
    cv2.rectangle(frame, (0, 0), (scene.width, 56 if status.reason else 40), (20, 20, 20), -1)
    _text(frame, title, (8, 16))
    _text(frame, motion, (8, 33))
    if status.reason:
        _text(frame, status.reason, (8, 50), (100, 180, 255))
    return frame


def _text(frame: Frame, value: str, origin: tuple[int, int], color: tuple[int, int, int] = (240, 240, 240)) -> None:
    cv2.putText(frame, value, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(frame, value, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)


class DebugWindow:
    title = "Robot pickup"

    def show(self, snapshot: VisionSnapshot, status: ControlStatus, beam: BeamSettings) -> bool:
        cv2.imshow(self.title, draw_overlay(snapshot, status, beam))
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
