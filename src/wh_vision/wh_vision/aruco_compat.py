"""aruco_compat.py – Lớp tương thích OpenCV 4.5.4 và ≥4.7 cho ArUco.

Dùng chung cho wh_vision (detection) và gen_tags.py (sinh texture).
"""

import cv2


class ArucoCompat:
    """Wrapper thống nhất API cũ và mới của cv2.aruco."""

    def __init__(self, dict_name: str = "DICT_4X4_50"):
        a = cv2.aruco
        self.dict = a.getPredefinedDictionary(getattr(a, dict_name))
        self.new_api = hasattr(a, "ArucoDetector")
        if self.new_api:
            p = a.DetectorParameters()
            p.cornerRefinementMethod = a.CORNER_REFINE_SUBPIX
            self.detector = a.ArucoDetector(self.dict, p)
        else:
            self.params = a.DetectorParameters_create()
            self.params.cornerRefinementMethod = a.CORNER_REFINE_SUBPIX

    def detect(self, gray):
        """Phát hiện ArUco marker trong ảnh grayscale."""
        if self.new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self.dict, parameters=self.params
            )
        return corners, ids

    def make_marker(self, marker_id: int, px: int):
        """Sinh ảnh marker kích thước px×px."""
        if hasattr(cv2.aruco, "generateImageMarker"):
            return cv2.aruco.generateImageMarker(self.dict, marker_id, px)
        return cv2.aruco.drawMarker(self.dict, marker_id, px)
