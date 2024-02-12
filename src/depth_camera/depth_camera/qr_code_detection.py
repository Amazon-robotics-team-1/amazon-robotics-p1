# Module to detect QR codes

import cv2


class QRCodeDetector:
    def __init__(self):
        self.qr_code_detected = False

    def detect_qr_code(self):
        """
        Scans and detects QR codes.
        """
        window_name = "QR Code Detector"
        delay = 1

        qcd = cv2.QRCodeDetector()
        cap = cv2.VideoCapture(0)

        while True:
            ret, frame = cap.read()

            if ret:
                ret_qr, decoded_info, points, _ = qcd.detectAndDecodeMulti(frame)
                if ret_qr:
                    for s, p in zip(decoded_info, points):
                        if s:
                            self.qr_code_detected = True
                            print(points)
                            color = (0, 255, 0)
                        else:
                            self.qr_code_detected = False
                            color = (0, 0, 255)
                        frame = cv2.polylines(frame, [p.astype(int)], True, color, 8)
                cv2.imshow(window_name, frame)
            if cv2.waitKey(delay) & 0xFF == ord("q"):
                break
        cv2.destroyWindow(window_name)