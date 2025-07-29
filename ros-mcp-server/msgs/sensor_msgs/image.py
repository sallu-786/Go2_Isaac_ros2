import base64
import json
from typing import Optional
from pathlib import Path
from typing import Protocol
from datetime import datetime
import numpy as np
import cv2

class Subscriber(Protocol):
    def receive_binary(self) -> bytes:
        ...
    def send(self, message: dict) -> None:
        ...

class Image:
    def __init__(self, subscriber: Subscriber, topic: str = "/camera/image_raw"):
        self.subscriber = subscriber
        self.topic = topic

    def subscribe(self, save_path: Optional[str] = None) -> Optional[bytes]:
        try:
            subscribe_msg = {
                "op": "subscribe",
                "topic": self.topic,
                "type": "sensor_msgs/Image"
            }
            self.subscriber.send(subscribe_msg)

            max_tries = 3
            tries = 0

            while tries < max_tries:
                raw = self.subscriber.receive_binary()
                if not raw:
                    print("[Image] No data received from subscriber")
                    tries += 1
                    continue

                if isinstance(raw, bytes):
                    raw = raw.decode("utf-8")

                data = json.loads(raw)

                if data.get("op") == "publish" and data.get("topic") == self.topic:
                    msg = data.get("msg")
                    if not msg:
                        tries += 1
                        continue

                    height = msg["height"]
                    width = msg["width"]
                    encoding = msg["encoding"]
                    data_b64 = msg["data"]

                    image_bytes = base64.b64decode(data_b64)
                    img_np = np.frombuffer(image_bytes, dtype=np.uint8)

                    if encoding == "rgb8":
                        img_np = img_np.reshape((height, width, 3))
                        img_cv = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
                    elif encoding == "bgr8":
                        img_cv = img_np.reshape((height, width, 3))
                    elif encoding == "mono8":
                        img_cv = img_np.reshape((height, width))
                    else:
                        print(f"[Image] Unsupported encoding: {encoding}")
                        return None

                    if save_path is None:
                        downloads_dir = Path(__file__).resolve().parents[2] / "screenshots"
                        downloads_dir.mkdir(parents=True, exist_ok=True)
                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                        save_path = downloads_dir / f"{timestamp}.png"

                    Path(save_path).parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(save_path), img_cv)
                    print(f"[Image] Saved to {save_path}")
                    return img_cv
                else:
                    tries += 1

            print("[Image] Max tries exceeded without receiving image data.")
            return None

        except Exception as e:
            print(f"[Image] Failed to receive or decode: {e}")
            return None

