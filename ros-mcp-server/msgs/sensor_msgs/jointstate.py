from typing import List, Any, Protocol

class Publisher(Protocol):
    def send(self, message: dict) -> None:
        ...

class JointState:
    def __init__(self, publisher: Publisher, topic: str = "/joint_states"):
        self.publisher = publisher
        self.topic = topic

    def publish(self, name: List[str], position: List[float], velocity: List[float], effort: List[float]):
        msg = {
            "op": "publish",
            "topic": self.topic,
            "msg": {
                "header": {},
                "name": name,
                "position": position,
                "velocity": velocity,
                "effort": effort
            }
        }
        self.publisher.send(msg)
        return msg

    def subscribe(self, timeout=2.0):
        subscribe_msg = {
            "op": "subscribe",
            "topic": self.topic
        }
        self.publisher.send(subscribe_msg)
        raw = self.publisher.receive_binary()
        if not raw:
            return None
        import json
        try:
            msg = json.loads(raw)
            if "msg" in msg:
                return json.dumps(msg["msg"], indent=2, ensure_ascii=False)
            return json.dumps(msg, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"[JointState] Failed to parse: {e}")
            return None