from typing import Tuple, List
from core.interfaces.i_payload_actuator import IPayloadActuator

class MockPayloadActuator(IPayloadActuator):
    def __init__(self):
        self.calls: List[Tuple[str, dict]] = []
        
    async def release_payload_at_mavi_altigen(self) -> bool:
        self.calls.append(('release_payload_at_mavi_altigen', {}))
        return True
        
    async def release_payload_at_kirmizi_ucgen(self) -> bool:
        self.calls.append(('release_payload_at_kirmizi_ucgen', {}))
        return True
        
    async def activate_pickup_mechanism(self) -> bool:
        self.calls.append(('activate_pickup_mechanism', {}))
        return True
        
    async def activate_drop_mechanism(self) -> bool:
        self.calls.append(('activate_drop_mechanism', {}))
        return True
