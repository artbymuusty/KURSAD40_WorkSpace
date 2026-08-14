from mavsdk_common.mavsdk_backend_base import MavsdkBackendBase

class GzFlightBackend(MavsdkBackendBase):
    """
    Gazebo SITL uçuş sistemi adaptörü.
    connection_string config'den okunur (örn. "udp://:14540").
    MavsdkBackendBase ile uçuş kontrol mantığı tamamen aynıdır.
    """
    pass
