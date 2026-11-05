# config.py
"""
Centralized multi-UAV configuration, shared by receiver_socket.py (port/session
routing) and service.py (default session auto-setup). Tambah UAV baru di sini
sekali, dua-duanya otomatis kebawa.
"""

UAV_CONFIG_TCP = {
    "uav1": {
        "port": 5001,
        "session_id": "uav_1",
        "auto_stitch_threshold": 5,
        "auto_stitch_enabled": True,
        "folder_monitoring_enabled": True,
        "output_name": "finalResult.png",
    },
    "uav2": {
        "port": 5002,
        "session_id": "uav_2",
        "auto_stitch_threshold": 5,
        "auto_stitch_enabled": True,
        "folder_monitoring_enabled": True,
        "output_name": "finalResult.png",
    },
    # "uav3": {"port": 5003, "session_id":
}

UAV_CONFIG_UDP = {
    "uav1": {
        "port": 6001,
        "session_id": "uav_1",
        "auto_stitch_threshold": 5,
        "auto_stitch_enabled": True,
        "folder_monitoring_enabled": True,
        "output_name": "finalResult.png",
    },
    "uav2": {
        "port": 6002,
        "session_id": "uav_2",
        "auto_stitch_threshold": 5,
        "auto_stitch_enabled": True,
        "folder_monitoring_enabled": True,
        "output_name": "finalResult.png",
    },
}