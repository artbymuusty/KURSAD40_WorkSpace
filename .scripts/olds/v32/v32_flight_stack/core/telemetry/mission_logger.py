import logging
import os
from datetime import datetime

def get_mission_logger(name: str) -> logging.Logger:
    """Proje genelinde kullanılacak standart logger oluşturur."""
    logger = logging.getLogger(name)
    
    if not logger.handlers:
        logger.setLevel(logging.INFO)
        
        # Konsol Handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        
        # Dosya Handler
        log_dir = "logs"
        os.makedirs(log_dir, exist_ok=True)
        time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        file_handler = logging.FileHandler(os.path.join(log_dir, f"mission_{time_str}.log"))
        file_handler.setLevel(logging.INFO)
        
        formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(name)s: %(message)s")
        console_handler.setFormatter(formatter)
        file_handler.setFormatter(formatter)
        
        logger.addHandler(console_handler)
        logger.addHandler(file_handler)
        
    return logger

def configure_all_loggers() -> None:
    """BUG FIX (runtime investigation, 2026-08-13): this module was never
    imported anywhere in the actual entrypoints (main_gz.py/main_real.py/
    main_dual.py) -- the get_mission_logger() calls that used to sit at
    this module's top level only ran if something imported this file, and
    nothing did. Every `logger = logging.getLogger(__name__)` +
    logger.info()/.warning() call throughout the entire codebase therefore
    had NO HANDLER anywhere in its propagation chain: only Python's bare
    `lastResort` stderr fallback (WARNING+ only, unformatted) could ever
    surface anything, meaning essentially every diagnostic log in this
    project (mission phase transitions, centering/payload progress, vision
    heartbeats) was silently going nowhere. Call this explicitly, once, at
    the very start of each entrypoint's main() -- before any core.* module
    does anything -- to actually attach console+file handlers."""
    for namespace in ("core", "real_system", "gz_system", "dual_system", "mavsdk_common"):
        get_mission_logger(namespace)
