# KURSAD40 Architecture Changelog

## BUG-001
**Description:**  
UISnapshot constructor mismatch after Phase 6 refactor.

**Root Cause:**  
The `UISnapshot` dataclass was extended with the required field `frame_id`, but the constructor call in `main.py` was not updated.

**Resolution:**  
Added the missing `frame_id` argument during `UISnapshot` construction.

**Impact:**  
Mission startup crash resolved.  
No architectural changes.

**Status:**  
CLOSED
