import zmq
import time

def main():
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    socket.connect("tcp://127.0.0.1:5555")
    
    print("Listening on tcp://127.0.0.1:5555 for frames...")
    
    msg_count = 0
    start_time = time.time()
    last_print = start_time
    
    while True:
        try:
            if socket.poll(1000):
                msg = socket.recv()
                msg_count += 1
                
                now = time.time()
                if now - last_print >= 1.0:
                    fps = msg_count / (now - last_print)
                    print(f"Received {msg_count} frames in last second (FPS: {fps:.1f}, size: {len(msg)} bytes)")
                    msg_count = 0
                    last_print = now
            else:
                print(f"No frames received for 1 second! (Timeout)")
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    main()
