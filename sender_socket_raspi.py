import socket
import struct
import time
import cv2

# IP laptop tujuan (bisa local, real ip, atau tailscale)
SERVER_IP = "100.94.80.118"
SERVER_PORT = 5001  # harus sama dengan receiver

def main():
    cam = cv2.VideoCapture(0)  # buka kamera default
    if not cam.isOpened():
        print("[CLIENT] Error: Could not open camera.")
        return
    
    # BUAT SOCKET 
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    print(f"[CLIENT] Connecting to {SERVER_IP}:{SERVER_PORT} ...")
    sock.connect((SERVER_IP, SERVER_PORT))
    
    if sock:
        print("[CLIENT] Connected to server!")
    
    try:
        frame_counter = 0
        while True:
            ret, frame = cam.read()
            if not ret:
                print("[CLIENT] Error: Could not read frame from camera.")
                break
            
            # Encode frame as JPEG
            ret, img_encoded = cv2.imencode('.jpg', frame)
            if not ret:
                print("[CLIENT] Error: Could not encode frame.")
                continue
            
            img_data = img_encoded.tobytes()
            length = len(img_data)
            
            # Kirim panjang data (4 bytes, big-endian)
            sock.sendall(struct.pack("!I", length))
            
            # Kirim data gambar
            try:
                sock.sendall(img_data)
            except Exception as e:
                print(f"[CLIENT] Error sending frame data: {e}")
                break
            
            frame_counter += 1
            print(f"[CLIENT] Sent frame {frame_counter}, size: {length} bytes")
            
            time.sleep(0.3)  # delay kecil untuk menghindari overload
    
    finally:
        print("[CLIENT] Closing connection.")
        sock.close()
        cam.release()

if __name__ == "__main__":
    main()
    
