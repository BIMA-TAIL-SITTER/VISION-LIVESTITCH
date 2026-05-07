#!/bin/bash

SESSION_ID="${1:-session_1}"

echo "Starting live stitching in separate terminals..."

# Start service in new terminal cihuy
echo "Opening Service terminal..."
gnome-terminal --title="Service - $SESSION_ID" -- bash -c "
    echo '========================================';
    echo 'Live Stitching Service';
    echo 'Session: $SESSION_ID';
    echo '========================================';
    echo '';
    python service.py;
    echo '';
    echo 'Service stopped. Press Enter to close...';
    read;
"

# Wait for service to start
echo "Waiting for service to initialize..."
sleep 3

# Check if service is running
echo "Checking service health..."
if curl -s http://127.0.0.1:8001/ > /dev/null 2>&1; then
    echo "✓ Service is running"
else
    echo "✗ Service failed to start!"
    exit 1
fi

# Start receiver in new terminal
gnome-terminal --title="Receiver - $SESSION_ID" -- bash -c "
    echo '========================================';
    echo 'Image Receiver';
    echo 'Session: $SESSION_ID';
    echo '========================================';
    
    # Enable monitoring
    echo '[1/3] Enabling folder monitoring...';
    RESPONSE=\$(curl -s -X POST 'http://127.0.0.1:8001/session/$SESSION_ID/toggle-monitoring?enable=true');
    echo \"      \$RESPONSE\";
    echo '';
    
    # Enable auto-stitch
    echo '[2/3] Enabling auto-stitch (threshold: 5 images)...';
    RESPONSE=\$(curl -s -X POST 'http://127.0.0.1:8001/session/$SESSION_ID/toggle-auto-stitch?enable=true');
    echo \"      \$RESPONSE\";
    echo '';
    
    # Check session status
    echo '[3/3] Session status:';
    curl -s 'http://127.0.0.1:8001/session/$SESSION_ID/status' | python -m json.tool 2>/dev/null || echo 'Session will be created when images arrive';
    echo '';
    
    echo '========================================';
    echo 'Ready to receive images on port 5001';
    echo 'Waiting for drone connection...';
    echo '========================================';
    echo '';
    
    python receiver_socket.py --session-id '$SESSION_ID' --port 5001;
    exec bash
"

echo "Two terminals opened:"
echo "  1. Service (shows stitching progress)"
echo "  2. Receiver (shows incoming images)"
echo ""
echo "Press Ctrl+C in each terminal to stop"