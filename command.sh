#!/bin/bash

SESSION_ID="$1"

# default to session_1 if no argument provided
if [ -z "$SESSION_ID" ]; then
  SESSION_ID="session_1"
fi

echo "Starting live stitching session: $SESSION_ID"

echo "[1/4] Starting stitching service..."
python service.py  & SERVICE_PID=$!
echo "Service started with PID: $SERVICE_PID"

sleep 3 

# Check if service is running
if ! kill -0 $SERVICE_PID 2>/dev/null; then
    echo "Error: Service failed to start!"
    exit 1
fi

# Enable folder monitoring
echo "[2/4] Enabling folder monitoring..."
RESPONSE=$(curl -s -X POST "http://127.0.0.1:8001/session/$SESSION_ID/toggle-monitoring?enable=true")
echo "      $RESPONSE"

# Enable auto-stitch
echo "[3/4] Enabling auto-stitch (threshold: 5 images)..."
RESPONSE=$(curl -s -X POST "http://127.0.0.1:8001/session/$SESSION_ID/toggle-auto-stitch?enable=true")
echo "      $RESPONSE"

echo "Monitoring and auto-stitch enabled for session: $SESSION_ID"

# check sessions status
echo "[4/4] Session status:"
curl -s "http://127.0.0.1:8001/session/$SESSION_ID/status" | python -m json.tool 2>/dev/null || echo "Session not found, it will be created when images arrive"

echo ""
echo "======================================"
echo "Ready to receive images on port 5001"
echo "Press Ctrl+C to stop"
echo "======================================"
echo ""

# wait $SERVICE_PID  # Wait for service to , but if wan to run receiver socket as well, cannot wait here

# Cleanup function
cleanup() {
    echo ""
    echo "======================================"
    echo "Shutting down..."
    echo "======================================"
    kill $SERVICE_PID 2>/dev/null
    wait $SERVICE_PID 2>/dev/null
    echo "Live Stitching service stopped. Goodbye!"
    exit 0
}

# Trap Ctrl+C
trap cleanup SIGINT SIGTERM

# # Start receiver
python receiver_socket.py --session-id "$SESSION_ID" --port 5001


