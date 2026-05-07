#!/bin/bash
# Script to activate virtual environment and install dependencies

VENV_PATH="./ortho_env/bin/activate"
REQ_FILE="./requirements.txt"

echo "📦 [1/2] Checking virtual environment..."

if [ -f "$VENV_PATH" ]; then
    echo "✅ Virtual environment found. Activating..."
    source "$VENV_PATH"
else
    echo "❌ Virtual environment not found at $VENV_PATH"
    echo "➡️ Creating new virtual environment ortho_env..."
    python3 -m venv ortho_env
    source ortho_env/bin/activate
fi

echo ""
echo "📄 [2/2] Installing requirements..."
if [ -f "$REQ_FILE" ]; then
    pip install --upgrade pip
    pip install -r "$REQ_FILE"
    echo "✅ Requirements installed."
else
    echo "⚠️ File $REQ_FILE not found. Skipping installation."
fi

echo ""
echo "🎉 Setup complete. Virtual environment is now active!"
