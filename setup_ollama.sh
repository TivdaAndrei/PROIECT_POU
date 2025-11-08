#!/bin/bash
# Setup Ollama for natural conversations with Pou

echo "🤖 Setting up Ollama for Natural Conversations"
echo ""

# Check if ollama is installed
if ! command -v ollama &> /dev/null; then
    echo "📥 Installing Ollama..."
    curl -fsSL https://ollama.com/install.sh | sh
    echo ""
fi

echo "✅ Ollama installed!"
echo ""

# Start Ollama service
echo "🚀 Starting Ollama service..."
ollama serve &
OLLAMA_PID=$!
sleep 3

echo ""
echo "📦 Pulling Llama 3.2 model (small, fast, ~2GB)..."
echo "   This will take a few minutes on first run..."
ollama pull llama3.2:latest

echo ""
echo "✅ Setup complete!"
echo ""
echo "📋 Ollama is now running with model: llama3.2:latest"
echo ""
echo "💡 Tips:"
echo "   - Ollama runs in background automatically"
echo "   - Uses about 2-4GB RAM"
echo "   - Responses are generated locally (private)"
echo "   - To use a different model: ollama pull mistral"
echo ""
echo "🎤 Now Pou can have natural conversations with you!"
echo "   Say 'Hi friend' and ask anything!"
echo ""
echo "To stop Ollama: pkill ollama"
