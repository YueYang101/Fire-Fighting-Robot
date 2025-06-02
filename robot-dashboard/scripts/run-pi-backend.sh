#!/usr/bin/env bash
# run-pi-backend.sh — launches the FastAPI micro-backend inside its venv

# go into the backend folder
cd "$(dirname "$0")/../backend"

# hand over to uvicorn (exec replaces the shell)
exec uvicorn pi_backend:app --host 0.0.0.0 --port 5000
