FROM python:3.11-slim

WORKDIR /app

# deps (keep it tiny – no node_modules-sized sins here)
RUN pip install --no-cache-dir \
    fastapi \
    "uvicorn[standard]" \
    python-socketio[asyncio_client] \
    redis==5.0.4

COPY . /app

# gun it
CMD ["uvicorn", "server:app", "--host", "0.0.0.0", "--port", "8000"]
