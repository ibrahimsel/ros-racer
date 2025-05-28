FROM ros:humble-ros-base

RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-pip ros-humble-ackermann-msgs && \
    pip3 install --no-cache-dir redis && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . /app
CMD ["python3", "speed_publisher.py"]
