FROM ros:humble-ros-base

RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-pip ros-humble-ackermann-msgs && \
    pip3 install --no-cache-dir redis==5.0.4 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY . /app

CMD ["python3", "ros_to_redis.py"]
