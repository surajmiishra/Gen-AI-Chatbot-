#!/bin/bash

# Variables
MODEL_DIR="d:/ROS/models"
CLOUD_BUCKET="gs://oorb-ai-models"
ROS_WORKSPACE="~/ros2_ws"

# Authenticate with cloud
gcloud auth activate-service-account --key-file=service-account.json

# Upload model to cloud
gsutil cp -r $MODEL_DIR $CLOUD_BUCKET

# Set up ROS2 environment
source /opt/ros/foxy/setup.bash
mkdir -p $ROS_WORKSPACE/src
cd $ROS_WORKSPACE
colcon build

# Run AI Agent node
source install/setup.bash
ros2 run ros2_nodes ai_agent_node

echo "Model deployed to $CLOUD_BUCKET and ROS2 node started."
