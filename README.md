# Gen AI Chatbot & Agent for Robotics

This project focuses on the development of a Gen AI Chatbot and Agent for Robotics, leveraging Retrieval-Augmented Generation (RAG) and Large Language Models (LLMs). The AI Agent is integrated with the ROS2 computational stack to enhance robotic automation and user interaction.

## Features
- **AI-Powered Chatbot**: Uses RAG and LLMs for intelligent query handling.
- **ROS2 Integration**: Interfaces with the ROS2 framework for robotic operations.
- **Cloud Deployment**: Supports deployment on Google Cloud Platform (GCP).
- **Data Preprocessing**: Includes utilities for preparing datasets for AI training.
- **Testing**: Comprehensive test scripts for validating AI Agent functionality.

## Technologies
- **Programming Languages**: Python, Shell Scripting
- **Frameworks**: PyTorch, Transformers, ROS2
- **Tools**: Google Cloud Platform, Colcon, rclpy
- **Libraries**: pandas, scikit-learn

## Project Structure
```
d:\ROS
├── ai_agent
│   └── rag_llm_agent.py       # AI Agent implementation
├── data
│   └── preprocess_data.py     # Data preprocessing script
├── deployment
│   └── deploy_model.sh        # Deployment script for GCP and ROS2
├── ros2_nodes
│   └── ai_agent_node.py       # ROS2 node for AI Agent
├── tests
│   └── test_ai_agent.py       # Test script for AI Agent
└── README.md                  # Project documentation
```

## Setup Instructions

### Prerequisites
1. Python 3.8+ installed.
2. ROS2 (e.g., Foxy) installed and configured.
3. Google Cloud SDK installed for cloud deployment.

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/surajmiishra/Gen-AI-Chatbot-.git
   cd Gen-AI-Chatbot-
   ```
2. Install Python dependencies:
   ```bash
   pip install torch transformers pandas scikit-learn
   ```
3. Set up ROS2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## Usage

### Running Locally
1. Start the ROS2 node:
   ```bash
   python3 d:/ROS/ros2_nodes/ai_agent_node.py
   ```
2. Test the AI Agent:
   ```bash
   python3 d:/ROS/tests/test_ai_agent.py
   ```

### Deploying to GCP
1. Make the deployment script executable:
   ```bash
   chmod +x d:/ROS/deployment/deploy_model.sh
   ```
2. Run the deployment script:
   ```bash
   bash d:/ROS/deployment/deploy_model.sh
   ```

## Testing
Run the test script to validate the AI Agent:
```bash
python3 d:/ROS/tests/test_ai_agent.py
```

## Contributing
Contributions are welcome! Please fork the repository and submit a pull request.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

## Contact
For questions or support, please contact [Suraj Mishra](https://github.com/surajmiishra).
