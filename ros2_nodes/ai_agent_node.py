import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ai_agent.rag_llm_agent import RAGLLMAgent

class AIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')
        self.agent = RAGLLMAgent()
        self.subscription = self.create_subscription(
            String,
            'robot_query',
            self.handle_query,
            10
        )
        self.publisher = self.create_publisher(String, 'robot_response', 10)

    def handle_query(self, msg):
        query = msg.data
        context = "Robot operational data here."  # Replace with actual context
        response = self.agent.generate_response(query, context)
        self.publisher.publish(String(data=response))

def main(args=None):
    rclpy.init(args=args)
    node = AIAgentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
