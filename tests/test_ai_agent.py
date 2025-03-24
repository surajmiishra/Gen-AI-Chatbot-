from ai_agent.rag_llm_agent import RAGLLMAgent

def test_agent():
    agent = RAGLLMAgent()
    query = "What is the robot's battery level?"
    context = "Battery level is at 85%."
    response = agent.generate_response(query, context)
    assert "85%" in response, "Test failed: Response does not contain expected information."

if __name__ == "__main__":
    test_agent()
    print("All tests passed!")
