import torch
from transformers import AutoModelForSeq2SeqLM, AutoTokenizer

class RAGLLMAgent:
    def __init__(self, model_name="facebook/bart-large"):
        self.tokenizer = AutoTokenizer.from_pretrained(model_name)
        self.model = AutoModelForSeq2SeqLM.from_pretrained(model_name)

    def generate_response(self, query, context):
        # Preprocess input
        inputs = self.tokenizer(query + context, return_tensors="pt", truncation=True)
        # Generate response
        outputs = self.model.generate(**inputs)
        return self.tokenizer.decode(outputs[0], skip_special_tokens=True)

# Example usage
if __name__ == "__main__":
    agent = RAGLLMAgent()
    query = "What is the status of the robot?"
    context = "The robot is currently in standby mode."
    print(agent.generate_response(query, context))
