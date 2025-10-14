from litellm import completion
from config import AZURE_API_BASE, AZURE_API_KEY, AZURE_API_VERSION
from server import (
    get_list_topics, get_topic_type, get_topic_message, get_topic_publishers, get_topic_subscribers)
class GenerateResponse:
    def __init__(self):
        self.MODELS = {"ChatGPT-4o": "azure/azure_openai_app_4o",
                       "Gemma3": "ollama/gemma3:latest",
                       }
        self.model = self.MODELS['ChatGPT-4o']         
        # self.api_base = os.getenv("LITELLM_API_BASE","http://localhost:11434")            #use line 11 and 12 for ollama based local model
        # self.api_key="local"
        self.api_base = AZURE_API_BASE 
        self.api_key = AZURE_API_KEY   
        self.api_version = AZURE_API_VERSION 
        self.system_message = "You are an assistant deployed on a Quadruped (Dog Robot)."
        self.user_message = "What's in this image? Tell me briefly. Also warn user of some warning signs or some dangerous situation."

        
    def llm_response(self,image_path=None):
        if image_path is not None:
            messages=[
                    {"role": "system", "content": f"{self.system_message}"},
                    {"role": "user", "content": [
                        {"type": "text", "text": f"{self.user_message}"},
                        {"type": "image_url", "image_url": {"url": image_path}}
                    ]}
                ]
        else:
            messages=[
                    {"role": "system", "content": f"{self.system_message}"},
                    {"role": "user", "content": [
                        {"type": "text", "text": f"{self.user_message}"},
                    ]}
                ]
        response = completion(
            model=self.model,
            messages=messages,
            max_tokens=150,
            temperature=0.7,
            stream=False,  # full response at once
            api_base=self.api_base,
            api_key=self.api_key
        )
        # Return the text content
        return response.choices[0].message.content.strip()