"""VLM client fork for Google Gemini API."""

import os
import re
import time
from typing import List, Optional, Tuple, Type, TypeVar, Union

import google.generativeai as genai
from omegaconf import DictConfig

from pragmabot.conversation_builder import ConversationBuilder

T = TypeVar("T")


def _to_gemini(messages):
    """Convert ConversationBuilder messages (OpenAI format) to Gemini format.

    Returns (system_instruction, gemini_contents).
    """
    system_parts = []
    gemini_contents = []

    for msg in messages:
        role = msg["role"]
        content = msg["content"]

        if role == "system":
            system_parts.append(content)
            continue

        gemini_role = "user" if role == "user" else "model"

        if isinstance(content, str):
            gemini_contents.append({"role": gemini_role, "parts": [{"text": content}]})
        else:
            parts = []
            for block in content:
                if block["type"] == "text":
                    parts.append({"text": block["text"]})
                elif block["type"] == "image_url":
                    url = block["image_url"]["url"]
                    m = re.match(r"data:([^;]+);base64,(.+)", url, re.DOTALL)
                    if m:
                        mime_type, data = m.group(1), m.group(2)
                        parts.append({"inline_data": {"mime_type": mime_type, "data": data}})
            gemini_contents.append({"role": gemini_role, "parts": parts})

    system_instruction = "\n\n".join(system_parts) if system_parts else None
    return system_instruction, gemini_contents


class GeminiVLMClient:
    """VLM client wrapping Google Gemini API with native Gemini embeddings."""

    def __init__(self, config: DictConfig) -> None:
        self.config = config
        genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))

    def query_structured(
        self,
        builder: ConversationBuilder,
        response_format: Type[T],
    ) -> Tuple[T, float, int]:
        system_instruction, gemini_contents = _to_gemini(builder.messages)

        model = genai.GenerativeModel(
            model_name=self.config.vlm_model,
            system_instruction=system_instruction,
        )
        generation_config = genai.GenerationConfig(
            response_mime_type="application/json",
            response_schema=response_format,
        )

        start_time = time.time()
        response = model.generate_content(
            contents=gemini_contents,
            generation_config=generation_config,
        )
        elapsed = time.time() - start_time

        prompt_tokens = (
            response.usage_metadata.prompt_token_count
            if response.usage_metadata
            else 0
        )
        builder.log_assistant_message(
            f"VLM reasoning time: {elapsed:.2f} s. # prompt tokens: {prompt_tokens}."
        )

        try:
            parsed = response_format.model_validate_json(response.text)
        except Exception as e:
            raise ValueError(
                f"Gemini structured output parse failed: {e}. Raw: {response.text}"
            )

        return parsed, elapsed, prompt_tokens

    def get_text_embedding(
        self,
        text: Union[str, List[str]],
        builder: Optional[ConversationBuilder] = None,
    ) -> Tuple[List[List[float]], float, int]:
        texts = [text] if isinstance(text, str) else list(text)

        start_time = time.time()
        embeddings = []
        token_count = 0
        for t in texts:
            result = genai.embed_content(
                model="models/text-embedding-004",
                content=t,
            )
            embeddings.append(result["embedding"])
            token_count += len(t.split())
        elapsed = time.time() - start_time

        if builder:
            builder.log_assistant_message(
                f"Embedding query time: {elapsed:.2f} s. # approx tokens: {token_count}."
            )

        return embeddings, elapsed, token_count
