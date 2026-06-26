"""VLM client fork for Anthropic Claude API."""

import os
import re
import time
from typing import List, Optional, Tuple, Type, TypeVar, Union

import anthropic
from omegaconf import DictConfig

from pragmabot.conversation_builder import ConversationBuilder

T = TypeVar("T")


def _to_anthropic(messages):
    """Convert ConversationBuilder messages (OpenAI format) to Anthropic format.

    Returns (system_prompt, anthropic_messages).
    """
    system_parts = []
    anthropic_messages = []

    for msg in messages:
        role = msg["role"]
        content = msg["content"]

        if role == "system":
            system_parts.append(content)
            continue

        if isinstance(content, str):
            anthropic_messages.append({"role": role, "content": content})
        else:
            new_content = []
            for block in content:
                if block["type"] == "text":
                    new_content.append({"type": "text", "text": block["text"]})
                elif block["type"] == "image_url":
                    url = block["image_url"]["url"]
                    m = re.match(r"data:([^;]+);base64,(.+)", url, re.DOTALL)
                    if m:
                        media_type, data = m.group(1), m.group(2)
                    else:
                        media_type, data = "image/jpeg", url
                    new_content.append({
                        "type": "image",
                        "source": {
                            "type": "base64",
                            "media_type": media_type,
                            "data": data,
                        },
                    })
            anthropic_messages.append({"role": role, "content": new_content})

    system_prompt = "\n\n".join(system_parts) if system_parts else ""
    return system_prompt, anthropic_messages


class ClaudeVLMClient:
    """VLM client wrapping Anthropic Claude API with sentence-transformers embeddings."""

    def __init__(self, config: DictConfig) -> None:
        self.config = config
        self.client = anthropic.Anthropic(api_key=os.getenv("ANTHROPIC_API_KEY"))
        self._embedder = None

    def _get_embedder(self):
        if self._embedder is None:
            from sentence_transformers import SentenceTransformer
            self._embedder = SentenceTransformer("all-MiniLM-L6-v2")
        return self._embedder

    def query_structured(
        self,
        builder: ConversationBuilder,
        response_format: Type[T],
    ) -> Tuple[T, float, int]:
        system_prompt, anthropic_messages = _to_anthropic(builder.messages)

        start_time = time.time()
        response = self.client.messages.parse(
            model=self.config.vlm_model,
            system=system_prompt,
            messages=anthropic_messages,
            output_format=response_format,
            thinking={"type": "adaptive"},
            max_tokens=4096,
        )
        elapsed = time.time() - start_time

        builder.log_assistant_message(
            f"VLM reasoning time: {elapsed:.2f} s. # prompt tokens: {response.usage.input_tokens}."
        )

        if response.parsed_output is None:
            raise ValueError("Claude did not return a parseable structured response.")

        return response.parsed_output, elapsed, response.usage.input_tokens

    def get_text_embedding(
        self,
        text: Union[str, List[str]],
        builder: Optional[ConversationBuilder] = None,
    ) -> Tuple[List[List[float]], float, int]:
        texts = [text] if isinstance(text, str) else list(text)

        embedder = self._get_embedder()
        start_time = time.time()
        vectors = embedder.encode(texts, convert_to_numpy=True)
        elapsed = time.time() - start_time

        token_count = sum(len(t.split()) for t in texts)
        if builder:
            builder.log_assistant_message(
                f"Embedding time: {elapsed:.2f} s (local sentence-transformers). "
                f"# approx tokens: {token_count}."
            )

        return [v.tolist() for v in vectors], elapsed, token_count
