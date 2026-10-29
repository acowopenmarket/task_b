"""Shared file/config helpers. Local data is never implicitly uploaded."""
import json
from pathlib import Path

PROJECT_ROOT=Path(__file__).resolve().parents[1]
DEFAULT_CONFIG=PROJECT_ROOT/'configs/handeye.json'

def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))

def save(path,value):
    destination=Path(path); destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(json.dumps(value,indent=2,ensure_ascii=False),encoding='utf-8')
