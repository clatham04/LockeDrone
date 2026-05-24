import shutil, os
from huggingface_hub import hf_hub_download

# Delete the cached copy
cache_dir = os.path.expanduser(
    r"~\.cache\huggingface\hub\models--pitangent-ds--YOLOv8-human-detection-thermal"
)
shutil.rmtree(cache_dir, ignore_errors=True)

# Download fresh into the current directory
model_path = hf_hub_download(
    repo_id="pitangent-ds/YOLOv8-human-detection-thermal",
    filename="model.pt",
    local_dir="."
)
print(model_path)