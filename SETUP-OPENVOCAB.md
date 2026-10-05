# Open-vocabulary setup

Open-vocabulary search uses local YOLOE proposals and an image-capable language
model to recognize the requested target and verify its bounding box. Detection
is on demand. Existing AI Vision and marker detection are untouched.


## Installation

Activate the same Python environment used to run Celeste, install the normal
platform requirements, then run from this repository:

```powershell
python -m pip install -r requirements-openvocab.txt
```

Obtain both model assets from the Ultralytics model distribution, or copy the
existing working assets from the development installation:

```text
Celeste/
  models/
    yoloe-26l-seg.pt
    mobileclip2_b.ts
  vex-aim-tools/
    Celeste.fsm
```

Both files are required: the first contains the detector weights; the second is
the text encoder for target phrases. The loader requires these local paths; it
does not download missing models. Missing assets or dependencies disable openvocab with a startup diagnostic.
Inference currently defaults to CPU.

## Provider and credentials

For OpenRouter, set `OPENROUTER_API_KEY` before launching Celeste, or place the
key alone in `vex-aim-tools/.openrouter-key`. The environment variable takes
precedence over the file. The key file is ignored by Git. 


`OPENVOCAB_MODEL` in `Celeste.fsm` selects the recognition and verification model;
the current development setting is `gpt-5.6-luna`.


## Build and validation

After editing `Celeste.fsm`, regenerate its Python output:

```powershell
python genfsm Celeste.fsm
```
Dependencies include Ultralytics, PyTorch, torchvision, and the pinned Ultralytics
CLIP tokenizer. OpenRouter uses the existing OpenAI SDK; no separate OpenRouter
library is needed. The CLIP source archive installation needs network access.

Add an optional download step for both model assets into `$ResourcesDir/models`.
The application does not currently provide an automatic model downloader. Verify
source URLs and file integrity before adding that step to the distributed installer;


## Run openvocab

Use the same configured environment and launch command as ordinary Celeste:

```powershell
simple_cli Celeste
```
Configure the OpenRouter key locally before launching. The startup provider log
should say `OpenRouter`; `Open-vocabulary detection unavailable` means the local
models or dependencies failed to load. Allow model warmup to finish.

# How openvocab works
If you ask the robot to "go to ( or find)" an object that isn’t already mapped, it first asks GPT to provide 3 different descriptions of the object, including its distinguishing visual features. GPT also checks the camera image to see whether the object is present and can provide additional descriptions.
These descriptions are sent to YOLOE for detection. YOLOE generates candidate bounding boxes with confidence scores. Some might not be the correct object, so we crop around the candidates and send the numbered crops, along with the full image, to GPT for verification. GPT chooses the bounding box that it thinks shows the correct object and checks whether its bottom edge corresponds to the object’s visible base on the tabletop or floor. If GPT is uncertain, we send the full image and a crop of the selected candidate for another verification. The images and responses are stored in snapshots/openvocab for diagnostics.
The ground-projection calculation uses the bottom-center of the bounding box to estimate the object’s ground-contact position. If the verified object is far away, Celeste may move closer and check again before mapping it.
Once an object is placed in the map, asking Celeste to go to it again skips the initial detection. After arrival, it checks whether the object is still there. If it reports the object absent, Celeste announces that it cannot see it where expected, removes the stale map entry, and searches again for its new location. If the check is uncertain, it stops and reports that it could not confirm the object.

#Testing it
Say "Find the blue coffee mug" or "Go to the blue coffee mug". Celeste generates
`#find` internally for an unmapped target. Recognition and verification require
internet access; YOLOE runs locally. There is no separate openvocab executable.
Check the green labeled camera overlay and `openvocab map:` log messages. Test a
mapped object again after moving/removing it to exercise arrival recovery.


