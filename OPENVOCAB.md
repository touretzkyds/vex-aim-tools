# Open-vocabulary setup

Open-vocabulary search uses local YOLOE proposals and an image-capable language
model to recognize the requested target and verify its bounding box. Detection
is on demand. Existing AI Vision and marker detection do not require these assets.

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
the current development setting is `gpt-5.6-luna`. Configure an image-capable model
available to the chosen account/provider. The OpenRouter adapter adds `openai/`
to an unqualified model name. Provider model availability must be checked when
setting up a new account. This feature does not establish OpenRouter compatibility
for the existing OpenAI document upload/vector-store APIs.

For a collaborator, share a dedicated credential privately and keep it local to
their installation. They do not need to replace the existing OpenAI key in code.

## Build and validation

After editing `Celeste.fsm`, regenerate its Python output:

```powershell
python genfsm Celeste.fsm
```
Go-to ends at the normal navigation destination without a close-contact approach.
For openvocab targets, a stationary recognition-only arrival check confirms
presence. An absent result announces the missing target, removes its stale entry from the map,
and retries search once. A successful search maps a fresh position. A failed search
leaves the old entry removed. An uncertain check reports failure instead of
claiming absence. The arrival check does not itself estimate a new position.

The detection state waits up to 120 seconds. This is not a total multi-view search
deadline and uses a 30-second HTTP timeout without SDK retries. Cancellation rejects late results and prevents further candidate calls; it does not forcibly terminate a running thread.

Diagnostics are saved under `snapshots/openvocab/` and are ignored by Git.


Dependencies include Ultralytics, PyTorch, torchvision, and the pinned Ultralytics
CLIP tokenizer. OpenRouter uses the existing OpenAI SDK; no separate OpenRouter
library is needed. The CLIP source archive installation needs network access.

Add an optional download step for both model assets into `$ResourcesDir/models`.
The application does not currently provide an automatic model downloader. Verify
source URLs and file integrity before adding that step to the distributed installer;

The launcher BAT does not need changing.

## Run openvocab

Use the same configured environment and launch command as ordinary Celeste:

```powershell
simple_cli Celeste
```
Configure the OpenRouter key locally before launching. The startup provider log
should say `OpenRouter`; `Open-vocabulary detection unavailable` means the local
models or dependencies failed to load. Allow model warmup to finish.

Say "Find the blue coffee mug" or "Go to the blue coffee mug". Celeste generates
`#find` internally for an unmapped target. Recognition and verification require
internet access; YOLOE runs locally. There is no separate openvocab executable.
Check the green labeled camera overlay and `openvocab map:` log messages. Test a
mapped object again after moving/removing it to exercise arrival recovery.


