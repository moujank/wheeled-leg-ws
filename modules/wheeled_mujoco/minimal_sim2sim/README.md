# Minimal Sim2Sim

This directory provides a minimal MuJoCo 3.4.0 `sim2sim` path used to validate the lowest-risk deployment loop.

It currently covers only the `wheeled-base-locked` task:

- Robot model: `modules/wheeled_mujoco/asset/mjcf/wheeled_leg.xml`
- Policy directory: `modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported`
- Default policy file: `policy.pt`

Design goals:

- Validate the loop `policy load -> observation build -> action inference -> MuJoCo actuator control`
- Avoid reusing the old humanoid-style `sim2sim.py` layout
- Avoid depending on `mujoco_viewer`
- Stay aligned with the action and observation definitions in `base_locked_locomotion_env_cfg.py`

## Running

Make sure PyTorch is available in the `mujoco` environment so the default `policy.pt` can be loaded:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate mujoco
pip install torch --index-url https://download.pytorch.org/whl/cpu
```

With no extra arguments, the script defaults to:

- `wheeled_leg.xml`
- `policy.pt`
- `torch` backend
- `isaac_play` mode
- MuJoCo official viewer enabled
- `0.5x` playback speed for visualization without changing `sim_dt`
- startup/reset tether at `z=0.32`
- `1s tether -> 1s release -> 2s stand -> policy`
- deterministic zero-velocity startup reset on first launch
- Isaac Play reset semantics for later resets
- default forward command `cmd_vx=0.5`
- leg action scale `0.5`
- wheel action scale `10.0`
- continuous execution until you stop it manually

Default launch:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate mujoco
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py
```

Useful examples:

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --stand-strategy policy_zero
```

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --mode experimental \
  --startup-tether 5 \
  --tether-release 1.5 \
  --cmd-vx 0.1
```

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --cmd-vx 0.5
```

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --playback-speed 0.25
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --playback-speed 1.0
```

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --log-path /tmp/wheeled_mujoco_diag.jsonl
```

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --mode experimental \
  --startup-tether 1.0 \
  --tether-release 0.2 \
  --cmd-vx 0.1 \
  --verbose-obs \
  --log-path /tmp/wheeled_handoff_diag.jsonl
```

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --wheel-action-scale 6.0
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --wheel-action-scale 10.0
```

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --no-viewer
```

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --no-real-time
```

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --cmd-vx 0.0
```

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate mujoco
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py --duration 5
```

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate mujoco
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --viewer \
  --real-time \
  --mode isaac_play \
  --cmd-vx 0.5 \
  --duration 30
```

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --policy modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported/policy.pt
```

```bash
python3 modules/wheeled_mujoco/minimal_sim2sim/run_base_locked.py \
  --backend onnx \
  --policy modules/wheeled_train/wheeled_leg/logs/rsl_rl/wheeled_base_locked/best/exported/policy.onnx
```

## Current Assumptions

- Observation dimension is `27`
- Action dimension is `6`
- Action order is `HipL HipR KneeL KneeR HubL HubR`
- Leg actions are position-offset control with scale `0.5`
- Wheel actions are velocity control with scale `10.0`
- Default startup flow is `1s tether -> 1s release -> 2s stand -> policy`
- Command observation is `[cmd_vx, cmd_vy, cmd_wz]`

## Notes

This is not the final deployment script. It is a minimal closed-loop validation tool. A fuller simulation-alignment workflow will still need:

- viewer or recording support
- reset / fail-safe handling
- strict Isaac/MuJoCo sensor alignment
- command sampling, noise, and randomization replication
- richer logging and metrics
