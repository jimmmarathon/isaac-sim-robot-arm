# Dual Arm Task Extension for Isaac Sim 5.1

This extension adds a simple UI window with:
- Start Task button
- Task status display
- Current step display
- Last error display
- JSON path field

## Folder structure

```text
isaacsim_5_1_dual_arm_extension/
├─ config/
│  └─ extension.toml
└─ user/
   └─ dual_arm/
      └─ task/
         ├─ __init__.py
         ├─ constants.py
         ├─ extension.py
         └─ task_runner.py
```

## How to install in Isaac Sim

1. Copy this folder to a location on your PC.
2. Open **Window > Extensions** in Isaac Sim.
3. Open the Extensions settings panel and add the **parent folder** of this extension to the search path.
4. Search for **Dual Arm Task** enable it.
5. A window named **Dual Arm Task** should appear.

## Before pressing Start Task

Make sure these are correct in `config/extension.toml`:
- `json_path`
- `arm1_prim_path`
- `arm1_ee_path`
- `arm2_prim_path`
- `arm2_ee_path`
- `goods_path`
- `lid_path`

Also make sure your JSON contains:
- arm1_home
- arm1_goods_above
- arm1_goods_pick
- arm1_box_above
- arm1_box_place
- arm1_safe_wait
- arm2_home
- arm2_lid_above
- arm2_lid_pick
- arm2_lid_open_place_above
- arm2_lid_open_place
- arm2_box_cover_above
- arm2_box_cover_place
- arm2_safe_wait

## 🎥 Demo Video

[![KUKA Dual Arm Demo](https://img.youtube.com/vi/3t4HDp8p4lI/0.jpg)](https://www.youtube.com/watch?v=3t4HDp8p4lI)

## Notes

- This version auto-plays the timeline if it is not already running.
- The box zone is protected by a lock so Arm1 and Arm2 do not enter it at the same time.
- The task flow is:
  1. Arm2 opens lid and moves aside
  2. Arm1 prepares goods in parallel
  3. Arm1 places goods into box
  4. Arm2 picks lid and closes box
