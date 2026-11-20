# Как использовать архив Gemini Lexium

**Статус:** здесь 15 оригинальных файлов Gemini, извлечённых без исправления API, и вспомогательный скрипт `check_assets.py`. Файлы прошли статическую компиляцию Python, но не запускались в Isaac Sim. Код имеет критические несоответствия Isaac Lab 3.0.0-EA и НЕ готов к обучению без правок.

## Размещение

1. Распакуйте архив в `~/Desktop/adi/` или любую папку проекта. В результате должно быть `~/Desktop/adi/lexium_pick_place/`.
2. Положите `lexium_cobot.urdf` в `lexium_pick_place/assets/`; STL из исходной папки `meshes` — в `lexium_pick_place/assets/meshes/`; XRDF (если есть) — рядом с URDF.
3. `python3 lexium_pick_place/check_assets.py` из корня распакованного архива. Вы увидите точные URDF-лимиты и отсутствующие STL/links.
4. `python3 -m compileall -q lexium_pick_place` проверяет только синтаксис.
5. Перед PhysX, RL: исправьте URDF conversion, spawn, Isaac Lab 3.0 cfg / imports, RSL-RL runner, sensors и event/reward logic. См. основное пояснение в чате.

## Документация ветки EA

- https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/setup/installation/index.html
- https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/how-to/import_new_asset.html
- https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/api/lab/isaaclab.sim.converters.html

## Особо критичные ошибки оригинала

- `robot_cfg.py`: неверный параметр `collider_type`, `spawn=UrdfConverterCfg`, часть импортов и actuator argument names требуют сверки.
- `pick_place_env_cfg.py`: current action config не реализует требуемый delta control, target marker не привязан к реальной команде, поза command вероятно в иной системе координат; @configclass у nested manager classes, debug state и physics настройки требуют сверки.
- `mdp/actions.py`: `binary_mode=False` вопреки бинарной логике в описании.
- `mdp/rewards.py`: жестко прописаны joint indices [6,7,8,9]; контакт любого объекта считается захватом кубика.
- `mdp/events.py`: material properties методы/поля требуют сверки с API 3.0.
- `agents/rsl_rl_ppo_cfg.py`: legacy `policy` вместо `actor` + `critic` для RSL-RL 5.x.
- `train.py`/`play.py`: runners imports and CLI differ from canonical 3.0 CLI.
- `check_robot.py`: SimulationContext instance() before creation; cfg/attributes require verification.
- В README оригинала `lexium_cobot.urdf` не был приложен, поэтому табличные пределы НЕ подтверждены из вложения; проверьте своим URDF с `check_assets.py`.
