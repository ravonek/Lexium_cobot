# LEXIUM GEMINI — STARTER ARCHIVE (10 ноября 2026)

Этот ZIP создан из реально доступных вложений беседы. **Он НЕ является полной копией текущего Ubuntu-проекта.**

## Что внутри
- `LEXIUM_GEMINI_HANDOFF_2026-11-10.md`: подробный контекст, архитектура проекта, реальные результаты Stage 3C.73/3C.74, ошибка Stage 3C.75.
- `model/robot_cfg.py`, `model/assets/lexium_cobot.urdf`: **реальные** исходные модельные файлы из архивного пакета Stage 3C.41. Это могут быть более ранние версии, чем файлы в текущем Ubuntu-проекте.
- `model/assets/meshes/`: **только 6 STL** для основания и звеньев руки; STL для `gripper_base_1` и четырёх пальцев в доступных вложениях отсутствуют.
- `historical_sources/`: доступные исходники/запускные скрипты примерно Stage 3C.22 и 3C.31–52.
- `historical_reports/`: JSON и другие ранние отчёты, отдельно `offline_examples/` — **не** реальные прогоны.
- `historical_logs/`: полученные в этой беседе терминальные логи Stage 3C.68 и Stage 3C.73.
- `collect_full_lexium_from_ubuntu.py`: создаёт более полную ZIP-копию **на твоём Ubuntu**, читая текущие локальные файлы, логи и ассеты. Скрипт не запускает симуляцию.

## Критически недостаёт в starter ZIP
- Актуальные Python-файлы Stage 3C.68–75, особенно:
  `lexium_stage3c73_physx_two_finger_precontact_trial.py`,
  `lexium_stage3c74_finger_cube_sensor_wiring.py`,
  `lexium_stage3c75_finger_cube_positive_control.py`,
  `lexium_stage3c75_diagnostic.py`.
- Файлы `~/Downloads/lexium_stage3c73_*.json`, Stage 3C.74 JSON/LOG и Stage 3C.75 LOG (они на Ubuntu, но не загружены сюда).
- Рабочая **USD/USDA-модель** и её вспомогательные ассеты.
- 5 gripper STL и, возможно, другие referenced assets — без них URDF не является самодостаточным для полноценной симуляции.
- **XRDF**: в доступных вложениях не найден. XRDF полезен для некоторых motion-planning workflows, но не требуется лишь для чтения модели и диагностики текущих Isaac Lab скриптов; не нужно его выдумывать.

## Как получить максимально полный ZIP на Ubuntu
1. Распакуй этот ZIP, например в `~/Downloads/lexium_gemini_starter/`.
2. Открой терминал и выполни:
   `python3 ~/Downloads/lexium_gemini_starter/collect_full_lexium_from_ubuntu.py`
3. Скрипт создаст `~/Downloads/LEXIUM_GEMINI_COMPLETE_LOCAL_2026-11-10.zip`.
4. Загрузи **этот локально собранный ZIP** в Gemini вместе с запросом продолжить с Stage 3C.75. Он будет содержать актуальные `.py`, URDF, XRDF (если есть), USD, STL/OBJ/DAE, JSON/логи (если файлы находятся в указанных директориях).
5. Скрипт выводит числа найденных URDF/XRDF/USD/meshes и перечень критически недостающих файлов. Проверяй, чтобы Stage 3C.73–75 действительно присутствовали.

Скрипт намеренно не включает `.venv`, Git, секреты, огромные checkpoints; отдельные файлы >300MB пропускаются с записью в манифест.

**Безопасность**: никакие результаты не дают разрешения запускать физический робот. Самостолкновения в актуальных PhysX-прогонах не валидированы. Stage 3C.75 не пройден; предположение о проблеме компенсации гравитации пока не проверено.
