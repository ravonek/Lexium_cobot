# ПОЛНЫЙ ТЕХНИЧЕСКИЙ КОНТЕКСТ ДЛЯ GEMINI — LEXIUM COBOT PICK & PLACE

**Дата среза:** 10 ноября 2026. **Язык общения:** русский. **Статус:** Stage 3C.73 PASS; Stage 3C.74 PASS; Stage 3C.75 НЕ ПРОЙДЕН; последнее препятствие — отсутствует созданный файл исправленного теста. Это документ передачи контекста, а не утверждение о наличии доступа к машине или физической готовности робота.

## 0. Как использовать этот текст

Ты — Gemini, новый технический помощник в продолжающемся проекте. Учитывай всю информацию ниже как контекст прошлой работы. **Не считай упомянутые планы и офлайновые проверки подтверждённой физической реализацией. Не повторяй заново подтверждённые этапы без причины.** Разделяй: (A) реальные успешные исполнения Isaac Lab / PhysX по терминальным логам, (B) офлайновые расчёты и геометрические тесты, (C) гипотезы, ошибки, непроверенные скрипты. Не заявляй, что физический захват или pick-and-place уже работает. Если нужно исправить код — опирайся на реальные локальные файлы и трассировки, не обещай работоспособность без прогона. Проси только конкретный необходимый вывод; давай готовые команды Ubuntu, желательно `cat > script.py <<'PY'`, `python3 -m py_compile`, после проверки — запуск `uv run isaaclab -p`. Пользователь переносит команды вручную, надёжная загрузка файлов из чата не всегда доступна. Длинные команды терминала могут повредиться при вставке, поэтому разделяй сборку/проверку/запуск.

## 1. Конечная цель проекта

**AI-Enabled Pick-and-Place Workcell**: **Schneider Electric Lexium Cobot**, 6-осевой манипулятор, модель в предыдущих материалах **LXMRL03S0000** (до ~3 кг), с четырёхпальцевым захватом **FAE4MR86M-D**. Долгосрочная цель — автоматическое **обнаружение томатов с камерой Intel RealSense RGB-D, определение их 3D-положения, планирование безопасного бережного захвата, взятие и перемещение в целевую зону**. Это **не** просто перемещение куба: куб 40 мм — промежуточный тестовый объект, чтобы проверить точность кинематики, контроллеров и контактной физики. Модель камеры (D435i/D455 и т.д.), место крепления (на захвате — eye-in-hand / над столом — eye-to-hand) и доступность реального робота ещё **не подтверждены**. В диалоге отмечалось возможное крепление камеры на захвате, но это пока не окончательная спецификация. Текущие эксперименты **исключительно в симуляции**; реальное оборудование не подключено.

Запланированная система: RealSense RGB + depth -> обнаружение/сегментация томатов (например, YOLO-seg), согласование RGB/depth и фильтрация глубины -> координаты в системе камеры через интринсики -> преобразование в базу робота после калибровки camera↔robot (hand-eye) -> выбор точки/подхода захвата, IK и планирование траектории -> мягкий контролируемый захват с контактным мониторингом -> подъём, перенос, размещение, отпускание. Позже можно использовать ROS 2 и Isaac Lab RL/PPO, но **пока нет доказательства работающего camera-to-grasp pipeline**.

## 2. Текущая программная среда и важные пути

- ОС хоста: Ubuntu 24.04 / 24.04.5 LTS; GPU NVIDIA GeForce RTX 5080 (~15 GiB); CUDA Toolkit 12.9, Warp 1.16.0 (из логов).
- **Isaac Lab 3.0.0-EA**, Isaac Sim **6.1.0**; Python **3.12** внутри `~/IsaacLab-3.0-EA/.venv/`; используется PhysX и `uv run isaaclab -p`.
- Хост в логах `aybergen@aybergen-Z790-D-AX`.
- Рабочие переменные:
  ```bash
  PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
  WORK="$HOME/Desktop/adi/lexium_rl_workspace"
  ISAACLAB="$HOME/IsaacLab-3.0-EA"
  D="$HOME/Downloads"
  ```
- Импорт Python проекта через `PYTHONPATH="$WORK"`; рабочий модуль `lexium_pick_place.robot_cfg`; URDF: `"$PROJECT/assets/lexium_cobot.urdf"`; существует/использовался USD `assets/usd/lexium_cobot/lexium_cobot.usda` и отдельные CAD/STL коллизии.
- Пример реального запуска:
  ```bash
  cd "$ISAACLAB"
  PYTHONPATH="$WORK" uv run isaaclab -p "$PROJECT/SCRIPT.py" --num-envs 1 --visualizer none ...
  ```
- Отчёты JSON и логи `.log` в `~/Downloads/`.
- Более старые Windows/старые USD-конфигурации и RL-модель **не отождествлять** с текущей Ubuntu-моделью без проверки: были варианты с 8 actions/2 sliders, тогда как **текущий подтверждённый URDF имеет 10 joints: 6 arm + 4 gripper**.

## 3. Модель робота, суставы и физические ограничения

**Текущие 10 суставов (из физического лога):**
1. `joint_1` — arm, [-π,+π] rad, stiffness 400, damping 40, effort limit 100 N·m, velocity limit 1 rad/s.
2. `joint_2` — arm, те же пределы, stiffness 800, damping 60.
3. `joint_3` — arm, stiffness 800, damping 60.
4. `joint_4` — arm, stiffness 400, damping 40.
5. `joint_5` — arm, stiffness 400, damping 40.
6. `joint_6_gripper_rotation` — arm, stiffness 400, damping 40.
7. `gripper_Revolute_1` — finger1, limits [-1.780, +0.131] rad, stiffness 2000, damping 100.
8. `gripper_Revolute_2` — finger2, limits [-1.571, +0.340] rad, stiffness 2000, damping 100.
9. `gripper_Revolute_5` — finger3, limits [-0.349, +1.571] rad, stiffness 2000, damping 100.
10. `gripper_Revolute_6` — finger4, limits [-0.340, +1.571] rad, stiffness 2000, damping 100.

В логах gripper velocity limits = 100, effort limits = 100 (значения сконфигурированной модели, **не** подтверждённые паспортные безопасные усилия для помидоров). Пальцы имеют сложные STL-меши. Имеются **старые пересечения поверхностей STL в HOME**; самостолкновения в актуальных PhysX-прогонах **отключены**, поэтому `robot contact=0` нельзя трактовать как доказательство отсутствия всех внутренних коллизий. Динамика пальцев и мягкое усилие для томатов не валидированы. Ранее модель страдала от проблем с фиксацией базы, тряски, коллизий; нынешние прогоны имеют fixed base в сцене, но нельзя автоматически переносить этот факт на старый USD или физическое железо.

В Stage3C74 тестовая сцена: стол размером 0.90×1.20×0.40 м (центр примерно [0.62,0,-0.20]); зелёный куб 0.04×0.04×0.04 м, масса 0.10 кг, начальная точка [0.30,-0.30,0.12]; контакт куба со столом после оседания ≈**0.981 N**, что соответствует 0.1 kg × 9.81 m/s².

## 4. Что уже выполнено — история существенных этапов

### Далёкие ранние этапы / RL
- URDF/STEP -> USD articulation; исходный импорт 10 joints/22 mesh refs, без ошибок на одной проверке; импорт USD ранее доходил до `assets/usd/lexium_cobot/lexium_cobot.usda`.
- Пробовались Isaac Lab env, robot/table/cube/target, PPO/RSL-RL, около 29 наблюдений и 8 действий, порядка 300 итераций / 9600 sim steps; были checkpoint/reward logs, но **это не означает обученный автономный захват**. Эта RL-схема могла основываться на другом варианте 8-DOF модели. Приоритет последних недель — механическая и контактная верификация текущей 10-joint модели.
- Старый проект и расположение USD на Windows были другими; не смешивать пути и результаты.

### Stage 3C.30 — реальный PhysX motion PASS
- Полная сцена HOME -> Candidate 1: position/TCP error **0.150 mm**, orientation error **0.058°**; peak `joint_2` torque **67.33 N·m**; robot external contact **0 N**, cube-table **0.981 N**. Это успешное движение в симуляции, **не grasp**.

### Stage 3C.31–3C.38 — gripper geometry / collision исследование
- Изучались реальные четыре пальца и их collision meshes. Stage3C32 выдал `GRASP_VALIDATED=False`: пальцы не двигались, отсутствие работоспособного grasp.
- Stage3C32B: limit-aware probe, небольшие команды по **0.004 rad**; JSON-отчёты с именем `_limit_aware_`.
- Stage3C33 static geometry; 3C34 virtual opening; 3C35 bottleneck/hinge; 3C36 local aperture, отчёт `/tmp/lexium_stage3c36_local_aperture_report.json`.
- Stage3C37: mesh export, `*_full_collision_triangles.npz`.
- Stage3C38 offline mesh poses: palm (ладонь) **5640 vertices / 11514 triangles**, lowest world Z gripper base ~**117.856 mm**, finger1 ~**167.536 mm**. Это **офлайновые геометрические** результаты, не физическая динамика.

### Stage 3C.40 — IK feasibility
- Base quaternion `(0.7071, 0, 0, 0.7071)`; TCP offset `[0.002, 0.1, 0.148]` м.
- Candidate index **5**: joint margin **41.48°**, gravity torque около **66.56 N·m**, discrete path PASS, IK PASS (проверка расчетов; не эквивалентна безопасному реальному движению).

### Stage 3C.50–3C.68 — подготовка и реальное PhysX движение
- Использовались preflight домашнего контакта, планирование waypoint, пред-подход и плотная траектория `Stage 3C.67`.
- Stage3C62 lifted wrist waypoint, Stage3C63 PhysX two-segment wrist trial; Stage3C67 dense joint path report.
- **Stage3C68** реальная симуляция HOME -> wrist clearance -> preapproach -> final target, `STAGE68_SIM_MOTION_PASS=True`.
- Stage68 final TCP error **0.32635 mm**, orientation **0.03769°**, peak arm effort **85.6813 N·m**, peak tracking error **0.01237 rad**, external robot contact **0 N**, cube stable; end arm q:
  `[-0.4562118053, 3.0264446735, 1.5111563206, 0.0114600481, 0.0767459795, -0.3735128939]` rad.
- ВАЖНО: `joint_2` близок к +π limit; нельзя необдуманно добавлять угол или двигать сквозь joint limits. Self-collision not validated.

### Stage 3C.69–3C.72 — подробная ОФЛАЙНОВАЯ геометрия пальцев
- Stage3C69: candidate6 центр относительно кубика `[-0.08,+0.08,-0.39] mm`; в HOME пальцы и gripper base геометрически не пересекают стол/куб; closest finger1 mesh-to-cube ~**21.37 mm**; finger3 lowest Z ~**7 mm**.
- Stage3C70: одинаковый положительный sweep до +0.4 rad: finger1 первое пересечение куба около **+0.300 rad**, finger3 около **+0.400 rad**, finger2 и finger4 до куба не дотянулись (на +0.4 rad gaps ~12.1 mm и 10.38 mm). Не возникло новых самопересечений в проверенном пути, но **существовали старые STL-пересечения**.
- Stage3C71: individual positive sweep: first cube intersection finger1 **+0.284967 rad**, finger3 **+0.395891 rad**; finger2 min gap **8.737 mm** even at +0.67064 rad; finger4 min gap **7.956 mm** at +1.9011 rad. Рассчитанные precontact goals по четырём gripper joints: `[-0.270033,-0.340300,+0.050591,-0.340300]` rad. 11 sampled configurations без *новых* offline self/environment mesh-overlaps.
- Stage3C72: оппозитный диагональный двухпальцевый захват из finger1 и finger3: finger1 faces `X+,Y+,Z+`; finger3 `X-,Y-`; противолежащие оси X/Y. Проверены 31 sampled precontact configs, no *new* mesh intersections, но старые 4 STL self-intersection pairs сохраняются. `STAGE72_TWO_FINGER_PRECONTACT_SAMPLED_CLEAR=True` — **это не физический захват**.

### Stage 3C.73 — ПОДТВЕРЖДЁННЫЙ PhysX PASS
- Собрали `lexium_stage3c73_physx_two_finger_precontact_trial.py`, исходник этапа был синтаксически проверен и затем реально запущен пользователем; финал `STAGE73_EXIT_CODE=0`, `LEXIUM_STAGE3C73_PHYSX_TWO_FINGER_PRECONTACT_COMPLETE=True`.
- Отработали HOME -> WRIST_CLEAR (600 steps) -> PREAPPROACH (2400) -> settle (180) -> TARGET_MOVE (3600) -> ARM_HOLD (240) -> движение только finger1/finger3 (1200) -> FINGER_HOLD (180), warmup 150. **Захват куба не командовался.**
- Лог: `STAGE73_REPLAY_STAGE63_ACTUAL_GATE_PASS=True`, `STAGE73_ARM_ENDPOINT_GATE_PASS=True`, `STAGE73_FINGER_PRECONTACT_PASS=True`, `STAGE73_SIM_MOTION_PASS=True`.
- Final arm q: `[-0.4562118053436, 3.0264475345612, 1.5111577510834, 0.0114276437089, 0.0767770633101, -0.3735889792442]`.
- Tool error **0.32423 mm**, orientation **0.03513°**; peak external contact **0.0 N**; peak arm effort **85.6813 N·m**; peak tracking **0.01237 rad**.
- Finger HOME q `[-0.550000,-0.340300,-0.340300,-0.340300]`;
  finger goal `[-0.2700329709,-0.3402999938,+0.0505908144,-0.3402999938]`;
  **final actual gripper q** `[-0.2700324357,-0.3404558599,+0.0510838404,-0.3394470513]`;
  max gripper error **0.000853 rad**, peak finger effort **4.9467 N·m**.
- `STAGE73_GRASP_EXECUTED=False`;
  `STAGE73_SELF_COLLISIONS_UNTESTED=True`;
  `NO_HARDWARE_MOTION_PERMISSION=True`.
- Report: `~/Downloads/lexium_stage3c73_physx_two_finger_precontact_trial.json`;
  log: `~/Downloads/lexium_stage3c73_physx_two_finger_precontact_trial.log`.

### Stage 3C.74 — ПОДТВЕРЖДЁННЫЙ PhysX SENSOR WIRING PASS
- Скрипт `~/Desktop/adi/lexium_rl_workspace/lexium_pick_place/lexium_stage3c74_finger_cube_sensor_wiring.py`.
- Сцена HOME. Два отдельные filtered contact sensors `finger1_cube`, `finger3_cube` на bodies `gripper_finger_1_1` и `gripper_finger_3_1`, `filter_prim_paths_expr=['{ENV_REGEX_NS}/Cube']`, `normal_force_matrix_w` shapes checked. Есть baseline sensors robot and cube.
- Реальный лог: 
  ```text
  STAGE74_FINGER_1_SENSOR_BODY_NAMES=["gripper_finger_1_1"]
  STAGE74_FINGER_1_FILTER_ACCESSOR=normal_force_matrix_w
  STAGE74_FINGER_3_SENSOR_BODY_NAMES=["gripper_finger_3_1"]
  STAGE74_FINGER_3_FILTER_ACCESSOR=normal_force_matrix_w
  STAGE74_SAMPLE step=30 cube_n=0.9810 robot_n=0.0000 finger1_cube_n=0.0000 finger3_cube_n=0.0000
  STAGE74_CUBE_TABLE_POSITIVE_CONTROL_PASS=True
  STAGE74_CUBE_FILTERED_SENSOR_WIRING_PASS=True
  STAGE74_FINGER_CONTACT_POSITIVE_CONTROL_PERFORMED=False
  STAGE74_CONTACT_CLOSURE_READY=False
  LEXIUM_STAGE3C74_COMPLETE=True
  STAGE74_EXIT_CODE=0
  ```
- Это подтверждает корректный доступ к показаниям и отсутствие ложного срабатывания вдали от куба, **но НЕ чувствительность датчиков при прямом контакте пальцев с кубом**.
- Report `~/Downloads/lexium_stage3c74_finger_cube_sensor_wiring.json`;
  log `~/Downloads/lexium_stage3c74_finger_cube_sensor_wiring.log`.

### Stage 3C.75 — НЕ ПРОЙДЕН, СУЩЕСТВЕННОЕ ТЕКУЩЕЕ ПРЕПЯТСТВИЕ
**Замысел:** в отдельной HOME-сцене удержать руку в HOME и **по очереди осторожно подвести тестовый кубик к finger1 и finger3**, проверить ненулевые индивидуальные filtered normal forces и нулевой отклик другого пальца. Никакого closing/lifting/grasp. Реальные контактные испытания в Stage75 ещё не состоялись.

**Файлы, которые предполагалось создать в `$PROJECT`:**
- `build_stage75.py` — генератор из рабочего `lexium_stage3c74_finger_cube_sensor_wiring.py`.
- `lexium_stage3c75_finger_cube_positive_control.py` — первичный сгенерированный Stage75.
- `diagnose_stage75.py`, `lexium_stage3c75_diagnostic.py` — диагностическая версия с трассировкой.
- `repair_stage75_gravity.py` — **предложенный, но не подтверждённый созданным файл** со сборкой gravity-fixed версии.
- `lexium_stage3c75_gravity_fixed.py` — **ОТСУТСТВУЕТ по последнему фактическому запуску**.

**Лог начального Stage75:** запуск показал заголовок и инициализацию сцены, но ни одной pass-строки; внешне напечатал `STAGE75_EXIT_CODE=0`. Следующая диагностическая версия доказала, что программа останавливается **на первом physics tick**, а не на сенсорах:
```text
STAGE75_DIAG_BEFORE_SIM_CREATE=True
STAGE75_DIAG_BEFORE_SCENE_CREATE=True
STAGE75_DIAG_BEFORE_SIM_RESET=True
STAGE75_DIAG_AFTER_SCENE_RESET=True
STAGE75_DIAG_SENSORS_INITIALIZED=True
STAGE75_DIAG_BEFORE_TICK_FUNCTION=True
STAGE75_DIAG_BEFORE_BASELINE=True
STAGE75_DIAG_TICK=1
STAGE75_DIAG_EXCEPTION_TYPE=RuntimeError
STAGE75_DIAG_EXCEPTION_REPR=RuntimeError('STAGE74_SENSOR_WIRING_ABORT=ROBOT_HOME_DRIFT')
STAGE75_DIAG_CLOSING_APP=True
STAGE75_DIAGNOSTIC_EXIT_CODE=0
STAGE75_VERIFIED_PASS=False
```
**ВАЖНО:** `STAGE74_SENSOR_WIRING_ABORT` здесь — не отказ Stage74, а переиспользованное название функции `abort()` из исходника Stage74. `ROBOT_HOME_DRIFT` сравнивает абсолютное отклонение joints относительно default HOME, порог **0.07 rad**, и сработал сразу. Причина **ещё не доказана**. Гипотеза: Stage75 держит `qhome` напрямую, без **gravity-compensated target** (`q_command = q_home + tau_g / K`) и/или есть неправильное состояние initial buffer/переходный первый physics step. Stage50/68/73 успешно использовали компенсацию гравитации. Следует вывести величины joint positions/velocities, max index/joint, actual q, qhome, command и убедиться, что проблема физическая, а не API/read issue.

**Последний фактический пользовательский запуск:**
```text
python: can't open file '/home/aybergen/Desktop/adi/lexium_rl_workspace/lexium_pick_place/lexium_stage3c75_gravity_fixed.py': [Errno 2] No such file or directory
STAGE75_PROCESS_EXIT_CODE=2
STAGE75_MISSING_PASS_MARKERS=[...]
STAGE75_VALID_REPORT=False
STAGE75_VERIFIED_PASS=False
STAGE75_VERIFICATION_EXIT_CODE=1
```
То есть **последняя ошибка — missing file**, а вовсе не успешное испытание; скрипт не был построен. Последний совет помощника — сначала `ls -lh` проверить наличие `repair_stage75_gravity.py`, исходного stage75 и исправленного файла, выполнить сборщик, `python3 -m py_compile`, только потом запускать. **Нет подтверждённого ответа пользователя после этой проверки.**

**Дополнительные потенциальные ошибки в ранее предложенном gravity repair, которые следует проверить, а не слепо выполнять:**
1. Генератор `repair_stage75_gravity.py` основан на точных текстовых строках; в предоставленном ранее коде Stage75 условие drift написано многострочно, а `patch()` ищет одноcтрочное `if vals['TRACKING']>.07:abort('ROBOT_HOME_DRIFT')`. Очень вероятен `STAGE75_PATCH_ANCHOR_COUNT=0`.
2. Генератор предполагает существование импортируемых функций/модулей `lexium_pick_place.lexium_stage3c22_urdf_gravity_torque_audit`, `lexium_stage3c25_corrected_gravity_route_audit`, `LEXIUM_ARM_JOINT_NAMES`; их наличие/сигнатуры не проверены. Вместо этого лучше сравнить **реально работающую** Stage50/Stage68/Stage73 реализацию гравитации и перенести её без выдумывания API.
3. Stage75 после устранения drift ещё может упасть на `final_gripper_base_pose_xyzw` в Stage50 JSON или на геометрии кубика. Эти параметры требуют проверки в файлах.
4. Для отладки Isaac Sim/Kit иногда завершает shell `exit=0` даже при `RuntimeError` внутри Python. **Не считать этап успешным без обязательных PASS-маркеров + валидного JSON.**
5. Геометрические точки drop на верхнюю поверхность пальцев заданы предшествующим помощником как `[−0.0850,−0.3295,0.2365]` и `[−0.1972,−0.3131,0.1794]` м — **это предварительные расчётные точки**, не подтверждённые в PhysX.

**Предлагаемый следующий шаг — ровно один:** на Ubuntu проверить реальные файлы Stage75, рабочий gravity controller Stage50/68, сгенерировать корректный, минимально изменённый Stage75 (или отдельную диагностическую версию) с проверенными API; добавить print actual q/default q/command по первым физическим шагам без ослабления safety gate; сначала убедиться, что HOME baseline держится и cube-table=0.981N, затем ПО ОДНОМУ проверить sensor positive controls. Не запускать захват/подъём. Если основные параметры Stage75 непонятны, попросить локальные первые 200 строк `.py`, relevant section Stage50 и Stage50 JSON keys.

## 5. Контрольные команды для текущей точки

Проверить наличие исходников и диагностического лога (не запускает симуляцию):
```bash
PROJECT="$HOME/Desktop/adi/lexium_rl_workspace/lexium_pick_place"
D="$HOME/Downloads"
cd "$PROJECT" || exit 1
ls -lh \
  lexium_stage3c74_finger_cube_sensor_wiring.py \
  lexium_stage3c75_finger_cube_positive_control.py \
  lexium_stage3c75_diagnostic.py \
  repair_stage75_gravity.py \
  lexium_stage3c75_gravity_fixed.py 2>&1
ls -lh "$D/lexium_stage3c75_diagnostic.log" \
  "$D/lexium_stage3c74_finger_cube_sensor_wiring.json" \
  "$D/lexium_stage3c73_physx_two_finger_precontact_trial.json" 2>&1
```
Проверить реальный исходник Stage75 перед патчем:
```bash
sed -n '1,250p' "$PROJECT/lexium_stage3c75_finger_cube_positive_control.py"
```
Проверить структуру Stage50 и пути gripper/robot gravity; не делай предположений об их значениях:
```bash
python3 - <<'PY'
import json, pathlib
p=pathlib.Path.home()/'Downloads/lexium_stage3c50_physx_contact_preflight.json'
if p.is_file():
    d=json.loads(p.read_text())
    print('\n'.join(sorted(d.keys())))
else:
    print('Stage50 report missing')
PY
```

## 6. Что считается success, что нет

- Stage73 **PASS** = достигли рабочей позы и предконтактных углов пальцев в PhysX с точным tracking; **НЕ означает контакт/грип/подъём**.
- Stage74 **PASS** = индивидуальные датчики подключены/читаются, HOME zero baseline, cube/table positive control; **НЕ означает регистрацию finger/cube контакта**.
- Stage75 **PENDING/FAIL** = первый физический шаг drift, затем не создан исправленный файл. **НЕТ результатов положительного контакта**.
- Отрицательные сигналы `robot_n=0` при отключённых self-collisions не доказывают отсутствие самопересечений STL. Старые геометрические пересечения требуют отдельной обработки и коллизионной валидации.
- Max joint effort и параметры actuator в PhysX не являются валидацией безопасного давления на настоящие томаты.
- Нет evidence того, что камера подключена, что 3D tomato detection работает, что hand-eye calibration сделана, что ROS2 соединён, что PPO освоил pickup, что куб или томат был реально захвачен/поднят/перенесён.

## 7. Дальнейший маршрут после исправления текущей ошибки

1. Stage3C75: HOME stability + физическое обнаружение отдельных контролируемых контактов `finger1↔cube`, `finger3↔cube` в PhysX; каждый датчик должен отвечать на свой контакт и не давать ложного сигнала на другом пальце.
2. Затем отдельный Stage3C76 (название условно): во **всё ещё только симуляции** медленно приблизить fingers1/3 от precontact позы в рабочей конфигурации, оценить contact forces, вмешательство self-collision, сдвиг кубика, solver instability; ограничить усилия, иметь немедленный stop по отклонениям и силам. Не переходить к подъёму прежде чем contact closure подтверждён.
3. Затем первая валидированная симуляция grasp/hold/lift/place с трекингом куба, индивидуальными контактами и планированием retreat; только после этого работа с томатами (сфера/деформируемая модель/мягкие накладки, гораздо меньшие силы).
4. Параллельно виртуальная RealSense/RGBD в Isaac Sim: RGB+depth, segmentation tomato, 3D point, calibration transforms, reachability, collision checks, approach pose, target placement; затем реальная RealSense по модели и креплению, когда появится железо. Преобразования: eye-in-hand `T_base_tomato=T_base_ee * T_ee_cam * T_cam_tomato`; eye-to-hand `T_base_tomato=T_base_cam*T_cam_tomato`. Депроекция `(X,Y,Z)=((u-cx)Z/fx,(v-cy)Z/fy,Z)`, с учётом depth-to-color alignment.
5. ROS2, если нужен, объединить восприятие/планировщик/управление; интерфейс и версия ROS2 под Ubuntu 24.04 / Isaac Sim 6.1 уточняются, **Humble не следует автоматически считать нативной системой в Ubuntu 24.04**. RL/PPO — отдельный следующий уровень, не блокирующий базовую детерминированную проверку физики.

## 8. Требования к следующему помощнику

- Отвечай по-русски, конкретно, с команды «что сделать сейчас», не пересобирай все 75 этапов и не придумывай фальшивые PASS.
- Прежде чем предлагать большой исправляющий скрипт, проверь исходную структуру конкретных файлов через `sed`, `grep`, `ast`, `json.keys()` или попроси пользователя вставить их содержимое; не делай хрупких `.replace()` со строками, которые не проверял.
- Каждая новая версия кода: сначала создание файла, затем `python3 -m py_compile`, `test -s`, и лишь потом `uv run isaaclab -p`. Если compilation fails, не переходить к симуляции.
- Для PhysX-этапов итоговый PASS требует конечных логических проверок + JSON, а не только `EXIT_CODE=0`.
- Не допускать реальных движений робота или резкого закрытия четырёх пальцев по этим результатам. Self-collisions выключены, грип не валидирован.
- Конечная цель — автоматизированный **tomato pick-and-place с Intel RealSense**, а не зацикливание на кубике/датчиках как самоцель. Успешные этапы сохраняй, с минимальным числом новых необоснованных стадий.

## 9. Точная формулировка текущей задачи Gemini

**«Продолжи этот проект с момента Stage 3C.75. В Stage73 успешно выполнено точное PhysX движение руки и пальцев1/3 в предконтакт; Stage74 подтвердил, что individual contact sensors читаются и cube-table contact=0.981N; Stage75 не прошёл: diagnostic выдал `ROBOT_HOME_DRIFT` на первом physics tick, а следующую gravity-fixed версию не удалось запустить, потому что файла `lexium_stage3c75_gravity_fixed.py` нет. Не делай вид, что Stage75 уже запущен. Первым делом помоги проверить локальные `.py`, воспроизвести/исправить startup drift с рабочей gravity compensation из Stage50/68/73, надёжно создать минимальный исправленный Stage75, протестировать контакты пальцев с кубиком по отдельности, и только потом предложить план для перехода к tomato pick-and-place с RealSense. Давай проверяемые команды Ubuntu; не нарушай safety gates и не путай offline geometric feasibility с PhysX или real hardware.»**

---
**КРАТКИЙ ИТОГ:** Самое продвинутое реально подтверждённое действие — точный выход к кубику и установка двух пальцев в precontact (3C73), с отдельно работающими датчиками контактов (3C74). Положительный контакт, сжатие, захват, подъём, размещение и распознавание томатов RealSense — ещё впереди. Текущий blocker Stage75: HOME drift в первом физическом шаге и отсутствующий файл якобы исправленной версии. Не принимать `exit 0` от Isaac Lab без внутренних PASS/JSON.
