# ТЕКУЩАЯ ТОЧКА ОСТАНОВКИ

- PASS: Stage 3C.73, движение руки и fingers 1/3 в precontact; TCP 0.32423 mm, 0.03513°; захват/подъём НЕ выполнен.
- PASS: Stage 3C.74 — filtered contact sensor bodies finger1/finger3, negative baseline, cube/table 0.981N. Чувствительность контакта ещё не доказана.
- FAIL/INCOMPLETE: Stage 3C.75 — на первом physics tick `STAGE74_SENSOR_WIRING_ABORT=ROBOT_HOME_DRIFT`, подтверждено diagnostic log. Позже предложили gravity compensation, но локальный `lexium_stage3c75_gravity_fixed.py` отсутствовал, поэтому его исполнение не произошло.
- На первой проверке убедиться в существовании файлов `repair_stage75_gravity.py` и `lexium_stage3c75_gravity_fixed.py`; внимательно сверить модель и исходники успешных Stage 3C.50/68/73 перед исправлением.
- Следующая цель: корректно удерживать HOME, проверить отдельные PhysX finger-cube filtered sensors на реальном касании (positive control) без закрытия пальцев, затем переходить к более сложным контактным экспериментам только с ограничением сил.
- В будущем: Intel RealSense RGB-D, детекция томатов, hand-eye calibration, вычисление координат в robot base, мягкий захват и pick/place.
