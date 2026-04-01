# software/llm/

> LLM integration layer: prompts for task planning, Ollama configs, and scripts connecting the language model to the compute pipeline.
> Слой интеграции LLM: промпты для планирования задач, конфиги Ollama, скрипты подключения языковой модели к вычислительному пайплайну.

Здесь хранится всё, что связано с использованием локальных LLM (через Ollama) для высокоуровневого планирования миссий и взаимодействия с `software/compute/`.

This folder contains everything related to running local LLMs (via Ollama) for high-level mission planning and integration with `software/compute/`.

---

## Что сюда класть / What goes here

- `modelfile` — Ollama Modelfile с параметрами модели (`model: llama3`, `temperature: 0.2`, и т.д.)
- `prompts/task_planning.txt` — промпт для разбивки высокоуровневой задачи на движения дрона
- `prompts/obstacle_reasoning.txt` — промпт для рассуждений об обходе препятствий
- `ollama_config.yaml` — конфигурация: endpoint, timeout, retry-политика
- `llm_bridge.py` — скрипт, принимающий команды от оператора и передающий в `software/compute/`
- `test_planning.py` — тест: проверка, что LLM возвращает валидный план

**Не класть сюда / Do NOT put here:** алгоритмы SLAM и path planning (→ `software/compute/`), ROS-ноды (→ `software/ros/`)
