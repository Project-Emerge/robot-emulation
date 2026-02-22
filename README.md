# Robot Emulation
This project simulates the robots used for the researcher nights at the University of Bologna.
It provides a simple physical interface to control the robots and observe their behavior.
They continuously send data about their state via MQTT, which can be visualized using the provided web interface.
## How to run
1. Install poetry
2. Install the dependencies with `poetry install`
3. Run the application with `poetry run python src/robot_emulation/main.py --robots 100 --mqtt mqtt://localhost:1883 --world-size 100.0`


## How to visualize the data
You should use the other repository [robot_emulation_web](https://github.com/Project-Emerge/robot-visualization-dashboard)
