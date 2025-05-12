
---

````markdown
# 🧭 Robot Navigation Dashboard (ROS + Jupyter Frontend)

This project provides a **real-time robot goal interface** built in a Jupyter Notebook environment, allowing interactive control of a mobile robot using ROS. The system allows users to:

- Send or cancel navigation goals.
- Monitor robot state (position and velocity).
- Track goal statistics (reached or cancelled).
- Visualize proximity to obstacles.
- View robot, goal, and obstacle positions on a dynamic 2D plot.

---

## 📦 Project Structure

### Backend Nodes

- `action_client_node.py`: Connects to a ROS action server to send and cancel goals, and reports back the result.
- Publishes:
  - `/robot_state` (`RobotState`): Robot's position and velocity.
  - `/goal_result` (`GoalResult`): Status and message of each goal result.
  - `/obstacle_position` (`Point`): Position of the closest detected obstacle.
  - `/obstacle_distance` (`Float32`): Scalar distance to the nearest obstacle.
- Subscribes:
  - `/odom` (`Odometry`): To extract current robot state.

### Frontend (Jupyter Notebook)

- Real-time widgets using `ipywidgets` for goal interaction.
- Live Matplotlib visualization with `FuncAnimation`.
- ROS subscribers to `/robot_state`, `/obstacle_position`, `/goal_result`, and `/obstacle_distance`.

---

## 📡 ROS Topics

| Topic                  | Type                | Description                          |
|------------------------|---------------------|--------------------------------------|
| `/goal_topic`          | `geometry_msgs/Point` | Publishes navigation goals           |
| `/cancel_goal_topic`   | `std_msgs/Float32`   | Triggers goal cancellation (1.0)     |
| `/robot_state`         | `assignment_2_2024/RobotState` | Robot’s current state       |
| `/goal_result`         | `assignment_2_2024/GoalResult` | Result of navigation goals |
| `/obstacle_position`   | `geometry_msgs/Point` | Position of closest obstacle         |
| `/obstacle_distance`   | `std_msgs/Float32`   | Distance to closest obstacle         |

---

## 📊 Features

- **Goal Control**: Set goal position (x, y) using sliders and send/cancel via buttons.
- **Goal Summary**: Bar chart showing total reached and cancelled goals.
- **Robot & Target Plot**:
  - Blue dot: current robot position.
  - Red dot: current target goal.
  - Black `X`: obstacle position.
- **Obstacle Awareness**:
  - Distance displayed live.
  - Map background turns pink if obstacle is closer than `1.0m`.

---

## 🛠️ How to Run

### 1. Backend (ROS Node)
Make sure you have your workspace set up correctly with ROS and the custom messages compiled.

```bash
roslaunch assignment_2_2024 assignment1.launch
````

Ensure the following messages exist in `msg/` and are properly defined:

* `RobotState.msg`
* `GoalResult.msg`

### 2. Frontend (Jupyter)

Activate your ROS environment and open the notebook:

```bash
source ~/catkin_ws/devel/setup.bash
jupyter notebook
```

Run the notebook that contains the frontend interface.

---

## 📁 Custom Messages

### `RobotState.msg`

```msg
float64 x
float64 y
float64 vel_x
float64 vel_z
```

### `GoalResult.msg`

```msg
int8 status  # 1 = succeeded, 0 = canceled, -1 = aborted
string message
```

---

## 🧪 Dependencies

Make sure you have the following Python and ROS packages installed:

* `rospy`
* `actionlib`
* `matplotlib`
* `ipywidgets`
* `geometry_msgs`
* `nav_msgs`
* `std_msgs`

For Jupyter integration:

```bash
pip install ipywidgets matplotlib
jupyter nbextension enable --py widgetsnbextension
```

---

## ✅ Future Improvements

* Visualize robot trajectory.
* Add automatic obstacle avoidance logic.
* Enable goal queuing or path planning interface.
* Sound or alert if too close to an obstacle.

---

## 🧑‍💻 Author
Mohamed Ismail Mohamed Sayed      6655420

```
