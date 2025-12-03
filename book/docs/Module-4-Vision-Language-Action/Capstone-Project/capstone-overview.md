---
id: capstone-overview
title: Humanoid Capstone Project Overview
sidebar_label: Capstone Project Overview
---

# Humanoid Capstone Project Overview

The **Capstone Project** is the final exam of this course. It is not a multiple-choice test. It is a proof of competence. You will build a complete **Vision-Language-Action (VLA)** system that allows a humanoid robot to function as a helpful assistant.

## The Challenge: "The Butler Test"

Your robot will be placed in a simulated home environment (Isaac Sim). It must pass the following scenario:

1. **User Command**: "I'm working at the desk and I'm thirsty. Can you bring me a soda from the kitchen?"
2. **Understanding**: The robot must parse "thirsty" -> "needs drink" -> "find soda".
3. **Execution**:
   - Navigate from the Living Room to the Kitchen.
   - Detect the soda can on a cluttered table.
   - Grasp the soda can.
   - Navigate to the Desk.
   - Place the soda can near the user.
4. **Feedback**: "Here is your soda. Do you need anything else?"

## System Architecture

You will integrate every technology learned in this course:

| Component | Technology | Function |
|-----------|------------|----------|
| **Brain (VLA)** | GPT-4 + Whisper | Understands speech, plans tasks. |
| **Eyes (Perception)** | Isaac ROS VSLAM | Localizes the robot in the house. |
| **Object Detection** | YOLOv8 / DOPE | Finds the soda can. |
| **Legs (Navigation)** | Nav2 | Plans path from Room A to Room B. |
| **Hands (Manipulation)** | MoveIt 2 | Plans arm trajectory to grasp. |
| **Body (Simulation)** | Isaac Sim | The physical world. |

## Requirements

### Minimum Viable Product (MVP)
- [ ] **Voice Interface**: Robot responds to "Hey Robot".
- [ ] **Navigation**: Robot can move between at least 2 rooms autonomously.
- [ ] **Grasping**: Robot can pick up at least 1 object type.
- [ ] **Safety**: Robot does not collide with walls or furniture.

### Distinction Features (Optional)
- [ ] **Memory**: "Where did I leave my keys?" (Robot remembers past locations).
- [ ] **Error Recovery**: If it drops the can, it tries again.
- [ ] **Multi-Modal**: "Bring me *that* red cup" (Pointing + Speech).

## Evaluation Criteria

Your project will be graded on a 100-point scale:

### 1. Autonomy (40 Points)
- **40**: Zero human intervention.
- **30**: One intervention (e.g., robot got stuck).
- **10**: Teleoperated for part of the task.
- **0**: Fully teleoperated.

### 2. System Integration (30 Points)
- **30**: Smooth hand-offs between VLA, Nav2, and MoveIt.
- **15**: High latency or "stop-and-go" behavior.
- **0**: Components work in isolation but not together.

### 3. Complexity (20 Points)
- **20**: Cluttered environment, dynamic obstacles.
- **10**: Empty room, static environment.

### 4. Presentation (10 Points)
- **10**: Clear video demo, professional documentation.

## Timeline

- **Week 12**: Build the Voice Interface and VLA Planner.
- **Week 13**: Integrate Perception (Isaac ROS) and Navigation.
- **Week 14**: Manipulation (Grasping) and Final Polish.

---

**Next:** Let's break down the problem in [Planning and Design](./planning-and-design.md).