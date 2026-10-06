# Interactive Physics Simulator for AI-Assisted STEM Learning

This project aims to develop an interactive and programmable physics simulator designed to support STEM learning through dynamic visualization of physical phenomena.

The simulator is being developed in **C++** using a modular architecture. Instead of placing the entire program in a single file, the project is divided into independent components, each responsible for a specific part of the simulation.

The initial version focuses on **kinematics**, including:

- Uniform motion
- Uniformly accelerated motion
- Free fall
- Projectile motion

The simulator will represent physical quantities such as **position, velocity, acceleration, and trajectories**, allowing users to observe how changes in initial conditions affect the behavior of a physical system.

## Project Architecture

The first version of the project is organized around three main components:

- **Vector2D**: represents two-dimensional physical quantities such as position, velocity, and acceleration.
- **PhysicalObject**: represents an object inside the simulation and stores its physical state.
- **Simulation**: controls simulation time and updates the state of the physical objects.

The project is separated into `.hpp` and `.cpp` files. Header files (`.hpp`) define the classes and their interfaces, while source files (`.cpp`) contain their implementation.

A simplified structure is:

```text
physics-simulator/
│
├── main.cpp
│
├── Vector2D.hpp
├── Vector2D.cpp
│
├── PhysicalObject.hpp
├── PhysicalObject.cpp
│
├── Simulation.hpp
├── Simulation.cpp
│
└── CMakeLists.txt
```
This modular design makes the simulator easier to understand, test, maintain, and extend.
Future Development
Future versions of the project are expected to include:
- Interactive graphical visualization
- Position, velocity, and acceleration vectors
- Trajectory visualization
- JSON-based simulation instructions
- Numerical integration methods
- User-configurable simulation parameters
- Integration with artificial intelligence
The long-term objective is to allow an AI assistant to translate a physical problem written in natural language into structured simulation instructions. These instructions could then be interpreted by the C++ simulation engine to generate an interactive visualization of the requested phenomenon.
The AI component will remain separated from the physics engine, allowing the simulator to operate independently and making the physical results easier to validate.

Esta versión mantiene la idea original del proyecto: primero construir un simulador independiente que interprete instrucciones y represente fenómenos de cinemática, dejando la integración con IA como una etapa posterior. :chatgpt-content-reference{index="0"}

Además, para GitHub me parece mejor esta redacción porque deja claro que **C++ es el motor del proyecto**, pero que la meta no es simplemente crear animaciones: la arquitectura está pensada para que posteriormente exista el flujo:

```text
Natural Language
       ↓
AI Assistant
       ↓
Simulation Instructions
       ↓
C++ Physics Engine
       ↓
Interactive Visualization
```
