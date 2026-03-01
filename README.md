Project Overview

This project is a Python-based GUI application that demonstrates intelligent pathfinding using:

A* Search

Greedy Best-First Search

The application is built using Tkinter and allows users to visually compare both algorithms in a grid-based environment. It also supports dynamic obstacle generation during path execution.

This project helps in understanding how informed search algorithms work in real-time.

Features

Interactive grid (custom size up to 50x50)

Set Start and Goal positions manually

Place and remove obstacles

Random maze generation

A* Search implementation

Greedy Best-First Search implementation

Manhattan and Euclidean heuristics

8-directional movement (including diagonals)

Dynamic obstacle mode with automatic replanning

Real-time performance metrics display

Path animation

Algorithms Implemented
A* Search

A* calculates:
f(n) = g(n) + h(n)

g(n): actual cost from start to current node

h(n): estimated cost to goal

It guarantees the shortest path when heuristic is admissible.

Greedy Best-First Search

Greedy calculates:
f(n) = h(n)

It selects the node that appears closest to the goal, ignoring actual path cost. It is faster in simple cases but does not guarantee optimal paths.

Performance Metrics Displayed

Nodes Visited

Frontier Size

Path Length

Path Cost

Execution Time (milliseconds)

Obstacle Count

Grid Dimensions

Dynamic Mode

When dynamic mode is enabled:

New obstacles may appear during path execution.

The agent automatically replans if the path becomes blocked.

This simulates real-world changing environments.

Technologies Used

Python

Tkinter (GUI)

heapq (Priority Queue)

Dataclasses

Object-Oriented Programming

How to Run

Install Python 3.x

Save the file as pathfinding.py

Run the program:

python pathfind.py

The GUI window will open.

Educational Purpose

This project is useful for:

Artificial Intelligence coursework

Understanding informed search algorithms

Visual learning of heuristic-based pathfinding

Comparing optimal vs non-optimal search strategies
