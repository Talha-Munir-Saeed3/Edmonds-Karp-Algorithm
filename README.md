<div align="center">

# 🌊 FLOW-NET: Max-Flow & Min-Cut Analysis Suite

### *Algorithmic Flow Network Optimization & Step-by-Step Visualizer*

[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)](#)
[![C++](https://img.shields.io/badge/C%2B%2B-11%20%7C%2014%20%7C%2017-00599C?logo=c%2B%2B&logoColor=white)](#)
[![Algorithm](https://img.shields.io/badge/Algorithm-Edmonds--Karp%20%7C%20Ford--Fulkerson-darkgreen)](#)
[![Visualization](https://img.shields.io/badge/Visualization-NetworkX%20%26%20Matplotlib-ff69b4)](#)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](#)

<p align="center">
  <b>A dual-language implementation of the Edmonds-Karp algorithm for computing Maximum Network Flow and Minimum Cut cuts, featuring step-by-step graphical state rendering in Python and bare-metal array-optimized execution in C++.</b>
</p>

[Dual Architecture](#-dual-architecture) •
[Core Theoretical Concepts](#-core-algorithms--data-structures) •
[Visual Pipeline](#-visual-pipeline-python) •
[Quickstart Guide](#-quickstart-guide) •
[Benchmarking & Complexities](#-computational-complexity)

---

</div>

## 📌 Overview

**Flow-Net** solves the classic maximum bipartite/multicommodity network routing problem: given a directed network with capacity limits on each edge, what is the maximum amount of flow that can pass from source ($s$) to sink ($t$)?

This repository provides two coordinated implementations of the **Edmonds-Karp Algorithm** (an implementation of Ford-Fulkerson using Breadth-First Search for shortest augmenting path selection):

1. **`Project Final.py` (Visual & Pedagogical Suite):** Generates step-by-step graphical representations of augmenting paths, residual capacity updates, bottleneck flows, and minimum $s-t$ cuts using `networkx` and `matplotlib`.
2. **`Project.cpp` (High-Performance Engine):** Bare-metal implementation utilizing static contiguous memory (`memset`, 2D adjacency tables) and custom circular queue implementations for minimal memory overhead and zero dependency requirements.

---

## ⚖️ Dual Architecture

| Feature | Python (`Project Final.py`) | C++ (`Project.cpp`) |
| :--- | :--- | :--- |
| **Primary Goal** | Visual simulation & interactive state inspection | Low-latency algorithmic execution |
| **Graph Representation** | Dynamic adjacency lists + NumPy capacity/flow matrices | Fixed static arrays (`Max_V = 100`) with zero heap churn |
| **Queue Architecture** | `collections.deque` | Custom dynamic `Queue` class / `std::queue` |
| **Output Type** | Multi-stage Matplotlib plots + on-canvas metrics | CLI standard stream (`stdout`) |
| **Dependencies** | `numpy`, `networkx`, `matplotlib` | Standard C++ Library (`<climits>`, `<cstring>`) |

---

## 🧠 Core Algorithms & Data Structures

<details open>
<summary><b>1. Edmonds-Karp Max-Flow </b></summary>
<br>

Rather than arbitrary augmenting path selection (which can cycle indefinitely on irrational capacities), Edmonds-Karp uses **Breadth-First Search (BFS)** to guarantee finding the path with the fewest edges:
1. **BFS Traversal:** Searches for a directed path from source $s$ to sink $t$ in the residual graph where $c_f(u, v) = c(u, v) - f(u, v) > 0$.
2. **Bottleneck Calculation:** Identifies the minimum residual capacity along the discovered path:
   $$\Delta f = \min_{(u, v) \in P} \left( c(u, v) - f(u, v) \right)$$
3. **Residual Graph Augmentation:** Updates forward and backward edges:
   $$f(u, v) \leftarrow f(u, v) + \Delta f$$
   $$f(v, u) \leftarrow f(v, u) - \Delta f$$

</details>

<details open>
<summary><b>2. Max-Flow Min-Cut Theorem & DFS Cut Partitioning</b></summary>
<br>

By the Max-Flow Min-Cut Theorem, the maximum flow value equals the capacity of the minimum cut:
$$\text{Max Flow} = \sum_{u \in S, v \in T} c(u, v)$$

- Once BFS finds no more augmenting paths ($\Delta f = 0$), a **Depth-First Search (DFS)** traverses all reachable vertices from $s$ in the residual network.
- All visited vertices form partition set $S$; unvisited vertices form set $T$.
- Any original edge crossing from $S \to T$ is flagged as a bottleneck belonging to the **Minimum Cut**.

</details>

---

## 🎨 Visual Pipeline (Python)

The visual script steps through the entire lifecycle of the graph:

```text
[1. Initial Graph] ──> [2. Augmenting Paths 1..k] ──> [3. Final Flow State] ──> [4. Min-Cut Highlight]
(Capacities only)       (Green highlight + metrics)     (Flow / Capacity format)   (Dashed red cut edges)


