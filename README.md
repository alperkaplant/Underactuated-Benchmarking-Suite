# Comparative Simulation Analysis of Underactuated Robotic Mechanisms

This repository contains the official Python simulation framework for the ISMSIT 2026 submission: **"Comparative Simulation Analysis and Control Benchmarking of Underactuated Robotic Mechanisms."**

It provides a lightweight, non-proprietary computational suite to benchmark nonlinear control strategies across three canonical underactuated systems, serving as the algorithmic precursor for a 32-bit ESP32-based physical deployment.

## Included Benchmarks
1. **Cart-Pole:** Passivity-based energy shaping with local LQR stabilization.
2. **Ball and Beam:** Recursive integrator backstepping for cascaded zero-dynamics.
3. **Planar Bicopter:** Cascaded PD-Lyapunov regulation for aerodynamic differential thrust.

## Installation and Usage
Ensure you have Python 3.8+ installed, then install the required dependencies:
```bash
pip install -r requirements.txt

```bash
python simulation_benchmark.py
