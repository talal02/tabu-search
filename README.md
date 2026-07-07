# Tabu Search — Capacitated Vehicle Routing Problem (CVRP)

A C++ implementation of the Tabu Search metaheuristic applied to the Capacitated Vehicle Routing Problem (CVRP), built as a group project for a course assignment.

## Overview

The program generates a random CVRP instance (customers with demands, scattered around a depot) and solves it in three stages:

1. **Greedy Construction** — builds an initial feasible solution by repeatedly assigning each vehicle to its nearest unrouted, capacity-feasible customer until every customer is served.
2. **Intra-Route Local Search** — improves each vehicle's own route by searching for beneficial customer-position swaps within that route (a 2-opt-style local search).
3. **Tabu Search** — improves the solution further by considering moves of customers *between* vehicle routes, tracking recently used moves in a tabu matrix to avoid cycling back to previously visited solutions, and keeping the best solution found across iterations.

At each stage the program prints the resulting route per vehicle and the total solution cost, along with the total running time.

## Problem Setup

- Customers and the depot are placed at random (x, y) coordinates.
- Each customer has a random demand.
- Each vehicle has a fixed capacity and must not exceed it across the customers on its route.
- Cost between any two points is the Euclidean distance, precomputed into a distance matrix.

Default instance size (configurable in `main()`):
- 500 customers
- 2,500 vehicles
- Vehicle capacity: 50
- Tabu list size: 100

## Project Structure

| File | Description |
|---|---|
| `main.cpp` | Full implementation: `Node`, `Vehicle`, and `Solution` structs, the greedy construction heuristic, intra-route local search, and the Tabu Search algorithm. |
| `Tabu Search.pdf` | Project write-up / assignment description. |

## Requirements

- A C++ compiler supporting C++11 or later (uses `std::chrono` for timing).

## Building

```bash
g++ -std=c++17 -O2 main.cpp -o tabu_search
```

## Usage

Run the compiled binary directly; the problem instance is generated internally, so no input is required:

```bash
./tabu_search
```

The program will print:
- The initial greedy solution's routes and cost.
- The routes and cost after intra-route local search.
- The routes and cost after Tabu Search.
- Total run time in milliseconds.

## Algorithm Notes

- **Tabu list**: implemented as a matrix keyed by customer-pair indices, with entries decremented each iteration until they expire, temporarily forbidding moves that would recreate a recently broken edge.
- **Stopping criteria**: Tabu Search stops after a fixed number of iterations or after a set number of consecutive non-improving iterations.
- **Best-solution tracking**: the best feasible solution found during Tabu Search is saved separately and restored at the end, so the final result is never worse than the best solution encountered.
