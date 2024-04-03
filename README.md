# Eight Puzzle Solver

## Overview

This project provides a comprehensive solution for solving the Eight Puzzle problem using various search strategies and heuristics. The Eight Puzzle consists of a 3x3 grid with tiles numbered 1 through 8 and one empty space. The goal is to move the tiles until they are arranged in numerical order. The project includes implementations of depth-first search, breadth-first search, uniform cost search, and A* search algorithms to find solutions. It also includes a method for generating solvable scenarios of the puzzle and evaluating the performance of different heuristics.

## Prerequisites

- Python 3.x
- Pandas library for handling data manipulation and exporting CSV files.
## Installation

- Ensure Python 3 is installed on your system. If not, download and install it from the official Python website.
- Install Pandas using pip:
```bash
pip install pandas
```
## Usage

To use the Eight Puzzle Solver, follow these steps:

- Prepare Puzzle Scenarios: Run the script to generate solvable puzzle scenarios and save them to a CSV file.
- Run the Solver: Execute the main script to solve the puzzles using different search strategies and heuristics. The script will evaluate the performance of each method and save the results to CSV files.
- Analyze Results: Review the CSV files to analyze the performance data, including the number of nodes expanded, solution depth, and fringe size.
## Features

- Implements the Eight Puzzle game mechanics and search problem.
- Provides depth-first, breadth-first, uniform cost, and A* search algorithms.
- Includes four heuristic functions for A* search optimization.
- Generates solvable puzzle scenarios and exports them to a CSV file.
- Evaluates and compares the performance of search algorithms and heuristics.
- Exports detailed and summary performance data to CSV files.
## Contributing

Contributions to improve the Eight Puzzle Solver are welcome. Feel free to fork the repository, make your changes, and submit a pull request with a description of your enhancements.

## License

This project is open source and available under the MIT License. It is intended for educational purposes, and as such, please respect the principles of academic honesty and integrity.
