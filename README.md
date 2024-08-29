# VANET-Dynamic-Graph

## Overview
In this project, a dynamic time-based graph is created. The members of this graph include neighboring nodes and edges representing the connections between these nodes. In this project, the graph is updated every 5 seconds. For each node, a score based on centrality is considered for scheduling the sending of messages by vehicles. Messages that exceed a certain threshold are divided into smaller tokens, and then routing is performed based on the efficiency related to the edges, and the tokens are sent.

## Project Objective
The objective of this project is to create a vehicular network simulation based on dynamic, time-based graphs. The project aims to explore and optimize communications between vehicles in a simulated environment.

## Getting Started

### Prerequisites
- [OMNeT++](https://omnetpp.org/) 
- [Veins](https://veins.car2x.org/)
- [SUMO](https://eclipse.dev/sumo/) 
