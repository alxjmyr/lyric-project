# lyric-projects
Mock/Demo implementation(s) for Lyric project presentation

## Includes two potential mock problems-
Shipt CVRP usecase was presented to Sara / Ganesh on 6/25

1) Shipt Routing / Dispatch (CVRP)
* `shipt_vrp.py` includes basic model formulation and a script to unpack routing results into some plots of a market
* `vrp_utils.py` includes functions to generate synthetic inputs for the problem, as well as a solution parser and evaluator. problem inputs / constraints can be controlled from input data method.


2) Target Inbound Trailer Scheduling
* Simple mock implementation of managing trailer unload scheduling for a Target distribution center. Modeled as a flexible job shop problem with considerations / modifications to handle live unloads with a pre-determined start time requirement.

## Run Book
1) Clone repository
2) run `poetry install` to install dependencies
