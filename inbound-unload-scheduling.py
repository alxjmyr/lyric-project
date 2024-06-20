from ortools.sat.python import cp_model
from json import load
from collections import defaultdict


with open("./inbound-trailers.json", "r") as f:
    trailer_data = load(f)


def estimate_unload_time(trailer):
    """
    creates dummy esimtate for unloading a trailer in a given line type
    In reallife this is managed by a machine learning model

    """

    auto = {"auto": 35, "ncon": 5}
    manual = {"auto": 18, "ncon": 9}
    ncon = {"auto": 7, "ncon": 10}

    if trailer["n_ncon"] / trailer["ttl_ctns"] > 0.15:
        return {
            "auto": -1,
            "manual": round(
                (trailer["n_auto"] / manual["auto"])
                + (trailer["n_ncon"] / manual["ncon"])
            ),
            "ncon": round(
                (trailer["n_auto"] / ncon["auto"]) + (trailer["n_ncon"] / ncon["ncon"])
            ),
        }
    else:
        return {
            "auto": round(
                (trailer["n_auto"] / auto["auto"]) + (trailer["n_ncon"] / auto["ncon"])
            ),
            "manual": round(
                (trailer["n_auto"] / manual["auto"])
                + (trailer["n_ncon"] / manual["ncon"])
            ),
            "ncon": round(
                (trailer["n_auto"] / ncon["auto"]) + (trailer["n_ncon"] / ncon["ncon"])
            ),
        }


# set up input data
# "machine" defs
# 0,1 both automated receipt doors
# 2 manual dock
# 3 non con dock
# 4 yard move in / out of door
num_doors = 5
num_unloads = len(trailer_data["trailers"])


# rules
# trailers > 15% non-con can't go in auto
# anything could potentially go through non-con or manual doors

# each unload consists of 1 task (unload)
# unlaoding each trailer is treated  as a job with multiple options for where to unload
unloads = []

for trailer in trailer_data["trailers"]:
    unload_estimates = estimate_unload_time(trailer)

    if unload_estimates["auto"] == -1:  # trailer can't go on auto dock
        task = [
            [(2, 4)],
            [(unload_estimates["manual"], 2), (unload_estimates["ncon"], 3)],
            [(2, 4)],
        ]
    else:
        task = [
            [(2, 4)],
            [
                (unload_estimates["auto"], 0),
                (unload_estimates["auto"], 1),
                (unload_estimates["manual"], 2),
                (unload_estimates["ncon"], 3),
            ],
            [(2, 4)],
        ]
    # check if trailer is for live unload and set required start interval for unload
    if trailer["is_live"]:
        unloads.append(
            {
                # if its a live trailer there is no need to complete the in/out yard moves from door
                "tasks": task,
                "live_start": trailer["live_start"],
                "live_end": trailer["live_end"],
            }
        )
    else:
        unloads.append({"tasks": task, "live_start": -1, "live_end": -1})

# for unload in unloads:
#     print(unload)

num_jobs = len(unloads)
all_jobs = range(num_jobs)

num_machines = 5
all_machines = range(num_machines)

# flexible job shop implementation using or-tools CP solver
model = cp_model.CpModel()

# need to calculate max horizon for completion
# this might not be totally right when accounting for
# constraint of live unloads
horizon = 0
jobs = unloads
for job in jobs:
    for task in job["tasks"]:
        max_task_duration = 0
        for alternative in task:
            max_task_duration = max(max_task_duration, alternative[0])
        horizon += max_task_duration

print("Horizon = %i" % horizon)

# Global storage of variables.
intervals_per_resources = defaultdict(list)
starts = {}  # indexed by (job_id, task_id).
presences = {}  # indexed by (job_id, task_id, alt_id).
job_ends = []

# Scan the jobs and create the relevant variables and intervals.
for job_id in all_jobs:
    job = jobs[job_id]
    num_tasks = len(job["tasks"])
    previous_end = None
    for task_id in range(num_tasks):
        task = job["tasks"][task_id]

        min_duration = task[0][0]
        max_duration = task[0][0]

        num_alternatives = len(task)
        all_alternatives = range(num_alternatives)

        for alt_id in range(1, num_alternatives):
            alt_duration = task[alt_id][0]
            min_duration = min(min_duration, alt_duration)
            max_duration = max(max_duration, alt_duration)

        # Create main interval for the task.
        suffix_name = "_j%i_t%i" % (job_id, task_id)
        if job["live_start"] > -1:
            start = model.NewIntVar(
                job["live_start"], job["live_end"], "start" + suffix_name
            )
        else:
            start = model.NewIntVar(0, horizon, "start" + suffix_name)
        duration = model.NewIntVar(min_duration, max_duration, "duration" + suffix_name)
        end = model.NewIntVar(0, horizon, "end" + suffix_name)
        interval = model.NewIntervalVar(start, duration, end, "interval" + suffix_name)

        # Store the start for the solution.
        starts[(job_id, task_id)] = start

        # Add precedence with previous task in the same job.
        if previous_end is not None:
            model.Add(start >= previous_end)
        previous_end = end

        # alternative intervals.
        if num_alternatives > 1:
            l_presences = []
            for alt_id in all_alternatives:
                alt_suffix = "_j%i_t%i_a%i" % (job_id, task_id, alt_id)
                l_presence = model.NewBoolVar("presence" + alt_suffix)
                # l_start = model.NewIntVar(0, horizon, "start" + alt_suffix)
                if job["live_start"] > -1:
                    l_start = model.NewIntVar(
                        job["live_start"], job["live_end"], "start" + suffix_name
                    )
                else:
                    l_start = model.NewIntVar(0, horizon, "start" + suffix_name)
                l_duration = task[alt_id][0]
                l_end = model.NewIntVar(0, horizon, "end" + alt_suffix)
                l_interval = model.NewOptionalIntervalVar(
                    l_start, l_duration, l_end, l_presence, "interval" + alt_suffix
                )
                l_presences.append(l_presence)

                # Link the master variables with the local ones.
                model.Add(start == l_start).OnlyEnforceIf(l_presence)
                model.Add(duration == l_duration).OnlyEnforceIf(l_presence)
                model.Add(end == l_end).OnlyEnforceIf(l_presence)

                # Add the local interval to the right machine.
                intervals_per_resources[task[alt_id][1]].append(l_interval)

                # Store the presences for the solution.
                presences[(job_id, task_id, alt_id)] = l_presence

            # Select exactly one presence variable.
            model.AddExactlyOne(l_presences)
        else:
            intervals_per_resources[task[0][1]].append(interval)
            presences[(job_id, task_id, 0)] = model.NewConstant(1)

    job_ends.append(previous_end)

# Create machines constraints.
for machine_id in all_machines:
    intervals = intervals_per_resources[machine_id]
    if len(intervals) > 1:
        model.AddNoOverlap(intervals)

# Makespan objective
makespan = model.NewIntVar(0, horizon, "makespan")
model.AddMaxEquality(makespan, job_ends)
model.Minimize(makespan)


# Solve model.
class SolutionPrinter(cp_model.CpSolverSolutionCallback):
    """Print intermediate solutions."""

    def __init__(self):
        cp_model.CpSolverSolutionCallback.__init__(self)
        self.__solution_count = 0

    def on_solution_callback(self):
        """Called at each new solution."""
        print(
            "Solution %i, time = %f s, objective = %i"
            % (self.__solution_count, self.WallTime(), self.ObjectiveValue())
        )
        self.__solution_count += 1


solver = cp_model.CpSolver()
solution_printer = SolutionPrinter()
status = solver.Solve(model, solution_printer)

# Print final solution.
for job_id in all_jobs:
    print("Job %i:" % job_id)
    for task_id in range(len(jobs[job_id]["tasks"])):
        start_value = solver.Value(starts[(job_id, task_id)])
        machine = -1
        duration = -1
        selected = -1
        for alt_id in range(len(jobs[job_id]["tasks"][task_id])):
            if solver.Value(presences[(job_id, task_id, alt_id)]):
                duration = jobs[job_id]["tasks"][task_id][alt_id][0]
                machine = jobs[job_id]["tasks"][task_id][alt_id][1]
                selected = alt_id
        print(
            "  task_%i_%i starts at %i (alt %i, machine %i, duration %i)"
            % (job_id, task_id, start_value, selected, machine, duration)
        )

print("Solve status: %s" % solver.StatusName(status))
print("Optimal objective value: %i" % solver.ObjectiveValue())
print("Statistics")
print("  - conflicts : %i" % solver.NumConflicts())
print("  - branches  : %i" % solver.NumBranches())
print("  - wall time : %f s" % solver.WallTime())
