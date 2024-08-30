from pr2.solver import TaskSolverBase


class TaskSolver(TaskSolverBase):
    def __init__(self) -> None:
        super().__init__()
        # implement your own TaskSolver here
        raise NotImplementedError("Implement your own TaskSolver here")

    def next_action(self, obs: dict) -> dict:
        # implement your own TaskSolver here
        raise NotImplementedError("Implement your own TaskSolver here")
