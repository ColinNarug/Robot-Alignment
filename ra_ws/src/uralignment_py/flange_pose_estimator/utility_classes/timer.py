import time


class Timer:
    
    def __init__(self) -> None:
        self.timers = {}


    def start(self, timer_id: str) -> None:
        """
        Start a timer with the given ID.
        """
        if timer_id in self.timers:
            raise ValueError(f"Timer with ID '{timer_id}' is already running.")
        self.timers[timer_id] = time.time()


    def stop(self, timer_id: str, DEBUG_TIME: bool = False) -> float:
        """
        Stop the timer with the given ID and optionally print the elapsed time.
        """
        if timer_id not in self.timers:
            raise ValueError(f"Timer with ID '{timer_id}' was not started.")
        start_time = self.timers.pop(timer_id)
        elapsed_time = time.time() - start_time
        if DEBUG_TIME:
            print(f"-[DEBUG_TIME] Timer '{timer_id}': {elapsed_time} [s]")
        return elapsed_time
    


    