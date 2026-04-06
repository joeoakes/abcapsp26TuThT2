

from dataclasses import dataclass
from typing import List, Optional, Tuple

from tools_maze import (
    get_redis,
    load_latest_session_id,
    load_static_maze,
    astar,
    validate_plan,
    store_plan,
    session_key,
)

# ollama
import requests
import json

OLLAMA_URL = "http://localhost:11434/api/generate"
OLLAMA_MODEL = "llama3"
# =========================
# 4.1 SHARED STATE
# =========================
@dataclass
class MazeState:
    width: int
    height: int
    cells: List[int]
    start: Tuple[int, int]
    goal: Tuple[int, int]

    current_pos: Tuple[int, int]

    plan: List[str]
    plan_index: int
    need_plan: bool

    action: Optional[str] = None
    reason: Optional[str] = None

    rag_context: Optional[str] = None


# =========================
# REDIS HELPERS
# =========================
def load_plan(r, session_id):
    import json

    raw = r.get(session_key(session_id, "plan"))
    if not raw:
        return []

    return json.loads(raw)


def load_plan_index(r, session_id):
    val = r.get(session_key(session_id, "plan_index"))
    return int(val) if val else 0


def save_plan_index(r, session_id, index):
    r.set(session_key(session_id, "plan_index"), index)


# =========================
# 4.2 EXECUTOR NODE
# =========================
def executor_node(state: MazeState, r, session_id) -> MazeState:
    x, y = state.current_pos

    # Goal check
    if (x, y) == state.goal:
        state.action = "DONE"
        state.reason = "Reached goal"
        return state

    # No plan or exhausted
    if not state.plan or state.plan_index >= len(state.plan):
        state.need_plan = True
        state.action = None
        state.reason = "Need new plan"
        return state

    # Execute next move
    move = state.plan[state.plan_index]
    state.plan_index += 1
    save_plan_index(r, session_id, state.plan_index)

    dx, dy = {
        "UP": (0, -1),
        "DOWN": (0, 1),
        "LEFT": (-1, 0),
        "RIGHT": (1, 0),
    }[move]

    state.current_pos = (x + dx, y + dy)
    state.action = move
    state.reason = "Executing plan"
    state.need_plan = False

    return state


# =========================
# ollama implementation
# =========================
def call_llm_for_plan(state: MazeState) -> List[str]:
    prompt = f"""
You are solving a maze.

Start: {state.current_pos}
Goal: {state.goal}

Return ONLY a JSON list of moves.
Valid moves: ["UP","DOWN","LEFT","RIGHT"]

Example:
["RIGHT","RIGHT","DOWN"]
"""

    try:
        response = requests.post(
            OLLAMA_URL,
            json={
                "model": OLLAMA_MODEL,
                "prompt": prompt,
                "stream": False
            },
            timeout=10
        )

        text = response.json().get("response", "").strip()

        # Extract JSON safely
        start = text.find("[")
        end = text.rfind("]")

        if start == -1 or end == -1:
            return []

        plan = json.loads(text[start:end+1])
        return [m.upper() for m in plan]

    except Exception as e:
        print("Ollama error:", e)
        return []


# =========================
# 4.3 PLANNER NODE
# =========================
def planner_node(state: MazeState, r, session_id) -> MazeState:
    if not state.need_plan:
        return state

    # 1. A* FIRST
    plan = astar(
        state.width,
        state.height,
        state.cells,
        state.current_pos,
        state.goal,
    )

    if plan:
        validate_plan(
            plan,
            state.width,
            state.height,
            state.cells,
            state.current_pos[0],
            state.current_pos[1],
        )

        store_plan(r, session_id, plan)

        state.plan = plan
        state.plan_index = 0
        state.need_plan = False
        state.reason = "A* plan generated"

        return state

    # 2. Optional RAG (skipped)

    # 3. LLM fallback
    llm_plan = call_llm_for_plan(state)

    try:
        validate_plan(
            llm_plan,
            state.width,
            state.height,
            state.cells,
            state.current_pos[0],
            state.current_pos[1],
        )

        store_plan(r, session_id, llm_plan)

        state.plan = llm_plan
        state.plan_index = 0
        state.need_plan = False
        state.reason = "LLM plan generated"
        return state

    except Exception:
        pass

    # 4. Final fallback
    state.plan = []
    state.plan_index = 0
    state.need_plan = False
    state.reason = "No valid plan found"

    return state


# =========================
# 4.4 GRAPH WIRING
# =========================
def run_multi_agent():
    r = get_redis()
    session_id = load_latest_session_id(r)

    maze = load_static_maze(r, session_id)

    state = MazeState(
        width=maze["width"],
        height=maze["height"],
        cells=maze["cells"],
        start=(maze["start_x"], maze["start_y"]),
        goal=(maze["goal_x"], maze["goal_y"]),
        current_pos=(maze["start_x"], maze["start_y"]),
        plan=load_plan(r, session_id),
        plan_index=load_plan_index(r, session_id),
        need_plan=True,
    )

    while True:
        # ENTRY → EXECUTOR
        state = executor_node(state, r, session_id)

        if state.action == "DONE":
            break

        # CONDITIONAL EDGE
        if state.need_plan:
            state = planner_node(state, r, session_id)

        # Loop → back to executor

    return state


# =========================
# RUN
# =========================
if __name__ == "__main__":
    final_state = run_multi_agent()

    print("Final position:", final_state.current_pos)
    print("Final action:", final_state.action)
    print("Reason:", final_state.reason)
