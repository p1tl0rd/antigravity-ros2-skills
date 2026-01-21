---
trigger: always_on
---

# Technical Standards (Universal)

Technical standards applied to all source code.

## 1. Naming Conventions

*   **Follow Language Standards:**
    *   **Python/C++:** `snake_case` for variables/functions. `PascalCase` for Classes.
    *   **JS/TS/Java:** `camelCase` for variables/functions. `PascalCase` for Classes.
    *   **C#/.NET:** `PascalCase` for Methods and Classes. `camelCase` for local variables/parameters.
    *   **Constants:** `SCREAMING_SNAKE_CASE` (Globally).
*   **Boolean:** prefix with `is`, `has`, `can`, `should` (e.g., `is_valid` or `isValid`).
*   **Language:** 100% English for all code identifiers.

✅ `customer_address` (Python) / `customerAddress` (JS)
✅ `CalculateTotal()` (C# Method)
❌ `addr`, `val`, `func1()`

## 2. Function & Logic Flow

*   **Early Return:** Avoid deep nesting (Arrow code). Check conditions and return/throw early.
*   **Single Responsibility:** 1 function = 1 task.
*   **Size Limit:** Keep functions short (e.g., < 50 lines).
*   **Parameter Limit:** Max 3 parameters. Use Structs/Objects/Data Classes if more are needed.

## 3. Type Safety

*   **No Magic Numbers:** ❌ `if status == 1` ✅ `if status == STATUS.PENDING`
*   **Strict Typing:**
    *   **Statically Typed (C++/Java/Go):** Always use explicit types.
    *   **Dynamically Typed (Python/JS):** Use Type Hints (`typing` module) or JSDoc/TS types where possible.
*   **Immutability:** Prefer immutable data structures where performance allows.

## 4. Error Handling

*   **Don't Swallow Errors:** Always log or re-throw in try/catch blocks.
*   **Structured Logging:** Include context (Node Name, Component ID, Input Params).
*   **Fail Fast:** Report errors immediately rather than propagating corrupted state.

## 5. Comments & Documentation

*   **Why > What:** Explain the intent and "Why", not just the syntax.
*   **TODO/FIXME:** Mark incomplete work clearly.
*   **Docstrings:** Required for all public API methods and classes.