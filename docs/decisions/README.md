# Decision Log

Every non-trivial decision — scope, sim environment, algorithm choice, message schema — lives here as an ADR.

## Format

Each doc:

```markdown
# NNN — Title

**Status:** Proposed | Accepted | Superseded by NNN
**Date:** YYYY-MM-DD

## Context
Why this decision is on the table.

## Options considered
List with pros/cons.

## Decision
The one chosen. In one sentence.

## Consequences
What now flows from this — what's easier, what's harder, what's ruled out.
```

## Index

| # | Title | Status |
|---|---|---|
| 000 | Scope and layer depth | Accepted |
| 001 | Sim environment | Pending (D1) |
| 002 | Roadmap slot / timing | Pending (D2) |
| 003 | Success target / venue | Pending (D3) |
| 004 | Category structure | Pending (D4) |
| 005 | Intent-parser tech | Pending (D5) |
| 006 | Real hardware stretch | Pending (D6) |
| 010 | State estimator | Blocked on 000 |
| 011 | Data association | Blocked on 000 |
| 012 | Classification fusion | Blocked on 000 |
| 020 | Task decomposer | Blocked on 000 |
| 021 | Allocator | Blocked on 000 |
| 022 | Coverage planner | Blocked on 000 |
| 030 | 3D viewer | Blocked on 000 |
| 031 | Data transport | Blocked on 000 |
| 040 | Interface schemas | Blocked on 001, 010, 020 |
